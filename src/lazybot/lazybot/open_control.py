import rclpy
from threading import Thread
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

from std_msgs.msg import Float32, Float32MultiArray, String, Int8MultiArray, Int8
from lazy_interface.msg import BotDebugInfo, LidarTowerInfo
from math import pi, radians, degrees, sin, cos
import time

class OpenNode(Node):
    def __init__(self):
        super().__init__('open_control')
        
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)

        self.throttle_pub = self.create_publisher(Float32, '/throttle', 1)
        self.steer_pub = self.create_publisher(Float32, '/steer', 1)
        
        self.create_timer(0.025, self.odom_loop)
        self.create_timer(0.025, self.control_loop)

        self.IS_SIM = False
        
        self.dir = 0 #CCW : 1 ; CW : -1 ; None : 0
        
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []

        self.maxSpeed : float = 0.65
        self.speed : float = 0.0
        self.strAngle : float = 0.0
        self.strRange = 1.0
        
        self.front_dist_thresh = 0.25

        self.new_lidar_val = False

        self.pos = Vector3()
        self.sectionAngle = 0.0
        self.turning = False
        self.gotWallD = False
        self.reached = True
        self.lapCount = 0
        self.targetLap = 3
        self.running = False
        self.endOffset = [0.0, 0.0]

        self.get_logger().info('Open Control node has been started.')
        self.lastTime = time.time()
    
    def control_loop(self):
        if not self.running:
            self.pubDrive(disable=True)
            return
        
        err = self.sectionAngle - self.pos.z
        if(abs(err) < 30):
            self.turning = False
        sA = self.remap(err, -radians(90), radians(90), -self.strRange, self.strRange)
        self.strAngle = self.clamp(sA, -self.strRange, self.strRange)
        
        frontDist = self.get_dst(0)
        leftDist = self.get_dst(-90)
        rightDist = self.get_dst(90)
        
        if(frontDist < self.front_dist_thresh):
            if(self.dir == 0):                
                if(leftDist > rightDist):
                    self.dir = 1
                    self.get_logger().info("Setting direction to CCW.")
                else:
                    self.dir = -1
                    self.get_logger().info("Setting direction to CW.")
                
            if(self.dir == 1):
                self.sectionAngle = self.sectionAngle + pi/2
            else:
                self.sectionAngle = self.sectionAngle - pi/2
            
            self.turning = True
            self.get_logger().info(f"Section angle changed: {degrees(self.sectionAngle)}")
        else:            
            pos = leftDist - rightDist
            self.strAngle = self.clamp(
                val = self.remap(pos, -0.5, 0.5, -self.strRange/2, self.strRange/2), 
                mini = -self.strRange/2,
                maxi = self.strRange/2
            )
        
        self.speed = self.remap(
            old_val = abs(self.strAngle), 
            old_min = 0, 
            old_max = self.strRange,
            new_min = self.maxSpeed,
            new_max = self.maxSpeed * 0.5
        )
        
        self.pubDrive()
        
    def odom_loop(self):
        if not self.running or not self.gotWallD:
            self.pubDrive(disable=True)
            return
        isInside = abs(self.pos.x) < 0.75 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
        if not self.reached:
            if isInside:
                self.reached = True
                self.lapCount += 1
                self.get_logger().info(f"Lap {self.lapCount} completed!")
        
        if not isInside and self.reached:
            self.get_logger().info("Moving on...")
            self.reached = False
        
        if self.lapCount >= self.targetLap:
            self.speedCap = 0.35
            
            self.running = False
            self.get_logger().info("Reached the destination, stopping the robot.")
            self.pubDrive(disable=True)
            return
        
        if abs(self.pos.z - self.sectionAngle) > radians(80):
            if(self.pos.z > self.sectionAngle):
                self.sectionAngle = self.sectionAngle + pi/2
            else:
                self.sectionAngle = self.sectionAngle - pi/2
            self.get_logger().info(f"Section angle changed: {degrees(self.sectionAngle)}")
    
    def pos_callback(self, msg: Vector3):
        self.pos.x = msg.x
        self.pos.y = msg.y
        self.pos.z = msg.z
        
    def cmd_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == "boot":
            self.lapCount = 0
            self.reached = True
            self.running = True
            self.gotWallD = False
            self.get_logger().info("Starting the robot.")
            self.cmd_pub(String(data="start_open"))
        elif command == "stop":
            self.running = False
            self.get_logger().info("Stopping the robot.")
    
    def lidar_callback(self, msg: LaserScan):
        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angInc = msg.angle_increment

        self.ranges = msg.ranges
        self.ints = msg.intensities
        
        if not self.running:
            return
        
        if(not self.gotWallD):
            wi = self.a2i(0.0)
            if(self.IS_SIM): self.ints[wi] = 1.0
            
            if(self.ints[wi] <= 0.1 or self.ranges[wi] > 3.0):
                self.ranges[wi] = self.fix_missing(wi)
                self.ints[wi] = 1.0
            
            self.endOffset[0] = self.ranges[wi] - 1.5
            self.endOffset[1] = self.ranges[wi] - 0.5
            
            self.get_logger().info(f"Wall Distance: {self.ranges[wi]:.2f}, End Offset: {self.endOffset}")
            
            self.gotWallD = True
            
            isInside = abs(self.pos.x) < 0.75 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
            
            if not isInside:
                self.lapCount = -1
                self.get_logger().info(f"Robot is outside the area, setting lap count to {self.lapCount}.")

    
    def fix_missing(self, i):
        if(i < 0 or i >= len(self.ints)):
            return 0
        if(self.ints[i] > 0.05 and self.ranges[i] <= 3.0):
            return self.ranges[i]
        first = i
        last = i
        while(first > 0 and (self.ints[first] == 0.0 or self.ranges[first] > 3.0)):
            first -= 1
            if(first < 0):
                first = 0
                break
        while(last < len(self.ints) and (self.ints[last] == 0.0 or self.ranges[last] > 3.0)):
            last += 1
            if(last >= len(self.ints)):
                last = len(self.ints) - 1
                break
        
        if(self.ints[first] == 0.0 or self.ints[last] == 0.0):
            return 0
        if(first == last):
            return self.ranges[first]
        
        return self.lerp(self.ranges[first], self.ranges[last], (i-first)/(last-first))

    def pubDrive(self, disable = False):
        if disable:
            self.speed = 0.0
            self.strAngle = 0.0
        throttle_msg = Float32()
        throttle_msg.data = self.speed
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = self.strAngle
        self.steer_pub.publish(steer_msg)
        
    def get_dst(self, ang):
        i = self.a2i(radians(ang))
        if(self.ints[i] <= 0.05 or self.ranges[i] > 3.0):
            self.ranges[i] = self.fix_missing(i)
            self.ints[i] = 1.0
        return self.ranges[i]
    
    def i2a(self, i, deg = False):
        if deg:
            return degrees(self.angMin + self.angInc*i)
        return self.angMin + self.angInc*i
    
    def indRng(self, Min, Max):
        return [self.a2i(Min), self.a2i(Max)]

    def a2i(self, ang):
        return round((ang -self.angMin) / self.angInc)

    def clamp(self, val, mini, maxi):
        tMin = mini
        tMax = maxi
        if (mini > maxi):
            tMin = maxi
            tMax = mini
        if (val < tMin):
            return tMin
        if (val > tMax):
            return tMax
        return val
            
    def remap(self, old_val, old_min, old_max, new_min, new_max):
        newVal = (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min
        return self.clamp(newVal, new_min, new_max)

    def lerp(self, a, b, t):
        return self.clamp(a + (b - a) * t, a, b)
    
    def norm_ang(self, a):
        return (a + pi) % (2 * pi) - pi
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = OpenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()