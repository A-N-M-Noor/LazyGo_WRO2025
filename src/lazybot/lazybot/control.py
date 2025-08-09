import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import numpy as np

from std_msgs.msg import Float32, Float32MultiArray, String
from lazy_interface.msg import BotDebugInfo, LidarTowerInfo
from math import pi, radians, degrees, sin, cos
import time
from std_msgs.msg import Header

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 10)
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.obj_sub = self.create_subscription(String, '/closest_obj', self.obj_callback, 1)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)

        self.objPub = self.create_publisher(Float32MultiArray, '/obj_data', 1)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 1)
        self.steer_pub = self.create_publisher(Float32, '/steer', 1)

        self.pubDebug = self.create_publisher(BotDebugInfo, '/lazybot/debug', 10)
        
        self.create_timer(0.025, self.odom_loop)
        
        self.IS_SIM = True
        
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []

        self.maxSpeed : float = 0.45
        self.speed : float = 0.0
        self.strAngle : float = 0.0
        self.strRange = 1.0
        self.str_ang_thresh = 60.0
        self.strSpd = 0.65
        
        self.prec = 81
        self.skip1 = 2
        self.skip2 = 1

        self.dangerDist = 0.25
        self.dangerAng = [25.0, 90.0]

        self.castRange = [0.15, 0.18]
        self.castR = 0.25
        self.lookRng = radians(80.0)
        self.lookRngS = radians(90.0)
        self.targetAng = 0.0
        self.targetD = 0.0
        
        self.pos = Vector3()
        self.gotWallD = False
        self.reached = True
        self.lapCount = 0
        self.running = False
        self.endOffset = [0.0, 0.0]

        self.objs = []
        self.cont_stack = []
        
        self.closest = "N"

        self.get_logger().info('Control node has been started.')
        self.lastTime = time.time()

    def odom_loop(self):
        if not self.running or not self.gotWallD:
            self.pubDrive(disable=True)
            return
        isInside = abs(self.pos.x) < 0.5 and self.pos.y > self.endOffset[0] and self.pos.y < self.endOffset[1]
        if not self.reached:
            if isInside:
                self.reached = True
                self.lapCount += 1
                self.get_logger().info(f"Lap {self.lapCount} completed!")
        
        if not isInside and self.reached:
            self.get_logger().info("Moving on...")
            self.reached = False
        
        if self.lapCount >= 3:
            self.running = False
            self.get_logger().info("Reached the destination, stopping the robot.")
            self.pubDrive(disable=True)
            return
        
        self.pubDrive()
    
    def cmd_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == "start":
            self.lapCount = 0
            self.reached = True
            self.running = True
            self.gotWallD = False
            self.get_logger().info("Starting the robot.")
        elif command == "stop":
            self.running = False
            self.get_logger().info("Stopping the robot.")
            
    def obj_callback(self, msg: String):
        self.closest = msg.data
        
    def lidar_callback(self, msg: LaserScan):
        if not self.running:
            return
        
        self.dt = time.time() - self.lastTime
        self.lastTime = time.time()

        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angInc = msg.angle_increment

        self.ranges = msg.ranges
        self.ints = msg.intensities
        
        if(not self.gotWallD):
            wi = self.a2i(0.0)
            if(self.IS_SIM): self.ints[wi] = 1.0
            
            if(self.ints[wi] <= 0.1 or self.ranges[wi] > 3.0):
                self.ranges[wi] = self.fix_missing(wi)
                self.ints[wi] = 1.0
            
            self.endOffset[0] = self.ranges[wi] - 1.5
            self.endOffset[1] = self.ranges[wi] - 1.0
            
            self.get_logger().info(f"Wall Distance: {self.ranges[wi]:.2f}, End Offset: {self.endOffset}")
            
            self.gotWallD = True
        
        self.castR = self.remap(self.speed/self.maxSpeed, 0.45, 1, self.castRange[0], self.castRange[1])

        self.pubObjData()

        maxD, tA = self.getMaxDOBJ()
        dS = self.dangerSense()

        if(dS):
            tA -= dS * self.strRange * 1.0

        delta = abs(tA-self.targetAng)
        self.targetAng = tA if(delta > 0.5) else self.lerp(self.targetAng, tA, 35*self.dt)
        self.targetAng = self.lerp(self.targetAng, tA, 0.1)
        self.targetAng = degrees(self.targetAng)
        self.targetD = maxD

        sAng = self.remap(self.targetAng, -self.str_ang_thresh, self.str_ang_thresh, -self.strRange, self.strRange)
        self.strAngle = self.lerp(self.strAngle, sAng, min(self.dt*5, 1.0)) if self.IS_SIM else sAng
        mult2 = self.remap(maxD, 1.0, 2.0, 0.65, 1.0)

        self.speed = self.maxSpeed * mult2

        self.pubDebugPoint()
    
    def pos_callback(self, msg: Vector3):
        self.pos.x = msg.x
        self.pos.y = msg.y
        self.pos.z = msg.z
    
    def getMaxDOBJ(self):
        _max = {"dst": 0, "ang": 0}
        chkRng = self.indRng(-self.lookRng, self.lookRng)
        
        self.objs = []
        self.cont_stack = []

        remove = "None" 
        if self.closest == "G":
            remove = "Left"
        elif self.closest == "R":
            remove = "Right"
        
        for i in range(chkRng[0], chkRng[1], self.skip1):
            if(self.IS_SIM): self.ints[i] = 1.0
            if(self.ints[i] <= 0.1 or self.ranges[i] > 3.0):
                self.ranges[i] = self.fix_missing(i)
                self.ints[i] = 1.0

            if(i < 1 or i >= len(self.ranges) or self.ints[i] <= 0.05 or self.ranges[i] > 3.0):
                continue
            
            dt = self.marching(i)
            if(dt["dst"] > _max["dst"]):
                _max = dt

            objectFound = self.detectContrast(i)
            
            if objectFound:
                if remove == "Left":
                    _max = dt
                if remove == "Right":
                    return _max["dst"], _max["ang"]
                
        return _max["dst"], _max["ang"]
    
    
    def detectContrast(self, i):
        slope = (self.ranges[i] - self.ranges[i-self.skip1]) / self.skip1
        if(slope < -0.15):
            self.cont_stack.append(i)
        elif(slope > 0.15):
            if(len(self.cont_stack) > 0):
                pop = self.cont_stack.pop()
                mid = (i + pop) // 2
                sz = self.ranges[mid] * abs(i-pop)*self.angInc
                if( sz > 0.03 and sz < 0.1):
                    ang = self.i2a(mid)
                    self.objs.append({
                        "index": mid,
                        "ang": ang,
                        "dst": self.ranges[mid],
                        "x": self.ranges[mid] * cos(ang),
                        "y": self.ranges[mid] * sin(ang),
                        "intensity": self.ints[mid]
                    })
                if(self.ranges[mid] < 1.0):
                    return True
        return False

    def marching(self, indx):
        rng = [indx-self.prec*self.skip2, indx+self.prec*self.skip2]

        targetRay = {
            "dst": self.ranges[indx],
            "ang": self.i2a(indx)
        }
        
        _min = {"dst": 1000, "ang": 0}
        for i in range(rng[0], rng[1], self.skip2):
            if(self.IS_SIM): self.ints[i] = 1.0
            if(i >= 0 and i < len(self.ranges)):
                ray = {
                    "dst": self.ranges[i],
                    "ang": self.i2a(i)
                }

                hit = self.hitC(targetRay, ray, self.castR)
                if(hit):
                    if(hit < _min["dst"]):
                        _min = {
                            "dst": ray["dst"],
                            "ang": targetRay["ang"]
                        }
        if _min["dst"] >= 1000:
            return {"dst": targetRay["dst"], "ang": targetRay["ang"]}
        else:
            return {"dst": _min["dst"], "ang": _min["ang"]}
    
    def fix_missing(self, i):
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

    def hitC(self, rayPnt, checkPnt, R):
        dTheta = abs(rayPnt["ang"] - checkPnt["ang"])     
        if(checkPnt["dst"] <= 0.0):
            return False   
        collAng = R/checkPnt["dst"]
        if(dTheta <= collAng):
            return checkPnt["dst"]
        
        return False
    
    def dangerSense(self):
        for i in range(len(self.ranges)):
            if(self.ranges[i] < self.dangerDist):
                ang = degrees(self.i2a(i))
                if(ang > self.dangerAng[0] and ang < self.dangerAng[1]):
                    return self.remap(self.ranges[i], 0, self.dangerDist, 1.0, 0.0)
                if(ang > -self.dangerAng[1] and ang < -self.dangerAng[0]):
                    return -self.remap(self.ranges[i], 0, self.dangerDist, 1.0, 0.0)
        return False

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

    def pubObjData(self):
        obj_data = Float32MultiArray()
        obj_data.data = []
        self.objs.sort(key=lambda x: x["dst"], reverse=True)
        for obj in self.objs:
            obj_data.data.append(obj["ang"])
        
        self.objPub.publish(obj_data)

    def pubDebugPoint(self):
        msg = BotDebugInfo()
        msg.target_angle = self.targetAng
        msg.target_distance = self.targetD
        msg.cast_radius = self.castR
        msg.danger_distance = self.dangerDist
        msg.danger_angles = self.dangerAng
        
        msg.towers = []
        for obj in self.objs:
            tower = LidarTowerInfo()
            tower.color = self.closest
            tower.index = obj["index"]
            tower.ang = obj["ang"]
            tower.dst = obj["dst"]
            tower.x = obj["x"]
            tower.y = obj["y"]
            msg.towers.append(tower)
        self.pubDebug.publish(msg)

    def i2a(self, i):
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

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()