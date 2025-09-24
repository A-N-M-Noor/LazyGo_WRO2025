import rclpy
from rclpy.node import Node

from threading import Thread
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

from std_msgs.msg import String, Int8
from math import pi, radians, degrees, sin, cos
import time

class Parking(Node):
    def __init__(self):
        super().__init__('parking')
        self.get_logger().info('Parking node has been started.')
        
        self.esp_cmd_pub = self.create_publisher(Int8, '/esp_cmd', 10)
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 1)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)
        
        self.obj_sub = self.create_subscription(String, '/closest_obj', self.obj_callback, 1)
        self.obj = "N"
        
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []
        
        self.pos = Vector3()
        
        self.state = "Idle"
        self.park_dir = "_"
        
        self.parking_thread = Thread(target=self.parking_loop)
        self.parking_thread.daemon = True
        self.parking_thread.start()
    
    def parking_loop(self):
        while rclpy.ok():
            if(self.state == "toTurn"):
                l = self.get_dst(-90)
                r = self.get_dst(90)
                
                if(l > r):
                    self.park_dir = "R"   # parking is on the right
                else:
                    self.park_dir = "L"  # parking is on the left
                
                if(self.park_dir == "L"):
                    self.send_c(1)
                    self.state = "TurningRight"
                else:
                    self.send_c(2)
                    self.state = "TurningLeft"
            
            if(self.state == "Turned"):
                if(self.obj == "G" and self.park_dir == "L"):
                    self.send_c(3)  # start parking
                    self.state = "Moving"
                elif(self.obj == "G" and self.park_dir == "R"):
                    self.send_c(4)  # start parking
                    self.state = "Moving"
                elif(self.obj == "R" and self.park_dir == "L"):
                    self.send_c(4)  # start parking
                    self.state = "Moving"
                elif(self.obj == "R" and self.park_dir == "R"):
                    self.send_c(3)  # start parking
                    self.state = "Moving"
                elif(self.obj == "N"):
                    self.send_c(3)
                    self.state = "Moving"
            
            time.sleep(0.1)
    
    def send_c(self, c:int):
        msg = Int8()
        msg.data = c
        self.esp_cmd_pub.publish(msg)
        
    def cmd_callback(self, msg: String):
        self.get_logger().info(f'Received command: {msg.data}')
        if(self.state == "Idle" and msg.data == "Boot"):
            self.state = "toTurn"
        
        if(msg.data == "Turned"):
            self.state = "Turned"
        if(msg.data == "Done"):
            self.state = "Idle"
            self.park_dir = "_"
            self.send_c(5)
            self.cmd_pub.publish(String(data="start"))
    
    def lidar_callback(self, msg: LaserScan):
        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angInc = msg.angle_increment
        self.ranges = msg.ranges
        self.ints = msg.intensities
    
    def pos_callback(self, msg: Vector3):
        self.pos = msg
        
    def obj_callback(self, msg: String):
        self.obj = msg.data
    
    def get_dst(self, ang):
        i = self.a2i(radians(ang))
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
    node = Parking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()