import rclpy
from rclpy.node import Node

from threading import Thread
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from lazy_interface.msg import BotDebugInfo, LidarTowerInfo

from std_msgs.msg import String, Int8, Int16MultiArray
from math import pi, radians, degrees, sin, cos, inf
import time

class Parking(Node):
    def __init__(self):
        super().__init__('parking')
        self.get_logger().info('Parking node has been started.')
        
        self.esp_cmd_pub = self.create_publisher(Int8, '/esp_cmd', 10)
        self.cmd_sub = self.create_subscription(String, '/cmd', self.cmd_callback, 1)
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.dir_dst_pub = self.create_publisher(Int16MultiArray, '/dir_dst', 1)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)
        self.offs_pub = self.create_publisher(Vector3, '/initial_offset', 10)
        
        self.debug_sub = self.create_subscription(BotDebugInfo, '/lazybot/debug', self.debug_callback, 10)
        
        self.obj_sub = self.create_subscription(String, '/closest_obj', self.obj_callback, 1)
        self.obj = "N"
        self.objs = []
        
        self.angMin = -pi
        self.angMax = pi
        self.angInc = radians(0.5)
        self.ranges = []
        self.ints = []
        
        self.pos = Vector3()
        
        self.state = "Idle"
        self.park_dir = "_"
        self.parking_offs_rng = [0.95, 1.45]
        self.parking_offs = self.parking_offs_rng[0]
        
        self.parking_thread = Thread(target=self.parking_loop)
        self.parking_thread.daemon = True
        self.parking_thread.start()
    
    def parking_loop(self):
        while rclpy.ok():
            if(self.state == "toTurn"):
                time.sleep(1.0)
                l = self.get_dst(80)
                r = self.get_dst(-80)
                
                if(l > r):
                    self.park_dir = "R"   # parking is on the right
                    self.cmd_pub.publish(String(data="ccw"))
                else:
                    self.park_dir = "L"  # parking is on the left
                    self.cmd_pub.publish(String(data="cw"))
                
                if(self.park_dir == "L"):
                    self.send_c(1)
                    self.state = "TurningRight"
                else:
                    self.send_c(2)
                    self.state = "TurningLeft"
            
            elif(self.state == "Turned"):
                time.sleep(0.5)
                clr = "N"
                if(len(self.objs) > 0):
                    clr = self.objs[0]["color"]
                    if(self.objs[0]["dst"] > 0.5):
                        clr = "N"
                    self.obj = clr
                self.set_state_table(self.obj)
            
            elif(self.state == "Offset_Calc"):
                time.sleep(1.0)
                l = self.get_dst(90)
                r = self.get_dst(-90)
                f = self.get_dst(0)
                offx = 0.0
                offy = 0.0

                if(self.park_dir == "R"):
                    offx = r - 0.5
                else:
                    offx = 0.5 - l
                
                offy = 1.5 - f

                self.state = "Idle"          
                self.send_c(5)
                self.cmd_pub.publish(String(data="start"))
                self.initial_offset = Vector3()
                self.initial_offset.x = offx
                self.initial_offset.y = offy
                self.initial_offset.z = 0.0
                self.offs_pub.publish(self.initial_offset)
                self.get_logger().info(f"Initial Offset set to: x={offx:.2f}, y={offy:.2f}, f={f:.2f}")
            

            elif(self.state == "RunEnd"):
                if(self.park_dir == "R"):
                    self.send_c(6)  # Park Right but Move first
                else:
                    self.send_c(8)  # Park Left but Move first
                self.state = "ParkingInit"
            
            # elif(self.state == "Oriented"):
            #     f = self.get_dst(0)
                
            #     tomove = f - self.parking_offs

            #     self.get_logger().info(f"Oriented: Dist: {f}, Moving {tomove:.2f} m")
            #     self.cmd_pub.publish(String(data=f"MOVE:{tomove:.2f}"))
            #     self.state = "Idle"
            # elif(self.state == "ParkReady"):
            #     if(self.park_dir == "R"):
            #         self.send_c(8)
            #     else:
            #         self.send_c(9)
            #     self.state = "Idle"
            
            # elif(self.state == "Inside"):
            #     self.get_logger().info("Deciding exit direction")
            #     l = self.get_dst(90)
            #     r = self.get_dst(-90)

            #     c_val = 11
            #     if(r < l):
            #         c_val = 10
            #     self.send_c(c_val)
            #     self.get_logger().info(f"Exiting parking to the {'right' if c_val == 10 else 'left'}")
            #     self.state = "Idle"
            time.sleep(0.1)
    
    def set_state_table(self, obj):
        if(obj == "G" and self.park_dir == "L"):
            self.send_c(3)  # start parking
            self.state = "Moving"
        elif(obj == "G" and self.park_dir == "R"):
            self.send_c(4)  # start parking
            self.state = "Moving"
        elif(obj == "R" and self.park_dir == "L"):
            self.send_c(4)  # start parking
            self.state = "Moving"
        elif(obj == "R" and self.park_dir == "R"):
            self.send_c(3)  # start parking
            self.state = "Moving"
        elif(obj == "N"):
            self.send_c(3)
            self.state = "Moving"
    
    def send_c(self, c:int):
        msg = Int8()
        msg.data = c
        self.esp_cmd_pub.publish(msg)
        
    def cmd_callback(self, msg: String):
        self.get_logger().info(f'Received command: {msg.data}')
        if(msg.data == "Boot"):
            self.state = "toTurn"
        
        if(msg.data == "Turned"):
            self.state = "Turned"
        if(msg.data == "Done"):
            self.state = "Offset_Calc"
        if(msg.data == "RunEnd"):
            self.state = "RunEnd"
        # if(msg.data == "Oriented"):
        #     self.state = "Oriented"
        # if(msg.data == "ParkReady"):
        #     self.state = "ParkReady"
        # if(msg.data == "Inside"):
        #     self.state = "Inside"
    
    def lidar_callback(self, msg: LaserScan):
        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angInc = msg.angle_increment
        self.ranges = msg.ranges
        self.ints = msg.intensities

        if(self.state == "ParkingInit"):
            l = self.get_dst(90)
            fl = self.get_dst(45)
            f = self.get_dst(0)
            fr = self.get_dst(-45)
            r = self.get_dst(-90)

            msg = Int16MultiArray()
            msg.data = [int(l*100), int(fl*100), int(f*100), int(fr*100), int(r*100)]
            self.dir_dst_pub.publish(msg)
    
    def debug_callback(self, msg: BotDebugInfo):        
        self.objs = []
        for tower in msg.towers:
            self.objs.append({
                "color": tower.color,
                "index": tower.index,
                "ang": tower.ang,
                "dst": tower.dst,
                "x": tower.x,
                "y": tower.y
            })
    
    def pos_callback(self, msg: Vector3):
        self.pos = msg
        
    def obj_callback(self, msg: String):
        self.obj = msg.data
    
    def get_dst(self, ang):
        i = self.a2i(radians(ang))
        return self.fix_missing(i)
    
    def fix_missing(self, i):
        if(i < 0 or i >= len(self.ints)):
            return 0
        if(self.ints[i] > 0.05 and self.ranges[i] <= 3.0 and self.ranges[i] != inf):
            return self.ranges[i]
        
        if(self.ranges[i] == inf):
            return 0
        return self.ranges[i]

        first = i
        last = i
        while(first > 0 and (self.ints[first] == 0.0 or self.ranges[first] > 3.0 or self.ranges[first] == inf)):
            first -= 1
            if(first < 0):
                first = 0
                break
        while(last < len(self.ints) and (self.ints[last] == 0.0 or self.ranges[last] > 3.0 or self.ranges[last] == inf)):
            last += 1
            if(last >= len(self.ints)):
                last = len(self.ints) - 1
                break
        
        if(self.ints[first] == 0.0 or self.ints[last] == 0.0):
            return 0
        if(first == last):
            return self.ranges[first]
        
        return self.lerp(self.ranges[first], self.ranges[last], (i-first)/(last-first))

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