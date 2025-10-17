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
    
def main(args=None):
    rclpy.init(args=args)
    node = OpenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()