import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray
from math import radians, degrees, tan, atan, pi

class LazyBridge(Node):
    def __init__(self):
        super().__init__('lazybridge')
        self.throttle = 0.0
        self.steer = 0.0
        
        self.maxSpeed = 47.0 # rad/s
        self.max_steer = radians(30)
        self.axle_length = 0.094
        self.wheelbase = 0.12
        self.hinge_d = 0.035

        self.throttle_sub = self.create_subscription(
            Float32,
            '/throttle',
            self.throttle_callback,
            10
        )
        self.steer_sub = self.create_subscription(
            Float32,
            '/steer',
            self.steer_callback,
            10
        )
        
        self.back_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub= self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        
        self.get_logger().info("LazyBridge Node has been started.")

    def throttle_callback(self, msg):
        self.throttle = msg.data

    def steer_callback(self, msg):
        self.steer = msg.data
        
        whl_spd = self.throttle * self.maxSpeed
        str_ang = self.steer * self.max_steer
        
        if (abs(str_ang) < 0.17):
            self.back_pub.publish(Float64MultiArray(data=[whl_spd, whl_spd]))
            self.steer_pub.publish(Float64MultiArray(data=[str_ang, str_ang]))
            return
        
        ang = abs(str_ang)
        r = self.wheelbase / tan(ang)
        
        ras = r - (self.axle_length / 2 - self.hinge_d)
        rbs = r + (self.axle_length / 2 - self.hinge_d)
        
        strL = atan(self.wheelbase / ras)
        strR = atan(self.wheelbase / rbs)
        
        ra = r - self.axle_length / 2
        rb = r + self.axle_length / 2
        
        spdL = whl_spd * (ra / r)
        spdR = whl_spd * (rb / r)
        
        if str_ang < 0:
            strL, strR = strR, strL
            spdL, spdR = spdR, spdL

        self.back_pub.publish(Float64MultiArray(data=[spdL, spdR]))
        self.steer_pub.publish(Float64MultiArray(data=[strL, strR]))

def main(args=None):
    rclpy.init(args=args)
    node = LazyBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()