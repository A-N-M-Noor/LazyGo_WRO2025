import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState, Imu
from math import radians, degrees, sin, cos, tan, atan, atan2, asin, pi

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw

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
        
        self.wheelD = 0.04

        self.throttle_sub = self.create_subscription(
            Float32,
            '/throttle',
            self.throttle_callback,
            1
        )
        self.steer_sub = self.create_subscription(
            Float32,
            '/steer',
            self.steer_callback,
            1
        )
        
        self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            1
        )
        
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1
        )

        self.motor_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub= self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.pos_pub = self.create_publisher(Vector3, '/lazypos', 10)
        
        self.yaw = 0.0
        self.prevYaw = 0.0
        
        self.heading = 0.0
        self.headingPrev = self.heading

        self.encoder = [0.0, 0.0]
        self.encoderPrev = self.encoder.copy()
        
        self.pos = [0.0, 0.0]
        
        self.create_timer(0.05, self.odom)
        
        self.get_logger().info("LazyBridge Node has been started.")

    def throttle_callback(self, msg: Float32):
        self.throttle = msg.data

    def steer_callback(self, msg: Float32):
        self.steer = msg.data
        
        whl_spd = self.throttle * self.maxSpeed
        str_ang = self.steer * self.max_steer
        
        if (abs(str_ang) < 0.17):
            self.motor_pub.publish(Float64MultiArray(data=[whl_spd, whl_spd, whl_spd, whl_spd]))
            self.steer_pub.publish(Float64MultiArray(data=[str_ang, str_ang]))
            return
        
        ang = abs(str_ang)
        r = self.wheelbase / tan(ang)
        hinge_l = self.axle_length / 2 - self.hinge_d
        
        ras = r - hinge_l
        rbs = r + hinge_l
        
        strL = atan(self.wheelbase / ras)
        strR = atan(self.wheelbase / rbs)
        
        ra = r - self.axle_length / 2
        rb = r + self.axle_length / 2
        
        raf = ras / cos(strL)
        rbf = rbs / cos(strR)
        
        spdBL = whl_spd * (ra / r)
        spdBR = whl_spd * (rb / r)
        spdFL = whl_spd * (raf / r)
        spdFR = whl_spd * (rbf / r)
        
        if str_ang < 0:
            strL, strR = -strR, -strL
            spdBL, spdBR = spdBR, spdBL

        self.motor_pub.publish(Float64MultiArray(data=[spdBL, spdBR, spdFL, spdFR]))
        self.steer_pub.publish(Float64MultiArray(data=[strL, strR]))

    def state_callback(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        p1 = pos['base_to_back_left_wheel'] * (self.wheelD / 2)
        p2 = pos['base_to_back_right_wheel'] * (self.wheelD / 2)
        if(abs(p1) > 0.01 and abs(p2) > 0.01):
            self.encoder = [p1, p2]
    
    def imu_callback(self, msg: Imu):
        yaw = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)[2]
        
        dA = yaw - self.prevYaw

        if(dA > pi):
            dA -= 2 * pi
        elif(dA < -pi):
            dA += 2 * pi

        self.yaw += dA
        self.prevYaw = yaw
        
        self.heading = self.yaw

    def odom(self):
        dS = ( (self.encoder[0] - self.encoderPrev[0]) + (self.encoder[1] - self.encoderPrev[1]) ) / 2
        H = (self.heading + self.headingPrev) / 2

        self.encoderPrev = self.encoder.copy()
        self.headingPrev = self.heading
        
        dX = dS * cos(H)
        dY = dS * sin(H)
        
        self.pos[0] += dX
        self.pos[1] += dY
        
        attitude = Vector3()
        attitude.x = self.pos[1]
        attitude.y = self.pos[0]
        attitude.z = self.heading
        self.pos_pub.publish(attitude)
        
        # self.get_logger().info(f"{self.pos[0]:.2f}, {self.pos[1]:.2f} | {degrees(self.heading):.2f}°")
    
def main(args=None):
    rclpy.init(args=args)
    node = LazyBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()