import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, Vector3, TransformStamped, Quaternion, Vector3
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from lazy_interface.msg import BotDebugInfo, LidarTowerInfo, GlobalTowerInfo
from math import pi, radians, degrees, sin, cos
from lazybot.helper.util import global2local

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug')
        
        self.debug_sub = self.create_subscription(BotDebugInfo, '/lazybot/debug', self.debug_callback, 10)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)
        
        self.pubDebug = self.create_publisher(PointCloud, '/target_point', 10)
        self.pubCache = self.create_publisher(PointCloud, '/cached_object', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.pos = Vector3()
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.pos.z = 0.0
        
        self.dangerDist = 0.25
        self.dangerAng = [25.0, 90.0]

        self.castR = 0.25
        self.targetAng = 0.0
        self.targetD = 0.0
        
        self.datass = []
        
        self.objs = []
        self.obj_cache = []
        
        self.pubOdom()
    
    def debug_callback(self, msg: BotDebugInfo):
        self.targetAng = msg.target_angle
        self.targetD = msg.target_distance
        self.castR = msg.cast_radius
        self.dangerDist = msg.danger_distance
        self.dangerAng = msg.danger_angles
        
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
        
        self.obj_cache = []
        for tower in msg.towerpool:
            self.obj_cache.append({
                        "color": tower.color,
                        "index": tower.index,
                        "x": tower.x,
                        "y": tower.y
                    })
        
        self.pubDebugPoint()
    
    def pos_callback(self, msg: Vector3):
        self.pos = msg
        self.pubOdom()
        
    
    def pubOdom(self):
        current_time = self.get_clock().now().to_msg()
        
        q = yaw_to_quaternion(self.pos.z - pi/2)
        x, y, z = self.pos.x, -self.pos.y, 0.0

        # self.get_logger().info(f"Publishing odom at x: {x}, y: {y}, yaw: {self.pos.z}")
        
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        self.odom_pub.publish(odom)

    def pubDebugPoint(self):
        targetPnts = PointCloud()
        targetPnts.channels.append(ChannelFloat32(name="intensity", values=[]))
        targetPnts.header.stamp = self.get_clock().now().to_msg()
        targetPnts.header.frame_id = "laser"

        ang = radians(self.targetAng)
        offX = self.castR*cos(ang + pi/2)
        offY = self.castR*sin(ang + pi/2)

        for i in range(0, int(self.targetD/0.1)):
            x = (i*0.1)*cos(ang) + offX
            y = (i*0.1)*sin(ang) + offY
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)
        offX = self.castR*cos(ang - pi/2)
        offY = self.castR*sin(ang - pi/2)
        for i in range(0, int(self.targetD/0.1)):
            x = (i*0.1)*cos(ang) + offX
            y = (i*0.1)*sin(ang) + offY
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)

        circle_points = 50
        for i in range(circle_points):
            ang = 2 * pi * i / circle_points
            x = self.dangerDist*cos(ang)
            y = self.dangerDist*sin(ang)
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))

            ang = degrees(ang)
            if(ang > self.dangerAng[0] and ang < self.dangerAng[1]):
                targetPnts.channels[0].values.append(0.0)
            elif(ang > -self.dangerAng[1] and ang < -self.dangerAng[0]):
                targetPnts.channels[0].values.append(1.0)
            else:
                targetPnts.channels[0].values.append(0.5)
        
        circle_points = 50
        
        for i in range(circle_points):
            theta = 2 * pi * i / circle_points
            x = self.castR * cos(theta)
            y = self.castR * sin(theta)
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)
        
        circle_points = 20
        circle_radius = 0.07

        for obj in self.objs:
            cx, cy = obj["x"], obj["y"]
            for j in range(circle_points):
                theta = 2 * pi * j / circle_points
                x = cx + circle_radius * cos(theta)
                y = cy + circle_radius * sin(theta)
                targetPnts.points.append(Point32(x=x, y=y, z=0.0))
                if(obj['color'] == "R"):
                    targetPnts.channels[0].values.append(0.0)
                elif(obj['color'] == "G"):
                    targetPnts.channels[0].values.append(0.2)
                else:
                    targetPnts.channels[0].values.append(0.6)
        
        circle_points = 12
        circle_radius = 0.09
        for obj in self.obj_cache:
            cx, cy = obj["x"], obj["y"]
            for j in range(circle_points):
                if(obj['color'] == "C"):
                    circle_radius = 0.18
                else:
                    circle_radius = 0.09
                theta = 2 * pi * j / circle_points
                x = cx + circle_radius * cos(theta)
                y = cy + circle_radius * sin(theta)
                targetPnts.points.append(Point32(x=x, y=y, z=0.0))
                if(obj['color'] == "R"):
                    targetPnts.channels[0].values.append(0.0)
                elif(obj['color'] == "G"):
                    targetPnts.channels[0].values.append(0.2)
                else:
                    targetPnts.channels[0].values.append(0.6)

        self.pubDebug.publish(targetPnts)
        
def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()