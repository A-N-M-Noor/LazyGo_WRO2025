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
    """Helper to convert a yaw angle (radians) into a ROS Quaternion."""
    q = Quaternion()
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q

class DebugNode(Node):
    """
    Visualization and Debugging Node.
    1. Subscribes to control logic state (target angles, detected objects).
    2. Publishes RVIZ-compatible markers (PointClouds) to visualize the robot's "mind".
    3. Broadcasts Odometry and TF transforms to visualize robot movement.
    """
    def __init__(self):
        super().__init__('debug')
        
        # --- Subscribers ---
        # Receives internal state from the Control Node
        self.debug_sub = self.create_subscription(BotDebugInfo, '/lazybot/debug', self.debug_callback, 10)
        # Receives calculated position (x, y, theta)
        self.pos_sub = self.create_subscription(Vector3, '/lazypos', self.pos_callback, 10)
        
        # --- Publishers ---
        # Visualizes target path, danger zones, and towers as a PointCloud
        self.pubDebug = self.create_publisher(PointCloud, '/target_point', 10)
        # Visualizes cached/global objects (optional)
        self.pubCache = self.create_publisher(PointCloud, '/cached_object', 10)
        # Standard Odometry for RVIZ
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster for coordinate frames (odom -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- State Variables ---

        # Current position variable
        self.pos = Vector3()
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.pos.z = 0.0
        
        # Default visualization parameters
        self.dangerDist = 0.25
        self.dangerAng = [25.0, 90.0]
        self.castR = 0.25
        self.targetAng = 0.0
        self.targetD = 0.0
        
        self.datass = []
        self.objs = []      # Currently detected towers
        self.obj_cache = [] # Historically tracked towers
        
        # Publish initial odom to establish TF tree immediately
        self.pubOdom()
    
    def debug_callback(self, msg: BotDebugInfo):
        """
        Updates internal state based on the latest control loop decision.
        Triggered whenever the Control Node publishes a debug snapshot.
        """
        self.targetAng = msg.target_angle
        self.targetD = msg.target_distance
        self.castR = msg.cast_radius
        self.dangerDist = msg.danger_distance
        self.dangerAng = msg.danger_angles
        
        # Unpack detected towers
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
        
        # Unpack global tower pool (if using mapping)
        self.obj_cache = []
        for tower in msg.towerpool:
            self.obj_cache.append({
                        "color": tower.color,
                        "index": tower.index,
                        "x": tower.x,
                        "y": tower.y
                    })
        
        # Update the visualization immediately
        self.pubDebugPoint()
    
    def pos_callback(self, msg: Vector3):
        """Updates robot position and broadcasts TF."""
        self.pos = msg
        self.pubOdom()
        
    
    def pubOdom(self):
        """
        Broadcasts the transform from 'odom' frame to 'base_link' frame.
        This allows RVIZ to show the robot moving in the map.
        """
        current_time = self.get_clock().now().to_msg()
        
        # Convert 2D yaw to Quaternion
        # Note: Adjusting by -pi/2 might be specific to the robot's zero-heading definition
        q = yaw_to_quaternion(self.pos.z - pi/2)
        x, y, z = self.pos.x, -self.pos.y, 0.0

        # 1. Broadcast Transform (TF)
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Odometry Message
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
        """
        Generates a PointCloud to visualize:
        1. The chosen path (Target Angle & Distance)
        2. The robot's cast radius (Width check)
        3. Danger zones (Emergency stop areas)
        4. Detected towers (Red/Green circles)
        """
        targetPnts = PointCloud()
        # 'intensity' channel is used to color points in RVIZ (e.g., Rainbow filter)
        targetPnts.channels.append(ChannelFloat32(name="intensity", values=[]))
        targetPnts.header.stamp = self.get_clock().now().to_msg()
        targetPnts.header.frame_id = "laser" # Points are relative to the LiDAR frame

        # --- 1. Visualize Path Corridor ---
        # Draws two parallel lines representing the robot's width along the target path
        ang = radians(self.targetAng)
        
        # Left boundary line
        offX = self.castR*cos(ang + pi/2)
        offY = self.castR*sin(ang + pi/2)
        for i in range(0, int(self.targetD/0.1)):
            x = (i*0.1)*cos(ang) + offX
            y = (i*0.1)*sin(ang) + offY
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2) # Color code
            
        # Right boundary line
        offX = self.castR*cos(ang - pi/2)
        offY = self.castR*sin(ang - pi/2)
        for i in range(0, int(self.targetD/0.1)):
            x = (i*0.1)*cos(ang) + offX
            y = (i*0.1)*sin(ang) + offY
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)

        # --- 2. Visualize Danger Zone ---
        # Draws a circle around the robot indicating the emergency stop radius
        circle_points = 50
        for i in range(circle_points):
            ang = 2 * pi * i / circle_points
            x = self.dangerDist*cos(ang)
            y = self.dangerDist*sin(ang)
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))

            # Color code segments based on whether they are active danger zones
            ang = degrees(ang)
            if(ang > self.dangerAng[0] and ang < self.dangerAng[1]):
                targetPnts.channels[0].values.append(0.0) # Active Left
            elif(ang > -self.dangerAng[1] and ang < -self.dangerAng[0]):
                targetPnts.channels[0].values.append(1.0) # Active Right
            else:
                targetPnts.channels[0].values.append(0.5) # Safe
        
        # --- 3. Visualize Cast Radius (Robot Body) ---
        # Draws a circle representing the robot's current effective width
        for i in range(circle_points):
            theta = 2 * pi * i / circle_points
            x = self.castR * cos(theta)
            y = self.castR * sin(theta)
            targetPnts.points.append(Point32(x=x, y=y, z=0.0))
            targetPnts.channels[0].values.append(0.2)
        
        # --- 4. Visualize Detected Towers ---
        # Draws small circles around detected obstacles
        circle_points = 20
        circle_radius = 0.07

        for obj in self.objs:
            cx, cy = obj["x"], obj["y"]
            for j in range(circle_points):
                theta = 2 * pi * j / circle_points
                x = cx + circle_radius * cos(theta)
                y = cy + circle_radius * sin(theta)
                targetPnts.points.append(Point32(x=x, y=y, z=0.0))
                
                # Color code based on tower type
                if(obj['color'] == "R"):
                    targetPnts.channels[0].values.append(0.0) # Red
                elif(obj['color'] == "G"):
                    targetPnts.channels[0].values.append(0.2) # Green
                else:
                    targetPnts.channels[0].values.append(0.6) # Unknown
        
        # --- 5. Visualize Cached Objects (Global Map) ---
        circle_points = 12
        for obj in self.obj_cache:
            cx, cy = obj["x"], obj["y"]
            for j in range(circle_points):
                # Larger radius for corners ("C"), smaller for towers
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