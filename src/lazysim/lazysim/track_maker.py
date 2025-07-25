import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Quaternion, Twist
import os, json, math


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)



class TrackMaker(Node):
    def __init__(self):
        super().__init__('track_maker')
        self.declare_parameter('template_path', '')
        self.declare_parameter('settings_path', '')
        
        self.template_path = self.get_parameter('template_path').get_parameter_value().string_value
        self.settings_path = self.get_parameter('settings_path').get_parameter_value().string_value
        self.template = ""
        self.settings = {}

        if os.path.isfile(self.settings_path):
            with open(self.settings_path, 'r') as f:
                self.settings = json.load(f)
        else:
            self.get_logger().warn(f"Settings file '{self.settings_path}' does not exist.")

        self.get_logger().info("Waiting for services to be available...")
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.spawn_client.wait_for_service()
        self.get_logger().info("SpawnEntity service is available.")
        
        
        self.set_entity_client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info("Waiting for SetEntityState service...")
        if not self.set_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetEntityState service not available after 5 seconds")
            return
        self.get_logger().info('SetEntityState service is available.')
        
        
        if os.path.isfile(self.template_path):
            with open(self.template_path, 'r') as f:
                self.template = f.read()
                self.get_logger().info(f"Template loaded from {self.template_path}")
        else:
            self.get_logger().warn(f"Template file '{self.template_path}' does not exist.")


        self.count = 1
        
        for tower in self.settings['towers']:
            self.get_logger().info(f"Spawning tower: {tower}")
            self.spawn_tower(tower)
        
        self.set_pose('lazyBot', 0.0, 1.0, 0.0, 90.0)

    def set_pose(self, target, x: float, y: float, z: float, yaw_deg: float):
        yaw_rad = math.radians(yaw_deg)
        quaternion = euler_to_quaternion(0.0, 0.0, yaw_rad)

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = target
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation = quaternion
        req.state.twist = Twist()

        future = self.set_entity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'{target} moved successfully.')
        else:
            self.get_logger().error(f'Failed to move {target}: {future.result().status_message}')

    
    def spawn_tower(self, tower):
        position = (1.0, 2.0, 0.5)  # (x, y, z)

        # SDF XML for a red cube
        sdf = self.template.format(
            color=tower['color'],
            pos_x=position[0],
            pos_y=position[1],
            pos_z=position[2]
        )

        req = SpawnEntity.Request()
        req.name = f'tower_{self.count}'
        req.xml = sdf
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose = Pose()
        req.initial_pose.position.x = position[0]
        req.initial_pose.position.y = position[1]
        req.initial_pose.position.z = position[2]

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Tower {self.count} spawned successfully!')
            self.count += 1
        else:
            self.get_logger().error(f'Failed to spawn tower: {future.result().status_message}')


def main():
    rclpy.init()
    node = TrackMaker()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
