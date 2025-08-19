import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Quaternion, Twist
import os, math
from ruamel.yaml import YAML


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
        self.declare_parameter('tower_template_path', '')
        self.declare_parameter('wall_template_path', '')
        self.declare_parameter('settings_path', '')

        self.tower_template_path = self.get_parameter('tower_template_path').get_parameter_value().string_value
        self.wall_template_path = self.get_parameter('wall_template_path').get_parameter_value().string_value
        self.settings_path = self.get_parameter('settings_path').get_parameter_value().string_value
        self.tower_template = ""
        self.wall_template = ""
        self.settings = {}

        yaml = YAML()
        if os.path.isfile(self.settings_path):
            with open(self.settings_path, 'r') as f:
                self.settings = yaml.load(f)
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
        
        
        if os.path.isfile(self.tower_template_path):
            with open(self.tower_template_path, 'r') as f:
                self.tower_template = f.read()
                self.get_logger().info(f"Template loaded from {self.tower_template_path}")
        else:
            self.get_logger().warn(f"Template file '{self.tower_template_path}' does not exist.")

        if os.path.isfile(self.wall_template_path):
            with open(self.wall_template_path, 'r') as f:
                self.wall_template = f.read()
                self.get_logger().info(f"Template loaded from {self.wall_template_path}")
        else:
            self.get_logger().warn(f"Template file '{self.wall_template_path}' does not exist.")


        self.count = 1
        
        for tower in self.settings['towers']:
            self.get_logger().info(f"Spawning tower: {tower}")
            self.spawn_tower(tower)
            
        self.spawn_parking(self.settings['parking'])
        self.set_pose('lazyBot', self.settings['start']['pos']['x'], self.settings['start']['pos']['y'], 0.0, self.settings['start']['angle'])

    def set_pose(self, target, x: float, y: float, z: float, yaw_deg: float):
        yaw_rad = math.radians(yaw_deg)
        quaternion = euler_to_quaternion(0.0, 0.0, yaw_rad)

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = target
        req.state.pose.position.x = x - 0.0825 * math.cos(yaw_rad)
        req.state.pose.position.y = y - 0.0825 * math.sin(yaw_rad)
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
        position = (tower['pos']['x'], tower['pos']['y'], 0.5)

        sdf = self.tower_template.format(
            color=tower['color']
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

        resp = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

        if resp.result().success:
            self.get_logger().info(f'Tower {self.count} spawned successfully!')
            self.count += 1
        else:
            self.get_logger().error(f'Failed to spawn tower: {resp.result().status_message}')

    def spawn_parking(self, parking):
        if(not parking['enabled']):
            return

        x, y, l = parking['pos']['x'], parking['pos']['y'], (parking['size']+0.02)/2
        ang_rad = math.radians(parking['angle'])
        
        pos1 = (
            x - l * math.cos(ang_rad),
            y - l * math.sin(ang_rad)
        )
        pos2 = (
            x + l * math.cos(ang_rad),
            y + l * math.sin(ang_rad)
        )
        
        self.spawn_wall("parking_wall_1", "Purple", pos1, ang_rad, 0.2, static=False)
        self.spawn_wall("parking_wall_2", "Purple", pos2, ang_rad, 0.2, static=False)

    def spawn_wall(self, name, color, pos, ang, size, static=True):
        position = (pos[0], pos[1], 0.05)

        sdf = self.wall_template.format(
            color=color,
            sz_x=size,
            sz_y=0.02,
            static=static
        )

        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose = Pose()
        req.initial_pose.position.x = position[0]
        req.initial_pose.position.y = position[1]
        req.initial_pose.position.z = position[2]
        req.initial_pose.orientation = euler_to_quaternion(0.0, 0.0, ang + math.pi / 2)

        resp = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

        if resp.result().success:
            self.get_logger().info(f'Wall {name} spawned successfully!')
        else:
            self.get_logger().error(f'Failed to spawn wall: {resp.result().status_message}')

def main():
    rclpy.init()
    node = TrackMaker()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
