import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    controller_config_file = os.path.join(get_package_share_directory('lazysim'), 'config', 'controller_config.yaml')
    
    return LaunchDescription([
        TimerAction(period=5.0, actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controller_config_file],
                output="screen",
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['velocity_controller'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['position_controller'],
                output='screen'
            )
        ])
    ])
