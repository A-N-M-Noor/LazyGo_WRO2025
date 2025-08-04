from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    package_directory = get_package_share_directory('lazysim')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(package_directory, 'config', 'lazySim.rviz')],
        parameters=[{'use_sim_time': True}],
        output='log',
        emulate_tty=False
    )

    robot_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', os.path.join(package_directory, 'description', 'lazyBot.xacro')])},
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        rviz_node,
        robot_publisher_node,
        joint_state_publisher
    ])