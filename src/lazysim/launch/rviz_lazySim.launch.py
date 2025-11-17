from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    disable_state_publisher_arg = DeclareLaunchArgument(
        'disable_state_publisher',
        default_value='false',
        description='Disable state publisher'
    )
    disable_state_publisher = LaunchConfiguration('disable_state_publisher')
    
    disable_robot_publisher_arg = DeclareLaunchArgument(
        'disable_robot_publisher',
        default_value='false',
        description='Disable robot state publisher'
    )
    disable_robot_publisher = LaunchConfiguration('disable_robot_publisher')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (Gazebo / bag playback)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    package_directory = get_package_share_directory('lazysim')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(package_directory, 'config', 'lazySim.rviz'), '--ros-args', '--log-level', 'ERROR'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
        emulate_tty=False
    )

    robot_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', os.path.join(package_directory, 'description', 'lazyBot.xacro')])},
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(disable_robot_publisher)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(disable_state_publisher)
    )

    return LaunchDescription([
        disable_state_publisher_arg,
        disable_robot_publisher_arg,
        use_sim_time_arg,
        rviz_node,
        robot_publisher_node,
        joint_state_publisher
    ])