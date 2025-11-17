import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    control = Node(
        package='lazybot',
        executable='control',
        name='control',
        parameters=[{'use_sim_time': True, 'IS_SIM': True}],
    )

    detect = Node(
        package='lazybot',
        executable='detect',
        name='detect',
        parameters=[{'use_sim_time': True}]
    )
    
    debug = Node(
        package='lazybot',
        executable='debug',
        name='debug',
        parameters=[{'use_sim_time': True}]
    )

    ld.add_action(control)
    ld.add_action(detect)
    ld.add_action(debug)
    return ld