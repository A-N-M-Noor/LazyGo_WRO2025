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
        name='control'
    )

    serial = Node(
        package='lazybot',
        executable='serial',
        name='serial'
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )

    ld.add_action(control)
    ld.add_action(serial)
    ld.add_action(rplidar_launch)
    return ld