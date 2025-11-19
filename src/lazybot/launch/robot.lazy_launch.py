import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction

from serial.tools import list_ports

def generate_launch_description():
    ld = LaunchDescription()

    control = Node(
        package='lazybot',
        executable='control',
        name='control'
    )
    parking = Node(
        package='lazybot',
        executable='parking',
        name='parking'
    )

    serial = Node(
        package='lazybot',
        executable='serial',
        name='serial'
    )

    detect = Node(
        package='lazybot',
        executable='detect_cam',
        name='detect_bot'
    )
    
    debug = Node(
        package='lazybot',
        executable='debug',
        name='debug'
    )

    ports = list_ports.comports()
    lidarPort = None
    for port in ports:
        if(port.product and port.product.startswith("CP2102N")):
            lidarPort = port.device
            print(f"RPLIDAR found on port: {lidarPort}")
            break

    
    if(lidarPort is None):
        print("No RPLIDAR found")
    
    else:
        rplidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rplidar_ros'),
                    'launch',
                    'rplidar_c1_launch.py'
                )
            ),
            launch_arguments={
                'serial_port': lidarPort,
                'inverted': 'true',
            }.items()
        )
        ld.add_action(rplidar_launch)

    # ld.add_action(control)
    ld.add_action(parking)
    ld.add_action(serial)
    ld.add_action(
        TimerAction(period=4.0, actions=[detect])
    )
    ld.add_action(debug)
    
    return ld