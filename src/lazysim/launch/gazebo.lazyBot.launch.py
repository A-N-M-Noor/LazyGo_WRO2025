import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    disable_rviz_arg = DeclareLaunchArgument(
        'disable_rviz',
        default_value='false',
        description='Disable RViz launch'
    )
    disable_rviz = LaunchConfiguration('disable_rviz')
    
    disable_bridge_arg = DeclareLaunchArgument(
        'disable_bridge',
        default_value='false',
        description='Disable Bridge launch'
    )
    disable_bridge = LaunchConfiguration('disable_bridge')

    package_directory = get_package_share_directory('lazysim')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_directory, 'launch', 'lazySim.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(package_directory, 'config', 'params_gazebo.yaml')

    world_file = os.path.join(package_directory, 'worlds', 'lazyWorld.world')
    
    materials_path = os.path.join(package_directory, 'worlds')
    existing_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    if existing_path:
        os.environ['GAZEBO_RESOURCE_PATH'] = existing_path + ':' + materials_path
    else:
        os.environ['GAZEBO_RESOURCE_PATH'] = materials_path
        
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                    'world': world_file,
                    'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
                }.items()
            )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                    '-entity', 'lazyBot'],
                        output='screen')
    
    bridge = Node(
                package='lazysim',
                executable='lazybridge',
                name='lazybridge',
                output='screen',
                condition=UnlessCondition(disable_bridge),
            )
    
    rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_directory, 'launch', 'rviz_lazySim.launch.py')]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'tf_buffer_duration': '30.0',
                    'disable_state_publisher': 'true',
                    'disable_robot_publisher': 'true',
                }.items(),
                condition=UnlessCondition(disable_rviz)
            )
    static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
            output='screen'
        )
    
    control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_directory, 'launch', 'control.launch.py')]),
                )
    
    track_creator = Node(
                package="lazysim",
                executable="track_maker",
                parameters=[{
                    "tower_template_path": "/home/duronto/WRO_25_ws/src/lazysim/config/object_template.sdf",
                    "settings_path": "/home/duronto/WRO_25_ws/src/lazysim/config/track.yaml"
                    }],
                output="screen",
            )
    
    ld = LaunchDescription([
        disable_rviz_arg,
        disable_bridge_arg,
        static_tf,
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        TimerAction(period=4.0, actions=[
            track_creator
        ]),
        control,
        TimerAction(period=2.0, actions=[
            rviz
        ]),
    ])

    return ld