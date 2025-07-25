import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
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
            )
    
    rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_directory, 'launch', 'rviz_lazySim.launch.py')]),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
    
    control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_directory, 'launch', 'control.launch.py')]),
                )
    
    track_creator = Node(
                package="lazysim",
                executable="track_maker",
                # parameters=[{"template_path": os.path.join(package_directory, "templates", "track_template.xml")}],
                parameters=[{
                    "template_path": "/home/duronto/WRO_25_ws/src/lazysim/config/object_template.sdf",
                    "settings_path": "/home/duronto/WRO_25_ws/src/lazysim/config/track.json"
                    }],
                output="screen",
            )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        # rviz,
        TimerAction(period=5.0, actions=[
            track_creator
        ]),
        control,
    ])