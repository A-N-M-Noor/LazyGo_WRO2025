#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/lazygo/rplidar_ws/install/setup.bash
source /home/lazygo/WRO_25_ws/install/setup.bash
# ros2 launch lazybot lazy_launch.py

tmux new-session -d -s lazy_session "ros2 launch lazybot lazy_launch.py"
