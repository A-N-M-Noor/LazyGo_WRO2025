# Instructions
Want to use this package for your robot? Or maybe you just want to use the simulator? You can find the instructions here.

[![Ubuntu](https://img.shields.io/badge/Ubuntu-DE4E1B?style=flat-square)](#installing-ubuntu)
[![Packages](https://img.shields.io/badge/Packages-4584b6?style=flat-square)](#installing-packages)
[![ROS2](https://img.shields.io/badge/ROS2-22314E?style=flat-square)](#installing-ros2)
[![Gazebo](https://img.shields.io/badge/Gazebo-EE7C11?style=flat-square)](#installing-gazebo)
[![Codes](https://img.shields.io/badge/Codes-gray?style=flat-square)](#use-our-codes)

## Installing Ubuntu
As we are using ROS2 for our robot, we installed Ubuntu on our Pi. Now, it is completely possible to use ROS2 without using Ubuntu, just on a Pi OS using Docker. But to keep things simple, we decided to install Ubuntu on our Pi.

#### Which version?
You can either use ROS2 Humble with Ubuntu 22.04, or you can use ROS2 Jazzy with Ubuntu 24.04.

There is no direct download link for Ubuntu 22.04 for ARM architecture (Please let us know if you find any). And you can't install Ubuntu 22.04 from Pi Imager for a Pi 5. Therefore, we are using ROS2 Jazzy with Ubuntu 24.04. But this version of Ubuntu comes with a few problems. For example,you'll face a lot of issues installing a working VNC here. But thankfully ROS2 comes with a lot of remote debugging tools. And you can even code inside VSCode with its remote SSH feature. So you probably won't even need any VNC.

#### Installation
Download the Official Ubuntu Image for ARM from [here](https://cdimage.ubuntu.com/daily-live/20240421/). Download the `64-bit ARM (ARMv8/AArch64) desktop image` iso file.

Install Raspberry Pi Imager from [here](https://www.raspberrypi.com/software/). Use the imager to burn the image file to an SD card. Or you can use any othe image writing software of your choice.

Insert the SD card into the Pi, and turn it on. The first time you'll need to use a monitor.


## Installing Packages
#### Pip3

```
sudo apt install python3-pip
```

#### Git
```
sudo apt install git-all
```

#### VSCode (Optional)
```
wget https://update.code.visualstudio.com/latest/linux-deb-arm64/stable -O code_arm64.deb
```
```
sudo apt install ./code_arm64.deb
```

## Installing ROS2
You can find all the instructions on the official [ROS2 website](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).


#### UTF8
First, check if you have UTF8
```
locale
```
If you don't have it,
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
Then verify again.

#### APT Repository
Ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

.

Add the ROS2 APT repository
```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Installation - ROS2 Jazzy Jelisco
Install ROS2 Jazzy Jelisco
```
sudo apt update
sudo apt install ros-dev-tools ros-jazzy-desktop
```

.

Install `colcon` for building ROS2 projects
```
sudo apt install python3-colcon-common-extensions
```

.

Install and initialize rosdep
```
sudo apt install python3-rosdep python3-vcstool build-essential
```
```
sudo rosdep init
rosdep update
```

#### Source some files
Add ROS2 and colcon to the ~/.bashrc file, so that theyare sourced on every terminal. Adding colcon will enable autocomplete of commands.

First, open the .bashrc file
```
sudo nano ~/.bashrc
```
At the end, add these lines:
```
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

## Installing Gazebo
If you want to run the simulation, you'll need to install Gazebo.

> [!WARNING]  
> Do not try to run the simulation on your Raspberry Pi!

To use the simulation, you should install ROS2 on a separate PC/Laptop and run the simulation there.

> We already had ROS2 Humble installed on a separate device, so we used that version to run the simulation. You can install the new ignition gazebo with ROS2 Jazzy. But we used Gazebo Classic 11 with ROS2 Humble ([Installation guide here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)).

After you have ROS2 Humble working on your device...

First, install Gazebo
```
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
```
Source the gazebo bash file
```
source /usr/share/gazebo/setup.sh
```

You should be able to run Gazebo using
```
ros2 launch gazebo_ros gazebo.launch.py
```


## Use our codes

First, clone our repository
```
git clone https://github.com/A-N-M-Noor/LazyGo_WRO2025.git
```
Head into the repo
```
cd LazyGo_WRO2025
```
Install the required python packages
```
pip3 install -r requirements.txt
```
If pip3 refuses to install the packages, use this command (Only on the Pi. Ideally you would want to create a virtual environment if you're doing this on your everyday device)
```
pip3 install -r requirements.txt --break-system-packages
```

Then build the packages
```
colcon build
```

Before you can run any packages, you'll need to source the workspace
```
source install/setup.bash
```

To run the simulation
```
ros2 launch lazysim gazebo.lazyBot.launch.py
```

This command will launch Gazebo simulator with an URDF model along with RVIZ with lidar and odom data. If you don't want to visualize odom, set the `Fixed Frame` in rviz from `odom` to `base_link`.

Now, edit -
- `/src/lazysim/config/track.yaml` to change the track.
- `/src/lazysim/config/lazySim.rviz` to change the rviz visualization.
- `/src/lazysim/description` files to edit the robot model.



## Launch it 🚀🚀
You'll find three packages inside our [`/src`](../src/) directory.

`lazysim`: This is the simulation package. This package handles launching gazebo with rviz2, and works as a bridge between the control package and gazebo.

```
ros2 launch lazysim gazebo.lazyBot.launch.py
```
Launches Gazebo, and RVIZ2 with custom URDF. Also launches a bridge node.
```
ros2 launch lazysim rviz_lazySim.launch.py
```
Only launches the RVIZ app, usefull for when you want to run the robot on track and only want to view debug info.

.

`lazy_interfaces`: This just contains some custom interfaces to communicate between different ROS Nodes.

.

`lazybot`: This is the control package. This includes all the nodes to run the robot.
```
ros2 launch lazybot open.lazy_launch.py
```
Launches a control node and serial node. Also launches a debug node.
```
ros2 launch lazybot robot.lazy_launch.py
```
Launches all the nodes from the previous one, in addition to a parking node and a detection node (using camera).

---

Good luck! Do reach out if you face any issue.