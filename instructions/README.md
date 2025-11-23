# Instructions
Want to use this package for your robot? Or maybe you just want to use the simulator? You can find the instructions here.

[![Ubuntu](https://img.shields.io/badge/Ubuntu-DE4E1B?style=flat-square)](#installing-ubuntu)
[![Packages](https://img.shields.io/badge/Packages-4584b6?style=flat-square)](#installing-packages)
[![ROS2](https://img.shields.io/badge/ROS2-22314E?style=flat-square)](#installing-ros2)
[![Gazebo](https://img.shields.io/badge/Gazebo-EE7C11?style=flat-square)](#installing-gazebo)
[![Usage](https://img.shields.io/badge/Usage-gray?style=flat-square)](#use-our-codes)


Hey! If you have any trouble, you may want to check out the [Issues](https://github.com/A-N-M-Noor/LazyGo_WRO2025/issues?q=is%3Aissue%20state%3Aclosed) page and find if there's any solution there.


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

## Package Usage Guide

### Package Overview

#### 1. `lazy_interface`
**Purpose**: Custom ROS2 message definitions for robot communication
- Contains custom message types used across all nodes
- Includes tower detection info, LiDAR data, and debug information
- **No direct launch required** - automatically built with other packages

#### 2. `lazybot` 
**Purpose**: Main control package for the physical robot
- Contains all control logic, sensor processing, and robot behavior
- Handles both open challenge and obstacle avoidance scenarios

#### 3. `lazysim`
**Purpose**: Gazebo simulation environment
- Provides virtual testing environment before deploying to physical robot
- Includes track generation and robot model visualization

#### 4. `ESP32 Code`
**Purpose**: Low-level motor control and sensor interfacing
- PlatformIO project for ESP32 microcontroller
- Handles motor control, encoders, IMU, and parking maneuvers

---

### Launch File Usage Guide

#### For Physical Robot Testing

##### **Open Challenge** (3 laps without obstacles)
```bash
ros2 launch lazybot open.lazy_launch.py
```
**What it launches**: 
- Control node for the open challenge rounds
- Serial node for ESP32 communication
- Debug node for visualization
- RPLiDAR C1 driver Node

##### **Obstacle Challenge** (3 laps with tower avoidance + parking)
```bash
ros2 launch lazybot robot.lazy_launch.py
```

**What it launches**:
- Control node for obstacle avoidance
- Parking node for parking logic
- Detection node for camera-based tower detection
- Serial node for ESP32 communication
- Debug node for visualization
- RPLiDAR C1 driver Node

---

#### For Simulation Testing

##### **Full Gazebo Simulation**
```bash
ros2 launch lazysim gazebo.lazyBot.launch.py
```
Or this one if you don't want to launch rviz:
```bash
ros2 launch lazysim gazebo.lazyBot.launch.py disable_rviz:=true
```

**When to use**:
- Algorithm development and testing
- Safe testing without physical robot
- Visualizing robot behavior in RVIZ

**What it launches**:
- Gazebo physics simulation with custom world
- Robot URDF model spawning
- Bridge node for Gazebo↔ROS communication
- Track creator node for dynamic track generation
- RVIZ2 for visualization (optional: `disable_rviz:=true`)
- ROS2 control systems

**Configuration**:
- Edit `track.yaml` to modify track layout
- Edit `lazySim.rviz` for visualization settings

##### **Simulation Control Only**
```bash
ros2 launch lazybot sim.lazy_launch.py
```
**When to use**:
- Running control logic in simulation after Gazebo is already launched
- Testing specific algorithms without restarting simulation

**What it launches**:
- Control node for robot control.
- Detection node for simulated detection
- Debug node for visualization

##### **Visualization Only**
```bash
ros2 launch lazysim rviz_lazySim.launch.py
```
**When to use**:
- Viewing debug data from physical robot
- Monitoring robot state without simulation
- Analyzing recorded data

---

### Usage Workflow

#### 1. **Development Phase** 
```bash
# Start with simulation for safe testing
ros2 launch lazysim gazebo.lazyBot.launch.py

# Test control algorithms
ros2 launch lazybot sim.lazy_launch.py
```

#### 2. **Physical Robot Testing**
```bash
# Basic navigation testing
ros2 launch lazybot open.lazy_launch.py

# Full competition testing  
ros2 launch lazybot robot.lazy_launch.py
```

#### 3. **Color Calibration** (Separate utility)
```bash
# For camera-based tower detection tuning
ros2 run lazybot color_calibration_node
```

#### 4. **Debug Visualization**
```bash
# View robot data in RVIZ (works with physical robot)
ros2 launch lazysim rviz_lazySim.launch.py
```

---

### Hardware Dependencies

- **LiDAR**: RPLiDAR C1 (auto-detected via CP2102N USB bridge)
- **Camera**: Logitech C270 for tower detection
- **ESP32**: Connected via serial for motor control and sensors
- **IMU**: BNO055 for orientation tracking

### Key Configuration Files

- `lazysim/config/track.yaml`: Simulation track layout
- `color_data.yaml`: Color detection parameters (Can be generated from the calibration node)

---

Good luck! Do reach out if you face any issue.