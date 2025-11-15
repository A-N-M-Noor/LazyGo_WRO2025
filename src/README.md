# Codebase of LazyBot
This directory contains all the codes of LazyBot (By Team LazyGo)

File structure:
```
src
├── ESP32 Code                      (PlatformIO Project for the ESP32)
│   └── src/                        (Contains the c++ code for esp32)
|
├── lazy_interface/                 (ROS Package - Message Types)
│   ├── msg/                        (Contains custom message types used by ROS to control the robot)
│   └── srv/                        (Currently, we're not using any custom ROS services)
|
├── lazybot/                        (ROS Package - Control Package)
│   ├── launch/                     (Contains launch files to launch necessary nodes together)
|   |   ├── open.lazy_launch.py     (Launch file for open challenge)
|   |   ├── robot.lazy_launch.py    (Launch file for obstacle challenge)
│   |   └── sim.lazy_launch.py      (Launch file for simulation run)
|   └── lazybot/                    (ROS Nodes - contains all the code files for control package)
|
└── lazysim/                        (ROS Package - Gazebo Simulation)
    ├── config/                     (Configure files for the simulation)
    |   └── track.yaml              (Configure the simulated track)
    └── launch/                     (Contains launch files to launch necessary nodes together)
        ├── control.launch.py       (ROS-Control package launcher: This is needed to control the robot in simulation world)
        ├── gazebo.lazyBot.launch.py    (Launches the simulation world with robot model)
        ├── lazySim.launch.py       (Only launches the robot model.)
        └── rviz_lazySim.launch.py  (Launches RVIZ with appropriate topics and settings)
```

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