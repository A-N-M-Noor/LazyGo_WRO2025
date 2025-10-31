<div align="center">
  <img alt="image" src="./assets/banner.png" />
</div>
  <strong>Team LazyGo</strong> is a robotics team from <strong>Bangladesh</strong>. Originally formed in 2022, LazyGo won <strong>Bronze</strong> at the WRO international round in the Future Engineers category. Now returns again with three experienced international participants.
<br><br>

We named our robot **LazyBot**, which is built for the **Future Engineers** category in the **World Robot Olympiad 2025**. This year, we are trying to bring some interesting changes from the typical way a robot is built for this category. Definitely check out [Odometry Calculation](#odometry), [Disparity Extender Algorithm](#avoidance-using-lidar), and [LiDAR Based Tower Detection](#obstacle-round) algorithms.

This repository contains all the files, codes, models, photos and everything about our team and the robot.

Visit our socials:

[![facebook](https://img.shields.io/badge/face-book-blue?style=flat-square&logo=facebook)](https://www.facebook.com/team.lazygo)
[![linkedin](https://img.shields.io/badge/linked-in-blue?style=flat-square)](https://www.linkedin.com/company/lazygo)


## Table of Contents

- [`Team Introduction`](#team-introduction)
- [`Mission Overview for WRO Future Engineers Rounds`](#mission-overview-for-wro-future-engineers-rounds)
- [`Repository`](#repository)
- [`Key Features`](#key-features)
- [`Components and Hardware`](#components-and-hardware)
- [`Algorithm`](#algorithm)
- [`Mobility Management`](#mobility-management)
- [`Power and Sense Management`](#power-and-sense-management)


---
## Team Introduction
<div>
  <img src="./assets/Prithul.png" width="180" height="180" align="left"/><br>
  <strong>Iqbal Samin Prithul</strong><br>
  Embedded Electronics<br>
  WRO 2022 International Bronze Medalist (Future Engineers Category)<br>
  <a href="mailto:prithul0218@gmail.com">prithul0218@gmail.com</a>
</div><br>

---
<div>
  <img src="./assets/Noor.png" width="180" height="180" align="right"/><br>
  <div align="right">
  <strong>A.N.M Noor</strong><br>
  Software and ROS<br>
  WRO 2023 International Participant (Future Engineers Category)<br>
  <a href="mailto:noornoorrohan15@gmail.com">noornoorrohan15@gmail.com</a></div>
</div><br>

---
<div>
  <img src="./assets/Rakibul.png" width="180" height="180" align="left"/><br>
  <strong>Rakibul Islam</strong><br>
  Hardware and CAD design<br>
  WRO 2024 International Participant (Future Engineers Category)<br>
  <a href="mailto:rakibul.rir06@gmail.com">rakibul.rir06@gmail.com</a>
</div><br>

---

---

## Mission Overview for WRO Future Engineers Rounds

<table>
  <tr>
    <td width="50%" valign="top" align="center"><h3>🏁 Round 1: Lap Completion</h3></td>
    <td width="50%" valign="top" align="center"><h3>🏆 Round 2: Lap Completion with Obstacle Avoidance and Parking</h3></td>
  </tr>
  <tr>
    <td width="50%" valign="top" align="left">
      <p>In <strong>Round 1</strong>, the robot must autonomously complete <strong>three laps</strong> on a pre-defined track. The goal of this round is for the bot to demonstrate stable navigation and precise lap tracking without any obstacle avoidance requirements.</p>
      <ul>
        <li><strong>Objective</strong>: Complete three laps on the track within the allotted time.</li>
        <li><strong>Key Tasks</strong>: Accurate path-following, speed control, and lap counting.</li>
      </ul>
      <div align="center" valign="bottom">
        <br><br><br><br>
        <img src="https://github.com/user-attachments/assets/823b29fa-8c92-479e-a78a-9fc96c407858" alt="Round 1 WRO Track" width="250" height="180" />
      </div>
    </td>
    <td width="50%" valign="top" align="left">
      <p>In <strong>Round 2</strong>, the bot must complete <strong>three laps</strong> while avoiding green and red obstacles:</p>
      <ul>
        <li><strong>Green Obstacles</strong>: The bot should move <strong>left</strong> to avoid.</li>
        <li><strong>Red Obstacles</strong>: The bot should move <strong>right</strong> to avoid.</li>
      </ul>
      <p>After completing the laps, the bot must accurately park within a designated zone.</p>
      <ul>
        <li><strong>Objective</strong>: Complete three laps, avoid obstacles, and park in the designated area.</li>
        <li><strong>Tasks</strong>: Obstacle detection, color-based avoidance, and precision parking.</li>
      </ul>
      <div align="center">
        <img src="https://github.com/user-attachments/assets/b578392d-b443-4315-8fe3-f03af828c39a" alt="Round 2 WRO Track" width="250" height="180" />
      </div>
    </td>
  </tr>
</table>

---
>[!IMPORTANT]
>**Important: WRO Future Engineers Rulebook**
>* **Thorough Reading:** Ensure that you thoroughly read the **WRO Future Engineers 2025 Rulebook** to understand all rules and guidelines.
>* **Official Link:** Access the rulebook here: [🔗 WRO Future Engineers 2025 Rulebook](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).

---
## Repository

This repository includes all files, designs, and code for **LazyBot**, our WRO 2025 robot. Below is the file structure:

### File Structure


<img align="right" alt="LazyBot" width="270" src="./assets/lazygo_logo_round.png">
Here’s a breakdown of the project folders:

- **[`assets`](./assets/)**: Contains all the images used in the README files of this repository.
- **[`instructions`](./instructions/)**: Contains all the instructions on how to setup and use the package.
- **[`models`](./models/)**: Contains 3D models and CAD designs of the robot.
- **[`src`](./src/)**: Source code for the robot's programming. This contains the ROS2 packages.
- **[`t-photos`](./t-photos/)**: Technical images of the robot build.
- **[`v-photos`](./v-photos/)**: Visual photos for aesthetics and showcasing.
- **[`video`](./video/)**: Performance and demo videos of LazyBot.

---
---


## Key Features

- **`Hybrid LEGO & 3D Printed Design`**: Using LEGO parts proved to be very helpful based on the previous experience of our members. That, along with 3D printed parts made it possible to harness LEGO's precision and the flexibility of 3D printing.
- **`Advanced Sensor Suite`**: LazyBot is equipped with a LiDAR (to perfectly sense the sorroundings), encoder motor (to precisely calculate the position), IMU (to calculate realtime orientation).
- **`Use of Robot Operating System`**: We used ROS2 to control the robot. We took this decision because Robot Operating System allows to make a project very modular and provide a lot of useful tools for simulation and visualization.
- **`Real-Time Odometry Calculation`**: This is one of the major feature of pur robot. Using the onboard IMU sensor and the value from the motor's encoder, the robot calculates the exact realtime position of the robot.
- **`Efficient Debugging`**: We've added an OLED display to the MCU of the robot which helps to debug the issues on the MCU side. And ROS2 provides with a lot of debugging tools that helps us debug any issues hapening on the Pi side.

---
---


## Components and Hardware

Our bot is equipped with various components that support its autonomous functionality. Below is a breakdown of the key hardware elements used in this project:

| Component                          | Description                                                                                      | Image                                                                                      |
|-------------------------------------|--------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------|
| **Raspberry Pi 5**                | Provides powerful onboard processing for computer vision, and higher-level navigation.            | <div align="center"><img src="./assets/Pi5.jpg" alt="Raspberry Pi 5" width="200"></div>                  |
| **RPLidar C1**                    | Enables 360-degree obstacle detection and environment mapping through LiDAR scanning.             | <div align="center"><img src="./assets/lidar.jpg" alt="RPLidar C1" width="200"></div>                       |
| **Logitech C270 Camera**                    | Using this to detect colored towers.             | <div align="center"><img src="./assets/camera.png" alt="Logi c270" width="200"></div>                       |
| **ESP32 Microcontroller**         | Manages real-time control such as motor commands, sensor data collection, and communication.      | <div align="center"><img src="./assets/ESP32.jpg" alt="ESP32" width="200"></div>                            |
| **BNO055 9Axis IMU Sensor**       | Tracks orientation and motion to assist with balance and movement stabilization. The internal microcontroller allows for precision orientation tracking. | <div align="center"><img src="./assets/BNO.jpg" alt="BNO055" width="200"></div>                          |
| **0.96" OLED Display (I2C)**      | Displays status information such as sensor readings, and debug info.                              | <div align="center"><img src="./assets/oled.jpg" alt="OLED Display" width="200"></div>                     |
| **Mini560 Buck Converter**        | Provides compact 5V power regulation for the servo.                        | <div align="center"><img src="./assets/mini560.jpg" alt="Mini560 Buck Converter" width="200"></div>           |
| **5V 5A Buck Converter (LM2596S)**   | Supplies a steady 5V 5A output for powering the raspberry pi and peripherals.           | <div align="center"><img src="./assets/5v5a.jpg" alt="LM2596 Buck Converter" width="200"></div>            |
| **25GA Gear Motors with Encoder** | Offers precise speed and position feedback for accurate wheel control and localization.           | <div align="center"><img src="./assets/25GA.jpg" alt="25GA Motor with Encoder" width="200"></div>          |
| **VNH2SP30 Motor Driver**         | High-power motor driver used to control brushed DC motors with PWM and direction control.         | <div align="center"><img src="./assets/VNH2SP30.jpg" alt="VNH2SP30 Motor Driver" width="200"></div>            |
| **PS1171MG Servo Motor**             | Controls precise angular movements, typically used for steering or actuation.                     | <div align="center"><img src="./assets/1171mg.jpg" alt="AM117 Servo" width="200"></div>                      |
| **Custom Secondary PCB**          | Integrates ESP32, BNO055, OLED, and buck converter for cleaner wiring and modularity.             | <div align="center"><img src="./assets/pcb_bottom.jpeg" alt="Custom PCB" width="200" style="aspect-ratio: 1 / 1; object-fit: cover;"></div>                       |
| **3rd Generation LEGO Differential** | Utilized in the drive system to enable turning with gear synchronization.                      | <div align="center"><img src="./assets/Diff.jpg" alt="LEGO Differential" width="200"></div>                |
| **3D Printed Body Frame**         | Provides a lightweight and modular chassis tailored for our component layout and design.          | <div align="center"><img src="./v-photos/right.jpeg" alt="3D Printed Frame" width="200"></div>                 |

---
---

## Algorithm
### Odometry
A very interesting feature of our robot is that it can calculate it's realtime position. We achieved this by fusing the realtime orientation value with the motor's encoder values. When the robot is moving in a straight line, it is possible to calculate how far the robot has moven using the encoder values. But it isn't sosimple when the robot turns. But when the robot is turning, we can actually divide it's curved path into tiny sentions that resembles straight lines. And if we accumulate those straight lines, taking their directions into account, we can find the actual displacement of the robot pretty accurately.

Here's how it is calculated:
1. Find the delta of the encoder's value. Let's call it `ds`
1. Find the average angle between the last calculation and current heading angle. Let's call it `A`.
1. The movement vector would be: _(x = `ds*cos(A)`, y = `ds*sin(A)`)_
1. By accumulating this movement vector, and we can find the realtime position of the robot.

Before getting an usable value, we had to calibrate our encoder to calculate values in metric unit.

```mermaid
flowchart LR
    C["Repeat process continuously"] --> D["Calculate delta encoder value: ds"]
    D --> E["Read current orientation angle"]
    E --> F["Find average angle (A) between last and current heading"]
    F --> G["Compute movement vector: x = ds * cos(A), y = ds * sin(A)"]
    G --> H["Accumulate movement vector to update position"]
    H --> I["Output realtime position"]
    I --> C
    A["Start"] --> C
```

### Open Round

#### Avoidance using LiDAR
During the open round, there is no towers on the track. So we don't need the camera. To move the robot in the track, we use a modified version of `Disparity Extender` algorithm. Here's a step by step description of the idea:

1. The LiDAR scans the area and gives a bunch of distances in many directions (one distance per ray).
1. For each ray, start with the ray’s measured distance as the “candidate” distance.
1. Look at the nearby rays around that ray. If any nearby ray’s obstacle is close enough that the car’s body would hit it when moving along the candidate ray, shorten the candidate distance to that nearby obstacle. (In other words: pretend the car is wide and see where it would first hit something.)
1. After doing that for every ray, you have a “safe distance” for every direction — the farthest you can travel in that direction without your body hitting something.
1. Pick the direction with the largest safe distance and steer the car toward it.

```mermaid
flowchart LR
    A[Start control loop] --> B[Perform LiDAR scan to get rays]
    B --> C[Iterate over each ray]
    C --> D[Set candidate distance from measured ray]
    D --> E[Check nearby rays for potential body collision]
    E --> F{Nearby ray would cause collision}
    F -- Yes --> G[Shorten candidate distance to nearby obstacle]
    F -- No  --> H[Keep candidate distance as is]
    G --> I[Save safe distance for current ray]
    H --> I
    I --> J{More rays to process}
    J -- Yes --> C
    J -- No  --> K[Pick direction with largest safe distance]
    K --> L[Steer car toward chosen direction]
    L --> M[Repeat control loop]
    M --> B
```

One drawback of this method is that on straight sections, the robot shows a tendency to point itself toward the next corner. This happens because corners often look like the direction with the most open space before discovering the next turn, so the algorithm treats them as the safest option - even though the robot should ideally stay centered on the straight path. But this algorithm works really well to move between tight gaps. So the little drawback doesn't really matter to us. And ofcourse, there are ways to improve on this issue.

One important thing to remember, the robot does not target the farthest distance, it targets the farthest `safe` distance - where it can move to without collision. Here's how it works:

<table>
    <tr>
        <td align="center">
            <img src="./assets/collision.png" alt="Targetting farthest distance">
        </td>
        <td align="center">
            <img src="./assets/no_collision.png" alt="Targetting farthest safe distance">
        </td>
    </tr>
    <tr>
        <td align="center"><sub>Targetting farthest distance</sub></td>
        <td align="center"><sub>Targetting farthest `safe` distance</sub></td>
    </tr>
</table>

#### Lap Count
Because we can precisely calculate odometry, keeping lap count is a very simple task. The robot just keeps track of how many times it passes through the starting section. When it reaches the desired lap count, it just stops there. And it worked really well. The robot always stops between a few centimeters from the dead center of the starting section.

### Obstacle Round
It works similar to the open round. But the robots needs to detect the towers. In our robot, tower detection is done in two ways.

1. `Using the camera`: This is a very basic color detection algorithm. From the camera feed, the robot detects the towers by masking colors.
1. `Using LiDAR data`: This approach is a bit more interesting. Towers create sudden changes (valley) in the LiDAR distance readings. If the readings suddenly get closer and then farther again, that usually means there’s an object in between. By checking how wide this change looks from the LiDAR’s point of view, we can estimate whether it matches the expected size of a tower. The object's size can be easily calculated using the formula `s = rθ`. If the size is around 5cm (width of a tower) the robot marks it as a possible tower. But it still confirms the color with the camera to be sure.

Here's how it works:

<table>
    <tr>
        <td align="center">
            <img src="./assets/Tower.png" alt="Tower">
        </td>
        <td align="center">
            <img src="./assets/notTower.png" alt="Not a tower">
        </td>
    </tr>
    <tr>
        <td align="center"><sub>The valley has a depth of more than 20cm and the calculated size is around 5cm</sub></td>
        <td align="center"><sub>These are not towers because either the valley depth is lower than 20cm, or the calculated size is much different from 5cm.</sub></td>
    </tr>
</table>


```mermaid
flowchart LR
    A[Start LiDAR scan] --> B[Read distance values across rays]
    B --> C[Look for sudden changes in distances]
    C --> D{Spike detected?}
    D -- No --> B
    D -- Yes --> E[Measure width of the detected spike]
    E --> F{Width matches tower size?}
    F -- No --> B
    F -- Yes --> H[Mark as possible tower]
    H --> I[Final tower detection result]
    I --> B[Continue scanning]

```

After detecting the towers, the robot needs to avoid them. The robot needs to move to the left of green objects. So what it does, it imagins a wall at the right of any green tower. That way,the robot is forced to pass the object from the left side. The opposite happens for red towers.

### Camera Placement

We attached the robot's main camera on a servo motor. Because we are detecting the towers' positions using the LiDAR values, we can easily point the camera towards the closest object using the servo. This makes it super easy to isolate a target tower. And this allowes us to use a lower Field of View (`FoV`) camera and cover a large area.

But does using a low `FoV` camera has any benefit?

Well, fish eye lenses introduce a lot of distortion. It can be fixed by calculating the lense intrinsics and camera matrix coefficients, but that also takes up some processing time as it needs to be done per frame. Although it is not too much of an issue if you just need to use color masking using opencv. But there is another issue with high `FoV` cameras. Generally, cameras with a higher `FoV` have lower image brightness (per unit area) if other settings (such as aperture and exposure time) remain the same. This is because the same amount of light gathered by the lens is distributed over a larger sensor area. We can increase the exposure, but that increases motion blur and sometimes even introduces lag. This is an issue especially for WRO Future Engineers Category. Over the years, we've noticed that the tower colors are usually very dark. This makes it very difficult to detect their colors. Here, a camera with more overall brightness will perform better than one with a lower overall brightness. So a low `FoV` camera helps in this sense as well. 

Also, as we are directly pointing the camera towards a tower, the chance to falsely detect, let's say, the parking walls as a red tower.


---
---


## Mobility Management

This segment outlines the mobility system of **LazyBot**, with two key features: a differential gear system, and ackermann steering.

---

### **Differential Drive System**

Our robot utilizes a differential gear system powered by one **25GA DC gear motor with encoders** and a **3rd generation LEGO differential**, allowing the wheels on the same axle to rotate at different speeds even though they are both connected toa single motor. This is crucial feature for smooth cornering and precise movement.

<table>
<tr>
<td width="50%">

#### How It Works:
- The motor is connected to the differential gearset. The gear system distributes torque to the wheels as needed.
- The differential automatically compensates for wheel speed differences when turning.
- Encoders provide real-time feedback to ensure accurate speed and distance tracking.

#### Benefits:
1. **Smooth Turning**: Independent wheel speeds allow efficient navigation.
2. **Precise Odometry**: Encoders enhance path planning and tracking.
3. **Compact & Modular**: LEGO-based integration makes the drivetrain easy to modify or maintain.

<div align="center">
  <img src="./assets/diff_drive_diagram.png" alt="Differential Gear Diagram" width="80%" style="max-width: 400px; border-radius: 20px">
</div>

</td>
<td width="50%">
<div align="center">
  <img src="./assets/differential.jpeg" alt="Differential Drive Setup" style="border-radius: 20px" width="300"/>
  <br>
  A 16GA Motor was being used when we took this picture.
</div>
</td>
</tr>
</table>

### **Steering System - 3D Printed Ackermann Mechanism**

The robot features a **3D printed Ackermann steering system** controlled by a **servo motor**. This configuration allows the front wheels to turn at appropriate angles during a corner, minimizing tire slippage and improving real-world steering accuracy.

##### Advantages:
1. **Realistic Geometry**: Replicates automotive-grade steering for smoother and more stable turns.
2. **Customizable Design**: 3D printed components allow fine-tuning and easy iteration.
3. **Precise Control**: Servo-driven mechanism ensures consistent and accurate angle adjustments.
4. **Mechanical Efficiency**: Reduces energy loss (friction) and component wear during turning.

<div align="center">
  <img src="./assets/steering.jpeg" alt="3D Printed Ackermann Steering System" style="border-radius: 20px" width="500"/>
</div>

---
---

## Power and Sense Management

The **Power and Sense Management** system of our robot has been meticulously designed to optimize performance while ensuring reliable power delivery, precise sensing, and efficient communication between components.

---

### Overview

Our system is powered by a 3-cell lithium battery and efficiently distributed using dedicated buck converters for high and low power domains. This structure ensures stable operation across core modules like the Raspberry Pi 5, RPLidar, ESP32, sensors, and motor systems.

---

### System Architecture

#### **1. Power Source: 3-Cell Lithium Polymer Battery**
- **Configuration**: 3S (3 cells in series)
- **Voltage**: 12.6V (fully charged) → ~11.1V (nominal) → ~9V (discharged)
- **Features**:
  - High energy density
  - Sufficient current delivery for high-load components like Motors, Pi and LiDAR
  - Rechargeable and easily purchasable

---

#### **2. 5V 5A Buck Converter with unknown chip**
- **Purpose**: Supplies 5V to **Raspberry Pi 5** and **RPLidar C1**
- **Input**: Directly from the 3S battery (~12.6V max)
- **Output**: Stable 5V / 5A
- **Benefits**:
  - Provides continuous high-current draw without overheating
  - Ensures stable real-time data processing and mapping

---

#### **3. Secondary 5V 5A Buck Converter (Mini560)**
- **Purpose**: Supplies 5V to **Servo**
- **Input**: Directly from the 3S battery (~12.6V max)
- **Output**: Rush currents at 5V / 5A
- **Benefits**:
  - Provides rush current with large electrolytic capacitors
  - Ensures separate 5V with separate ground path so that the servo current does not affect other modules

---

#### **4. VNH2SP30 Motor Driver with Built-in Regulator**
- **Purpose**: Drives the **20GA motor with encoders**
- **Regulation**: Built-in buck handles motor voltage directly from the 3S battery
- **Features**:
  - No external 5V needed
  - Simplifies wiring and improves efficiency
  - Has very low RDS(on) resistance with integrated protections

---

### Voltage Distribution Table

| Component                     | Voltage Supplied | Power Source / Converter        |
|-------------------------------|------------------|----------------------------------|
| **Raspberry Pi 5**            | 5V               | Unknown chip 5A Buck             |
| **RPLidar C1**                | 5V               | Unknown chip 5A Buck             |
| **ESP32 + Sensors + OLED**    | 5V               | Unknown chip 5A Buck             |
| **Servo Motor**               | 5V               | Mini560 5A Buck                  |
| **Motors (20GA with encoder)**| Battery Voltage  | VNH2SP30 Motor Driver (built-in) |

```mermaid
---
config:
  theme: redux
---
flowchart TD
    n1["3s LiPo Battery"] --> n3["5V 5A Buck<br>Converter"] & n4["Mini 360 Buck<br>Converter"] & n5["VNH2SP30<br>Motor Driver"]
    n3 --> n6["Raspberry Pi 5"]
    n6 --> n7["RP Lidar C1"] & n8["USB Camera"] & n9["Main PCB"]
    n9 --> n10["ESP32<br>IMU<br>IR Sensors<br>OLED<br>Encoder"]
    n4 --> n11["Servo"]
    n5 --> n12["Motor"]
    n1@{ shape: rounded}
```
---
---

## Our PCB is completely custom homemade. 


### PCB Views

| **Top View of PCB**                 | **Bottom View of PCB**             |
|-------------------------------------|-------------------------------------|
| <img src="./assets/pcb_top.jpeg" width="800"/> | <img src="./assets/pcb_bottom.jpeg" width="800"/> |



