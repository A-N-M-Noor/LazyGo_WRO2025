<div align="center">
  <img alt="image" src="https://github.com/user-attachments/assets/7d07546e-424a-4d2b-ba96-8149f166cd7d" />
</div>

---

## 👥 Team Introduction
<table width="100%" cellspacing="0" cellpadding="10" border="0">
  <tr>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/5c2665d3-2c25-455c-9a63-0e6dbe2a04a4" width="180" height="180"/>
    </td>
    <td width="75%" valign="middle">
      <strong>Iqbal Samin Prithul</strong><br>
      Electronics Developer<br>
      WRO 2022 International Bronze Medalist (Future Engineers Category)<br>
      <a href="mailto:prithul0218@gmail.com">prithul0218@gmail.com</a>
    </td>
  </tr>
</table>
<table width="100%">
  <tr>
    <td width="75%" align="right">
      <strong>Iqbal Samin Prithul</strong><br>
      Electronics Developer<br>
      WRO 2022 International Bronze Medalist (Future Engineers Category)<br>
      <a href="mailto:prithul0218@gmail.com">prithul0218@gmail.com</a>
    </td>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/5c2665d3-2c25-455c-9a63-0e6dbe2a04a4" width="180" height="180"/>
    </td>
    
  </tr>
</table>
<table width="100%">
  <tr>
    <td width="25%" align="center">
      <img src="https://github.com/user-attachments/assets/5c2665d3-2c25-455c-9a63-0e6dbe2a04a4" width="180" height="180"/>
    </td>
    <td width="75%" valign="middle">
      <strong>Iqbal Samin Prithul</strong><br>
      Electronics Developer<br>
      WRO 2022 International Bronze Medalist (Future Engineers Category)<br>
      <a href="mailto:prithul0218@gmail.com">prithul0218@gmail.com</a>
    </td>
  </tr>
</table>
<br>

---

<div>
  <img src="https://github.com/user-attachments/assets/5c2665d3-2c25-455c-9a63-0e6dbe2a04a4" width="180" height="180" align="left"/><br>
  <strong>Iqbal Samin Prithul</strong><br>
  Electronics Developer<br>
  WRO 2022 International Bronze Medalist (Future Engineers Category)<br>
  <a href="mailto:prithul0218@gmail.com">prithul0218@gmail.com</a>
</div><br>

---



## 🎉 Project Overview
<img align="right" alt="Lazy_bot" width="350" src="#">

This repository includes all files, designs, and code for **Lazy_Bot**, our WRO 2025 robot. Below is the folder structure:

## 📂 Structure

Here’s a breakdown of the project folders:

- **[`models`](./models/)**: Contains 3D models and CAD designs.
- **[`src`](./src/)**: Source code for robot programming.
- **[`t-photos`](./t-photos/)**: Technical images of the robot build.
- **[`v-photos`](./v-photos/)**: Visual photos for aesthetics and showcasing.
- **[`video`](./video/)**: Performance and demo videos of SMOKI.

---






----

## Table of Contents

- <span style="color:green;">Mission Overview for WRO Future Engineers Rounds</span>
- <span style="color:orange;">Components and Hardware</span>
- <span style="color:teal;">Mobility Management</span>
- <span style="color:navy;">Power and Sense Management</span>




Our bot, **Lazy_bot**, is built for excellence in the **World Robot Olympiad 2025** in the Future Engineers category. From its custom 3D-printed chassis and LEGO differential drive system to its powerful computing duo of the **Raspberry Pi 5** and **ESP32 microcontroller**, Lazy_bot is engineered to tackle the complex challenges of autonomous navigation, real-time sensing, and dynamic obstacle avoidance.

---

### Mission Overview for WRO Future Engineers Rounds

<table>
  <tr>
    <td width="50%" valign="top" align="left">
      <h3>🏁 Round 1: Lap Completion</h3>
      <p>In <strong>Round 1</strong>, the robot must autonomously complete <strong>three laps</strong> on a pre-defined track. The goal of this round is for the bot to demonstrate stable navigation and precise lap tracking without any obstacle avoidance requirements.</p>
      <ul>
        <li><strong>Objective</strong>: Complete three laps on the track within the allotted time.</li>
        <li><strong>Key Tasks</strong>: Accurate path-following, speed control, and lap counting.</li>
      </ul>
      <div align="center">
        <br><br><br><br><br>
        <img src="https://github.com/user-attachments/assets/823b29fa-8c92-479e-a78a-9fc96c407858" alt="Round 1 WRO Track" width="250" height="180" />
      </div>
    </td>
    <td width="50%" valign="top" align="left">
      <h3>🏆 Round 2: Lap Completion with Obstacle Avoidance and Parking</h3>
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
---


## 🧩 Components and Hardware

Our bot is equipped with various components that support its autonomous functionality. Below is a breakdown of the key hardware elements used in this project:

| Component                          | Description                                                                                      | Image                                                                                      | Purchase Link                                                                                     |
|-----------------------------------|--------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------|
| **Raspberry Pi 5**                | Provides powerful onboard processing for computer vision, SLAM, and higher-level navigation.     | <div align="center"><img src="#" alt="Raspberry Pi 5" width="100"></div>                  | [Purchase Raspberry Pi 5](#)                                                                      |
| **RPLidar C1**                    | Enables 360-degree obstacle detection and environment mapping through LiDAR scanning.            | <div align="center"><img src="#" alt="RPLidar C1" width="100"></div>                       | [Purchase RPLidar C1](#)                                                                          |
| **ESP32 Microcontroller**         | Manages real-time control such as motor commands, sensor data collection, and communication.     | <div align="center"><img src="#" alt="ESP32" width="100"></div>                            | [Purchase ESP32](#)                                                                               |
| **MPU6050 Gyroscope/Accelerometer** | Tracks orientation and motion to assist with balance and movement stabilization.                 | <div align="center"><img src="#" alt="MPU6050" width="100"></div>                          | [Purchase MPU6050](#)                                                                             |
| **0.96" OLED Display (I2C)**      | Displays status information such as battery, sensor readings, and debug info.                    | <div align="center"><img src="#" alt="OLED Display" width="100"></div>                     | [Purchase OLED Display](#)                                                                        |
| **Mini560 Buck Converter**        | Provides compact 5V power regulation for the Raspberry Pi and peripherals.                        | <div align="center"><img src="#" alt="Mini560 Buck Converter" width="100"></div>           | [Purchase Mini560 Buck Converter](#)                                                              |
| **LM2596 5V 5A Buck Converter**   | Supplies a steady 5V 5A output for powering high-draw components like the motor driver.           | <div align="center"><img src="#" alt="LM2596 Buck Converter" width="100"></div>            | [Purchase LM2596 Buck Converter](#)                                                               |
| **20GA Gear Motors with Encoder** | Offers precise speed and position feedback for accurate wheel control and localization.          | <div align="center"><img src="#" alt="20GA Motor with Encoder" width="100"></div>          | [Purchase 20GA Encoder Motor](#)                                                                  |
| **VNH2SP30 Motor Driver**         | High-power motor driver used to control brushed DC motors with PWM and direction control.        | <div align="center"><img src="#" alt="VNH2SP30 Motor Driver" width="100"></div>            | [Purchase VNH2SP30](#)                                                                            |
| **AM117 Servo Motor**             | Controls precise angular movements, typically used for steering or actuation.                    | <div align="center"><img src="#" alt="AM117 Servo" width="100"></div>                      | [Purchase AM117 Servo](#)                                                                         |
| **Custom Secondary PCB**          | Integrates ESP32, MPU6050, OLED, and buck converter for cleaner wiring and modularity.           | <div align="center"><img src="#" alt="Custom PCB" width="100"></div>                       | N/A                                                                                               |
| **3rd Generation LEGO Differential** | Utilized in the drive system to enable turning with gear synchronization.                        | <div align="center"><img src="#" alt="LEGO Differential" width="100"></div>                | [Purchase LEGO Differential](#)                                                                   |
| **3D Printed Body Frame**         | Provides a lightweight and modular chassis tailored for our component layout and design.         | <div align="center"><img src="#" alt="3D Printed Frame" width="100"></div>                 | N/A                                                                                               |

---

## 🚀 Key Features

- **Hybrid LEGO & 3D Printed Design**: Combines modular LEGO Technic elements with custom 3D-printed mounts for adaptability and structural stability.
- **High-Performance Computing with Raspberry Pi 5**: Handles advanced algorithms, LiDAR processing, and high-level control logic.
- **Real-Time Control via ESP32**: Manages low-latency tasks like motor control, sensor polling, and communication through a custom secondary PCB.
- **Efficient Power Management**: Dual buck converters (Mini560 & LM2596) ensure stable voltage delivery across the system.
- **Advanced Sensor Suite**: Equipped with **RPLidar C1**, **MPU6050**, **motor encoders**, and a **0.96" OLED display** for feedback, navigation, and obstacle mapping.

---





----


# 🚗 Mobility Management

This segment outlines the mobility system of **Lazy_bot**, the first version of our robot developed for **WRO 2025 - Future Engineers**.

---

### **Differential Drive System**

Our robot utilizes a differential drive system powered by two **20GA DC gear motors with encoders**. These motors are connected through a **3rd generation LEGO differential**, allowing the wheels on the same axle to rotate at different speeds — a critical feature for smooth cornering and precise movement.

<table>
<tr>
<td width="50%">

##### 🔧 How It Works:
- Each motor drives one side of the robot.
- The differential automatically compensates for wheel speed differences when turning.
- Encoders provide real-time feedback to ensure accurate speed and distance tracking.

##### ✅ Benefits:
1. **Smooth Turning**: Independent wheel speeds allow efficient navigation.
2. **Precise Odometry**: Encoders enhance path planning and tracking.
3. **Compact & Modular**: LEGO-based integration makes the drivetrain easy to modify or maintain.

</td>
<td width="50%">
<div align="center">
  <img src="https://github.com/user-attachments/assets/7a50709c-d077-462a-b6b7-bfc616fd4920" alt="Differential Drive Setup" width="300"/>
</div>
</td>
</tr>
</table>

---

### **Motor Driver - VNH2SP30**

To control the high-current 20GA motors, we use the **VNH2SP30 full-bridge motor driver**. It allows for bidirectional motor control and supports high current output with excellent thermal performance.

##### ⚙️ Features:
- **High Power Handling**: Supports up to 30A peak, suitable for robust DC motors.
- **PWM Support**: Allows smooth speed control using ESP32 PWM signals.
- **Thermal Shutdown & Protection**: Keeps the system safe during heavy load.

---

### **Steering System - 3D Printed Ackermann Mechanism**

The robot features a **3D printed Ackermann steering system** controlled by a **servo motor**. This configuration allows the front wheels to turn at appropriate angles during a corner, minimizing tire slippage and improving real-world steering accuracy.

##### 🛠️ Advantages:
1. **Realistic Geometry**: Replicates automotive-grade steering for smoother and more stable turns.
2. **Customizable Design**: 3D printed components allow fine-tuning and easy iteration.
3. **Precise Control**: Servo-driven mechanism ensures consistent and accurate angle adjustments.
4. **Mechanical Efficiency**: Reduces energy loss and component wear during turning.

<div align="center">
  <img src="https://github.com/user-attachments/assets/6fde062d-c3a9-425e-8087-3d79e93b38eb" alt="3D Printed Ackermann Steering System" width="300"/>
</div>

---

### ✅ Summary

The first version of Lazy_bot features a simple yet effective mobility system combining:
- **Differential drive** with encoder feedback for stable movement.
- **VNH2SP30 motor driver** for high-efficiency control.

This configuration allowed us to build a robust platform capable of handling the dynamic challenges of autonomous navigation in the WRO Future Engineers category.

----
----

# ⚡ Power and Sense Management

The **Power and Sense Management** system of our robot has been meticulously designed to optimize performance while ensuring reliable power delivery, precise sensing, and efficient communication between components.

---

## 📜 Overview

Our system is powered by a 3-cell lithium battery and efficiently distributed using dedicated buck converters for high and low power domains. This structure ensures stable operation across core modules like the Raspberry Pi 5, RPLidar, ESP32, sensors, and motor systems.

---

## ⚙️ System Architecture

### **1. Power Source: 3-Cell Lithium Polymer Battery**
- **Configuration**: 3S (3 cells in series)
- **Voltage**: 12.6V (fully charged) → ~11.1V (nominal) → ~9V (discharged)
- **Features**:
  - High energy density
  - Sufficient current delivery for high-load components like Pi and LiDAR
  - Rechargeable and lightweight

---

### **2. LM2596 / Mini560 5A Buck Converter**
- **Purpose**: Supplies 5V to **Raspberry Pi 5** and **RPLidar C1**
- **Input**: Directly from the 3S battery (~12.6V max)
- **Output**: Stable 5V / 5A
- **Benefits**:
  - Powers high-demand modules without overheating
  - Ensures stable real-time data processing and mapping

---

### **3. Secondary 5V Buck Converter**
- **Purpose**: Powers the **ESP32**, **MPU6050**, **OLED display**, and other secondary PCB components
- **Input**: 12.6V battery input
- **Output**: 5V regulated
- **Use Cases**:
  - Enables low-power real-time control and sensing
  - Protects ESP32 and peripherals from voltage fluctuations

---

### **4. VNH2SP30 Motor Driver with Built-in Regulator**
- **Purpose**: Drives two **20GA motors with encoders**
- **Regulation**: Built-in buck handles motor voltage directly from the 3S battery
- **Features**:
  - No external buck needed
  - Simplifies wiring and improves efficiency
  - Supports high current with integrated protections

---

## 🔌 Voltage Distribution Table

| Component                     | Voltage Supplied | Power Source / Converter        |
|-------------------------------|------------------|----------------------------------|
| **Raspberry Pi 5**            | 5V               | LM2596 / Mini560 (5A Buck)       |
| **RPLidar C1**                | 5V               | LM2596 / Mini560 (5A Buck)       |
| **ESP32 + Sensors + OLED**    | 5V               | Secondary Buck Converter         |
| **Motors (20GA with encoder)**| Battery Voltage  | VNH2SP30 Motor Driver (built-in) |

---

## 🔧 Installation and Configuration

1. **Battery Setup**:
   - Connect the 3S LiPo pack to power rail or XT60 input
   - Ensure battery management safety using a BMS or proper low-voltage cut-off

2. **Main 5A Buck Converter (Pi + Lidar)**:
   - Adjust output to **5.1V** for Pi and LiDAR
   - Connect output through a capacitor filter for extra stability

3. **Secondary Buck Converter (ESP32 PCB)**:
   - Adjust output to **5V**
   - Wire output to ESP32 VIN and sensor lines

4. **Motor Driver (VNH2SP30)**:
   - Connect input power directly from battery
   - No need for external buck or regulator

5. **Final Check**:
   - Use multimeter to test all voltages before powering up components
   - Ensure cooling and airflow for high-power regulators

---

## 🛠 Maintenance Tips

- **Check Connectors Weekly**:
  - Secure all screw terminals and pin headers
- **Monitor Battery Health**:
  - Never discharge below 9V or overcharge past 12.6V
- **Inspect Buck Converters**:
  - Touch test for overheating and ensure proper heatsinking
- **Use a Multimeter**:
  - Regularly verify output voltages and current draw

---

This modular and carefully balanced power system allows **Lazy_bot** to run smoothly under demanding conditions while maintaining safety and long-term reliability.
tubuntu

## 🚀 Future Improvements
- Integrate a smart power monitoring system to dynamically adjust power distribution.
- Upgrade to more energy-efficient components as technology advances. ( WE WILL BE AROUND HOPEFULLY)

---
## 🎥 Camera Placement and Functionality

The robot's main camera is positioned at the top and angled slightly downwards. This setup enhances object detection capabilities by providing:
- **🔍 Close-Range Detection**: The camera can identify objects in close proximity with high accuracy.
- **🌐 Extended-Range Detection**: Ensures objects further away are detected effectively.

The camera feeds data to the **Raspberry Pi 5**, which processes image recognition algorithms to detect towers and corner lines. The processed data is then transmitted to the **ESP32 microcontroller** for real-time navigation and obstacle avoidance.

---


---

## 🛠️ Our PCB is custom made. 

---
# Project Name

## 🖼️ System Visuals

### PCB Views

| **Top View of PCB**                 | **Bottom View of PCB**             |
|-------------------------------------|-------------------------------------|
| <img src="https://github.com/user-attachments/assets/b1cfbc71-9583-4432-a8bf-8053735f607d" width="800"/> | <img src="https://github.com/user-attachments/assets/fe572fad-c348-459d-ab76-1f389a12ebcd" width="800"/> |



