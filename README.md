# Autonomous Navigation Control for KasyTwin Sewer Inspection Robot

An autonomous navigation and centering control system for sewer inspection robots, developed as part of the KaSyTwin (Kanal System Digital Twin) research initiative. This project implements a hierarchical control architecture combining low-level PID control with high-level PCA-based perception to maintain optimal robot alignment and centering within sewer pipes.

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
</div>

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Technical Approach](#technical-approach)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Core Components](#core-components)
- [Configuration](#configuration)



---

## Overview

This autonomous navigation control system enables a differential-drive robot to navigate through sewer pipes while maintaining optimal centering and alignment. The system handles real-world challenges including:

- **Curved pipes** with varying bend radii
- **Asymmetric pipe structures** and deformations
- **Damaged sections** with holes, cracks, and irregularities
- **Variable pipe diameters** and cross-sections

### Core Innovation

The system implements a **hierarchical two-level control architecture**:

1. **High-Level Control (Perception)**: PCA-based algorithm analyzes 2D point clouds from pipe walls to estimate optimal heading direction
2. **Low-Level Control (Actuation)**: PID controller corrects robot yaw and roll based on reference values from high-level perception

This separation of concerns allows the system to handle complex pipe geometries while maintaining stable, predictable control behavior.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                   Autonomous Sewer Navigation System                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │               HIGH-LEVEL CONTROL (Perception)               │    │
│  ├─────────────────────────────────────────────────────────────┤    │
│  │                                                             │    │
│  │  ┌──────────────┐        ┌─────────────────────────────┐    │    │
│  │  │ 3D PointCloud│───────>│    3D to 2D PointCloud      │    │    │
│  │  │ (Pipe Walls) │        │      Converter              │    │    │
│  │  └──────────────┘        └─────────────┬───────────────┘    │    │
│  │                                         │                   │    │
│  │                                         v                   │    │
│  │                           ┌─────────────────────────────┐   │    │
│  │                           │   PCA Algorithm             │   │    │
│  │                           │  • Extract pipe geometry    │   │    │
│  │                           │  • Estimate principal axis  │   │    │
│  │                           │  • Calculate optimal yaw    │   │    │
│  │                           └─────────────┬───────────────┘   │    │
│  │                                         │                   │    │
│  │                                         v                   │    │
│  │                           ┌─────────────────────────────┐   │    │
│  │                           │  Reference Manager          │   │    │
│  │                           │  • Generate ref yaw/roll    │   │    │
│  │                           │  • Compute centering target │   │    │
│  │                           └─────────────┬───────────────┘   │    │
│  └─────────────────────────────────────────┬───────────────────┘    │
│                                            │                        │
│                        Reference Values    │                        │
│                        (Yaw, Roll, Center) │                        │
│                                            v                        │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │               LOW-LEVEL CONTROL (Actuation)                 │    │
│  ├─────────────────────────────────────────────────────────────┤    │
│  │                                                             │    │
│  │  ┌──────────────┐        ┌─────────────────────────────┐    │    │
│  │  │     IMU      │───────>│   Quaternion to Euler       │    │    │
│  │  │(Orientation) │        │   Converter                 │    │    │
│  │  └──────────────┘        └─────────────┬───────────────┘    │    │
│  │                                        │                    │    │
│  │                    Current State       │                    │    │
│  │                    (Yaw, Roll)         │                    │    │
│  │                                        v                    │    │
│  │  Reference ───>        ┌─────────────────────────────┐      │    │
│  │  Values                │   PID Controller             │     │    │
│  │                        │  • Yaw correction (angular)  │     │    │ 
│  │                        │  • Roll correction (lateral) │     │    │
│  │                        │  • Centering control         │     │    │
│  │                        └─────────────┬───────────────┘      │    │
│  │                                      │                      │    │
│  │                                      v                      │    │
│  │                        ┌─────────────────────────────┐      │    │
│  │                        │   Motor Command Interface   │      │    │
│  │                        │  • Twist to wheel commands  │      │    │
│  │                        │  • Differential drive ctrl  │      │    │
│  │                        └─────────────┬───────────────┘      │    │
│  └──────────────────────────────────────┬──────────────────────┘    │
│                                         │                           │
│                                         v                           │
│                           ┌─────────────────────────┐               │
│                           │   Robot Actuators       │               │
│                           │   (Differential Drive)  │               │
│                           └─────────────────────────┘               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Technical Approach

### Hierarchical Control Architecture

#### 1. High-Level Control: PCA-Based Perception

**Challenge**: Sewer pipes are not perfectly cylindrical and often contain:
- Bends and curves requiring heading adjustments
- Asymmetric damage (holes, cracks)
- Varying cross-sections
- Partial obstructions

**Solution**: Principal Component Analysis (PCA) on 2D point clouds

The PCA algorithm processes point clouds from both sides of the pipe wall to:

1. **Extract geometric features** from the pipe structure
2. **Identify the principal axis** of the pipe (dominant direction)
3. **Estimate the optimal heading direction** for maintaining alignment
4. **Compensate for asymmetric structures** that would otherwise bias simple centering algorithms

**Why PCA?**
- **Robust to outliers**: Handles holes and cracks without losing track of pipe geometry
- **Adapts to curves**: Naturally follows the pipe's principal direction even in bends
- **Handles asymmetry**: Statistical approach is not fooled by one-sided damage
- **Computationally efficient**: Real-time processing on embedded hardware

#### 2. Low-Level Control: PID Controllers

**Dual-axis PID control** maintains robot orientation and position:

**Yaw Control (Heading)**
- Input: Reference yaw from PCA algorithm
- Current: IMU-measured yaw angle
- Output: Angular velocity command
- Purpose: Keeps robot aligned with pipe direction

**Roll Control (Lateral Position)**
- Input: Reference roll (typically 0° for level)
- Current: IMU-measured roll angle
- Output: Corrective lateral movement
- Purpose: Prevents robot from tilting or climbing walls

### Sensor Fusion

**3D PointCloud**
- 360° scanning of pipe walls
- Converted to 2D for PCA processing
- Update rate: 10-20 Hz typical

**IMU (Inertial Measurement Unit)**
- Provides orientation (quaternion)
- Converted to Euler angles (roll, pitch, yaw)
- High-frequency updates: 50-100 Hz
- Drift compensation through sensor fusion


---

## Features

### Navigation Capabilities

✅ **Autonomous Pipe Centering**: Maintains optimal position in pipe center  
✅ **Curve Handling**: Adapts to bends and curves using PCA-derived heading  
✅ **Damage Resilience**: Continues navigation despite holes, cracks, and asymmetry  
✅ **Multi-Diameter Support**: Works across various pipe sizes  
✅ **Real-Time Processing**: Low-latency control loop for responsive navigation

### Perception & Sensing

✅ **PCA-Based Direction Estimation**: Statistical geometric analysis for robust heading   
✅ **IMU Orientation Tracking**: Precise roll, pitch, yaw measurement  
✅ **Sensor Fusion**: Combines LiDAR and IMU for enhanced accuracy

### Control Systems

✅ **Hierarchical Architecture**: Separates perception from actuation  
✅ **Dual-Axis PID Control**: Independent yaw and roll correction    
✅ **Dynamic Tuning**: Adjustable PID gains for different environments

### Development & Testing

✅ **Full Gazebo Integration**: Physics-based simulation of sewer environments  
✅ **Custom Pipe Models**: Realistic tube geometries for testing  
✅ **ROS 2 Native**: Built on modern ROS 2 architecture  
✅ **Hardware/Simulation Parity**: Same codebase for sim and real robot

---

## Prerequisites

### Software Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **C++ Compiler**: GCC 11+ with C++17 support
- **Gazebo**: Ignition Gazebo (Fortress or Garden)

### Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-pcl-ros \
  ros-humble-laser-geometry
```

### Python Dependencies

```bash
pip3 install numpy scipy scikit-learn transforms3d
```

---

## Installation

### 1. Create ROS 2 Workspace

```bash
mkdir -p ~/sewer_robot_ws/src
cd ~/sewer_robot_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/omarelnahas/Autonomous-Navigation-Control-for-KasyTwin-Robot. control_1
```

### 3. Build Package

```bash
cd ~/sewer_robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Verify Installation

```bash
ros2 launch control_1 rsp.launch.py
```

---

## Usage

### Simulation Mode (Testing & Development)

**Full Simulation with Sewer Pipe Environment:**

```bash
# Launch complete simulation stack
ros2 launch control_1 rsp.launch.gazebo.py

# Optional: Monitor system health
ros2 topic echo /diagnostics
```

**What's Running:**
- Gazebo with custom sewer pipe model
- Robot spawned at pipe entrance
- All sensors (LiDAR, IMU, odometry) bridged to ROS 2
- Complete navigation stack active

### Hardware Mode (Physical Robot)

**Deploy on Real Robot:**

```bash
# Launch hardware stack (no Gazebo)
ros2 launch control_1 rsp.launch.py

# Optional: Monitor system health
ros2 topic echo /diagnostics
```

### Manual Testing of Components

**Test PCA Algorithm:**

```bash
# Launch just the PCA node
ros2 run control_1 pca_node --ros-args --params-file $(ros2 pkg prefix control_1)/share/control_1/config/config.yaml

# Play recorded bag file with PointCloud data
ros2 bag play sewer_test_data.bag
```

**Test PID Controller:**

```bash
# Launch PID controller
ros2 run control_1 pid_control_node --ros-args --params-file $(ros2 pkg prefix control_1)/share/control_1/config/config.yaml

# Publish reference values
ros2 topic pub /karo/reference/cmd_vel geometry_msgs/Twist "..."
```
---

## Core Components

### 1. PCA Node (High-Level Perception)

**Node**: `pca_node`  
**Purpose**: Analyzes 2D point clouds to estimate optimal robot heading in pipe


### 2. Reference Manager Node (High-Level Control)

**Node**: `reference_manager_node`  
**Purpose**: Generates reference trajectories and target values for low-level controller


### 3. PID Controller Node (Low-Level Control)

**Node**: `pid_control_node`  
**Purpose**: Implements dual-axis PID control for yaw and roll correction


### 4. Motor Command Node (Actuation Interface)

**Node**: `motor_command_node`  
**Purpose**: Converts Twist commands to differential drive motor commands


### 5. IMU Quaternion to Euler Node

**Node**: `imu_quat_to_euler_node`  
**Purpose**: Converts IMU quaternion orientation to Euler angles (roll, pitch, yaw)

### 6. Health Monitor Node

**Node**: `topic_health_monitor_node`  
**Purpose**: Monitors system health, topic rates, and sensor status

---

## Configuration

### Main Configuration File: `config/config.yaml`

---

### Gazebo Simulation Configuration

Custom sewer pipe model with realistic dimensions and materials:

**Tube Model (`description/tube_model.sdf`)**

---

## Contact
 
**Institution**: [INATECH/University of Freiburg]  
**Project**: KaSyTwin Autonomous Navigation Research

**Repository**: [github.com/omarelnahas/Autonomous-Navigation-Control-for-KasyTwin-Robot](https://github.com/omarelnahas/Autonomous-Navigation-Control-for-KasyTwin-Robot)

For questions, issues, or collaboration inquiries:
- Open an issue on GitHub
- Email: [omarelnahas54@gmail.com]

---