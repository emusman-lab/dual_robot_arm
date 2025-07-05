# ISAAC SIM DUAL ROBOT MANIPULATION

This project demonstrates the simulation and manipulation of **Two Dobot Nova5 robotic arms** in **NVIDIA Isaac Sim 4.2.0** using **MoveIt 2** and **MoveIt Task Constructor**.

---

## 📑 Table of Contents

- [ISAAC SIM DUAL ROBOT MANIPULATION](#isaac-sim-dual-robot-manipulation)
  - [📑 Table of Contents](#-table-of-contents)
  - [📖 Overview](#-overview)
  - [✅ Requirements](#-requirements)
  - [🛠️ Installation Instructions](#️-installation-instructions)
    - [Install MoveIt 2](#install-moveit-2)
    - [Install Isaac Sim 4.2.0](#install-isaac-sim-420)
  - [📁 Project Setup](#-project-setup)
    - [Clone the Repository](#clone-the-repository)
    - [Build and Source the Workspace](#build-and-source-the-workspace)
  - [🚀 Running the Simulation](#-running-the-simulation)
    - [Launch Isaac Sim](#launch-isaac-sim)
    - [Launch MoveIt 2 and RViz2](#launch-moveit-2-and-rviz2)
    - [Launch MoveIt Task Constructor](#launch-moveit-task-constructor)
  - [🔗 Links \& References](#-links--references)

---

## 📖 Overview

This package controls and coordinates two **Dobot Nova5 arms** in a simulated environment provided by **NVIDIA Isaac Sim**. The arms are controlled using **MoveIt 2**, and complex manipulation tasks (like pick-and-place) are executed using **MoveIt Task Constructor (MTC)**. This setup is ideal for testing dual-arm industrial manipulation, coordinated motion planning, and task sequencing.

---

## ✅ Requirements

- **ROS 2 Humble** (or later)
- **MoveIt 2**
- **Isaac Sim 4.2.0** or later
- **NVIDIA GPU** with proper drivers
- **Colcon build system**
- **Python 3.10**

---

## 🛠️ Installation Instructions

### Install MoveIt 2

Follow the MoveIt 2 installation instructions for your ROS 2 distro:

🔗 https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

Make sure MoveIt 2 is **properly sourced** in your terminal:

```bash
source /opt/ros/humble/setup.bash
```

---

### Install Isaac Sim 4.2.0

Install Isaac Sim 4.2.0 by following the official guide:

🔗 https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_isaacsim.html

Also follow the guide for ROS 2 bridge integration:

🔗 https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_setup.html

Make sure Isaac Sim can run with `python.sh` and ROS 2 environment is sourced in it.

---

## 📁 Project Setup

### Clone the Repository

Clone this repository into your desired workspace:

```bash
git clone https://github.com/emusman-lab/dual_robot_arm.git
cd dual_robot_arm
```

---

### Build and Source the Workspace

Build your workspace using colcon:

```bash
colcon build --base-paths src/
```

Then source it:

```bash
source install/setup.bash
```

> ✅ Replace the source path as needed, e.g., `/home/username/dual_robot_arm/install/setup.bash`

---

## 🚀 Running the Simulation

### Launch Isaac Sim

Launch Isaac Sim with the dual-arm control script:

```bash
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh \
  /home/usman/projects_ws/dual_robot_arm/isaac/scripts/isaac_moveit_dobot_dual.py
```

Make sure your Isaac Sim environment and ROS2 bridge are fully initialized.

---

### Launch MoveIt 2 and RViz2

Launch the MoveIt2 config and visualizer with:

```bash
ros2 launch dual_dobot_control dual_launch_config_dobot.py
```

> 🛠 This file should load both robot descriptions and bring up the joint state publishers, controllers, and MoveGroup interface.

---

### Launch MoveIt Task Constructor

Launch the dual-arm manipulation logic:

```bash
ros2 launch dual_dobot_control dual_pick_and_place_dobot.py
```

> 📦 This script uses MoveIt Task Constructor to execute predefined pick-and-place tasks using both Dobot Nova5 arms.

---

## 🔗 Links & References

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [Isaac Sim ROS 2 Bridge Docs](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_setup.html)
- [MoveIt Task Constructor](https://ros-planning.github.io/moveit_task_constructor/)
- [Dobot Nova5 Official Page](https://www.dobot.cc/products/dobot-nova.html)
- [Isaac Sim Python API Reference](https://docs.omniverse.nvidia.com/isaacsim/latest/reference/python/python_api_overview.html)

---
