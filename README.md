# Multi-agent-robot-swarm

# Bharatforge Inter-IIT Mid Prep 
# Centralized Intelligence for Dynamic Swarm Navigation

Welcome to the **Centralized Intelligence for Dynamic Swarm Navigation** project. This initiative focuses on designing and implementing a centralized control system for navigating dynamic swarms of autonomous robots. The project leverages advanced algorithms for real-time decision-making, robust communication frameworks for efficient data sharing, and dynamic path planning to ensure optimal swarm behavior in complex and uncertain environments. Key features include centralized coordination, adaptive navigation strategies, and real-time obstacle avoidance, enabling seamless operation in dynamic and constrained spaces.

We have provided the following files/folders in the code folder
- custom_planner_teb - module
- custom_world - module
- deepsort_ros -module
- m-explore module
- move_models module
- object_detection module
- rl_controls module
- ros_numpy module
- teb_local_planner - module
- turtlebot3_description - module
- turtlebot_ddpg -module
- ultralytics - module
- ultralytics_ros - module
- velodyne_simulator - module
- yolov8_ros - module
- requirements.txt
---

## Table of Contents

- [Video Demonstration](#video-demonstration)
- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Launching the UI](#launching-the-ui)
- [Resolving Errors](#resolving-errors)

---

## Video Demonstration

To experience the capabilities of the centralized swarm navigation system, including dynamic path planning, real-time obstacle avoidance, and seamless coordination, explore the video demonstration available in the **videos** folder.

---

## Overview

The Centralized Intelligence for Dynamic Swarm Navigation project addresses the challenges of designing a singular brain for robot swarms tasked with performing optimized path planning in highly dynamic environments. This solution emphasizes centralized control, seamless communication, and real-time adaptability, ensuring efficiency and scalability in complex navigation tasks. The key features and modules include:

**custom_planner_teb**: A custom implementation of the Timed Elastic Band (TEB) local planner, which is used for path planning and trajectory optimization, especially for mobile robots in dynamic environments.

**custom_world**: A custom world setup for the robot simulation environment, to simulate various scenarios for testing and training robots.

**deepsort_ros**: An integration of the Deep SORT (Simple Online and Realtime Tracking) algorithm with ROS. It is used for multi-object tracking in real-time, paired with object detection algorithms.

**m-explore**: A module for exploration or path planning, related to exploration in environments, used in autonomous robots for efficient map-building or mission completion.

**move_models**: A module containing scripts for movement of obstacles.

**object_detection**: A module dedicated to detecting objects in the environment, using computer vision techniques to identify and classify objects.

**rl_controls**: Module for reinforcement learning-based control systems, where reinforcement learning is used to optimize the robot's control policies for tasks such as navigation, object manipulation, or decision-making.

**ros_numpy**: A library to bridge between ROS messages and NumPy arrays. This is useful for performing numerical computations on sensor data in ROS.

**teb_local_planner** A ROS package for local path planning. It allows robots to plan and optimize paths in dynamic environments while avoiding obstacles and staying on course.

**turtlebot3_description**: A ROS package that provides the URDF (Unified Robot Description Format) model of the TurtleBot3 robot, including its sensors and actuators, for simulation and visualization.

**turtlebot_ddpg**: A module implementing Accelerated Deep Deterministic Policy Gradient (DDPG) algorithm for obstacle avoidance and exploration under dynamic scenarios.

**ultralytics**: A library for object detection model, used for real-time applications like autonomous navigation and surveillance.

**ultralytics_ros**: A ROS interface for integrating YOLO-based models (YOLOv8) with the ROS ecosystem for real-time object detection, providing a bridge between the object detection system and the robot’s ROS framework.

**velodyne_simulator**: A module for Velodyne LiDAR sensors, used in autonomous vehicles and robots for 3D mapping and obstacle detection.

**yolov8_ros**: A ROS integration for YOLOv8.

**requirements.txt**: File which specifies the dependencies required to run the project.

---

## System Requirements

Before you begin, ensure you have the following:

- **Operating System**: Ubuntu 20.04 recommended for a stable build environment.
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen 7 and above).
- **RAM**: 16 GB is recommended for handling complex simulations and concurrent processes.

---

# Installation
### Install ROS Noetic
- : Follow the ROS Noetic [Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu) (ros-noetic-desktop-full is recommended along with section 1.6).

### Build the catkin Workspace
Commands to run in Terminal:
```bash
   sudo apt-get python3-catkin-tools
   mkdir catkin_ws
   cd catkin_ws/
```
Extract the src.zip inside catkin_ws/

```bash
   cd catkin_ws/src/
   catkin_init_workspace
   cd ~/catkin_ws/
   catkin build
   source devel/setup.bash
   echo "source /devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
```
Resolve any ros dependencies if occur during the build.
### Setup the models correctly 
Extract the models from the models.zip inside the directory ~/.gazebo/models

### Setup Python3.6 Virtual environment (so that DDPG RL algorithm can run properly without dependency and version conflicts)
Ensure Python3.6 is also installed in your system.
Commands to run in Terminal inside the Catkin Workspace:
```bash
    python3.6 -m venv rl_env
    source rl_env/bin/activate
    pip install -r requirments.txt
    deactivate
```
NOTE : Only activate virtual environment when using Accelerated DDPG for Exploration and Obstacle Avoidance.
# Running the Simulation
## Launching individual nodes using Terminal: 
Command to launch the Gazebo world:
```bash
    roslaunch custom_world custom_world.launch num_bots:=4
```
Command to launch the gmapping and map merging node:
```bash
    custom_world map_merger.launch num_bots:=4
```
Command to start the movement of Obstacles run:
```bash
    
    python3 move_
```
### Exploration can be started using 2 methods:
- Using the Accelerated DDPG + Voronoi approach
or
- Using the Frontier Exploration Method

For the first approach
activate the virtual env (only when starting DDPG_test node) using the command:
```bash
    source rl_env/bin/activate
```
Now run the DDPG node:
```bash
python3 /catkin_ws/src/rl_controls/src/DDPG_launch.py
```
For the Frontier point approach

Command to start the teb planner node:
```bash
    roslaunch rl_controls teb.launch num_bots:=4
```
Command to start the Exploration node:
```bash
    roslaunch explore_lite explore.launch num_bots:=4
```
## Launching the UI
Commands to run in terminal:
```bash
    cd ~/catkin_ws
    chmod +x run.sh
    ./run.sh
```
Open the LocalHost Link provided in the terminal.
### For training the Accelerated DDPG algorithm on custom world,  
```bash
   cd ~/catkin_ws/src/turtlebot_ddpg/scripts/fd_replay/play_human_data
   python3 ddpg_network_turtlebot3_amcl_fd_replay_human.py
```
### To monitor the progess of Rewards and Q-values during our DDPG RL training, kindly check step_Q.mat and step_reward.mat inside:
```bash
    ~/catkin_ws/src/turtlebot_ddpg/scripts/fd_replay/play_human_data
```

# Resolving Errors

If you encounter the sudoers error:

    username is not in the sudoers file. This incident will be reported.  

Follow these steps to resolve it:

Switch to the root user by typing the following command in the terminal:

    su 

You will be prompted to enter the root password.

Open the sudoers file for editing by executing:

    visudo -f /etc/sudoers  

Scroll to the end of the file and add the following line:

    user_name ALL=(ALL) ALL  

Replace user_name with your actual username.

Save and exit the editor by pressing Ctrl+X, followed by Y, and then Enter.

Switch back to your user account with the command:

    su user_name  

Verify the fix by running a command with sudo (e.g., sudo apt update).

This modification grants the user sudo privileges, enabling administrative tasks without encountering the error.

