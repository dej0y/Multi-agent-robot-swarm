<h1 align="center">Multi-agent-robot-swarm</h1>

<h2 align="center">Centralized Intelligence for Dynamic Swarm Navigation</h2>

This project aims on designing a centralized control system for navigating dynamic swarms of autonomous robots. The project explores multi-agent robot swarm coordination using real-time communication, path planning, and navigation strategies. It demonstrates how multiple agents can collaboratively operate in dynamic environments with features like centralized control, adaptive navigation, and obstacle avoidance.

## Contents

- [Video Demonstration](#video-demonstration)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Launching the UI](#launching-the-ui)

---

## Video Demonstration

### [Video 1: Dynamic Swarm Navigation Overview](https://youtu.be/78zE4rJ4cDY)

### [Video 2: Obstacle Avoidance in Action](https://youtu.be/fuIKSx9zPPc)

### [Video 3: Exploration and Path Planning with DDPG](https://youtu.be/MFlaDMs3FBI)

### [Video 4: DDPG Training](https://youtu.be/fHyaJAUsxMM)

### [Video 5: Frontier exploration-based navigation of a swarm of robots in dynamic environment](https://youtu.be/MU8Pb9cezhg)

### [Video 6: Frontier exploration-based navigation of a larger swarm of robots in a larger dynamic environment](https://youtu.be/mjY70b-c7WM)

### [Video 7: Chatbot Interface with LLM](https://youtu.be/b1DNU8ecjX0)

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

