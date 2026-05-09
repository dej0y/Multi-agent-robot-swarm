import gym
from gym import spaces
import rospy
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Twist
import os
import random
import subprocess



class MultiRobotEnv(gym.Env):
    def __init__(self):
        super(MultiRobotEnv, self).__init__()

        # Define map and laser scan sizes
        # self.map_size = 40000  # 200x200 cells for a 10m x 10m map
        self.map_size = 4000000 
        self.laser_scan_size = 360

        # Define observation and action spaces
        self.observation_space = spaces.Box(
            low=-1, high=1, shape=(self.map_size + 3 * self.laser_scan_size,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1, high=1, shape=(3, 2), dtype=np.float32  # 3 robots with (linear, angular) velocities each
        )

        # Initialize global map and laser scans for each robot
        self.global_map = None
        self.laser_scans = {"bot1": None, "bot2": None, "bot3": None}

        # Initialize service proxy to reset robots' positions
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # ROS initialization, subscribers, etc.
        rospy.init_node('multi_robot_env', anonymous=True)
        rospy.Subscriber("/map_merged", OccupancyGrid, self.global_map_callback)
        rospy.Subscriber("/bot1/scan", LaserScan, self.laser_scan_callback, "bot1")
        rospy.Subscriber("/bot2/scan", LaserScan, self.laser_scan_callback, "bot2")
        rospy.Subscriber("/bot3/scan", LaserScan, self.laser_scan_callback, "bot3")
        self.cmd_vels = {}
        self.pre_frac_exp =0
        self.count =0

    # def global_map_callback(self, data):
    #     # Trim the map data to a 10m x 10m area (200x200 cells)
    #     grid_data = np.array(data.data).reshape(data.info.height, data.info.width)
    #     trimmed_map = grid_data[:200, :200]  # Adjust based on the map’s orientation and data layout
    #     self.global_map = trimmed_map.flatten()[:self.map_size]  # Flatten and limit to map_size

    # def global_map_callback(self, data):
    #     # Trim the map data to a 10m x 10m area (200x200 cells)
    #     grid_data = np.array(data.data).reshape(data.info.height, data.info.width)
    #     # trimmed_map = grid_data[:200, :200]  # Adjust based on the map’s orientation and data layout
    #     trimmed_map = grid_data[:4000:2, :4000:2]
    #     self.global_map = trimmed_map.flatten()[:self.map_size]  # Flatten and limit to map_size
    


    # normalised map instead of 0 to 100 probabilistic 
    def global_map_callback(self, data):
        # Convert the map data into a 2D NumPy array
        grid_data = np.array(data.data).reshape(data.info.height, data.info.width)

        # Trim the map data to the desired area (e.g., 10m x 10m, assuming 200x200 cells)
        trimmed_map = grid_data[:4000:2, :4000:2]  # Adjust based on the map resolution and area

        # Create a mask for unknown values (-1)
        unknown_mask = trimmed_map == -1

        # Normalize valid values [0, 100] to [0, 1]
        normalized_map = trimmed_map.astype(np.float32) / 100.0

        # Restore -1 values for unknowns
        normalized_map[unknown_mask] = -1

        # Flatten and limit the map data to the required size
        self.global_map = normalized_map.flatten()[:self.map_size]




    # def laser_scan_callback(self, data, bot_name):
    #     # Store the latest laser scan for each bot
    #     self.laser_scans[bot_name] = np.array(data.ranges[:self.laser_scan_size])



    # normalised lase data based no max ranage to exclude the inf data and replace it by the max range
    def laser_scan_callback(self, data, bot_name):
        # Convert the laser scan ranges to a NumPy array
        ranges = np.array(data.ranges[:self.laser_scan_size], dtype=np.float32)

        # Replace inf values with the maximum range of the LiDAR (or a predefined cap)
        max_range = data.range_max  # The maximum range of the LiDAR from the message
        ranges[np.isinf(ranges)] = max_range

        # Optional: Normalize the laser scan data to a range [0, 1]
        ranges = ranges / max_range

        # Store the processed laser scan data
        self.laser_scans[bot_name] = ranges


    def reset(self):
        # Reset the map merging
        self.reset_map_merging()

        # # Reset robots' positions to initial state in Gazebo
        self.reset_robots_positions()

        # Wait for map and laser scans to update
        rospy.sleep(1)

        # Return the initial observation
        return self._get_observation()

    # def reset_map_merging(self):
    #     # Logic to reset map merging can be implemented here, 
    #     # depending on the map_merge node’s reset capability
    #     pass

    # logic 1

    # def reset_map_merging(self):
    #     # Restart the map-merging node
    #     rospy.loginfo("Restarting multibot_map_merging node.")
        
    #     # Stop the map-merging node
    #     os.system("rosnode kill /map_merge/map_merge")
    #     os.system("rosnode kill /bot1/slam_gmapping")
    #     os.system("rosnode kill /bot2/slam_gmapping")
    #     os.system("rosnode kill /bot3/slam_gmapping")
        
    #     # Restart the map-merging node (adjust as necessary for your ROS setup)
    #     os.system("roslaunch rl_controls multi_bot_all.launch spawn_bots_num:=3 &")

    #     rospy.sleep(2)  # Give time for the node to restart
    #     rospy.loginfo("Merged map reset by restarting the node.")


    # logic 1 as a background process
    def reset_map_merging(self):
        # Restart the map-merging node
        rospy.loginfo("Restarting multibot_map_merging node.")

        # Stop the map-merging node
        subprocess.call(["rosnode", "kill", "/map_merge/map_merge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["rosnode", "kill", "/bot1/slam_gmapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["rosnode", "kill", "/bot2/slam_gmapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["rosnode", "kill", "/bot3/slam_gmapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Restart the map-merging node as a background process (adjust as necessary for your ROS setup)
        subprocess.Popen(
            ["roslaunch", "rl_controls", "multi_bot_all.launch", "spawn_bots_num:=3"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        rospy.sleep(2)  # Give time for the node to restart
        rospy.loginfo("Merged map reset by restarting the node.")



    # logic 2

    # def reset_map_merging(self):
    #     # Attempt to call the reset service if it exists
    #     try:
    #         rospy.wait_for_service('/multibot_map_merging/reset', timeout=5)
    #         reset_service = rospy.ServiceProxy('/multibot_map_merging/reset', Empty)
    #         reset_service()
    #         rospy.loginfo("Merged map reset successfully.")
    #     except (rospy.ServiceException, rospy.ROSException) as e:
    #         rospy.logwarn(f"Failed to reset merged map: {e}")

            # Function to generate a random position
    def generate_random_position(self):
        x = random.uniform(-5, 5)  # Random x between -5 and 5
        y = random.uniform(-5, 5)  # Random y between -5 and 5
        z = 0.1  # z is fixed at 0.1
        return (x, y, z)


    def reset_robots_positions(self):
        # initial_positions = [
        #     {"name": "bot1", "position": (0, 0, 0)},
        #     {"name": "bot2", "position": (2, 0, 0)},
        #     {"name": "bot3", "position": (4, 0, 0)}
        # ]

        # Initial positions with random values
        initial_positions = [
            {"name": "bot1", "position": self.generate_random_position()},
            {"name": "bot2", "position": self.generate_random_position()},
            {"name": "bot3", "position": self.generate_random_position()}
        ]
        
        for pos in initial_positions:
            req = SetModelStateRequest()
            req.model_state = ModelState()
            req.model_state.model_name = pos["name"]
            req.model_state.pose.position.x = pos["position"][0]
            req.model_state.pose.position.y = pos["position"][1]
            req.model_state.pose.position.z = 0
            req.model_state.pose.orientation.z = pos["position"][2]
            req.model_state.reference_frame = "world"
            self.set_model_state(req)

    # def _get_observation(self):
    #     # Ensure the map and laser scans are available
    #     if self.global_map is None or any(scan is None for scan in self.laser_scans.values()):
    #         return np.zeros(self.observation_space.shape, dtype=np.float32)

    #     # Concatenate map and laser scans
    #     observation = np.concatenate([
    #         self.global_map,
    #         self.laser_scans["bot1"],
    #         self.laser_scans["bot2"],
    #         self.laser_scans["bot3"]
    #     ])

    #     return observation


    def _get_observation(self):
        # Ensure the map and laser scans are available
        if self.global_map is None or any(scan is None for scan in self.laser_scans.values()):
            return np.zeros(self.observation_space.shape, dtype=np.float32)

        self.global_map = np.clip(self.global_map, -1, 1)
        # Concatenate map and laser scans
        observation = np.concatenate([
            self.global_map.astype(np.float32),
            self.laser_scans["bot1"].astype(np.float32),
            self.laser_scans["bot2"].astype(np.float32),
            self.laser_scans["bot3"].astype(np.float32),
        ])
        # print(f"Observation stats: min={np.min(observation)}, max={np.max(observation)}, mean={np.mean(observation)}")
        # print("Observations", observation)
        # print("map data", self.global_map.astype(np.float32))



        return observation


    def step(self, action):
        # Publish commands to each bot based on action
        self.publish_actions(action)

        # Calculate rewards based on exploration
        reward = self._calculate_reward()

        # Define termination conditions
        done = self._check_done()

        # Return observation, reward, done, and info (empty for now)
        return self._get_observation(), reward, done, {}

    def publish_actions(self, action):
        # Send the velocity commands to each robot based on PPO action
        for i, bot_name in enumerate(["bot1", "bot2", "bot3"]):
            vel_msg = Twist()
            vel_msg.linear.x = action[i][0]
            vel_msg.angular.z = action[i][1]
            rospy.Publisher(f"/{bot_name}/cmd_vel", Twist, queue_size=10).publish(vel_msg)
            self.cmd_vels[bot_name] = vel_msg

    # def _calculate_reward(self):
    #     # Reward computation based on map exploration
    #     if self.global_map is None:
    #         return 0

    #     map_data = np.array(self.global_map.data)

    #     # Calculate fractions for rewards
    #     explored_fraction = np.count_nonzero(map_data >= 0) / map_data.size
    #     unexplored_fraction = np.count_nonzero(map_data == -1) / map_data.size

    #     # Rewards based on map exploration regions
    #     reward = 0
    #     for cell in map_data:
    #         if cell == 0:  # Empty region
    #             reward += 30
    #         elif cell == -1:  # Unexplored region
    #             reward -= 50
    #         elif cell > 0:  # Occupied region
    #             reward += 50

    #     # Multiply reward by fraction of total explored region
    #     reward *= explored_fraction
    #     print('reward : ', reward)

    #     return reward

    def _calculate_reward(self):
        # Initialize reward components
        reward = 0
        R_explore = 0
        R_collision = 0
        R_idle = 0  # New component for idle penalty

        # Check if global_map is available
        if self.global_map is None:
            return 0

        # Convert the global map data to a numpy array
        map_data = np.array(self.global_map.data)

        # Total number of cells in the map
        total_cells = map_data.size

        # Calculate the number of explored cells (cells with value >= 0)
        current_explored_cells = np.count_nonzero(map_data >= 0)

        # Calculate the increase in explored area (ΔA)
        if hasattr(self, 'previous_explored_cells'):
            delta_explored_cells = current_explored_cells - self.previous_explored_cells
        else:
            # If no previous data, assume no increase for the first step
            delta_explored_cells = 0

        # Update the previous explored cells for the next time step
        self.previous_explored_cells = current_explored_cells

        # Exploration Reward (R_explore)
        alpha = 10  # Weight for exploration reward (adjust as needed)
        R_explore = alpha * delta_explored_cells

        # Collision Penalty (R_collision)
        beta = 1  # Weight for collision penalty (adjust as needed)
        C = 0      # Collision indicator: 1 if collision occurred, else 0

        # Check for collisions for each robot
        for robot_id in self.laser_scans:
            if self.check_collision(robot_id):
                C += 1  # Increment collision count
        if C <5:
            C=0

        # Total collision penalty
        R_collision = -beta * C

        # Idle Penalty (R_idle)
        epsilon = 10  # Weight for idle penalty (adjust as needed)
        I = 0         # Idle indicator: 1 if robot is idle, else 0

        # Check for idleness for each robot
        for robot_id in self.cmd_vels:
            if self.check_idle(robot_id):
                I += 1  # Increment idle count

        # Total idle penalty
        R_idle = -epsilon * I

        # Total reward
        reward = R_explore + R_collision + R_idle

        # Debug statements to trace computation values
        print(f"Total Reward: {reward}")
        print(f"  Exploration Reward (R_explore): {R_explore}")
        print(f"  Collision Penalty (R_collision): {R_collision}")
        print(f"  Idle Penalty (R_idle): {R_idle}")

        return reward

    def check_idle(self, robot_id):
        # Access the robot's cmd_vel data
        cmd_vel = self.cmd_vels[robot_id]  # Assuming cmd_vels is a dict storing the latest cmd_vel for each robot

        # Define thresholds for considering the robot as moving
        min_linear_velocity = 0.05  # m/s
        min_angular_velocity = 0.05  # rad/s

        # Check if velocities are below thresholds
        if (abs(cmd_vel.linear.x) < min_linear_velocity and
            abs(cmd_vel.linear.y) < min_linear_velocity and
            abs(cmd_vel.angular.z) < min_angular_velocity):
            
            # Initialize idle_times if it doesn't exist
            if not hasattr(self, 'idle_times'):
                self.idle_times = {}
            
            # Update idle time for this robot
            if robot_id not in self.idle_times:
                self.idle_times[robot_id] = 1
            else:
                self.idle_times[robot_id] += 1
            
            # Define the duration threshold (e.g., 5 time steps)
            idle_duration_threshold = 5  # Adjust as needed
            
            if self.idle_times[robot_id] >= idle_duration_threshold:
                return True  # Robot is idle
        else:
            # Reset idle time if the robot is moving
            if hasattr(self, 'idle_times') and robot_id in self.idle_times:
                self.idle_times[robot_id] = 0
        
        return False

    def check_collision(self, robot_id):
        # Access the robot's laser scan data
        laser_scan = self.laser_scans[robot_id]
        
        # Define a minimum safe distance threshold
        min_safe_distance = 0.2  # Adjust based on robot specifications
        
        # Check if any laser scan readings are below the safe distance
        if np.any(laser_scan < min_safe_distance):
            return True
        else:
            return False




    def _check_done(self):
        # Define a condition for completing exploration
        if self.global_map is None:
            return False

        map_data = np.array(self.global_map.data)
        explored_fraction = np.count_nonzero(map_data >= 0) / map_data.size
        done_threshold = 0.95  # Mark done when 95% of the map is explored
        print('explored fraction : ', explored_fraction)
        if abs(self.pre_frac_exp - explored_fraction) <0.001:
            self.count +=1
        else:
            self.pre_frac_exp = explored_fraction
            self.count =0
        if self.count >1000:
            return True

        return explored_fraction >= done_threshold
