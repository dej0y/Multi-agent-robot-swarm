import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
import gym
import numpy as np
from gym import spaces
import random
import os
import subprocess
import time
import signal

class MultiAgentEnv(gym.Env):
    def __init__(self, num_bots):
        super(MultiAgentEnv, self).__init__()
        
        # ROS setup
        rospy.init_node("multi_agent_rl_training", anonymous=True)
        self.num_bots = num_bots

        # Data storage
        self.odom_data = {i: None for i in range(num_bots)}
        self.scan_data = {i: None for i in range(num_bots)}
        self.map_data = None
        self.map_change = 1
        self.map_update = 0

        # Initialize service proxy to reset robots' positions
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.odom_subs = [rospy.Subscriber(f"/bot{i}/odom", Odometry, self._odom_callback, callback_args=i) for i in range(num_bots)]
        self.scan_subs = [rospy.Subscriber(f"/bot{i}/scan", LaserScan, self._scan_callback, callback_args=i) for i in range(num_bots)]
        self.map_sub = rospy.Subscriber("/map_merged", OccupancyGrid, self._map_callback)
        self.cmd_vel_pubs = [rospy.Publisher(f"/bot{i}/cmd_vel", Twist, queue_size=10) for i in range(num_bots)]
        
        # Observation space
        obs_dim = 3 + 360 + 3 * 2 + 10 * 2  # Pose(3), Lidar(360), Nearest Bots(5x2), Boundary Points(100x2)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        #self.observation_space = spaces.Dict({
         #   f"bot{i}": spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
          #  for i in range(self.num_bots)
        #})
        self.observation_space = spaces.Dict({
            f"bot{i}": spaces.Box(low=-100.0, high=100.0, shape=(obs_dim,), dtype=np.float32)
            for i in range(self.num_bots)
        })
        
        # Action space
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(num_bots,2), dtype=np.float32)  # Linear & Angular Velocity
        #self.action_space = spaces.Dict({
        #    f"bot{i}": spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        #    for i in range(self.num_bots)
        #})

        self.global_explored_fraction = []
        
    
    def _odom_callback(self, msg, bot_id):
        self.odom_data[bot_id] = msg

    def _scan_callback(self, msg, bot_id):
        self.scan_data[bot_id] = msg

    # def _map_callback(self, msg):
    #     self.map_data = msg
    def _map_callback(self, msg):
        self.map_data = msg
        # print("map_data callback")
        # print('map data : ', msg)
        self.map_update = 1
        self.global_map = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))



    def _get_observation(self, bot_id):
        # Get bot pose and yaw angle
        if self.odom_data[bot_id] is None:
            return np.zeros(self.observation_space.shape)
        pose = self.odom_data[bot_id].pose.pose
        yaw = np.arctan2(2 * (pose.orientation.w * pose.orientation.z),
                         1 - 2 * (pose.orientation.z ** 2))
        bot_pose = [pose.position.x, pose.position.y, yaw]
        
        # Lidar scan data
        if self.scan_data[bot_id] is None:
            lidar_data = np.zeros(360)
        else:
            lidar_data = np.array(self.scan_data[bot_id].ranges)
            lidar_data *= 0.001
            lidar_data = np.minimum(lidar_data, 3.0)
        
        # Nearest bots
        nearest_bots = []
        for i in range(self.num_bots):
            if i != bot_id and self.odom_data[i]:
                other_pose = self.odom_data[i].pose.pose.position
                nearest_bots.append([other_pose.x - pose.position.x, other_pose.y - pose.position.y])
        while len(nearest_bots) < 3:
            nearest_bots.append([0, 0])  # Allocator for missing bots

        # Nearest boundary points
        boundary_points = self._get_nearest_boundary_points(bot_pose)
        while len(boundary_points) < 10:
            boundary_points.append([0, 0])  # Allocator for missing points
        
        observation = np.concatenate([bot_pose, lidar_data, np.array(nearest_bots).flatten(), np.array(boundary_points).flatten()])
        # print('obs' , observation)
        return observation

    # def _get_nearest_boundary_points(self, bot_pose):
    #     # Extract boundary points from the map
    #     if self.map_data is None:
    #         return []
    #     boundary_points = []
    #     resolution = self.map_data.info.resolution
    #     origin_x = self.map_data.info.origin.position.x
    #     origin_y = self.map_data.info.origin.position.y
    #     width = self.map_data.info.width
    #     height = self.map_data.info.height
    #     map_data = np.array(self.map_data.data).reshape((height, width))
        
    #     for x in range(width):
    #         for y in range(height):
    #             if map_data[y, x] == -1:  # Unexplored region
    #                 if any(map_data[ny, nx] == 0 for nx, ny in self._get_neighbors(x, y, width, height)):
    #                     world_x = origin_x + x * resolution
    #                     world_y = origin_y + y * resolution
    #                     boundary_points.append([world_x - bot_pose[0], world_y - bot_pose[1]])
    #                     if len(boundary_points) == 100:
    #                         return boundary_points
    #     return boundary_points

    def _get_nearest_boundary_points(self, bot_pose):
        # Extract boundary points from the map
        t = time.time()
        if self.map_data is None:
            # print("inside the get nearest points but no map data")
            return []
        boundary_points = []
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_data = np.array(self.map_data.data).reshape((height, width))
        bot_x, bot_y = bot_pose[0], bot_pose[1]
        for x in range(width):
            for y in range(height):
                if map_data[y, x] == -1:  # Unexplored region
                    if any(map_data[ny, nx] == 0 for nx, ny in self._get_neighbors(x, y, width, height)):
                        world_x = origin_x + x * resolution
                        world_y = origin_y + y * resolution
                        boundary_points.append([world_x - bot_x, world_y - bot_y])
                        if len(boundary_points) == 10:
                            # print("boundary_points", boundary_points)
                            print("the time for boundary pts calc :", time.time() - t)
                            return boundary_points
        # print("boundary_points", boundary_points)
        print("the time for boundary pts calc :", time.time() - t)
        return boundary_points
    
    def _get_neighbors(self, x, y, width, height):
        neighbors = [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < width and 0 <= ny < height]

    # def _compute_reward(self, bot_id, action):
    #     # Check explored points from boundary
    #     boundary_points = self._get_nearest_boundary_points(self.odom_data[bot_id].pose.pose.position)
    #     explored_points = sum(1 for point in boundary_points if self._is_point_explored(point))
        
    #     # Penalize for obstacles
    #     if self.scan_data[bot_id] is None:
    #         obstacle_penalty = 0
    #     else:
    #         obstacle_penalty = sum(1 for dist in self.scan_data[bot_id].ranges if dist < 0.2)
        
    #     reward = explored_points - obstacle_penalty
    #     return reward

    def _compute_reward(self, bot_id, action):
        t = time.time() 
        if self.odom_data[bot_id] is None:
            rospy.logwarn(f"Odom data for bot {bot_id} is None.")
            return 0  # Default reward
        rospy.logwarn(f"bot with {bot_id}")
        pose = self.odom_data[bot_id].pose.pose
        bot_pose = [pose.position.x, pose.position.y]
        
        # Compute boundary points
        boundary_points = self._get_nearest_boundary_points(bot_pose)
        # print("crossed boundary point detection")
        explored_points = sum(1 for point in boundary_points if self._is_point_explored(point))
        # print("crossed explored points detection")
        
        #Penalize for obstacles
        if self.scan_data[bot_id] is None:
            obstacle_penalty = 0
        else:
            obstacle_penalty = sum(1 for dist in self.scan_data[bot_id].ranges if dist < 0.2)
        if self.map_change:
            reward = explored_points - obstacle_penalty 
        else:
            reward = explored_points - obstacle_penalty - 50 
        print(reward)
        print("after rewards calc t : ", time.time() - t)
        return reward




    def _is_point_explored(self, point):
        t = time.time()
        if self.map_data is None:
            # print("map data is none in is point explored")
            return False
        # print("inside is point explored")
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        x, y = int((point[0] - origin_x) / resolution), int((point[1] - origin_y) / resolution)
        map_data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        # print("outside is point explored")

        print("the time for is point explored check is :", time.time() - t)
        return 0 <= x < self.map_data.info.width and 0 <= y < self.map_data.info.height and map_data[y, x] == 0

    # def step(self, actions):
    #     print('actions' , actions)
    #     for i, action in enumerate(actions):
    #         twist = Twist()
    #         print(f'action bot{i}' , action)
    #         twist.linear.x = action[0]
    #         twist.angular.z = action[1]
    #         self.cmd_vel_pubs[i].publish(twist)
        
    #     rospy.sleep(0.1)  # Allow actions to take effect
        
    #     # observations = [self._get_observation(i) for i in range(self.num_bots)]
    #     # rewards = [self._compute_reward(i, actions[i]) for i in range(self.num_bots)]
        
    #     # Collect observations and rewards for each bot
    #     observations = {f"bot{i}": self._get_observation(i) for i in range(self.num_bots)}
    #     rewards = {f"bot{i}": self._compute_reward(i, actions[i]) for i in range(self.num_bots)}

    #     # done = False  # Adjust this condition based on the task
    #     done = self._check_done()
    #     info = {}
    #     # print('observation : ', observations)
    #     print('rewards : ', rewards)
        
    #     return observations, rewards, done, info

    def step(self, actions):
        t = time.time()
        print('actions', actions)
        for i, action in enumerate(actions):
            twist = Twist()
            # print(f'action bot{i}', action)
            twist.linear.x = action[0]
            twist.angular.z = action[1]
            self.cmd_vel_pubs[i].publish(twist)
        
        rospy.sleep(0.1)  # Allow actions to take effect

        # observations = [self._get_observation(i) for i in range(self.num_bots)]
        # rewards = [self._compute_reward(i, actions[i]) for i in range(self.num_bots)]
        
        # Collect observations and rewards for each bot
        observations = {f"bot{i}": self._get_observation(i) for i in range(self.num_bots)}
        rewards_dict = {f"bot{i}": self._compute_reward(i, actions[i]) for i in range(self.num_bots)}
        print("step done")
        # Aggregate rewards into a single scalar
        reward = sum(rewards_dict.values())  # Sum rewards from all bots
        done = self._check_done()
        info = {}
        # info = {"rewards_dict": rewards_dict}
        print('rewards_dict : ', rewards_dict)
        print("for one step time :", time.time() - t)

        return observations, reward, done, info




    def reset(self):
        # Reset the environment and return initial observations
        self.reset_robots_positions()
        self.reset_map_merging()
        rospy.loginfo("Environment reset")
        # return [self._get_observation(i) for i in range(self.num_bots)]
        # Return a dictionary with observations for each bot
        return {f"bot{i}": self._get_observation(i) for i in range(self.num_bots)}

    # def _check_done(self):
    #     # Define a condition for completing exploration
    #     if self.global_map is None:
    #         return False

    #     map_data = np.array(self.map_data.data)
    #     explored_fraction = np.count_nonzero(map_data >= 0) / map_data.size
    #     done_threshold = 0.95  # Mark done when 95% of the map is explored

    #     return explored_fraction >= done_threshold
    
    def _check_done(self):
        t = time.time()
        # print("inside check done")
        # Check if the map data is available
        if self.map_data is None:
            # print("map_data is none")
            return False

        # Process the map to calculate exploration progress
        map_data = np.array(self.map_data.data)
        explored_fraction = np.count_nonzero(map_data >= 0) / map_data.size
        done_threshold = 0.0008  # Mark done when 95% of the map is explored
        # print("outside check done")
        print("explored :", explored_fraction)
        if self.map_update:
            if len(self.global_explored_fraction) <= 10:
                self.global_explored_fraction.append(explored_fraction)
            else:
                self.global_explored_fraction.pop(0)
                self.global_explored_fraction.append(explored_fraction)
            self.map_update = 0
        
        if abs(sum(self.global_explored_fraction)/len(self.global_explored_fraction) - explored_fraction) < 0.01:
            self.map_change = 1  # this should be 0 for termaniting and resetting the env 
        else:
            self.map_change = 1

        print("for check_done the time is :", time.time() - t)
        return explored_fraction >= done_threshold or not(self.map_change)


    def reset_robots_positions(self):
        # initial_positions = [
        #     {"name": "bot1", "position": (0, 0, 0)},
        #     {"name": "bot2", "position": (2, 0, 0)},
        #     {"name": "bot3", "position": (4, 0, 0)}
        # ]

        initial_positions = []
        for i in range(1, self.num_bots + 1):
            x = random.uniform(-3, 3)
            y = random.uniform(-3, 3)
            z = 0  # Assuming 3D positions, z is included
            initial_positions.append({"name": f"bot{i}", "position": (x, y, z)})
        
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
    
    # def reset_map_merging(self):
    #     # Restart the map-merging node
    #     rospy.loginfo("Restarting multibot_map_merging node.")
        
    #     # Stop the map-merging node
    #     os.system("rosnode kill /map_merge/map_merge > /dev/null 2>&1")
    #     os.system("rosnode kill /bot1/slam_gmapping > /dev/null 2>&1")
    #     os.system("rosnode kill /bot2/slam_gmapping > /dev/null 2>&1")
    #     os.system("rosnode kill /bot3/slam_gmapping > /dev/null 2>&1")
        
    #     # Restart the map-merging node (adjust as necessary for your ROS setup)
    #     os.system("roslaunch rl_controls multi_bot_all.launch spawn_bots_num:=3 > /dev/null 2>&1 &")

    #     rospy.sleep(2)  # Give time for the node to restart
    #     rospy.loginfo("Merged map reset by restarting the node.")

    # def reset_map_merging(self):
    #     # Restart the map-merging node
    #     rospy.loginfo("Restarting multibot_map_merging node.")

    #     # Stop the map-merging node
    #     subprocess.call(["rosnode", "kill", "/map_merge/map_merge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #     subprocess.call(["rosnode", "kill", "/bot1/slam_gmapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #     subprocess.call(["rosnode", "kill", "/bot2/slam_gmapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #     subprocess.call(["rosnode", "kill", "/bot3/slam_gmapping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #     time.sleep(1)
    #     subprocess.call(["rosnode", "cleanup"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #     time.sleep(1)

    #     # Restart the map-merging node as a background process (adjust as necessary for your ROS setup)
    #     subprocess.Popen(
    #         ["roslaunch", "rl_controls", "multi_bot_all.launch", "spawn_bots_num:=3"],
    #         stdout=subprocess.DEVNULL,
    #         stderr=subprocess.DEVNULL
    #     )

    #     rospy.sleep(2)  # Give time for the node to restart
    #     rospy.loginfo("Merged map reset by restarting the node.")

    # def reset_map_merging(self):
    #     rospy.loginfo("Restarting multibot_map_merging node.")
        
    #     # Kill specific ROS nodes
    #     nodes_to_kill = [
    #         "/map_merge/map_merge",
    #         "/bot1/slam_gmapping",
    #         "/bot2/slam_gmapping",
    #         "/bot0/slam_gmapping"
    #     ]
    #     for node in nodes_to_kill:
    #         subprocess.call(["rosnode", "kill", node], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #         time.sleep(1)
    #     time.sleep(1)

    #     # Clean up orphaned nodes
    #     subprocess.call(["rosnode", "cleanup"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    #     # Kill the `roslaunch` process by name
    #     rospy.loginfo("Killing any existing `roslaunch` processes.")
    #     subprocess.call(["pkill", "-f", "roslaunch rl_controls multi_bot_all.launch"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    #     time.sleep(1)  # Allow time for processes to terminate cleanly

    #     # Start the map-merging node as a background process
    #     rospy.loginfo("Starting new map merging node.")
    #     map_merging_process = subprocess.Popen(
    #         ["roslaunch", "rl_controls", "multi_bot_all.launch", "spawn_bots_num:=3"],
    #         stdout=subprocess.DEVNULL,
    #         stderr=subprocess.DEVNULL
    #     )

    #     # Give some time for the node to restart
    #     rospy.sleep(2)

    #     # Optionally, forcefully terminate the process if it becomes unresponsive (failsafe)
    #     try:
    #         rospy.loginfo("Verifying map merging process health.")
    #         if map_merging_process.poll() is None:  # Process is still running
    #             rospy.loginfo("Map merging process running successfully.")
    #         else:
    #             rospy.logwarn("Map merging process terminated unexpectedly. Attempting force kill.")
    #             os.kill(map_merging_process.pid, signal.SIGTERM)
    #     except Exception as e:
    #         rospy.logerr(f"Error while managing map merging process: {e}")

    #     rospy.loginfo("Merged map reset by restarting the node.")


    def reset_map_merging(self):
        # Restart the map-merging node
        rospy.loginfo("Restarting multibot_map_merging node.")
        
        # Stop the map-merging node
        os.system("rosnode kill /map_merge/map_merge > /dev/null 2>&1")
        os.system("rosnode kill /bot1/slam_gmapping > /dev/null 2>&1")
        os.system("rosnode kill /bot2/slam_gmapping > /dev/null 2>&1")
        os.system("rosnode kill /bot3/slam_gmapping > /dev/null 2>&1")
        
        # Restart the map-merging node (adjust as necessary for your ROS setup)
        os.system("roslaunch rl_controls multi_bot_all.launch spawn_bots_num:=3 > /dev/null 2>&1 &")
        self.map_update = 0
        while not(self.map_update):
            print("waiting for merged map to pub once")
            time.sleep(1)
        rospy.sleep(2)  # Give time for the node to restart



# Example usage
# if __name__ == "__main__":
#     num_bots = 3  # Set number of bots
#     env = MultiAgentEnv(num_bots)
#     # Train RL model using this environment
