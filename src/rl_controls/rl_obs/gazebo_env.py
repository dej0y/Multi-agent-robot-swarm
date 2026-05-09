import gym
import rospy
import numpy as np
from gym import spaces
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import math

class GazeboEnv(gym.Env):
    def __init__(self):
        super(GazeboEnv, self).__init__()

        # Initialize ROS node
        rospy.init_node('ppo_training_node', anonymous=True)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/bot1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/bot1/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/map_merged', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/estimated_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/goal', PoseStamped, self.goal_callback)

        # Observation and Action Space
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        obs_size = 360 + 3  # Lidar data (360) + relative goal position (x, y) + robot's orientation (theta)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32)

        # Initialize variables
        self.lidar_data = None
        self.map_data = None
        self.current_pose = None
        self.goal_pose = None
        self.prev_distance_to_goal = None
        print('env initialized')

    def lidar_callback(self, data):
        # print('lidar callback')
        self.lidar_data = np.array(data.ranges)

    def map_callback(self, data):
        # print('map callback')
        self.map_data = data  # Process if needed

    def pose_callback(self, data):
        # print('pose callback')
        self.current_pose = data

    def goal_callback(self, data):
        # print('goal callback')
        self.goal_pose = data

    def step(self, action):
        # print('step ')
        # Send action to the robot
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(vel_cmd)

        # Wait for observations to update
        rospy.sleep(0.1)

        # Construct observation
        observation = self._get_observation()

        # Calculate reward
        reward = self._compute_reward()

        # Check if episode is done
        done = self._is_done()

        info = {}

        return observation, reward, done, info

    def reset(self):
        # print('reset')
        # Reset the simulation (you can call a Gazebo service to reset the world if needed)
        self._reset_simulation()

        # Wait for valid observations
        while self.lidar_data is None or self.current_pose is None or self.goal_pose is None:
            rospy.sleep(0.1)

        # Initialize distance to goal
        self.prev_distance_to_goal = self._distance_to_goal()

        return self._get_observation()

    def _get_observation(self):
        # print('get observation')
        # Process lidar data (normalizing)
        lidar_processed = np.clip(self.lidar_data, 0, 10) / 10.0  # Lidar max range assumed 10 meters

        # Calculate relative goal position
        rel_goal_pos = self._calculate_relative_goal_position()

        # Get robot orientation
        # _, _, theta = self._get_robot_orientation()
        theta = self._get_robot_orientation()

        # Combine into a single observation array
        observation = np.concatenate([lidar_processed, rel_goal_pos, [theta]])

        return observation

    def _compute_reward(self):
        # print('reward calc')
        # Distance to the goal
        distance_to_goal = self._distance_to_goal()
        progress_reward = self.prev_distance_to_goal - distance_to_goal  # Positive if moving towards the goal

        # Collision penalty
        min_lidar_distance = np.min(self.lidar_data)
        collision_penalty = -1.0 if min_lidar_distance < 0.2 else 0  # Collision threshold

        # Goal reached reward
        goal_reward = 10.0 if distance_to_goal < 0.3 else 0  # Goal proximity threshold

        # Total reward
        reward = progress_reward * 10 + collision_penalty + goal_reward

        # Update previous distance
        self.prev_distance_to_goal = distance_to_goal

        return reward

    def _is_done(self):
        # Check if the robot is close enough to the goal
        if self._distance_to_goal() < 0.3:
            return True
        # Check if a collision occurred
        if np.min(self.lidar_data) < 0.2:
            return True
        return False

    def _reset_simulation(self):
        # Implement any necessary code to reset the Gazebo simulation, like resetting robot position
        pass

    def _calculate_relative_goal_position(self):
        # Calculate relative position of the goal from the robot's current position
        rel_x = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        rel_y = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        return np.array([rel_x, rel_y])

    def _distance_to_goal(self):
        # Euclidean distance to goal
        if self.current_pose and self.goal_pose:
            return math.sqrt((self.goal_pose.pose.position.x - self.current_pose.pose.position.x) ** 2 +
                             (self.goal_pose.pose.position.y - self.current_pose.pose.position.y) ** 2)
        return float('inf')

    def _get_robot_orientation(self):
        # Get the robot's yaw orientation from its quaternion
        orientation_q = self.current_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def render(self, mode='human'):
        pass

    def close(self):
        pass
