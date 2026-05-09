import rospy
import numpy as np
import gym
from gym import spaces
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from your_voronoi_library import VoronoiExplore  # Make sure to import your Voronoi library

class FrontierExploreEnv(gym.Env):
    def __init__(self, bot_name='bot1'):
        super(FrontierExploreEnv, self).__init__()

        # Set bot name (for multi-agent)
        self.bot_name = bot_name

        # Initialize ROS node
        rospy.init_node(f'{self.bot_name}_explore_env', anonymous=True)

        # Define action and observation space
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3 + 360, ), dtype=np.float32)

        # Subscribers and Publishers
        self.frontier_sub = rospy.Subscriber(f"/{self.bot_name}/explore/frontier", Pose, self.frontier_callback)
        self.laser_sub = rospy.Subscriber(f"/{self.bot_name}/scan", LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber(f"/{self.bot_name}/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher(f"/{self.bot_name}/cmd_vel", Twist, queue_size=1)

        # Initialize robot state and frontier
        self.frontier_points = []
        self.current_pose = None
        self.laser_data = None

        # Voronoi class for selecting the best goal
        self.voronoi = VoronoiExplore()

        # Initialize next target node
        self.next_target_node = {}

    def frontier_callback(self, msg):
        """Callback to update the frontier points."""
        # Assuming `msg` is a Pose array (list of frontier points)
        self.frontier_points = np.array([[pose.position.x, pose.position.y] for pose in msg.poses])

    def laser_callback(self, msg):
        """Callback to process laser data."""
        self.laser_data = np.array(msg.ranges)

    def odom_callback(self, msg):
        """Callback to update robot's current position."""
        self.current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def step(self, action):
        """Take a step in the environment based on the action."""
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(cmd)

        # Get updated observations and compute reward
        observation = self.get_observation()
        reward, done, info = self.compute_reward(observation)
        
        return observation, reward, done, info

    def get_observation(self):
        """Combine the laser data, current pose, and velocity into a single observation."""
        laser_features = self.laser_data if self.laser_data is not None else np.zeros(360)
        velocity_features = np.array([self.current_pose[0], self.current_pose[1]])  # Use position as proxy for velocity
        return np.concatenate([laser_features, velocity_features])

    def compute_reward(self, observation):
        """Compute the reward based on the current state."""
        reward = 0
        done = False
        
        # Check if the robot has reached a frontier point
        if self.reached_frontier():
            reward += 10  # Reward for reaching a frontier point
        if self.hit_obstacle():
            reward -= 5  # Penalty for hitting obstacles
        
        return reward, done, {}

    def reached_frontier(self):
        """Check if the robot has reached a frontier point."""
        if self.frontier_points:
            goal = self.frontier_points[0]  # Select the first frontier point (this can be optimized)
            distance = np.linalg.norm(self.current_pose - goal)
            if distance < 0.5:  # Threshold for reaching goal
                return True
        return False

    def hit_obstacle(self):
        """Check if the robot has hit an obstacle."""
        if np.any(np.array(self.laser_data) < 0.5):  # If any laser reading is less than 0.5m
            return True
        return False

    def reset(self):
        """Reset the environment to its initial state."""
        self.frontier_points = []
        self.current_pose = np.zeros(2)
        return self.get_observation()

    def update_goal(self):
        """Select the best goal using Voronoi and frontier points."""
        if not self.frontier_points:
            return

        # Use the frontier points as candidate goal points
        selected_goals = self.voronoi_select_goal(self.frontier_points)
        
        # Select the goal closest to the robot
        self.next_target_node = self.get_min_distance_goal(selected_goals)

    def voronoi_select_goal(self, frontier_points):
        """Use Voronoi to select the best goal from frontier points."""
        # You may want to apply Voronoi-based logic here, e.g., distance-based goal selection
        # Assuming you have a VoronoiExplore method in your Voronoi library
        return self.voronoi.select_goal(frontier_points, self.current_pose)

    def get_min_distance_goal(self, selected_goals):
        """Select the goal that is closest to the current position."""
        distances = np.linalg.norm(selected_goals - self.current_pose, axis=1)
        closest_goal_idx = np.argmin(distances)
        return selected_goals[closest_goal_idx]
