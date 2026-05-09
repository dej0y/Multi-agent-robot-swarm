#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from stable_baselines3 import PPO
import numpy as np

# Initialize publishers and PPO model
cmd_vel_pub = {
    "bot1": rospy.Publisher('/bot1/cmd_vel', Twist, queue_size=10),
    "bot2": rospy.Publisher('/bot2/cmd_vel', Twist, queue_size=10),
    "bot3": rospy.Publisher('/bot3/cmd_vel', Twist, queue_size=10)
}
model = PPO.load("ppo_exploration_model")  # Pre-trained or newly trained model

# Global variables to store map data and laser scans
global_map = None
laser_scans = {"bot1": None, "bot2": None, "bot3": None}

def global_map_callback(data):
    global global_map
    global_map = data

def laser_scan_callback(data, bot_name):
    global laser_scans
    laser_scans[bot_name] = data

def get_observation():
    # Process global map and laser scans as PPO inputs
    if global_map is None or any(scan is None for scan in laser_scans.values()):
        return None
    # Convert global_map and laser_scans to suitable PPO observation space format
    map_data = np.array(global_map.data).reshape((global_map.info.height, global_map.info.width))
    scan_data = [np.array(scan.ranges) for scan in laser_scans.values()]
    return np.concatenate([map_data.flatten(), *scan_data])

def main():
    rospy.init_node("ppo_exploration")

    # Subscribe to global map and laser scans
    rospy.Subscriber("/map_merge", OccupancyGrid, global_map_callback)
    rospy.Subscriber("/bot1/bot1/scan", LaserScan, laser_scan_callback, "bot1")
    rospy.Subscriber("/bot2/bot2/scan", LaserScan, laser_scan_callback, "bot2")
    rospy.Subscriber("/bot3/bot3/scan", LaserScan, laser_scan_callback, "bot3")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        observation = get_observation()
        if observation is not None:
            action, _ = model.predict(observation)
            publish_velocity_commands(action)
        rate.sleep()

def publish_velocity_commands(action):
    for i, bot_name in enumerate(["bot1", "bot2", "bot3"]):
        vel_msg = Twist()
        # Map actions to robot velocities (assuming action format from PPO)
        vel_msg.linear.x = action[i][0]
        vel_msg.angular.z = action[i][1]
        cmd_vel_pub[bot_name].publish(vel_msg)

if __name__ == "__main__":
    main()
