#!/usr/bin/env python3
import rospy
import os
import json
import yaml
from threading import Lock
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer

script_dir = os.path.dirname(os.path.abspath(__file__))

class RobotHandler:
    def __init__(self):
        rospy.init_node('robot_handler', anonymous=True)

        with open(os.path.join(script_dir, "../../../ui/database/config.yaml"), 'r') as file:
            self.config = yaml.safe_load(file)

        self.bot_topic = [f"bot{i+1}/odom" for i in range(self.config['num_bots'])]
        self.refresh_rate = rospy.get_param("~refresh_rate")

        self.robots = []
        self.lock = Lock()

        self.subscriber = [Subscriber(self.bot_topic[i], Odometry) for i in range(self.config['num_bots'])]
        self.sync = ApproximateTimeSynchronizer(
            self.subscriber, 
            queue_size=10, 
            slop=0.1)
        self.sync.registerCallback(lambda *args: self.synchronized_callback(*args))
        self.refresh_timer = rospy.Timer(rospy.Duration(self.refresh_rate), self.refresh_callback)

        self.robots_path = os.path.join(script_dir, "../../../ui/database/robots.json")

    def synchronized_callback(self, *odom_msgs):
        self.robots = [
            {"name": f"bot{i+1}", 
            "loc_x": odom_msg.pose.pose.position.x,
            "loc_y": round(odom_msg.pose.pose.position.y, 2), 
            "loc_z": round(odom_msg.pose.pose.position.z, 2)}
            for i, odom_msg in enumerate(odom_msgs)
        ]

    def refresh_callback(self, event):
        with self.lock:
            self.save_robots_to_database()        

    def save_robots_to_database(self):
        data_to_save = self.robots
        try:
            with open(self.robots_path, 'w') as f:
                json.dump(data_to_save, f, indent=4)
        except Exception as e:
            print(f"Error occurred: {e}")

if __name__ == "__main__":
    try:
        db_handler = RobotHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass