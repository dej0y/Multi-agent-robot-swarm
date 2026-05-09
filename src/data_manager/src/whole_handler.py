#!/usr/bin/env python3
import rospy
import os
import json
import yaml
from threading import Lock

script_dir = os.path.dirname(os.path.abspath(__file__))

class WholeHandler:
    def __init__(self):
        rospy.init_node('whole_handler', anonymous=True)

        self.lock = Lock()
        self.refresh_rate = rospy.get_param("~refresh_rate")
        self.refresh_timer = rospy.Timer(rospy.Duration(self.refresh_rate), self.refresh_callback)

        self.whole_path = os.path.join(script_dir, "../../../ui/database/whole.json")
        self.robots_path = os.path.join(script_dir, "../../../ui/database/robots.json")
        self.objects_path = os.path.join(script_dir, "../../../ui/database/objects.json")

    def refresh_callback(self, event):
        with self.lock:
            self.save_whole_to_database()        

    def save_whole_to_database(self):
        try:
            with open(self.robots_path, 'r') as rob_file, open(self.objects_path, 'r') as obj_file:
                robots_data = json.load(rob_file)
                objects_data = json.load(obj_file)

            whole_data = {
                "map": "./database/map.png",
                "bots": robots_data,
                "objects": objects_data
            }

            with open(self.whole_path, 'w') as out_file:
                json.dump(whole_data, out_file, indent=4)
        except Exception as e:
            print(f"Error occurred: {e}")


if __name__ == "__main__":
    try:
        db_handler = WholeHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass