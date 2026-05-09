#!/usr/bin/env python3
import rospy
import os
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

script_dir = os.path.dirname(os.path.abspath(__file__))

class MapHandler:
    def __init__(self):
        rospy.init_node('map_saver_node', anonymous=True)
        
        self.map_topic = rospy.get_param("~map_topic")
        self.map_subscriber = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)

        self.bridge = CvBridge()
        self.map_path = os.path.join(script_dir, "../../../ui/database/map.png")

    def map_callback(self, msg):
        map_data = np.array(msg.data, dtype=np.uint8)
        map_width = msg.info.width
        map_height = msg.info.height
        map_data = map_data.reshape((map_height, map_width))

        print(self.map_path)
        cv2.imwrite(self.map_path, map_data)
        cv2.imshow(map_data, "image")
        cv2.waitKey(1)

    def save_map(self, file_path):
        try:
            rospy.loginfo(f"Saving map to {file_path}")
            response = self.map_saver(map_file_name=file_path)
            if response.success:
                rospy.loginfo(f"Map successfully saved to {file_path}")
            else:
                rospy.logwarn("Failed to save the map.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        map_saver = MapHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
