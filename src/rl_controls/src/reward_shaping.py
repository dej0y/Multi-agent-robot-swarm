#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def merged_map_callback(data):
    # Extract the map dimensions
    width = data.info.width
    height = data.info.height

    # Convert the OccupancyGrid data to a NumPy array and reshape it
    grid = np.array(data.data).reshape((height, width))
    trimmed_map = grid[:4000:2, :4000:2]
    # Print the shape of the grid
    print(f"Merged Map Shape: {grid.shape}")
    print(f"trimmed map shape: {trimmed_map.shape}")

def merged_map_listener():
    rospy.init_node('merged_map_subscriber', anonymous=True)

    # Subscribe to the /merged_map topic
    rospy.Subscriber('/map_merged', OccupancyGrid, merged_map_callback)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    try:
        merged_map_listener()
    except rospy.ROSInterruptException:
        pass
