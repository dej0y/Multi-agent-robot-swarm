#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import numpy as np

class PointCloudToLaserScan:
    def __init__(self):
        # Define the ranges for x, y, z filtering
        self.x_min, self.x_max = -10.0, 10.0
        self.y_min, self.y_max = -10.0, 10.0
        self.z_min, self.z_max = -0.17, 10.0

        # Define LaserScan parameters
        self.laser_angle_min = -np.pi  # -180 degrees
        self.laser_angle_max = np.pi   # 180 degrees
        self.laser_angle_increment = np.radians(1.0)  # 0.5-degree increments

        # Get the namespace from the ROS parameter server
        self.namespace = rospy.get_param("~namespace", "default_namespace")

        # Subscribers for both robots (namespaced topic names)
        self.sub_bot1 = rospy.Subscriber(f"/{self.namespace}/velodyne_points", PointCloud2, self.callback_bot1)

        # Publishers for filtered laser data for both robots (namespaced topic names)
        self.pub_laser_bot1 = rospy.Publisher(f"/{self.namespace}/laser_scan", LaserScan, queue_size=10)
      
    def callback_bot1(self, msg):
        self.filter_and_convert(msg, self.pub_laser_bot1)

    def filter_and_convert(self, msg, publisher):
        # Convert PointCloud2 to numpy array
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array(points, dtype=np.float32)

        # Apply range filtering for x, y, z
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        range_mask = (
            (x >= self.x_min) & (x <= self.x_max) &
            (y >= self.y_min) & (y <= self.y_max) &
            (z >= self.z_min) & (z <= self.z_max)
        )
        filtered_points = points[range_mask]

        # Convert to polar coordinates (range and angle)
        ranges = np.sqrt(filtered_points[:, 0]**2 + filtered_points[:, 1]**2)
        angles = np.arctan2(filtered_points[:, 1], filtered_points[:, 0])

        # Initialize LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = msg.header
        # scan_msg.header.frame_id = f"/{self.namespace}/base_scan"
        scan_msg.angle_min = self.laser_angle_min
        scan_msg.angle_max = self.laser_angle_max
        scan_msg.angle_increment = self.laser_angle_increment
        scan_msg.range_min = 0.12  # Minimum range (set to a small positive value)
        scan_msg.range_max = 100.0  # Maximum range (set to a suitable max range)

        # Create ranges array for LaserScan
        num_bins = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        scan_msg.ranges = [float('inf')] * num_bins  # Initialize all ranges to 'inf'

        # Fill in the ranges array with the filtered points
        for r, angle in zip(ranges, angles):
            # Compute the index in the ranges array (wrap angles between -pi to pi)
            bin_idx = int(((angle - scan_msg.angle_min) % (2 * np.pi)) / scan_msg.angle_increment)

            # Ensure bin_idx is within the valid range
            if 0 <= bin_idx < num_bins:
                scan_msg.ranges[bin_idx] = min(scan_msg.ranges[bin_idx], r)

        # Publish the LaserScan message
        publisher.publish(scan_msg)

if __name__ == "__main__":
    rospy.init_node('point_cloud_to_laser_scan')
    PointCloudToLaserScan()
    rospy.spin()
