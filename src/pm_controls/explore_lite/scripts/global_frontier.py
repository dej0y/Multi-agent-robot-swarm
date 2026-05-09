#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

class FrontierMarkerPublisher:
    def __init__(self):
        # ROS node setup
        self.map_sub = rospy.Subscriber('/merged_map', OccupancyGrid, self.map_callback)
        self.marker_pub = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)
        self.frontier_count_pub = rospy.Publisher('/frontier_count', Int32, queue_size=10)
        rospy.loginfo("FrontierMarkerPublisher initialized.")

    def map_callback(self, msg):
        # Extract map data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin
        data = msg.data

        # Convert map to 2D grid
        grid = [data[i:i + width] for i in range(0, len(data), width)]

        # Identify frontier points
        frontiers = self.find_frontiers(grid, width, height)

        # Publish frontier points as markers
        self.publish_markers(frontiers, resolution, origin)

        # Publish the number of frontiers
        self.publish_frontier_count(len(frontiers))

    def find_frontiers(self, grid, width, height):
        frontiers = []
        for y in range(height):
            for x in range(width):
                if grid[y][x] == 0:  # Free space
                    if self.is_frontier(grid, x, y, width, height):
                        frontiers.append((x, y))
        return frontiers

    def is_frontier(self, grid, x, y, width, height):
        neighbors = [
            (x + dx, y + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
            if 0 <= x + dx < width and 0 <= y + dy < height
        ]
        for nx, ny in neighbors:
            if grid[ny][nx] == -1:  # Unknown space
                return True
        return False

    def publish_markers(self, frontiers, resolution, origin):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Point size
        marker.scale.y = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Fully opaque

        # Add frontier points to the marker
        for point in frontiers:
            px = point[0] * resolution + origin.position.x
            py = point[1] * resolution + origin.position.y
            marker.points.append(Point(x=px, y=py, z=0.0))

        marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def publish_frontier_count(self, count):
        # Publish the number of frontiers
        frontier_count_msg = Int32()
        frontier_count_msg.data = count
        self.frontier_count_pub.publish(frontier_count_msg)
        rospy.loginfo(f"Frontier count published: {count}")

if __name__ == "__main__":
    rospy.init_node("frontier_marker_publisher", anonymous=True)
    publisher = FrontierMarkerPublisher()
    rospy.spin()
