#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import os

script_dir = os.path.dirname(os.path.abspath(__file__))

class MapSaver:
    def __init__(self):
        rospy.init_node('map_saver', anonymous=True)
        self.map_data = None
        self.map_png_path = os.path.join(script_dir, "../../../ui/database/map.png")
        self.map_yaml_path = os.path.join(script_dir, "../../../ui/database/map.yaml")
        self.subscriber = rospy.Subscriber('/merged_map', OccupancyGrid, self.map_callback)
        self.rate = rospy.Rate(0.7)  # Save map every 1 second

        rospy.loginfo(f"Map saver initialized. Saving PNG to {self.map_png_path} and YAML to {self.map_yaml_path}")

    def map_callback(self, msg):
        self.map_data = msg
        rospy.loginfo("Received new map data.")

    def save_map(self):
        if self.map_data is None:
            rospy.logwarn("No map data available to save yet.")
            return

        try:
            # Save map as PNG
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin = self.map_data.info.origin.position
            data = self.map_data.data

            # Convert map data to an image
            image = Image.new('L', (width, height))
            image_data = [
                255 - (cell if cell != -1 else 205)  # Unknown cells (-1) are set to gray (205)
                for cell in data
            ]
            image.putdata(image_data)
            image = image.transpose(Image.FLIP_TOP_BOTTOM)  # Correct orientation
            image.save(self.map_png_path)
            rospy.loginfo(f"Saved PNG map to {self.map_png_path}")

            # Save YAML metadata
            yaml_content = f"""image: {self.map_png_path}
            resolution: {resolution}
            origin: [{origin.x}, {origin.y}, {origin.z}]
            negate: 0
            occupied_thresh: 0.65
            free_thresh: 0.196
            """
            with open(self.map_yaml_path, 'w') as yaml_file:
                yaml_file.write(yaml_content)
            rospy.loginfo(f"Saved YAML metadata to {self.map_yaml_path}")

        except Exception as e:
            rospy.logerr(f"Failed to save map: {e}")

    def run(self):
        while not rospy.is_shutdown():
            self.save_map()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        map_saver = MapSaver()
        map_saver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down map saver node.")
