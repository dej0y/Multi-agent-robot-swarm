#!/usr/bin/env python3
import rospy
import os
import json
from threading import Lock
from scipy.spatial import KDTree
from deepsort_ros.msg import ObjectLocation

script_dir = os.path.dirname(os.path.abspath(__file__))

class ObjectHandler:
    def __init__(self):
        rospy.init_node('object_handler', anonymous=True)

        self.expiry_time = rospy.get_param("~expiry_time")
        self.refresh_rate = rospy.get_param("~refresh_rate")
        self.tracker_topic = rospy.get_param("~tracker_topic")
        self.duplicate_threshold = rospy.get_param("~duplicate_threshold", 1.0)

        self.objects = {}
        self.lock = Lock()

        self.subscriber = rospy.Subscriber(self.tracker_topic, ObjectLocation, self.object_callback)
        self.refresh_timer = rospy.Timer(rospy.Duration(self.refresh_rate), self.refresh_callback)

        self.objects_path = os.path.join(script_dir, "../../../ui/database/objects.json")

    def object_callback(self, msg):
        ts = msg.ts
        obj_id = msg.id
        obj_cls = msg.cls
        bot_num = msg.num
        x, y, z = msg.x, msg.y, msg.z

        with self.lock:
            if bot_num not in self.objects:
                self.objects[bot_num] = {}

            if obj_id in self.objects[bot_num]:
                self.objects[bot_num][obj_id]["timestamp"] = ts
                self.objects[bot_num][obj_id]["position"] = (x, y, z)
                self.objects[bot_num][obj_id]["class"] = obj_cls
            else:
                self.objects[bot_num][obj_id] = {
                    "position": (x, y, z),
                    "timestamp": ts,
                    "class": obj_cls,
                }

    def refresh_callback(self, event):
        with self.lock:
            self.remove_expired_objects()
            self.remove_duplicate_objects()
            self.save_objects_to_database()        

    def remove_expired_objects(self):
        current_time = rospy.Time.now().to_sec()
        for bot_num in list(self.objects.keys()):
            to_remove = [
                obj_id
                for obj_id, obj_data in self.objects[bot_num].items()
                if current_time - obj_data["timestamp"] > self.expiry_time
            ]
            for obj_id in to_remove:
                rospy.loginfo(f"Removing ID {obj_id} for Bot {bot_num} due to timeout.")
                del self.objects[bot_num][obj_id]

            if not self.objects[bot_num]:
                del self.objects[bot_num]

    def remove_duplicate_objects(self):
        all_objects = []
        id_to_bot_map = {}

        for bot_num, objects in self.objects.items():
            for obj_id, obj_data in objects.items():
                obj_cls = obj_data["class"]
                position = obj_data["position"]
                all_objects.append((obj_id, obj_cls, position))
                id_to_bot_map[obj_id] = bot_num

        positions = [obj[2] for obj in all_objects]
        ids = [obj[0] for obj in all_objects]
        classes = [obj[1] for obj in all_objects]

        if positions:
            tree = KDTree(positions)
            duplicates = set()

            for idx, position in enumerate(positions):
                nearby_indices = tree.query_ball_point(position, r=self.duplicate_threshold)
                for nearby_idx in nearby_indices:
                    if idx == nearby_idx:
                        continue
                    if classes[idx] == classes[nearby_idx]:
                        duplicates.add(ids[nearby_idx])

            for obj_id in duplicates:
                bot_num = id_to_bot_map[obj_id]
                if obj_id in self.objects[bot_num]:
                    rospy.loginfo(f"Removing duplicate object ID {obj_id} for Bot {bot_num}.")
                    del self.objects[bot_num][obj_id]

            for bot_num in list(self.objects.keys()):
                if not self.objects[bot_num]:
                    del self.objects[bot_num]

    def save_objects_to_database(self):
        data_to_save = []
        for bot_num, objects in self.objects.items():
            for obj_id, obj_data in objects.items():
                formatted_object = {
                    "name": obj_data["class"],
                    "loc_x": round(obj_data["position"][0], 2),
                    "loc_y": round(obj_data["position"][1], 2),
                    "loc_z": round(obj_data["position"][2], 2)
                }
                data_to_save.append(formatted_object)
        try:
            with open(self.objects_path, 'w') as f:
                json.dump(data_to_save, f, indent=4)
        except Exception as e:
            print(f"Error occurred: {e}")

if __name__ == "__main__":
    try:
        db_handler = ObjectHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
