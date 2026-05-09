#!/usr/bin/env python3
import os
import rospy
from scipy.spatial import KDTree
from custom_world.msg import ObjectPosition

dir = os.getcwd()

names_path = os.path.join(dir, 'coco.names')
classNames = []
with open(names_path, 'r') as f:
    classNames = f.read().splitlines()

class ObjectSaver:
    def __init__(self):
        rospy.init_node('object_saver', anonymous=True)
        self.pos_dict = {class_name: {'pos': [], 'tree': None} for class_name in classNames}
        self.pos_sub = rospy.Subscriber('/obj/position', ObjectPosition, self.position_callback)

    def position_callback(self, pos_msg):
        obj_cls = pos_msg.class_name
        pos = [pos_msg.x, pos_msg.y, pos_msg.z]

        pos_data = self.positions_dict[obj_cls]
        if pos_data['tree'] is not None:
            distance, _ = pos_data['tree'].query(pos)
            if distance < 0.5: return

        pos_data['pos'].append(pos)
        pos_data['tree'] = KDTree(pos_data['pos'])

if __name__ == '__main__':
    try:
        object_saver = ObjectSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
