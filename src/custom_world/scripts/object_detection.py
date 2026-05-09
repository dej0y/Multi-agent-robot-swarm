#!/usr/bin/env python3
import os
import cv2
import rospy
import random
import numpy as np
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_world.msg import ObjectPosition

dir = os.getcwd()

names_path = os.path.join(dir, 'coco.names')
weight_path = os.path.join(dir, 'frozen_inference_graph.pb')
config_path = os.path.join(dir, 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt')

net = cv2.dnn_DetectionModel(weight_path, config_path)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

classNames = []
with open(names_path,'r') as f:
    classNames = f.read().splitlines()

class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)

        self.bridge = CvBridge()
        self.obj_msg = ObjectPosition()
        self.bot_id = rospy.get_param('~bot_id')

        self.rgb_sub = message_filters.Subscriber('/bot{}/camera/rgb/image_raw'.format(self.bot_id), Image)
        self.dep_sub = message_filters.Subscriber('/bot{}/camera/depth/image_raw'.format(self.bot_id), Image)

        self.obj_pub = rospy.Publisher('/obj/position', ObjectPosition, queue_size=10)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.dep_sub], 
            queue_size=10, 
            slop=0.1)
        self.ts.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, rgb_msg, dep_msg):
        rgb_frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        dep_frame = self.bridge.imgmsg_to_cv2(dep_msg, "16UC1")

        # Object detection on RGB frame
        classIds, confs, bbox = net.detect(rgb_frame, confThreshold=0.5)
        indices = cv2.dnn.NMSBoxes(bbox, confs, 0.5, 0.1)

        for i in indices.flatten() if indices.size > 0 else []:
            x, y, w, h = bbox[i]
            cv2.rectangle(rgb_frame, (x, y), (x + w, y + h), (0, 0, 255), thickness=2)
            x_center = x + w // 2
            y_center = y + h // 2

            valid_depths = []
            for _ in range(10):
                x_rand = x_center + random.randint(-w // 4, w // 4)
                y_rand = y_center + random.randint(-h // 4, h // 4)
                depth = dep_frame[y_rand, x_rand]
                if depth in range(1, 10):
                    valid_depths.append(depth)
            median_depth = np.median(valid_depths)

            Z = median_depth
            Y = (y_center - 240) * Z / 500
            X = (x_center - 320) * Z / 500

            self.obj_msg.x = X
            self.obj_msg.y = Y
            self.obj_msg.z = Z
            self.obj_msg.class_name = classNames[classIds[i]]
            self.obj_pub.publish(self.obj_msg)

if __name__ == "__main__":
    try:
        object_detection_node = ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass