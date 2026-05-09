#!/usr/bin/env python3
import random
import rospy
import os
import tf
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image
from ultralytics_ros.msg import YoloRstArr
from deepsort_ros.msg import ObjectLocation
from message_filters import Subscriber, ApproximateTimeSynchronizer
from deep_sort import generate_detections as gdet
from deep_sort import preprocessing as prep
from deep_sort.tracker import Tracker
from deep_sort.detection import Detection
from deep_sort.nn_matching import NearestNeighborDistanceMetric
from cv_bridge import CvBridge


class DeepsortTracker:
    def __init__(self):
        rospy.init_node('deepsort_tracker', anonymous=True)

        self.bot_num = rospy.get_param("~bot_num")
        self.deep_track_topic = rospy.get_param("~deep_track_topic")
        self.odom_track_topic = rospy.get_param("~odom_track_topic")
        self.yolo_image_topic = rospy.get_param("~yolo_image_topic")
        self.yolo_depth_topic = rospy.get_param("~yolo_depth_topic")
        self.yolo_result_topic = rospy.get_param("~yolo_result_topic")

        max_cosine_distance = 0.2
        nn_budget = 100
        metric = NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        self.tracker = Tracker(metric)
        model_name = rospy.get_param("~model_name")
        model_path = os.path.join(os.path.abspath(__file__), "..", "deep_sort", "model_data", model_name)
        self.encoder = gdet.create_box_encoder(model_path)

        self.bridge = CvBridge()
        yolo_image_sub = Subscriber(self.yolo_image_topic, Image)
        yolo_depth_sub = Subscriber(self.yolo_depth_topic, Image)
        odom_track_sub = Subscriber(self.odom_track_topic, Odometry)
        yolo_result_sub = Subscriber(self.yolo_result_topic, YoloRstArr)
        self.sync = ApproximateTimeSynchronizer(
            [odom_track_sub, yolo_image_sub, yolo_depth_sub, yolo_result_sub], 
            queue_size=10, 
            slop=0.1,
            allow_headerless=True)
        self.sync.registerCallback(self.synchronized_callback)

        self.obj_msg = ObjectLocation()
        self.trackers_pub = rospy.Publisher(self.deep_track_topic, ObjectLocation, queue_size=10)

    def synchronized_callback(self, odom_msg, image_msg, depth_msg, result_msg):
        rgb_frame = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        dep_frame = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        clses = []
        scores = []
        detections = []
        for obj in result_msg.detections:
            clses.append(obj.object_id)
            scores.append(float('%.2f' % obj.score))
            detections.append(
                np.array([obj.center_x-obj.size_x//2, obj.center_y-obj.size_y//2, obj.size_x, obj.size_y])
            )
        features = self.encoder(rgb_frame, detections)
        detections_new = [
            Detection(bbox, cls, score, feature) for bbox, cls, score, feature in zip(detections, clses, scores, features)
        ]

        boxes = np.array([d.tlwh for d in detections_new])
        scores_new = np.array([d.confidence for d in detections_new])
        indices = prep.non_max_suppression(boxes, 1.0, scores_new)
        detections_new = [detections_new[i] for i in indices]

        self.tracker.predict()
        self.tracker.update(detections_new)

        for track in self.tracker.tracks:
            if not track.is_confirmed() or track.time_since_update > 1:
                continue
            bbox = track.to_tlbr()
            obj_id = track.track_id
            obj_cls = track.object_id
            x_center, x_delta = (bbox[0]+bbox[2])//2, (bbox[2]-bbox[0])//4
            y_center, y_delta = (bbox[1]+bbox[3])//2, (bbox[3]-bbox[1])//4

            valid_depths = []
            for _ in range(8):
                x_rand = x_center + random.randint(-x_delta, x_delta)
                if x_rand<=0: x_rand=0
                if x_rand>=640: x_rand=639
                y_rand = y_center + random.randint(-y_delta, y_delta)
                if y_rand<=0: y_rand=0
                if y_rand>=640: y_rand=639
                depth = dep_frame[int(y_rand), int(x_rand)]
                if depth>0: valid_depths.append(depth)
            median_depth = np.median(valid_depths)
            if np.isnan(median_depth): continue

            Z_cam = median_depth
            X_cam = (x_center - 320) * Z_cam / 500
            Y_cam = (y_center - 240) * Z_cam / 500

            robot_x = odom_msg.pose.pose.position.x
            robot_y = odom_msg.pose.pose.position.y
            robot_z = odom_msg.pose.pose.position.z

            orientation_q = odom_msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

            Y_world = Y_cam 
            X_world = math.cos(yaw) * Z_cam + math.sin(yaw) * X_cam
            Z_world = -math.sin(yaw) * Z_cam + math.cos(yaw) * X_cam

            
            self.obj_msg.x = robot_x + X_world
            self.obj_msg.y = robot_y - Z_world
            self.obj_msg.z = robot_z + Y_world
            self.obj_msg.id = obj_id
            self.obj_msg.cls = obj_cls
            self.obj_msg.num = self.bot_num
            self.obj_msg.ts = rospy.get_rostime().to_sec()
            self.trackers_pub.publish(self.obj_msg)


if __name__ == "__main__":
    try:
        tracker = DeepsortTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
