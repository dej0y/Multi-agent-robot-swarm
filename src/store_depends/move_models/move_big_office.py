#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import time
from threading import Thread

# Function to move a single model back and forth between two points with yaw
def move_back_and_forth_for_model(model_name, point_a, point_b, yaw_a, yaw_b, velocity):
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = Pose()
        model_state.pose.position = point_a
        model_state.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw_a))
        model_state.reference_frame = "world"
        
        set_model_state(model_state)
        rospy.loginfo(f"Model {model_name} initial position set to {point_a} with yaw {yaw_a}")

        while not rospy.is_shutdown():
            rospy.loginfo(f"Model {model_name}: Moving from {point_a} to {point_b}")
            move_model_between_points(set_model_state, model_state, point_a, point_b, yaw_a, yaw_b, velocity)
            
            rospy.loginfo(f"Model {model_name}: Moving from {point_b} to {point_a}")
            move_model_between_points(set_model_state, model_state, point_b, point_a, yaw_b, yaw_a, velocity)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

# Function to move a model between two points with interpolated yaw
def move_model_between_points(set_model_state, model_state, start_point, end_point, start_yaw, end_yaw, velocity):
    step_size = 0.01
    current_pos = model_state.pose.position
    current_yaw = start_yaw
    yaw_step = (end_yaw - start_yaw) / ((start_point.x - end_point.x)**2 + (start_point.y - end_point.y)**2)**0.5 / step_size

    while abs(current_pos.x - end_point.x) > step_size or abs(current_pos.y - end_point.y) > step_size:
        direction = Point(end_point.x - current_pos.x, end_point.y - current_pos.y, 0)
        dist = (direction.x**2 + direction.y**2)**0.5
        direction.x /= dist
        direction.y /= dist

        new_position = Point(current_pos.x + direction.x * step_size,
                             current_pos.y + direction.y * step_size,
                             current_pos.z)
        
        current_yaw += yaw_step
        new_orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, current_yaw))
        model_state.pose.position = new_position
        model_state.pose.orientation = new_orientation
        set_model_state(model_state)

        current_pos = new_position
        time.sleep(step_size / velocity)

# Function to start movement for multiple models
def move_multiple_models():
    points = [
        (Point(-8.616920, 10.636900, 0.0), Point(-8.616920, 4.110901, 0.0), 0.2, 0.0, 0.0),
        (Point(-9.749172, -6.557289, 0.142754), Point(-11.358500, -14.642485, 0.142754), 0.2, 1.411359, 1.411359), 
        (Point(-0.440037, -2.787240, 0.0), Point(-6.094445, -2.970711, 0.0), 0.15, 1.603240, 1.603240), 
        (Point(-6.319410, -5.227580, 0.0), Point(-6.319410, -10.107514, 0.0), 0.15, 0.0, 0.0),
        (Point(-4.647590, 10.389200, 0.0), Point(-4.647590, 3.483236, 0.0), 0.15, 0.0, 0.0)
    ]
    
    model_names = [
        "mars_rover",
        "husky_clone", 
        "person_walking",
        "casual_female_0",
        "elegant_male_0"
    ]
    
    threads = []
    for i in range(5):
        point_a, point_b, velocity, yaw_a, yaw_b = points[i]
        thread = Thread(target=move_back_and_forth_for_model, args=(model_names[i], point_a, point_b, yaw_a, yaw_b, velocity))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    rospy.loginfo("All models completed their movements!")

if __name__ == '__main__':
    try:
        rospy.init_node('move_back_and_forth_multiple_models', anonymous=True)
        move_multiple_models()
    except rospy.ROSInterruptException:
        pass
