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
        
        # Set the initial position and orientation (at point_a with yaw_a)
        model_state.pose = Pose()
        model_state.pose.position = point_a
        model_state.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw_a))
        model_state.reference_frame = "world"
        
        set_model_state(model_state)
        rospy.loginfo(f"Model {model_name} initial position set to {point_a} with yaw {yaw_a}")

        # Move back and forth
        while not rospy.is_shutdown():
            rospy.loginfo(f"Model {model_name}: Moving from {point_a} to {point_b}")
            move_model_between_points(set_model_state, model_state, point_a, point_b, yaw_a, yaw_b, velocity)
            
            rospy.loginfo(f"Model {model_name}: Moving from {point_b} to {point_a}")
            move_model_between_points(set_model_state, model_state, point_b, point_a, yaw_b, yaw_a, velocity)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# Function to move a model between two points with interpolated yaw
def move_model_between_points(set_model_state, model_state, start_point, end_point, start_yaw, end_yaw, velocity):
    step_size = 0.01  # Distance increment per step
    current_pos = model_state.pose.position
    current_yaw = start_yaw
    yaw_step = (end_yaw - start_yaw) / ((start_point.x - end_point.x)**2 + (start_point.y - end_point.y)**2)**0.5 / step_size

    while abs(current_pos.x - end_point.x) > step_size or abs(current_pos.y - end_point.y) > step_size:
        # Calculate direction vector and normalize
        direction = Point(end_point.x - current_pos.x, end_point.y - current_pos.y, 0)
        dist = (direction.x**2 + direction.y**2)**0.5
        direction.x /= dist
        direction.y /= dist

        # Update position incrementally
        new_position = Point(current_pos.x + direction.x * step_size,
                             current_pos.y + direction.y * step_size,
                             current_pos.z)
        
        # Interpolate yaw
        current_yaw += yaw_step
        new_orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, current_yaw))
        
        # Update the model's state
        model_state.pose.position = new_position
        model_state.pose.orientation = new_orientation
        set_model_state(model_state)

        # Update current position
        current_pos = new_position
        time.sleep(step_size / velocity)

# Function to start movement for multiple models
def move_multiple_models():
    # Define initial points, velocities, and yaw for each model
    points = [
        (Point(-3.625500, 5.592080, 0.0), Point(-3.625500, 1.166016, 0.0), 0.1, 0.0, 0.0),  
        (Point(-6.709127, 6.982453, 0.142754), Point(-4.950048, 2.959033, 0.142754), 0.2, -1.155175, -1.155175),  
        (Point(-8.880123, 1.391257, 0.142754), Point(-12.164167, 1.400892, 0.142754), 0.15, 0.139878, 0.139878) 
    ]
    
    # Define model names
    model_names = [
        "casual_female",
        "husky_clone", 
        "husky"
    ]
    
    # Create threads for each model to move them simultaneously
    threads = []
    for i in range(3):
        point_a, point_b, velocity, yaw_a, yaw_b = points[i]
        thread = Thread(target=move_back_and_forth_for_model, args=(model_names[i], point_a, point_b, yaw_a, yaw_b, velocity))
        threads.append(thread)
        thread.start()

    # Wait for all threads to finish
    for thread in threads:
        thread.join()

    rospy.loginfo("All models completed their movements!")

if __name__ == '__main__':
    try:
        rospy.init_node('move_back_and_forth_multiple_models', anonymous=True)
        move_multiple_models()
    except rospy.ROSInterruptException:
        pass
