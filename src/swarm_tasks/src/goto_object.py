#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def send_goal(bot_namespace, x, y, yaw):
    """
    Sends a goal to the move_base action server for a specific robot.
    Args:
        bot_namespace (str): Namespace of the robot (e.g., 'bot1').
        x (float): X coordinate of the goal.
        y (float): Y coordinate of the goal.
        yaw (float): Orientation of the goal in radians.
    """
    # Initialize the action client for the move_base server in the specified namespace
    action_server = f"{bot_namespace}/move_base"
    client = actionlib.SimpleActionClient(action_server, MoveBaseAction)
    
    rospy.loginfo(f"Waiting for {action_server} action server...")
    client.wait_for_server()  # Wait until the server is up
    rospy.loginfo(f"Connected to {action_server} action server.")

    # Define the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Use the appropriate frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set position
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    # Set orientation (convert yaw to quaternion)
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    rospy.loginfo(f"Sending goal to {bot_namespace}: x={x}, y={y}, yaw={yaw}")
    client.send_goal(goal)

    # Wait for result
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Goal reached successfully by {bot_namespace}!")
    else:
        rospy.logwarn(f"{bot_namespace} failed to reach the goal.")

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node("goal_publisher", anonymous=True)

        bot = rospy.get_param("~bot")
        loc_x = rospy.get_param("~loc_x")
        loc_y = rospy.get_param("~loc_y")
        goal_yaw = 0.0

        # Publish the goal for bot1
        send_goal(bot, loc_x, loc_y, goal_yaw)

    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted.")
