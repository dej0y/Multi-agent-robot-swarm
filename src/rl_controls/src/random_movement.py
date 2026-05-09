#!/usr/bin/env python3

import rospy
import random
import argparse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

# Global variables
safe_distance = 1.0  # Minimum distance from an obstacle
linear_speed_limit = 0.7  # Maximum linear speed
angular_speed_limit = 1.5  # Maximum angular speed
repulsion_strength = 2.0  # Strength of repulsive force
attraction_strength = 0.5  # Strength of random attractive force

# Robot state
obstacle_detected = False
repulsive_force = [0, 0]  # X and Y components of the repulsive force
random_direction = [1, 0]  # Random goal force direction (unit vector)

def scan_callback(scan_data):
    global obstacle_detected, repulsive_force
    
    # Reset repulsive force
    repulsive_force = [0, 0]
    obstacle_detected = False

    for angle, distance in enumerate(scan_data.ranges):
        if distance < safe_distance and distance > scan_data.range_min:
            obstacle_detected = True
            
            # Calculate the angle of the obstacle in radians
            obstacle_angle = scan_data.angle_min + angle * scan_data.angle_increment
            
            # Calculate repulsive force components (in robot's local frame)
            repulsive_x = -math.cos(obstacle_angle) / (distance**2)
            repulsive_y = -math.sin(obstacle_angle) / (distance**2)
            
            # Accumulate forces
            repulsive_force[0] += repulsive_x
            repulsive_force[1] += repulsive_y

def normalize_vector(vector):
    """Normalize a 2D vector."""
    magnitude = math.sqrt(vector[0]**2 + vector[1]**2)
    if magnitude == 0:
        return [0, 0]
    return [vector[0] / magnitude, vector[1] / magnitude]
def random_movement(pub):
    global random_direction, repulsive_force
    
    twist_msg = Twist()
    
    # Normalize the random direction vector
    random_direction = normalize_vector(random_direction)
    
    # Calculate total force
    total_force_x = repulsion_strength * repulsive_force[0] + attraction_strength * random_direction[0]
    total_force_y = repulsion_strength * repulsive_force[1] + attraction_strength * random_direction[1]
    
    # Normalize total force to get a direction
    total_force = normalize_vector([total_force_x, total_force_y])
    
    # Convert force vector to motion commands
    base_linear_x = max(0.1, 0.85 * min(linear_speed_limit, math.sqrt(total_force[0]**2 + total_force[1]**2)))
    base_angular_z = math.atan2(total_force_y, total_force_x)
    
    # Apply bias to angular velocity for smoother movement
    if obstacle_detected:
        # If obstacle detected, prioritize angular velocity for avoidance
        twist_msg.angular.z = max(-angular_speed_limit, min(angular_speed_limit, base_angular_z))
    else:
        # Reduce angular velocity when no obstacle is detected
        twist_msg.angular.z = base_angular_z * 0.7  # Apply scaling factor for smoother turns
        twist_msg.angular.z = max(-angular_speed_limit, min(angular_speed_limit, twist_msg.angular.z))
    
    # Add randomness to linear and angular velocities
    random_linear_offset = random.uniform(-0.1, 0.1)  # Max deviation of 0.1 m/s
    random_angular_offset = random.uniform(-0.1, 0.1) * angular_speed_limit  # Scale angular randomness
    
    twist_msg.linear.x = max(0.1, min(linear_speed_limit, base_linear_x + random_linear_offset))
    twist_msg.angular.z = max(-angular_speed_limit, min(angular_speed_limit, twist_msg.angular.z + random_angular_offset))
    
    # Publish the command
    pub.publish(twist_msg)

    # Periodically change the random direction to ensure variability
    if random.random() < 0.1:  # 10% chance to change direction
        random_direction = [random.uniform(-1, 1), random.uniform(-1, 1)]

def main(scan_topic, cmd_vel_topic, namespace):
    rospy.init_node(f'{namespace}_random_movement', anonymous=True)
    
    # Subscriber to laser scan topic
    rospy.Subscriber(scan_topic, LaserScan, scan_callback)
    
    # Publisher to cmd_vel topic for robot movement
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    # Set loop rate
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        random_movement(cmd_vel_pub)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Random movement with obstacle avoidance")
    parser.add_argument('--scan_topic', type=str, default='/scan', help="Topic name for laser scan data")
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel', help="Topic name for velocity commands")
    parser.add_argument('--namespace', type=str, default='robot', help="Namespace for the node")

    args = parser.parse_args()
    
    try:
        main(args.scan_topic, args.cmd_vel_topic, args.namespace)
    except rospy.ROSInterruptException:
        pass
