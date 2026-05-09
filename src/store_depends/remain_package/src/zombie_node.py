#!/usr/bin/env python3
import rospy

if __name__ == "__main__":
    rospy.init_node("map_explored")
    rospy.loginfo("Node 'map_explored' initialized and running.")
    rospy.spin()