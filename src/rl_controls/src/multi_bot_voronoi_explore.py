#! /usr/bin/env python

from nav_msgs import msg
import rospy
import rospkg
from rospy import names
from rospy.timer import sleep
import tf
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped, Pose
from math import cos, radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion

import threading
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState 

from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
#from kobuki_msgs.msg import BumperEvent
import time

import tensorflow
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input, merge
from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam, get
import keras.backend as K
import gym
import numpy as np
import math
import random

from std_srvs.srv import Empty

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Int8MultiArray


class InfoGetter(object):
    def __init__(self):
        #event that will block until the info is received
        self._event = threading.Event()
        #attribute for storing the rx'd message
        self._msg = None

    def __call__(self, msg):
        #Uses __call__ so the object itself acts as the callback
        #save the data, trigger the event
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=None):
        """Blocks until the data is rx'd with optional timeout
        Returns the received message
        """
        self._event.wait(timeout)
        return self._msg
    
class GameState:

    def __init__(self, num_bots_online=4):
        self.talker_node = rospy.init_node('game_state_handler', anonymous=True)
        self.pose_ig = InfoGetter()
        self.collision_ig = InfoGetter()
        
        self.move_cmd = Twist()
        self.mode = 0
        # tf
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)

        self.odom_frame = '/odom'
        self.base_frame = '/base_footprint'

        # Number of bots
        self.num_bots_online = num_bots_online

        # Dynamically create robot names
        self.robot_name = [f'bot{i+1}' for i in range(self.num_bots_online)]

        # Initialize position, rotation, record_info_node, next_target_node, and arr_info_node dictionaries
        self.position = {name: Point() for name in self.robot_name}
        self.rotation = {name: 0.0 for name in self.robot_name}
        self.record_info_node = {name: [] for name in self.robot_name}
        self.next_target_node = {name: [] for name in self.robot_name}
        self.arr_info_node = {name: False for name in self.robot_name}
        self.laser_crashed_value = {name: False for name in self.robot_name}

        # whether complete
        self.done = False


        # each robot share its own position max distance  ## used for voronoi patches 
        self.communication_max_range = 8

        # Is there any information node point within the detection range
        self.detect_info_node = False

        self.rate = rospy.Rate(100) # 100hz

        # Create publishers Dict 
        self.cmd_vel_pub = {name: rospy.Publisher(f'{name}/cmd_vel', Twist, queue_size=10) for name in self.robot_name}
        # print("cmd_vel_pub", self.cmd_vel_pub)
        self.goal_pt_sub = {name: rospy.Subscriber(f'{name}/task_goalpoint', Point, queue_size=10) for name in self.robot_name}


        self.map_merge_data = OccupancyGrid()


        ############## map completion logic needs to be fixed ######################
        self.map1_free_num = 100000000.0
        self.target_explored_region_rate = 0.8
        ############################################################################

        # set tf & get position
        for name in self.robot_name:
            self.set_tf(name)
            self.position[name],self.rotation[name] = self.get_odom(name)
            time.sleep(1)


        # crush default value
        self.crash_indicator = 0

        # observation_space and action_space
        self.state_num = 28                   # samples in laser scan data + 4  # for half = 180 samples # for full = 360
        self.action_num = 2                     # linear vel, angular vel
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)

        self.laser_reward = 0

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)


    def get_init_info_node(self):
        for name in self.robot_name:
            
            laser_ig = InfoGetter()
            rospy.Subscriber(name+'/scan', LaserScan, laser_ig)
            
            laser_msg = laser_ig.get_msg()
            self.laser_msg_range_max = laser_msg.range_max
            laser_values = laser_msg.ranges
            
            option_target_point = []
            
            for j in range(len(laser_values)):
                if laser_values[j] == np.inf:
                    theta = self.rotation[name] + j * (laser_msg.angle_increment) + (math.pi/2 - laser_msg.angle_max)
                    option_target_point_x = self.position[name].x + (self.laser_msg_range_max * math.sin(theta) )
                    option_target_point_y = self.position[name].y - (self.laser_msg_range_max * math.cos(theta) )
                    
                    option_target_point.append([option_target_point_x,option_target_point_y])
            
            # print(option_target_point)
            if len(option_target_point) == 0:
                option_target_point.append([0,0])
            option_target_point = self.voronoi_select_point(name,option_target_point)
            self.next_target_node[name] = self.get_min_Omega_distance_point(name,option_target_point)
            # self.next_target_node[name] = random.choice(option_target_point)
            self.record_info_node[name].append(self.next_target_node[name])

    def reset(self):
        self.laser_crashed_value = {name: False for name in self.robot_name}
        self.rate.sleep()
        self.crash_indicator = 0
        current_time = rospy.Time.now()

        rospy.wait_for_service('/gazebo/set_model_state')

        # Stop all robots
        for name in self.robot_name:
            cmd_vel_pub = self.cmd_vel_pub[name]
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            cmd_vel_pub.publish(self.move_cmd)
            rospy.sleep(0.1)  # Use rospy.sleep for better ROS timing
            cmd_vel_pub.publish(self.move_cmd)
            self.rate.sleep()

        # Location initialization
        state_msgs = []
        # small warehouse
        x_positions = [-5.4,-5,-2,-13.5,-1,-3,2]  
        y_positions = [-5.4,0,3,7.49,4,2,-4]
        # big office/warehouse   
        # x_positions = [5,-11,-3,-5,6,6,-15]  
        # y_positions = [2,5,-7,-12,14.5,-17,-20]

        for i, name in enumerate(self.robot_name):
            state_msg = ModelState()
            state_msg.model_name = name

            # Assign x and y positions dynamically by cycling through the lists
            state_msg.pose.position.x = x_positions[i % len(x_positions)]
            state_msg.pose.position.y = y_positions[i % len(y_positions)]
            state_msg.pose.position.z = 0

            # Orientation
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0

            # Twist
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            state_msg.twist.angular.x = 0.0
            state_msg.twist.angular.y = 0.0
            state_msg.twist.angular.z = 0.0

            state_msgs.append(state_msg)

        rospy.wait_for_service('/gazebo/reset_simulation')

        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            for state_msg in state_msgs:
                set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # Stop all robots again after resetting positions
        for name in self.robot_name:
            cmd_vel_pub = self.cmd_vel_pub[name]
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            cmd_vel_pub.publish(self.move_cmd)
            rospy.sleep(0.1)
            cmd_vel_pub.publish(self.move_cmd)
            self.rate.sleep()

        # Get positions of robots
        for name in self.robot_name:
            self.base_frame = name + '/base_footprint'
            self.position[name], self.rotation[name] = self.get_odom(name)
            rospy.sleep(0.1)
            self.record_info_node[name].append([self.position[name].x, self.position[name].y])

        # Get target nodes
        self.get_init_info_node()
        for name in self.robot_name:
            print(name, ":target position", self.next_target_node[name][0], self.next_target_node[name][1])

        # Map data
        free_num = 0

        self.map_merge_data.data = (-1,) * 2560000 ## 1600 * 1600(width and height of map) just to reset the map merged data 

        map_pub = rospy.Publisher('/merged_map', OccupancyGrid, queue_size=10)
        map_pub.publish(self.map_merge_data)
        self.rate.sleep()
        rospy.sleep(1)
        for data in self.map_merge_data.data:
            if data == 0:
                free_num += 1
        # print("free_num", free_num)

        initial_state = np.ones(self.state_num)
        initial_state[self.state_num - 1] = 0
        initial_state[self.state_num - 2] = 0
        initial_state[self.state_num - 3] = 0
        initial_state[self.state_num - 4] = 0

        self.rate.sleep()
        initial_state = [initial_state] * len(self.robot_name)  # Dynamically adjust for the number of robots
        return initial_state


    def set_tf(self,robot_name):
        try:
            self.tf_listener.waitForTransform(robot_name+self.odom_frame, robot_name+'/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = robot_name + '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(robot_name+ self.odom_frame, robot_name+'/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = robot_name + '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")


    def get_odom(self,robot_name):
        try:
            # (trans, rot) = self.tf_listener.lookupTransform("/map", self.base_frame, rospy.Time(0))
            (trans, rot) = self.tf_listener.lookupTransform(robot_name+ self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans), rotation[2]


    def line_distance(self,position0_x,position0_y,position1_x,position1_y):
        return math.sqrt((position0_x - position1_x)**2 + (position0_y - position1_y)**2)
    

    def num_robot_site(self,robot_name):
        option_site = []
        for name in self.robot_name:
            if robot_name == name:
                pass
            else:
                if self.line_distance(self.position[robot_name].x,self.position[robot_name].y,self.position[name].x,self.position[name].y) <= self.communication_max_range:
                    option_site.append(name)
        #print(option_site)
        return option_site


    # select new option_target_point through voronoi algorithm
    def voronoi_select_point(self,robot_name,option_target_point):
        option_site = self.num_robot_site(robot_name)
        voronoi_option_target_point = []
        if len(option_site)==0:
            return option_target_point
        for i in range(len(option_target_point)):
            j=0
            for name in option_site:
                distance = self.line_distance(self.position[name].x,self.position[name].y,option_target_point[i][0],option_target_point[i][1])
                if distance > self.laser_msg_range_max:
                    j+=1
                if distance < self.laser_msg_range_max:
                    pass
                if j== len(option_site):
                    voronoi_option_target_point.append(option_target_point[i])
        return voronoi_option_target_point
    

    def avoid_repeat_select_point(self,robot_name,option_target_point):
        avoid_repeat_option_target_point = []
        for i in range(len(option_target_point)):
            for j in range(len(self.record_info_node[robot_name])):
                if self.line_distance(option_target_point[i][0],option_target_point[i][1],self.record_info_node[robot_name][j][0],self.record_info_node[robot_name][j][1])>0.1:
                    avoid_repeat_option_target_point.append(option_target_point[i])
        return avoid_repeat_option_target_point
    

    def distance_other_point(self,robot_name,option_target_point,i):
        distance = 0.0
        for name in self.robot_name:
            if name != robot_name:
                if self.line_distance(self.position[robot_name].x,self.position[robot_name].y,self.position[name].x,self.position[name].y) <=self.communication_max_range:
                    distance += self.line_distance(self.position[name].x,self.position[name].y,option_target_point[i][0],option_target_point[i][1])
        return distance
    

    def get_min_Omega_distance_point(self,robot_name,option_target_point):
        Omega = 0 # distance of d_ik and phi_ik
        min_Omega = np.inf
        index = 0
        for i in range(len(option_target_point)):
            Omega = 0.2*(self.line_distance(self.record_info_node[robot_name][0][0],self.record_info_node[robot_name][0][1],option_target_point[i][0],option_target_point[i][1])) \
                     + 0.6*(self.line_distance(self.record_info_node[robot_name][-1][0], self.record_info_node[robot_name][-1][1],option_target_point[i][0],option_target_point[i][1])) \
                        -0.2*(self.distance_other_point(robot_name,option_target_point,i))
            if Omega < min_Omega:
                min_Omega = Omega
                index =  i

        return option_target_point[index]
    

    def get_record_next_info_node(self,robot_name,option_target_point):
        if(self.arr_info_node[robot_name] == True):
            option_target_point = self.voronoi_select_point(robot_name,option_target_point) # Further select the next point through the Voronoi algorithm
            option_target_point = self.avoid_repeat_select_point(robot_name,option_target_point)
            if len(option_target_point) == 0:
                return False
            else:
                self.next_target_node[robot_name] = self.get_min_Omega_distance_point(robot_name,option_target_point)
                print(robot_name,":target position",self.next_target_node[robot_name][0],self.next_target_node[robot_name][1])
                self.record_info_node[robot_name].append(self.next_target_node[robot_name])
                self.arr_info_node[robot_name] = False
                return True
            

    def map_data_handle(self):
        free_num=0
        explored_region_rate =0.0
        for data in self.map_merge_data.data:
            if data == 0:
                free_num += 1
        explored_region_rate = free_num/self.map1_free_num
        # print(explored_region_rate)
        if explored_region_rate >= self.target_explored_region_rate:
            self.done = True


    def game_step(self, robot_name, time_step=0.1, linear_x=0.8, angular_z=0.3):
        cmd_vel_pub =  self.cmd_vel_pub[robot_name]
        # path_pub =  self.path_pub[robot_name]

        map_ig = InfoGetter()
        rospy.Subscriber('/merged_map',OccupancyGrid,map_ig)
        map_msg = map_ig.get_msg()
        self.map_merge_data.data = map_msg.data

        self.map_data_handle()
       
        start_time = time.time()
        current_time = rospy.Time.now()
        record_time = start_time
        record_time_step = 0
        self.move_cmd.linear.x = linear_x*0.26
        self.move_cmd.angular.z = angular_z
        self.rate.sleep()

        self.base_frame = robot_name + '/base_footprint'

        self.position[robot_name], self.rotation[robot_name]= self.get_odom(robot_name)
        turtlebot_x_previous = self.position[robot_name].x
        turtlebot_y_previous = self.position[robot_name].y

        while (record_time_step < time_step) and (self.crash_indicator==0):
            cmd_vel_pub.publish(self.move_cmd)
            self.rate.sleep()
            record_time = time.time()
            record_time_step = record_time - start_time

        self.position[robot_name], self.rotation[robot_name]= self.get_odom(robot_name)
        turtlebot_x = self.position[robot_name].x
        turtlebot_y = self.position[robot_name].y
        turtlebot_z = self.rotation[robot_name]   # yaw for getting theta (not z)

        angle_turtlebot = self.rotation[robot_name]


        target_x = self.next_target_node[robot_name][0]
        target_y = self.next_target_node[robot_name][1]
        if self.line_distance(turtlebot_x,turtlebot_y,target_x,target_y)<1:
            self.arr_info_node[robot_name] = True
        else:
            self.arr_info_node[robot_name] = False

        
        # get list of optional target point
        #self.arr_info_node[robot_name] = True
        laser_ig = InfoGetter()
        rospy.Subscriber(robot_name+'/scan', LaserScan, laser_ig)
        laser_msg = laser_ig.get_msg()
        laser_values = laser_msg.ranges

        for i in range(len(laser_values)):
            if (laser_values[i] < 0.12):
                #self.reset()
                break
        #self.arr_info_node[robot_name] = True
        if self.arr_info_node[robot_name] == True:
            if not(self.mode):
                option_target_point = []
                theta= 0
                for i in range(len(laser_values)):
                    if laser_values[i] == np.inf:
                        theta = i*laser_msg.angle_increment + turtlebot_z + (math.pi/2-laser_msg.angle_max)
                        option_target_point_x = turtlebot_x + (self.laser_msg_range_max * math.sin(theta))
                        option_target_point_y = turtlebot_y - (self.laser_msg_range_max * math.cos(theta))
                        option_target_point.append([option_target_point_x,option_target_point_y])
            elif self.mode == 1:
                option_target_point = []
                goal_point = self.goal_pt_sub[robot_name]
                option_target_point.append([goal_point.x, goal_point.y])
            if len(option_target_point):
                self.get_record_next_info_node(robot_name,option_target_point)
            else:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                while (record_time_step < time_step):
                    cmd_vel_pub.publish(self.move_cmd)
                    self.rate.sleep()
                    record_time = time.time()
                    record_time_step = record_time - start_time
                self.laser_crashed_value[robot_name] =1 


        # make input, angle between the turtlebot and the target
        angle_turtlebot_target = atan2(target_y - turtlebot_y, target_x- turtlebot_x)

        if angle_turtlebot < 0:
            angle_turtlebot = angle_turtlebot + 2*math.pi

        if angle_turtlebot_target < 0:
            angle_turtlebot_target = angle_turtlebot_target + 2*math.pi


        angle_diff = angle_turtlebot_target - angle_turtlebot
        if angle_diff < -math.pi:
            angle_diff = angle_diff + 2*math.pi
        if angle_diff > math.pi:
            angle_diff = angle_diff - 2*math.pi



        # prepare the normalized laser value and check if it is crash

        normalized_laser = list(laser_values)
        for i in range(len(normalized_laser)):
            if normalized_laser[i] == np.inf:
                normalized_laser[i] = 1.0
            else:
                normalized_laser[i] = normalized_laser[i]/self.laser_msg_range_max

        current_distance_turtlebot_target = math.sqrt((target_x - turtlebot_x)**2 + (target_y - turtlebot_y)**2)

        state = np.append(normalized_laser, current_distance_turtlebot_target)
        state = np.append(state, angle_diff)
        state = np.append(state, linear_x*0.26)
        state = np.append(state, angular_z)

        state = state.reshape(1, self.state_num)

        return state



if __name__ == '__main__':
    try:
        sess = tensorflow.Session()
        K.set_session(sess)

        game_state = GameState()
        game_state.reset()
        # for i in range(10):
        #     print(game_state.position[game_state.robot_name[0]])
        #     game_state.game_step(game_state.robot_name[0],time_step=0.1,linear_x=0,angular_z=0)
        #     print(game_state.position[game_state.robot_name[0]])
        #     game_state.game_step(game_state.robot_name[0],time_step=0.01,linear_x=0.0,angular_z=0)
    except rospy.ROSInterruptException:
        pass