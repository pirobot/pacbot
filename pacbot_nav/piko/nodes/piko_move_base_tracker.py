#!/usr/bin/env python

""" move_base_tracker.py - Version 1.1 2016-06-20

    Subscribe to the /target_topic to get a goal pose and move the robot to with a given distance of the target.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf2_py
import tf
from tf.transformations import quaternion_from_euler
from math import atan2, cos, sin, sqrt, copysign, pow

class Tracker():
    def __init__(self):
        rospy.init_node('move_base_tracker', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How quickly should we update the tracker
        self.rate = rospy.Rate(rospy.get_param("~rate", 1))
        
        # Linear and angular threshold differences from last goal before updating goal
        self.linear_tracking_threshold = rospy.get_param("~linear_tracking_threshold", 0.2)
        self.angular_tracking_threshold = rospy.get_param("~angular_tracking_threshold", 0.2)

        # Minimum distance to keep between robot and target
        self.min_distance = rospy.get_param("~min_distance", 0.8)
        
        # Tolerance around min distance
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.1)
        
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # Cancel any left over goals from previous run
        self.move_base.cancel_all_goals()
        
        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Allow tf to catch up        
        rospy.sleep(2)
        
        self.last_target_map_pose = PoseStamped()
        
        self.last_goal_sent = rospy.Time.now()
        
        # Wait for the target topic to become alive
        rospy.loginfo("Waiting for target topic...")
        
        rospy.wait_for_message('target_topic', PoseStamped)
        
        rospy.wait_for_message('odom', Odometry)
        
        # Subscribe to the goal_pose topic
        rospy.Subscriber('target_topic', PoseStamped, self.track_target)
        
        self.goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size=1)
                
        rospy.loginfo("Target messages detected. Starting tracker...")
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        
    def track_target(self, msg):
        lookup_time = msg.header.stamp
        success = False
        while not success:
            try:
                # Get the target location in the map frame
                self.tf.waitForTransform("/map", msg.header.frame_id, lookup_time, rospy.Duration(3.0))
                target_in_map =  self.tf.transformPose('/map', msg)

                # Get the target location in the base frame
                self.tf.waitForTransform("/base_link", msg.header.frame_id, lookup_time, rospy.Duration(3.0))
                target_in_base = self.tf.transformPose('/base_link', msg)
                
                success = True
            except (Exception, tf2_py.ExtrapolationException, tf2_py.ConnectivityException, tf2_py.LookupException):
                rospy.sleep(0.1)
        
        # x-y coordinates of target in map frame
        x_map = target_in_map.pose.position.x
        y_map = target_in_map.pose.position.y
        
        # x-y coordinates of target in base frame
        x_base = target_in_base.pose.position.x
        y_base = target_in_base.pose.position.y
        
        # x-y coordinates of last target postion in map frame
        last_target_x_map = self.last_target_map_pose.pose.position.x
        last_target_y_map = self.last_target_map_pose.pose.position.y
        
        # Distance between current target and last target in the map frame
        distance_target_moved = sqrt(pow(x_map - last_target_x_map, 2) + pow(y_map - last_target_y_map, 2))
        
        # Distance between target and robot
        distance_to_target = sqrt(x_base * x_base + y_base * y_base)

        # Compute the yaw angle to the target in the base frame
        yaw = atan2(y_base, x_base)
        q_array = quaternion_from_euler(0, 0, yaw)
        quat = Quaternion(q_array[0], q_array[1], q_array[2], q_array[3])
                
        if (last_target_x_map == 0 and last_target_y_map == 0) or distance_target_moved > self.linear_tracking_threshold:
            self.last_target_map_pose = target_in_map
            goal_pose = Pose()
            goal_pose.position = target_in_base.pose.position

            if abs(distance_to_target - self.min_distance) > self.distance_tolerance:
                # Set the goal location to the minimum distance from the target
                goal_distance = max(0, distance_to_target - self.min_distance)
                goal_pose.position.x = goal_distance * cos(yaw)
                goal_pose.position.y = goal_distance * sin(yaw)
        else:
            return

        # Set the orientation parallel to the vector between the robot and target
        goal_pose.orientation = quat
        
        # Zero out the z component of the goal
        goal_pose.position.z = 0.0
        
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.stamp = rospy.Time.now()
        
        rospy.loginfo(goal)
        
        self.last_goal_sent = rospy.Time.now()
        
        self.move_base.send_goal(goal)
        
        self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Tracker test finished.")
