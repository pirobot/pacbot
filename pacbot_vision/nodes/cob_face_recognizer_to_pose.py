#!/usr/bin/env python

"""
    face_recognizer_to_pose.py - Version 1.0 2016-06-22
    
    Republish a cob_perception_msgs/DetectionArray message as a PoseStamped message
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2016 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from geometry_msgs.msg import PoseStamped
from cob_perception_msgs.msg import DetectionArray
from tf.transformations import quaternion_from_euler

class PubPose():
    def __init__(self):
        rospy.init_node('head_position_to_pose')

        # Subscribe to the head_positions topic
        rospy.Subscriber('input_topic', DetectionArray, self.pub_pose_message)
        
        # The PoseStamped message pubtlisher
        self.pose_pub = rospy.Publisher('output_topic', PoseStamped, queue_size=5)
        
        rospy.loginfo("Re-publishing COB Face Positions message as PoseStamped message")
        
    def pub_pose_message(self, msg):
        # Pick off the first detection
        try:
            detection = msg.detections[0]
            target_pose = detection.pose
        except:
            return
        
        head_arrow = quaternion_from_euler(0, -1.57, 0)
        pose = PoseStamped()
        target_pose.pose.orientation.x = head_arrow[0]
        target_pose.pose.orientation.y = head_arrow[1]
        target_pose.pose.orientation.z = head_arrow[2]
        target_pose.pose.orientation.w = head_arrow[3]
                
        self.pose_pub.publish(target_pose)
 
if __name__ == '__main__':
    try:
        target = PubPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
