#!/usr/bin/env python

"""
    simulated_walker.py - Version 1.0 2016-06-29
    
    Publish a moving PoseStamped message and marker relative to a given reference frame.
    
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
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose
#from dynamic_reconfigure.server import Server
#import dynamic_reconfigure.client
#from rbx2_utils.cfg import Pub3DTargetConfig
from math import sin, cos, pi

class SimulatedWalker():
    def __init__(self):
        rospy.init_node('simulated_walker')
        
        # How fast should we move the target in m/s
        walker_speed = rospy.get_param('~walker_speed', 0.1)
        
        # How big is the path (e.g. circle, square) the walker will follow
        path_size = rospy.get_param('~path_size', 5.0)
        
        # The rate at which we publish target messages
        rate = rospy.get_param('~rate', 20)
        
        # The rate at which we update the target position (controls the smoothness of the motion)
        move_target_rate = rospy.get_param('~move_target_rate', 20)
        
        # Publish the target pose relative to this frame
        target_frame = rospy.get_param('~target_frame', 'map')

        # Convert the rates into ROS rates
        r = rospy.Rate(rate)
        r_move_target = 1.0 / move_target_rate
        
        # Time interval between publishing cycles
        tick = 1.0 / rate
        
        # Fire up the dynamic_reconfigure server
        #dyn_server = Server(Pub3DTargetConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        #dyn_client = dynamic_reconfigure.client.Client("pub_3d_target", timeout=60)

        # The target pose publisher
        target_pub = rospy.Publisher('target_topic', PoseStamped, queue_size=5)
        
        # The marker publisher so we can view the target in RViz
        marker_pub = rospy.Publisher('target_marker', Marker, queue_size=5)
        
        # Parameters for publishing target marker
        marker_scale = rospy.get_param('~marker_scale', 0.2)
        marker_lifetime = rospy.get_param('~marker_lifetime', 1/rate) # 0 = forever
        marker_ns = rospy.get_param('~marker_ns', 'target_point')
        marker_id = rospy.get_param('~marker_id', 0)
        marker_color = rospy.get_param('~marker_color', {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        
        # Define the marker as a green sphere
        marker = Marker()
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(marker_lifetime)
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.scale.z = marker_scale
        marker.color.r = marker_color['r']
        marker.color.g = marker_color['g']
        marker.color.b = marker_color['b']
        marker.color.a = marker_color['a']
        
        # Define the target as a PoseStamped message
        target = PoseStamped()
        target.header.frame_id = target_frame
        
        target.header.frame_id = target_frame
        target.pose.orientation.x = 0
        target.pose.orientation.y = 0
        target.pose.orientation.z = 0
        target.pose.orientation.w = 1
        
        rospy.loginfo("Publishing target point on frame " + str(target_frame))
        
        # Variables to control the target motion
        timer = 0.0
        
        # Starting position of the target
        x_start = 0.0
        y_start = 0.0
        
        target.pose.position.x = x_start
        target.pose.position.y = y_start
        target.pose.position.z = 0
        
        path_time = 0.0
        start_outer_path = False
        
        # Start the walker
        while not rospy.is_shutdown():
            # Update the target position at the appropriate intervals
            if timer == 0.0 or timer > r_move_target:
                timer = 0.0
                if target.pose.position.x < path_size and not start_outer_path:
                    target.pose.position.x = walker_speed * path_time
                    target.pose.position.y = 0.0
                else:
                    if not start_outer_path:
                        path_time = 0.0        
                        start_outer_path = True
                    target.pose.position.x = path_size * cos(path_time * walker_speed / path_size)
                    target.pose.position.y = path_size * sin(path_time * walker_speed / path_size)
                            
            now = rospy.Time.now()
                    
            target.header.stamp = now
            
            marker.header.stamp = now
            marker.header.frame_id = target.header.frame_id
            marker.pose.position = target.pose.position
            marker.pose.position.z = 0.5
            marker.id = 0

            # Publish the target
            target_pub.publish(target)

            # Publish the marker for viewing in RViz
            marker_pub.publish(marker)                       

            r.sleep()
            
            timer += tick
            path_time += tick
            
#     def dynamic_reconfigure_callback(self, config, level):
#         if self.rate != config['rate']:
#             self.rate = config['rate']
#             self.r = rospy.Rate(self.rate)
#             self.tick = 1.0 / self.rate
#             
#         if self.move_target_rate != config['move_target_rate']:
#             self.move_target_rate = config['move_target_rate']
#             self.r_move_target = 1.0 / self.move_target_rate
#             
#         if self.speed != config['speed']:
#             self.speed = config['speed']
#                     
#         return config
                   
if __name__ == '__main__':
    try:
        target = SimulatedWalker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulated walker shut down.")
