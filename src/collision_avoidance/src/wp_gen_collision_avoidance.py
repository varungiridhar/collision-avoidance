#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy
import tf
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32

from math_utils import euler_from_quaternion
class WaypointGen():
    def __init__(self):
        rospy.set_param('frame_id', 'base_link')        
        rospy.set_param('num_of_wps', 10)
        rospy.set_param('wp_topic_name', 'wps')
        rospy.set_param('input_odom_topic_name', '/terrasentia/ekf')
        rospy.set_param('distance_between_wps', 0.4)
        
        self.heading = Float32()
        self._run_step = False;
        self._wps = Path()
        
        self._wps.header.frame_id = rospy.get_param('frame_id')
        #not sure if I need the resize function
        #self._wps.poses.resize(rospy.get_param('_num_of_wps'))
        
        self._pub_wps = rospy.Publisher(rospy.get_param('wp_topic_name'), Path, queue_size=1)
        rospy.Subscriber(rospy.get_param('input_odom_topic_name'), Odometry, self.heading_callback, queue_size=1)

    def step(self):
        if (self._run_step is False): 
            return 0
        self.generate_waypoints()
        self._pub_wps.publish(self._wps)
        self._run_step = False

    def generate_waypoints(self):
        #TODO: add decision making logic that relies on collision_avoidance.py
        self.generate_waypoints_obstacle_avoidance()
        return 0

    def generate_waypoints_obstacle_avoidance(self):
        #clear waypoints
        del self._wps.poses[:]
         
        #round about manoever
        #TODO: create an actual motion plan that produces smooth points
        self.generate_straight_wps_xyzh(0, 0, 0, self.heading, rospy.get_param('num_of_wps')) 
        #straight waypoint manoever
        #TODO: migrate this to its own method
        
        
        #TODO: come back to orginial path
    
    def generate_straight_wps_xyzh(self, x, y, z, heading, num_of_wps):
        
        p = Point()
        
        for i in range(0, rospy.get_param('num_of_wps')/2):
            pose = PoseStamped()
            p.y = y
            p.x = x+0.4*i
            p.z = z
            p = self.rotate(p, -heading)
            pose.pose.position = p
            self._wps.poses.append(pose)
        self._wps.poses.pop()
    def rotate(self, i, heading):
        p = Point()
        print(heading)
        p.x = i.x*math.cos(heading) - i.y*math.sin(heading)
        p.y = i.x*math.sin(heading) + i.y*math.cos(heading)
        return p 

    def heading_callback(self, data):
        #convert quaternion to yaw
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        _, _, yaw_z = euler_from_quaternion(x, y, z, w) 
        self.heading = yaw_z 
        self._run_step = True 
