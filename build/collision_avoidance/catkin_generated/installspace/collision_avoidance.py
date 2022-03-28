#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import signal, sys
import numpy as np
import math
import cv2
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import ros_numpy
import scipy.spatial as spatial

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Bool
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from tf.transformations import euler_from_quaternion

from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist

class CrashDetector():
    def __init__(self):
        self.data = LaserScan()
        self.lidar = np.ones(1081)*1000
        self.xy_array = []
        self.rate = 40
        self.lp = lg.LaserProjection()
        
        #receiving topic: /terrasentia/scan_processed
        rospy.Subscriber('/terrasentia/scan_processed', LaserScan, self.lidar_callback)
        plt.ion()
        plt.show()       
        rate = rospy.Rate(self.rate) # Run at 50Hz

        while not rospy.is_shutdown(): 
            self.collision_detector()
            rate.sleep()
    
    def lidar_callback(self, data):
        self.data = data
        
    def collision_detector(self):
        pc2_msg = self.lp.projectLaser(self.data)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)    
        self.xy_array = []
        
        #creates array with only x, y points.
        #TODO: use z coordinates as a part of lidar normalization taking into accound the pitch and roll of the robot
        for point in xyz_array:
            self.xy_array.append(point[0:2])
        
        if (len(self.xy_array) is not 0):
            self.xy_array = np.array(self.xy_array)
            self.plot_points()

    #plots the 2d point cloud        
    def plot_points(self):
        if (len(self.xy_array) is not 0):
            plt.clf()
            plt.scatter(*zip(*self.xy_array), s=10)
            self.search_nearest_neighbor() 
            self.place_ego()
            plt.xlim([-6, 6])
            plt.ylim([-6, 6]) 
            plt.pause(0.00000000001)

    #searches for closest neighbor within certain distance of robot
    def search_nearest_neighbor(self):
        dist_threshold = 2 #objects within dist_threshold will be flagged as objects to avoid.
        point_tree = spatial.cKDTree(self.xy_array)
        neighbors = point_tree.data[point_tree.query_ball_point([0, 0], dist_threshold)]
        if (len(neighbors) is not 0):
            plt.scatter(*zip(*neighbors), c='red')
            print("obstacle found in radius")
    #places robot into frame 
    def place_ego(self):
        rectangle = plt.Rectangle((-0.2,-0.2), 0.4, 0.4, fc='white',ec="green")
        plt.gca().add_patch(rectangle)

