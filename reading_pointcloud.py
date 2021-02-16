#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Tue May 26 14:28:11 2020

@author: kush
"""

from sensor_msgs.msg import PointCloud2
import numpy as np
import rospy
from sensor_msgs.msg import PointField # not used in this code
import sensor_msgs.point_cloud2 as pc2

class LidarPC():
    
    def __init__(self):
        '''
        __init__ method for the LidarPC class

        Returns
        -------
        sends PointCloud2 data to the lidar_callback function as its first argument

        '''
        self.pcdata = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        
    def lidar_callback(self, msg):
        '''
        Callback function to post-process the pointcloud data coming from /velodyne_points topic
        
        Parameters
        ----------
        msg : PointCloud2
            Lidar PointCloud2 data 
        Returns
        -------
        Minimum distacne to the object in front of the Lidar in Y direction

        '''
        for xyz_i in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z","intensity","ring")):
            z_point = xyz_i[2]
            ring_number = xyz_i[4]
            if ring_number==8 and 0.0075>xyz_i[0]>-0.0075:
                y_point = np.amin(xyz_i[1])
                intensity = xyz_i[3]
            
                print ('Minimum distance to object is {}'.format(y_point))
        

if __name__ == '__main__':
    """
    this if statement makes sure that this code will only run if the file is run directly i.e. when __name__=='__main__'
    """
    rospy.init_node('lidarpc') # this will initialise the new ros node 'lidarpc'
    lidarpc = LidarPC()
    rospy.spin()
