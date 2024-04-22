#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np 
 


class LaserScanFilter(Node):

    def __init__(self):
        super().__init__('laser_scan_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.filter,
            10)

        self.dist_arr = np.zeros([1080])
        
        self.publisher_ = self.create_publisher(LaserScan, '/postfilter_scan', 10)


    def filter(self, prefiltered_scan):

        self.dist_arr = np.array(prefiltered_scan.ranges)
        self.dist_arr[841:1000] = np.inf           # angle filter 
        # self.dist_arr[self.dist_arr>3.2] = np.inf # distance filter 
        
        prefiltered_scan.ranges = self.dist_arr.tolist()
        
        self.publisher_.publish(prefiltered_scan)
    
