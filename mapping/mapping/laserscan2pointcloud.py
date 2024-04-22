#!/usr/bin/env python3


from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

class LaserScan2PointCloudConverter(Node):
    def __init__(self):
        super().__init__("Converter")
        self.get_logger().info("Converter has been started")

        self.projector = LaserProjection()

        self.pointcloud2_publisher = self.create_publisher(
            PointCloud2, "/lidar_pc", 10
        )

        self.sense_pose_subscriber = self.create_subscription(
            LaserScan, "/postfilter_scan", self.interesting_callback, 10
        )

    def interesting_callback(self, ls: LaserScan):
        
        pc2 = self.projector.projectLaser(ls)
        self.pointcloud2_publisher.publish(pc2)
