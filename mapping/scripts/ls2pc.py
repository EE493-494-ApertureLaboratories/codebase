#!/usr/bin/env python3

import rclpy
from mapping.laserscan2pointcloud import LaserScan2PointCloudConverter


def main(args=None):
    rclpy.init(args=args)
    node = Converter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
