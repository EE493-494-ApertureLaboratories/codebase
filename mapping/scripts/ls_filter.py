#!/usr/bin/env python3

import rclpy
from mapping.laser_scan_filter import LaserScanFilter


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
