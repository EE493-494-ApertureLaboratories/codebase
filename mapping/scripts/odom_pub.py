#!/usr/bin/env python3

import rclpy
from mapping.octomap_odom.py import OctoMapOdom


def main(args=None):
    rclpy.init(args=args)
    node = OctoMapOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
