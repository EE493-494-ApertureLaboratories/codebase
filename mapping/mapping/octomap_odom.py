#!/usr/bin/env python3


from rclpy.node import Node

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

import tf_transformations
from geometry_msgs.msg import Quaternion

class OctoMapOdom(Node):
    def __init__(self):
        super().__init__("OctoMapOdom")
        self.get_logger().info("OctoMapOdom has been started")

        self.odom_pub = self.create_publisher(
            Odometry, "/odom",  10
            )

        self.global_odom = self.create_subscription(
            Odometry, "/global_odom", self.global_odom_cb, 10
        )
        
        self.local_odom = self.create_subscription(
            Odometry, "/partial_odom", self.local_odom_cb, 10
        )

        self.joy_subscriber = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )

        self.timer = self.create_timer(0.032, self.publish_octomap_odom)

        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.octomap_x = 0.0
        self.octomap_y = 0.0
        self.octomap_quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.global_x = 0.0
        self.global_y = 0.0
        self.global_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)

        self.local_x = 0.0
        self.local_y = 0.0
        self.local_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)

    def joy_callback(self, joy: Joy):
        
        if joy.buttons[5]:
            self.octomap_x = self.global_x + self.local_x
            self.octomap_y = self.global_y + self.local_y
            
            q_glob = [self.global_quat.w, self.global_quat.x, self.global_quat.y, self.global_quat.z]
            q_local = [self.local_quat.w, self.local_quat.x, self.local_quat.y, self.local_quat.z]

            q =  tf_transformations.quaternion_multiply(q_glob, q_local)

            self.octomap_quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        elif joy.buttons[4]:
            octomap_x = self.global_x
            octomap_y = self.global_y
            octomap_quat = self.global_quat
        
    def global_odom_cb(self, global_odom: Odometry):
        self.global_x = global_odom.pose.pose.position.x
        self.global_y = global_odom.pose.pose.position.y
        self.global_quat = global_odom.pose.pose.orientation

    def local_odom_cb(self, local_odom: Odometry):
        self.local_x = local_odom.pose.pose.position.x
        self.local_y = local_odom.pose.pose.position.y
        self.local_quat = local_odom.pose.pose.orientation
    
    def publish_octomap_odom(self):
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.octomap_x
        msg.pose.pose.position.y = self.octomap_y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation = self.octomap_quat

        self.odom_pub.publish(msg)

        
        