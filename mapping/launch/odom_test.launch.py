#!/usr/bin/env python3

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Display Scans in rviz2?'), 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mapping'),
                    'launch',
                    'lidar_test.launch.py'
                ])
            ]),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
            }.items()
        ),

        Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/partial_odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'partial_odom',
                    'init_pose_from_topic' : '',
                    'freq' : 10.0}],
            ),

    ])