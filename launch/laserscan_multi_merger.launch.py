#!/usr/bin/env python3
"""Launch file for laserscan_multi_merger node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for laserscan_multi_merger."""
    return LaunchDescription([
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            output='screen',
            parameters=[{
                'destination_frame': 'cart_frame',
                'cloud_destination_topic': '/merged_cloud',
                'scan_destination_topic': '/scan_multi',
                'laserscan_topics': '/scansx /scandx',
                'angle_min': -2.0,
                'angle_max': 2.0,
                'angle_increment': 0.0058,
                'time_increment': 0.0,
                'scan_time': 0.0333333,
                'range_min': 0.30,
                'range_max': 50.0,
            }],
        ),
    ])
