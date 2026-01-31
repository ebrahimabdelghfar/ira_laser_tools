#!/usr/bin/env python3
"""Launch file for laserscan_virtualizer node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for laserscan_virtualizer."""
    return LaunchDescription([
        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ira_static_broadcaster1',
            arguments=['0', '0', '0', '0', '0.3', '0', 'laser_frame', 'scansx'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ira_static_broadcaster2',
            arguments=['0', '0', '0', '0', '0.0', '0', 'laser_frame', 'scandx'],
        ),
        # Laserscan virtualizer node
        Node(
            package='ira_laser_tools',
            executable='laserscan_virtualizer',
            name='laserscan_virtualizer',
            output='screen',
            parameters=[{
                'cloud_topic': '/cloud_in',
                'base_frame': 'laser_frame',
                'output_laser_topic': '/scan',
                'virtual_laser_scan': 'scansx scandx',
                'angle_min': -2.36,
                'angle_max': 2.36,
                'angle_increment': 0.0058,
                'time_increment': 0.0,
                'scan_time': 0.0333333,
                'range_min': 0.45,
                'range_max': 25.0,
            }],
        ),
    ])
