#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # lunar_mono_driver.py
        Node(
            package='ros2_orb_slam3',
            executable='down_mono_driver.py',
            name='down_mono_py_node',
            output='screen',
            parameters=[
                {'settings_name': 'down_mono'},
                {'image_topic': '/drone/down_mono'},
            ]
        ),
        # mono_node_cpp
        Node(
            package='ros2_orb_slam3',
            executable='mono_node_cpp',
            name='mono_slam_cpp',
            output='screen',
            parameters=[
                {'node_name.arg': 'mono_slam_cpp'}
            ]
        )
    ])