#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        # Battery Monitor Node
        Node(
            package='terrain_mapping_drone_control',
            executable='battery_monitor.py',
            name='battery_monitor',
            parameters=[{
            'use_sim_time': True,
            }],
            output='screen'
        ),

        # Cylinder Landing Node
        Node(
            package='terrain_mapping_drone_control',
            executable='cylinder_landing_node.py',
            name='cylinder_landing_node',
            parameters=[{
            'use_sim_time': True,
            }],
            output='screen'
        ),

        

        # Aruco Tracker
        Node(
            package='terrain_mapping_drone_control',
            executable='aruco_tracker.py',
            name='aruco_tracker',
            parameters=[{
            'use_sim_time': True,
            }],
            output='screen'
        ),

        # Geometry Tracker
        Node(
           package='terrain_mapping_drone_control',
           executable='geometry_tracker.py',
           name='geometry_tracker',
           parameters=[{
           'use_sim_time': True,
           }],
           output='screen'
        )

    ])