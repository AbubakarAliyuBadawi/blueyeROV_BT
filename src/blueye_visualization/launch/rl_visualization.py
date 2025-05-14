#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blueye_visualization',
            executable='rov_enhanced_2d_trajectory',
            name='rov_enhanced_2d_trajectory',
        ),
        Node(
            package='blueye_visualization',
            executable='rov_enhanced_3d_trajectory',
            name='rov_enhanced_3d_trajectory',
        ),
        Node(
            package='blueye_visualization',
            executable='system_health_visualizer',
            name='system_health_visualizer'
        ),
    ])
    
