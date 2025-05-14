#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blueye_visualization',
            executable='2d_rl',
            name='rl_2d',
        ),
        Node(
            package='blueye_visualization',
            executable='3d_rl',
            name='rl_3d'
        ),
        Node(
            package='blueye_visualization',
            executable='rov_trajectory_plotter_rl',
            name='rov_trajectory_plotter_rl'
        ),
        Node(
            package='blueye_visualization',
            executable='system_health_visualizer',
            name='system_health_visualizer'
        ),
    ])
    
