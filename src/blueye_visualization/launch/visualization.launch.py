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
        # Node(
        #     package='blueye_visualization',
        #     executable='rov_visual_2d_xy',
        #     name='rov_visual_2d_xy'
        # ),
        Node(
            package='blueye_visualization',
            executable='rov_trajectory_plotter',
            name='rov_trajectory_plotter'
        ),
        # Node(
        #     package='blueye_visualization',
        #     executable='rov_trajectory_plotter_rl',
        #     name='rov_trajectory_plotter_rl'
        # ),
        Node(
            package='blueye_visualization',
            executable='rov_viz_matplotlib_3d',
            name='rov_viz_matplotlib_3d'
        ),
        Node(
            package='blueye_visualization',
            executable='obstacle_avoidance_visualizer',
            name='obstacle_avoidance_visualizer'
        ),
        Node(
            package='blueye_visualization',
            executable='system_health_visualizer',
            name='system_health_visualizer'
        ),
    ])
    
