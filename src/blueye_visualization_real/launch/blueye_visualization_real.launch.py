#!/usr/bin/env python3
"""
Launch file for Blueye Visualization package.
This launches the telemetry node and both 2D and 3D visualization nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    drone_ip_arg = DeclareLaunchArgument(
        'drone_ip',
        default_value='192.168.1.101',
        description='IP address of the Blueye ROV'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish telemetry data (Hz)'
    )
    
    # Access the parameters
    drone_ip = LaunchConfiguration('drone_ip')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Create the telemetry node
    telemetry_node = Node(
        package='blueye_visualization_real',
        executable='blueye_telemetry.py',
        name='blueye_telemetry_node',
        parameters=[
            {'drone_ip': drone_ip},
            {'publish_rate': publish_rate}
        ],
        output='screen'
    )
    
    # Create the 2D visualization node
    visualization_2d_node = Node(
        package='blueye_visualization_real',
        executable='blueye_visulization_2d_real.py',
        name='blueye_visualization_2d_node',
        output='screen'
    )
    
    # Create the 3D visualization node
    visualization_3d_node = Node(
        package='blueye_visualization_real',
        executable='blueye_visulization_3d_real.py',
        name='blueye_visualization_3d_node',
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        drone_ip_arg,
        publish_rate_arg,
        telemetry_node,
        visualization_2d_node,
        visualization_3d_node
    ])