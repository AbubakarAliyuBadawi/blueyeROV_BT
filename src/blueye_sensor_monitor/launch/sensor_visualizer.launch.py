from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blueye_sensor_monitor',  # Your correct package name
            executable='sensor_visualizer_node',  
            name='sensor_visualizer',
            output='screen'
        )
    ])