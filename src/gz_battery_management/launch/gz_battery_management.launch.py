from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gz_battery_management',
            executable='dock_distance_calc',
            name='dock_distance_calc',
            output='screen'
        ),
        Node(
            package='gz_battery_management',
            executable='battery_management',
            name='battery_management',
            parameters=[{
                'window_size': 300.0,
                'safety_margin': 30.0,
                'update_frequency': 0.1,
                'min_samples_for_average': 10
            }],
            output='screen'
        ),
        Node(
            package='gz_battery',
            executable='gz_battery_node',
            name='gz_battery',
            output='screen'
        ),
        Node(
            package='gz_sonar',
            executable='gz_sonar',
            name='gz_sonar',
            output='screen'
        )
    ])