from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='blueye_mission',
        #     executable='rov_mission',
        #     name='rov_mission',
        #     output='screen'
        # ),
        Node(
            package='blueye_mission',
            executable='dock_distance_calc',
            name='dock_distance_calc',
            output='screen'
        ),
        Node(
            package='blueye_mission',
            executable='battery_management',
            name='battery_management',
            parameters=[{
                'window_size': 300.0,
                'safety_margin': 30.0,
                'update_frequency': 0.1,
                'min_samples_for_average': 10
            }],
            output='screen'
        )
    ])