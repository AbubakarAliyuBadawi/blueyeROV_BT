from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    battery_management_node = Node(
        package='blueye_bt',
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
    
    dock_distance_node = Node(
        package='blueye_bt',
        executable='dock_distance_calc',
        name='dock_distance_calc',
        output='screen'
    )
    
    battery_percentage_node = Node(
        package='blueye_bt',
        executable='battery_percentage_node',
        name='battery_percentage_node',
        output='screen'
    )
    
    battery_node = Node(
        package='gz_battery',
        executable='gz_battery_node',
        name='gz_battery',
        output='screen'
    )
    
    ld = LaunchDescription([
        battery_management_node,
        battery_percentage_node,
        battery_node,
        dock_distance_node,
    ])
    
    return ld