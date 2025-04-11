from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    bt_filename_arg = DeclareLaunchArgument(
        'bt_filename',
        default_value='main_mission.xml',
        description='BT XML filename'
    )
    
    drone_ip_arg = DeclareLaunchArgument(
        'drone_ip',
        default_value='192.168.1.101',
        description='Blueye drone IP address'
    )
    
    drone_timeout_arg = DeclareLaunchArgument(
        'drone_timeout',
        default_value='30',
        description='Blueye drone connection timeout in seconds'
    )
    
    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate_ms',
        default_value='100',
        description='BT tick rate in milliseconds'
    )
    
    # Create node
    bt_executor_node = Node(
        package='blueye_bt_real',
        executable='blueye_bt_executor',
        name='blueye_bt_executor',
        parameters=[{
            'bt_filename': LaunchConfiguration('bt_filename'),
            'drone_ip': LaunchConfiguration('drone_ip'),
            'drone_timeout': LaunchConfiguration('drone_timeout'),
            'tick_rate_ms': LaunchConfiguration('tick_rate_ms')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        bt_filename_arg,
        drone_ip_arg,
        drone_timeout_arg,
        tick_rate_arg,
        bt_executor_node
    ])