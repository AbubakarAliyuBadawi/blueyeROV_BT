from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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
    
    drone_ip = LaunchConfiguration('drone_ip')
    publish_rate = LaunchConfiguration('publish_rate')
    
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
    
    rov_bt_pkg_dir = get_package_share_directory('blueye_bt_real')
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    behavior_tree_path = os.path.join(bt_dir, 'Mission.xml') 
    
    if not os.path.exists(behavior_tree_path):
        raise FileNotFoundError(f"Behavior tree file not found: {behavior_tree_path}")

    blueye_mission_node_real = Node(
        package='blueye_bt_real',
        executable='blueye_bt_real',
        name='blueye_mission_real',
        output='screen',
        parameters=[{
            'behavior_tree_path': behavior_tree_path,
            'bt_directory': bt_dir
        }]
    )
    
    delayed_bt_node = TimerAction(
        period=5.0,
        actions=[blueye_mission_node_real]
    )
    
    return LaunchDescription([
        drone_ip_arg,
        publish_rate_arg,
        telemetry_node,  
        delayed_bt_node,  
    ])