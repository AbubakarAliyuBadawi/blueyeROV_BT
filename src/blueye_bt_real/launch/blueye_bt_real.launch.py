from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rov_bt_pkg_dir = get_package_share_directory('blueye_bt_real')
    
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    behavior_tree_path = os.path.join(bt_dir, 'Mission2.xml')
    
    if not os.path.exists(behavior_tree_path):
        raise FileNotFoundError(f"Behavior tree file not found: {behavior_tree_path}")

    blueye_mission_node_real= Node(
        package='blueye_bt_real',
        executable='blueye_bt_real',
        name='blueye_mission_real',
        output='screen',
        parameters=[{
            'behavior_tree_path': behavior_tree_path,
            'bt_directory': bt_dir
        }]
    )
    
    ld = LaunchDescription([
        blueye_mission_node_real,
    ])
    
    return ld