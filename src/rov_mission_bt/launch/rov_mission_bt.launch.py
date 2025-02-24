from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    rov_bt_pkg_dir = get_package_share_directory('rov_mission_bt')
    
    # Create the behavior tree path
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    behavior_tree_path = os.path.join(bt_dir, 'mission.xml')
    
    # Verify file exists
    if not os.path.exists(behavior_tree_path):
        raise FileNotFoundError(f"Behavior tree file not found: {behavior_tree_path}")

    # Create the new BT-based rov_mission node
    rov_mission_node = Node(
        package='rov_mission_bt',
        executable='rov_mission_bt',
        name='rov_mission',
        output='screen',
        parameters=[{
            'behavior_tree_path': behavior_tree_path,
            'bt_directory': bt_dir
        }]
    )

    # Battery management nodes
    battery_management_node = Node(
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
    )

    dock_distance_node = Node(
        package='gz_battery_management',
        executable='dock_distance_calc',
        name='dock_distance_calc',
        output='screen'
    )

    battery_node = Node(
        package='gz_battery',
        executable='gz_battery_node',
        name='gz_battery',
        output='screen'
    )

    sonar_node = Node(
        package='gz_sonar',
        executable='gz_sonar',
        name='gz_sonar',
        output='screen'
    )

    # Create and return LaunchDescription with all nodes
    ld = LaunchDescription([
        rov_mission_node,
        battery_management_node,
        dock_distance_node,
        battery_node,
        sonar_node
    ])
    
    return ld