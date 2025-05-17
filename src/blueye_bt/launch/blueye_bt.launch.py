from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rov_bt_pkg_dir = get_package_share_directory('blueye_bt')
    
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    # behavior_tree_path = os.path.join(bt_dir, 'MissionControl.xml')
    # behavior_tree_path = os.path.join(bt_dir, 'TestLearningMission.xml')
    behavior_tree_path = os.path.join(bt_dir, 'TestDocking_24.xml')

    
    if not os.path.exists(behavior_tree_path):
        raise FileNotFoundError(f"Behavior tree file not found: {behavior_tree_path}")

    blueye_mission_node = Node(
        package='blueye_bt',
        executable='blueye_bt',
        name='blueye_mission',
        output='screen',
        parameters=[{
            'behavior_tree_path': behavior_tree_path,
            'bt_directory': bt_dir
        }]
    )

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
    battery_percentage_node = Node(
        package='blueye_bt',
        executable='battery_percentage_node',
        name='battery_percentage_node',
        output='screen'
    )
    
    dock_distance_node = Node(
        package='blueye_bt',
        executable='dock_distance_calc',
        name='dock_distance_calc',
        output='screen'
    )
    
    altitude_controller_node = Node(
    package='blueye_bt',
    executable='altitude_controller',
    name='altitude_controller',
    parameters=[{
        'target_altitude': 2.0,
        'altitude_control_enabled': False,
        'heave_speed': 0.3,
        'min_altitude': 0.5,
        'altitude_safety_margin': 0.2
    }],
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
    
    enhanced_visualization_node = Node(
        package='blueye_visualization',
        executable='rov_enhanced_2d_trajectory',
        name='rov_enhanced_2d_trajectory',
        output='screen'
    )
    
    ld = LaunchDescription([
        blueye_mission_node,
        # battery_management_node,
        # dock_distance_node,
        altitude_controller_node,
        # battery_percentage_node,
        # battery_node,
        # enhanced_visualization_node,
        # sonar_node,
    ])
    
    return ld