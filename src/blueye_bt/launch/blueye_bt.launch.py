from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rov_bt_pkg_dir = get_package_share_directory('blueye_bt')
    
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    behavior_tree_path = os.path.join(bt_dir, 'MissionControl.xml')
    
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

    
#     waypoint_obstacle_avoidance_node = Node(
#     package='blueye_bt',
#     executable='waypoint_obstacle_avoidance',
#     name='waypoint_obstacle_avoidance',
#     parameters=[{
#         'safety_distance': 2.0,
#         'deviation_distance': 6.0,
#         'deviation_velocity': 0.1,
#         'obstacle_avoidance_enabled': True,
#         'waypoint_resume_delay': 0.1
#     }],
#     output='screen'
# )
    
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

    ld = LaunchDescription([
        blueye_mission_node,
        battery_management_node,
        dock_distance_node,
        altitude_controller_node,
        # waypoint_obstacle_avoidance_node,
        battery_node,
        sonar_node,
    ])
    
    return ld