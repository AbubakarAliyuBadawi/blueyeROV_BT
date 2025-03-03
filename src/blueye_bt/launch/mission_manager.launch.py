from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rov_bt_pkg_dir = get_package_share_directory('blueye_bt')
    
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    pipeline_mission_file = os.path.join(bt_dir, 'PipeLineInspection.xml')
    lawnmower_mission_file = os.path.join(bt_dir, 'lawnmower.xml')
    
    mission_manager_node = Node(
        package='blueye_bt',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'pipeline_mission_file': pipeline_mission_file,
            'lawnmower_mission_file': lawnmower_mission_file,
            'station_keeping_duration': 20
        }]
    )

    return LaunchDescription([
        mission_manager_node
    ])