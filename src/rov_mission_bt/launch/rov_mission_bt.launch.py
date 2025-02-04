from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    rov_bt_pkg_dir = get_package_share_directory('rov_mission_bt')
    
    # Create the behavior tree path
    behavior_tree_path = os.path.join(rov_bt_pkg_dir, 'behavior_trees', 'dock_undock_mission.xml')
    
    # Create the new BT-based rov_mission node
    rov_mission_node = Node(
        package='rov_mission_bt',
        executable='rov_mission_bt', 
        name='rov_mission',
        output='screen',
        parameters=[{
            'behavior_tree_path': behavior_tree_path
        }]
    )

    # Create and return LaunchDescription with the node
    ld = LaunchDescription()
    ld.add_action(rov_mission_node)  # Add the node to the launch description
    return ld