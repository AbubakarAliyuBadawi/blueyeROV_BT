from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('rov_mission_bt')
    
    # Declare launch arguments
    mission_name_arg = DeclareLaunchArgument(
        'mission_name',
        default_value='dock_undock_mission',
        description='Name of the mission to execute'
    )

    # Default config
    default_config = os.path.join(pkg_dir, 'config', 'mission_params.yaml')
    
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to config file'
    )

    # Create node
    node = Node(
        package='rov_mission_bt',
        executable='rov_mission_bt_node',
        name='rov_mission_bt',
        parameters=[
            LaunchConfiguration('config_file'),
            {'mission_name': LaunchConfiguration('mission_name')},
            {'mission_directory': os.path.join(pkg_dir, 'missions', 'xml')}
        ],
        output='screen'
    )

    return LaunchDescription([
        mission_name_arg,
        config_arg,
        node
    ])