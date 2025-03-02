import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchContext


def generate_launch_description():

    # Get shared directories
    pkg_project_vehicle_interfaces = get_package_share_directory("mundus_mir_vehicle_interfaces")
    pkg_project_simulator_launch = get_package_share_directory("mundus_mir_simulator_launch")

    # Parse robot configuration file
    robot_config_path = os.path.join(pkg_project_simulator_launch, 'config', "mundus_mir_pipeline_world", 'robot.yaml')
    with open(robot_config_path, 'r') as file:
        robot_config = yaml.safe_load(file)

    blueye_simulator_interface = Node(
        package="mundus_mir_vehicle_interfaces",
        executable="blueye_simulator_interface",
        name="blueye_simulator_interface",
        output=['screen'],
    )

    # DVL

    return LaunchDescription([
        blueye_simulator_interface
    ])

