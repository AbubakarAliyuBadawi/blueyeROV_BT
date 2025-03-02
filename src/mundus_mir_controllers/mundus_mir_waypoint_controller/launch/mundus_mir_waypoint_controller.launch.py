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
    # Get shared directories and load configs
    pkg_project_simulator_launch = get_package_share_directory("mundus_mir_simulator_launch")
    
    # Parse robot configuration file
    joystick_config_path = os.path.join(pkg_project_simulator_launch, 'config', "generated_mundus_mir_simple_world_waypoint", 'joystick.yaml')
    with open(joystick_config_path, 'r') as file:
        joystick_config = yaml.safe_load(file)

    controller_config_path = os.path.join(pkg_project_simulator_launch, 'config', "generated_mundus_mir_simple_world_waypoint", 'controller.yaml')
    with open(controller_config_path, 'r') as file:
        controller_config = yaml.safe_load(file)
        
    # First define the parameters
    velocity_controller_params = {
        "topic_subscriber_odometry": controller_config["estimate_topic"],
        "topic_subscriber_desired": controller_config["vel_topic"],
        "topic_publisher": controller_config["cmd_topic"],
        "rt_surge_kp": controller_config["surge_kp"],
        "rt_surge_ki": controller_config["surge_ki"],
        "rt_surge_kd": controller_config["surge_kd"],
        "rt_sway_kp": controller_config["sway_kp"],
        "rt_sway_ki": controller_config["sway_ki"],
        "rt_sway_kd": controller_config["sway_kd"],
        "rt_depth_kp": controller_config["heave_kp"],
        "rt_depth_ki": controller_config["heave_ki"],
        "rt_depth_kd": controller_config["heave_kd"],
        "rt_yaw_kp": controller_config["yaw_kp"],
        "rt_yaw_ki": controller_config["yaw_ki"],
        "rt_yaw_kd": controller_config["yaw_kd"],
        "rt_surge_feedforward": controller_config["surge_feedforward"],
        "rt_sway_feedforward": controller_config["sway_feedforward"],
        "rt_depth_feedforward": controller_config["heave_feedforward"],
        "rt_yaw_feedforward": controller_config["yaw_feedforward"],
    }
    
    packages_to_launch = []

    # Launch nodes
    joystick_node = Node(
        package='mundus_mir_waypoint_controller',
        executable='waypoint_controller',
        name="waypoint_controller",
        parameters=[{
            "odometry_topic": controller_config["estimate_topic"],
            "desired_vel_topic": controller_config["vel_topic"],
            "heading_tolerance": controller_config["heading_tolerance"],
            "circle_of_acceptance": controller_config["circle_of_acceptance"],
            "max_heading_rate": controller_config["max_heading_rate"],
            "max_heave_rate": controller_config["max_heave_rate"],
            "max_station_keep_rate": controller_config["max_station_keep_rate"],
        }]
    )
    packages_to_launch.append(joystick_node)

    velocity_controller = Node(
        package='velocity_controller_4dof_ros2',
        executable='velocity_controller_4dof',
        name="velocity_controller",
        parameters=[velocity_controller_params]  # Use the defined params
    )
    packages_to_launch.append(velocity_controller)

    return LaunchDescription(packages_to_launch)
