import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    # Find shared directories
    pkg_project_simulator_launch = get_package_share_directory("mundus_mir_simulator_launch")

    robot_config_path = os.path.join(pkg_project_simulator_launch, 'config', 'mundus_mir_pipeline_world', "robot.yaml")
    with open(robot_config_path, 'r') as file:
        robot_config = yaml.safe_load(file)

    sensors_to_launch = []

    # If DVL
    if ("dvl" in robot_config and robot_config["dvl"]):
        dvl_node = Node(
            package="gz_dvl",
            executable="gz_dvl",
            name="gz_dvl",
            output="screen",
            parameters=[{
                "dvl_topic": robot_config["dvl_topic"],
                "range_topic": robot_config["dvl_topic"] + "/ray",
                "dvl_body_topic": robot_config["model_name"] + "/odometry_frd/gt",
                "max_valid_altitude": float(robot_config["dvl_max_valid_altitude"]),
                "publish_frequency": float(robot_config["dvl_update_rate"]),
                "velocity_sensor_noise": float(robot_config["dvl_lin_vel_noise_mean"]),
            }]
        )
        sensors_to_launch.append(dvl_node)

    if ("usbl" in robot_config and robot_config["usbl"]):
        usbl_node = Node(
            package="gz_usbl",
            executable="gz_usbl",
            name="gz_usbl",
            output="screen",
            parameters=[{
                "gz_usbl_topic": robot_config["usbl_topic"],
                "gz_usbl_odometry_topic": robot_config["model_name"] + "/odometry_frd/gt",
            }]
        )
        sensors_to_launch.append(usbl_node)

    if ("battery" in robot_config and robot_config['battery']):
        battery_node = Node(
            package="gz_battery",
            executable="gz_battery_node",
            name="gz_battery",
            output="screen",
            parameters=[{
                "battery_topic": robot_config["battery_topic"],
                "publish_frequency": float(robot_config["battery_update_rate"]),
                "voltage_sensor_noise_mean": float(robot_config["battery_voltage_sensor_noise_mean"]),
                "voltage_sensor_noise_stddev": float(robot_config["battery_voltage_sensor_noise_stddev"]),
                "temperature_sensor_noise_mean": float(robot_config["battery_temperature_sensor_noise_mean"]),
                "temperature_sensor_noise_stddev": float(robot_config["battery_temperature_sensor_noise_stddev"]),
            }]
        )
        sensors_to_launch.append(battery_node)


    # Launch
    return LaunchDescription(
        sensors_to_launch,
    )
