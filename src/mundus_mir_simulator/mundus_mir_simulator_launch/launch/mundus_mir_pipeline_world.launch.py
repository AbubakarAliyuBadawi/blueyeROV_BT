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


def generate_launch_description():

    # Find shared directories
    pkg_project_bringup = get_package_share_directory('gz_bringup')
    pkg_project_blueye_simulator = get_package_share_directory('mundus_mir_vehicle_interfaces')
    pkg_project_ps4_controller = get_package_share_directory('mundus_mir_blueye_joystick_cpp')
    pkg_project_sensors = get_package_share_directory('gz_sensor_launch')

    # Launch world
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, 'launch', 'mundus_mir_pipeline_world.launch.py')),
    )

    # Launch blueye simulator interface
    launch_blueye_simulator_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_blueye_simulator, 'launch', 'blueye_simulator_interface.launch.py')),
    )

    # Launch state estimation
    launch_ps4_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_ps4_controller, 'launch', 'mundus_mir_joystick_controller.launch.py')),
    )

    # Start sensors
        # Launch non-gz sensors
    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_sensors, 'launch', 'gz_sensor.launch.py')),
    )


    return LaunchDescription([
        launch_gazebo,
        launch_blueye_simulator_interface,
        launch_ps4_controller,
        launch_sensors,
    ])