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
    pkg_project_bringup = get_package_share_directory('gz_bringup')
    pkg_project_blueye_simulator = get_package_share_directory('mundus_mir_vehicle_interfaces')
    pkg_project_ps4_controller = get_package_share_directory('mundus_mir_blueye_joystick_cpp')
    pkg_project_waypoint_controller = get_package_share_directory('mundus_mir_waypoint_controller')
    pkg_project_sensors = get_package_share_directory('gz_sensor_launch')
    pkg_project_sdf_generator = get_package_share_directory('gz_sdf_generator')

    config_path = os.path.join(pkg_project_simulator_launch, 'config', 'generated_mundus_mir_pipeline_world', "robot.yaml")


    # SDF Generator
    generate_sdf_node = Node(
        package="gz_sdf_generator",
        executable="gz_sdf_generator",
        name="sdf_generator",
        output="screen",
        parameters=[{
            "generate_sdf": True,
            "config_path": config_path,
            "launch_file": "generated_mundus_mir_pipeline_world",
        }]
    )


    # Launch world
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, 'launch', 'generated_mundus_mir_pipeline_world.launch.py')),
    )

    # Launch blueye simulator interface
    launch_blueye_simulator_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_blueye_simulator, 'launch', 'blueye_simulator_interface.launch.py')),
    )

    # Launch non-gz sensors
    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_sensors, 'launch', 'generated_gz_sensor.launch.py')),
    )

    # Launch PS4 Controller
    launch_ps4_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_ps4_controller, 'launch', "mundus_mir_joystick_controller.launch.py")),
    )
    
    # Launch waypoint controller
    launch_waypoint_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_waypoint_controller, 'launch', "mundus_mir_waypoint_controller.launch.py")),
    )
    
    launch_rest = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_sdf_node,
            on_exit=[
                launch_gazebo,
                launch_blueye_simulator_interface,
                launch_waypoint_controller,
                launch_ps4_controller,
                launch_sensors,
            ]
        )
    )

    # Launch controller

    return LaunchDescription([
        generate_sdf_node,
        launch_rest
    ])