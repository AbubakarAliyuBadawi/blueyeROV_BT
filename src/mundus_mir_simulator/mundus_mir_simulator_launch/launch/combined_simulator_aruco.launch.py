import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Find shared directories
    pkg_project_simulator_launch = get_package_share_directory("mundus_mir_simulator_launch")
    pkg_project_bringup = get_package_share_directory('gz_bringup')
    pkg_project_blueye_simulator = get_package_share_directory('mundus_mir_vehicle_interfaces')
    pkg_project_waypoint_controller = get_package_share_directory('mundus_mir_waypoint_controller')
    pkg_project_sensors = get_package_share_directory('gz_sensor_launch')
    pkg_project_sdf_generator = get_package_share_directory('gz_sdf_generator')

    # Path for simulator config
    config_path = os.path.join(pkg_project_simulator_launch, 'config', 'generated_mundus_mir_simple_world_waypoint', "robot.yaml")

    # Declare arguments for EKF
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('blueye_description'), 'config', 'robot_localization.yaml'
        ]),
        description='Path to the EKF configuration file'
    )

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # SDF Generator Node
    generate_sdf_node = Node(
        package="gz_sdf_generator",
        executable="gz_sdf_generator",
        name="sdf_generator",
        output="screen",
        parameters=[{
            "generate_sdf": True,
            "config_path": config_path,
            "launch_file": "generated_mundus_mir_simple_world_waypoint",
        }]
    )

    # Launch Simulator Components
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, 'launch', 'generated_mundus_mir_simple_world_waypoint.launch.py')),
    )

    launch_blueye_simulator_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_blueye_simulator, 'launch', 'blueye_simulator_interface.launch.py')),
    )

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_sensors, 'launch', 'generated_gz_sensor.launch.py')),
    )

    launch_waypoint_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_waypoint_controller, 'launch', "mundus_mir_waypoint_controller.launch.py")),
    )

    # ArUco Detection Node
    pose_estimation_aruco = Node(
        package='image_processing',
        executable='pose_estimation_aruco_node',
        name='pose_estimation_aruco',
        output='screen',
        remappings=[
            # Remap topics to match your simulator's topics
            ("blueye/camera", "/blueye/camera_1/image_raw"),
            ("/blueye/odometry", "/blueye/odometry_flu/gt"),
        ]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
    )

    # Launch rest of the nodes after SDF generation
    launch_rest = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_sdf_node,
            on_exit=[
                launch_gazebo,
                launch_blueye_simulator_interface,
                launch_waypoint_controller,
                launch_sensors,
                pose_estimation_aruco,
                robot_state_publisher,
                ekf_node,  # Added EKF node here
            ]
        )
    )

    # Create launch description
    ld = LaunchDescription()

    # Add all actions
    ld.add_action(params_file_arg)  # Add EKF params argument
    ld.add_action(generate_sdf_node)
    ld.add_action(launch_rest)

    return ld