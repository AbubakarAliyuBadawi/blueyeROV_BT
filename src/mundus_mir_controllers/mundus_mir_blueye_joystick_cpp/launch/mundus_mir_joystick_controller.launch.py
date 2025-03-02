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
    pkg_project_simulator_launch = get_package_share_directory("mundus_mir_simulator_launch")

    # Parse robot configuration file
    joystick_config_path = os.path.join(pkg_project_simulator_launch, 'config', "generated_mundus_mir_pipeline_world", 'joystick.yaml')
    with open(joystick_config_path, 'r') as file:
        joystick_config = yaml.safe_load(file)

    controller_config_path = os.path.join(pkg_project_simulator_launch, 'config', "generated_mundus_mir_pipeline_world", 'controller.yaml')
    with open(controller_config_path, 'r') as file:
        controller_config = yaml.safe_load(file)

    packages_to_launch = []

    # Launch joystick node
    joystick_parser = Node(
        package='joy',
        executable='joy_node',
        name="joy_node",
    )

    # Launch joystick parser
    joystick_node = Node(
        package='mundus_mir_blueye_joystick_cpp',
        executable='joystick_controller',
        name="joystick_controller",
        parameters=[{
            'quadratic_mapping': controller_config['quadratic_mapping'],
            'max_thrust_surge': controller_config['max_thrust_surge'],
            'max_thrust_sway': controller_config['max_thrust_sway'],
            'max_thrust_heave': controller_config['max_thrust_heave'],
            'max_thrust_yaw': controller_config['max_thrust_yaw'],
            'joy_topic': joystick_config['joy_topic'],
            'cmd_topic': controller_config['cmd_topic'],
            'left_joystick_x_axis': joystick_config['x_axis_left_stick'],
            'left_joystick_y_axis': joystick_config['y_axis_left_stick'],
            'right_joystick_x_axis': joystick_config['x_axis_right_stick'],
            'right_joystick_y_axis': joystick_config['y_axis_right_stick'],
            'x_button': joystick_config['cross_button'],
            'circle_button': joystick_config['circle_button'],
            'square_button': joystick_config['square_button'],
            'triangle_button': joystick_config['triangle_button'],
            'velocity_control': controller_config['velocity_controller'],
            'max_velocity_x': controller_config['max_velocity_surge'],
            'max_velocity_y': controller_config['max_velocity_sway'],
            'max_velocity_z': controller_config['max_velocity_heave'],
            'max_angular_velocity_yaw': controller_config['max_velocity_yaw'],
            'velocity_topic': controller_config['vel_topic'],
        }]
    )

    packages_to_launch.append(joystick_parser)
    packages_to_launch.append(joystick_node)

    if controller_config["velocity_controller"]:
        velocity_controller = Node(
            package='velocity_controller_4dof_ros2',
            executable='velocity_controller_4dof',
            name="velocity_controller",
            parameters=[{
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
            }]
        )
        packages_to_launch.append(velocity_controller)



    return LaunchDescription(
    packages_to_launch,
    )

