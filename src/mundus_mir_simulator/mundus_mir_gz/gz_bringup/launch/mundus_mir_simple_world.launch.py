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
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Parse robot configuration file
    robot_config_path = os.path.join(pkg_project_simulator_launch, 'config', "mundus_mir_simple_world", 'robot.yaml')
    with open(robot_config_path, 'r') as file:
        robot_config = yaml.safe_load(file)

    robot_name = "blueye"
    x = str(robot_config['x'])
    y = str(robot_config['y'])
    z = str(robot_config['z'])
    R = str(robot_config['R'])
    P = str(robot_config['P'])
    Y = str(robot_config['Y'])
    sdf_file = "blueye.sdf"

    # Gazebo sim 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            'models',
            'worlds',
            'mundus_mir_simple_world.sdf',
        ])}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
                    package='ros_gz_sim',
                    executable='create',
                    name="spawn_sdf_entity_1",
                    arguments=['-file', PathJoinSubstitution([
                            'models',
                            'dynamic_assets',
                            'blueye',
                            sdf_file,
                    ]),
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-R', R,
                    '-P', P,
                    '-Y', Y,
                    ],
                    output='screen'
    )

    # Bridge for ROS2 and Gazebo
    bridge_config_path = os.path.join(pkg_project_simulator_launch, 'config', "mundus_mir_simple_world", 'bridge.yaml')
    ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # TF broadcaster -> Look through this and make sure it makes sense
    tf_transforms = Node(
             package="gz_tf_broadcaster",
             executable="gz_tf2_broadcaster",
             name="gz_tf_broadcaster_blueye",
             parameters=[{"robot_name" : robot_name}],
             output=['screen']
             )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        ros_bridge,
        tf_transforms
    ])

