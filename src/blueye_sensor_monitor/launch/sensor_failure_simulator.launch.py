from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for sensor failure simulator."""
    
    # Launch arguments
    camera_failure_duration = LaunchConfiguration('camera_failure_duration')
    sonar_failure_duration = LaunchConfiguration('sonar_failure_duration')
    auto_recover = LaunchConfiguration('auto_recover')
    
    return LaunchDescription([
        # Declare launch arguments with default values
        DeclareLaunchArgument(
            'camera_failure_duration',
            default_value='30.0',
            description='Duration of camera failure in seconds'),
            
        DeclareLaunchArgument(
            'sonar_failure_duration',
            default_value='30.0',
            description='Duration of sonar failure in seconds'),
            
        DeclareLaunchArgument(
            'auto_recover',
            default_value='true',
            description='Whether to automatically recover from failures'),
            
        # Launch the sensor failure simulator node
        Node(
            package='blueye_sensor_monitor',  # Changed to your actual package name
            executable='sensor_failure_simulator',  # Removed .py extension
            name='sensor_failure_simulator',
            output='screen',
            parameters=[{
                'camera_failure_duration': camera_failure_duration,
                'sonar_failure_duration': sonar_failure_duration,
                'auto_recover': auto_recover
            }]
        )
    ])