#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage
from std_srvs.srv import SetBool
import time
import argparse
import sys
import threading
import numpy as np

class SensorFailureSimulator(Node):
    """
    Node to simulate sensor failures to test emergency behavior.
    Can block/intercept camera and sonar messages to simulate failures.
    """
    def __init__(self):
        super().__init__('sensor_failure_simulator')
        
        # Parameters - default durations in seconds
        self.declare_parameter('camera_failure_duration', 30.0)
        self.declare_parameter('sonar_failure_duration', 30.0)
        self.declare_parameter('auto_recover', True)
        
        self.camera_failure_duration = self.get_parameter('camera_failure_duration').value
        self.sonar_failure_duration = self.get_parameter('sonar_failure_duration').value
        self.auto_recover = self.get_parameter('auto_recover').value
        
        # State variables
        self.blocking_camera = False
        self.blocking_sonar = False
        self.camera_failure_end_time = 0
        self.sonar_failure_end_time = 0
        
        # Create proxies for the camera and sonar topics
        # Subscribe to original topics
        self.camera_sub = self.create_subscription(
            Image, 
            '/blueye/camera_1/image_raw', 
            self.camera_callback, 
            10)
        
        self.sonar_sub = self.create_subscription(
            ProjectedSonarImage, 
            '/mundus_mir/sonar', 
            self.sonar_callback, 
            10)
        
        # Create replacement publishers with '_proxy' suffix
        # These allow us to selectively block messages
        self.camera_pub = self.create_publisher(
            Image, 
            '/blueye/camera_1/image_raw_proxy', 
            10)
        
        self.sonar_pub = self.create_publisher(
            ProjectedSonarImage, 
            '/mundus_mir/sonar_proxy', 
            10)
        
        # Create services to trigger failures - use SetBool for both services
        self.camera_failure_srv = self.create_service(
            SetBool, 
            '/failure_simulator/trigger_camera_failure', 
            self.trigger_camera_failure_callback)
        
        self.sonar_failure_srv = self.create_service(
            SetBool,  # Changed from Bool to SetBool 
            '/failure_simulator/trigger_sonar_failure', 
            self.trigger_sonar_failure_callback)
        
        # Create a timer to check for auto-recovery
        self.timer = self.create_timer(1.0, self.check_recovery)
        
        # Command-line interface in separate thread
        self.cli_thread = threading.Thread(target=self.cli_interface)
        self.cli_thread.daemon = True
        self.cli_thread.start()
        
        self.get_logger().info('Sensor Failure Simulator initialized')
        self.get_logger().info('Use "c" to simulate camera failure, "s" for sonar failure, "b" for both, "r" to recover all')
    
    def camera_callback(self, msg):
        """Forward camera messages unless blocking is active"""
        if not self.blocking_camera:
            self.camera_pub.publish(msg)
    
    def sonar_callback(self, msg):
        """Forward sonar messages unless blocking is active"""
        if not self.blocking_sonar:
            self.sonar_pub.publish(msg)
    
    def trigger_camera_failure(self, duration=None):
        """Start blocking camera messages"""
        if duration is None:
            duration = self.camera_failure_duration
            
        self.blocking_camera = True
        if self.auto_recover:
            self.camera_failure_end_time = time.time() + duration
        
        self.get_logger().warn(f'Camera failure triggered for {duration} seconds')
        
        # Return success
        return True
    
    def trigger_sonar_failure(self, duration=None):
        """Start blocking sonar messages"""
        if duration is None:
            duration = self.sonar_failure_duration
            
        self.blocking_sonar = True
        if self.auto_recover:
            self.sonar_failure_end_time = time.time() + duration
        
        self.get_logger().warn(f'Sonar failure triggered for {duration} seconds')
        
        # Return success
        return True
    
    def trigger_camera_failure_callback(self, request, response):
        """Service callback to trigger camera failure"""
        response.success = self.trigger_camera_failure()  # Changed from data to success
        response.message = "Camera failure triggered"  # Added message field
        return response
    
    def trigger_sonar_failure_callback(self, request, response):
        """Service callback to trigger sonar failure"""
        response.success = self.trigger_sonar_failure()  # Changed from data to success
        response.message = "Sonar failure triggered"  # Added message field
        return response
    
    def recover_camera(self):
        """Stop blocking camera messages"""
        self.blocking_camera = False
        self.camera_failure_end_time = 0
        self.get_logger().info('Camera restored')
    
    def recover_sonar(self):
        """Stop blocking sonar messages"""
        self.blocking_sonar = False
        self.sonar_failure_end_time = 0
        self.get_logger().info('Sonar restored')
    
    def recover_all(self):
        """Restore all sensors"""
        self.recover_camera()
        self.recover_sonar()
    
    def check_recovery(self):
        """Check if it's time to auto-recover sensors"""
        current_time = time.time()
        
        if self.auto_recover:
            if self.blocking_camera and current_time > self.camera_failure_end_time > 0:
                self.recover_camera()
            
            if self.blocking_sonar and current_time > self.sonar_failure_end_time > 0:
                self.recover_sonar()
    
    def cli_interface(self):
        """Command line interface for interactive control"""
        print("Sensor Failure Simulator CLI")
        print("----------------------------")
        print("Commands:")
        print("  c - Trigger camera failure")
        print("  s - Trigger sonar failure")
        print("  b - Trigger both failures")
        print("  r - Recover all sensors")
        print("  q - Quit")
        
        while rclpy.ok():
            try:
                cmd = input("> ")
                
                if cmd.lower() == 'c':
                    self.trigger_camera_failure()
                elif cmd.lower() == 's':
                    self.trigger_sonar_failure()
                elif cmd.lower() == 'b':
                    self.trigger_camera_failure()
                    self.trigger_sonar_failure()
                elif cmd.lower() == 'r':
                    self.recover_all()
                elif cmd.lower() == 'q':
                    self.get_logger().info('Exiting...')
                    rclpy.shutdown()
                    break
                else:
                    print("Unknown command")
            except Exception as e:
                print(f"Error: {e}")
                
            time.sleep(0.1)  # Don't hog the CPU

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Parse command line arguments, but filter out ROS arguments first
    parser = argparse.ArgumentParser(description='Simulate sensor failures')
    parser.add_argument('--camera', type=int, help='Immediately trigger camera failure for N seconds')
    parser.add_argument('--sonar', type=int, help='Immediately trigger sonar failure for N seconds')
    parser.add_argument('--no-auto-recover', action='store_true', help='Disable auto-recovery')
    
    # This is the important part - remove ROS args before parsing
    # Use ros_args=None to tell rclpy to filter them automatically
    parsed_args, _ = parser.parse_known_args()  # Fixed syntax error here
    
    # Create node
    node = SensorFailureSimulator()
    
    # Apply command line arguments
    if parsed_args.no_auto_recover:
        node.auto_recover = False
    
    if parsed_args.camera:
        node.trigger_camera_failure(parsed_args.camera)
    
    if parsed_args.sonar:
        node.trigger_sonar_failure(parsed_args.sonar)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()