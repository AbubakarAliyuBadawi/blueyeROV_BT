#!/usr/bin/env python3
"""
ROS service-based script to hold drone position and move backwards.
Uses existing ROS node connection instead of creating new drone connection.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import WrenchStamped
import time
import logging
import sys

class PositionHoldClient(Node):
    """ROS client for position hold and reverse movement."""
    
    def __init__(self):
        super().__init__('position_hold_client')
        
        # Create service clients
        self.depth_hold_client = self.create_client(SetBool, '/blueye/depth_hold')
        self.heading_hold_client = self.create_client(SetBool, '/blueye/heading_hold')
        self.stationkeep_client = self.create_client(SetBool, '/blueye/stationkeep')
        
        # Create command publisher
        self.cmd_pub = self.create_publisher(WrenchStamped, '/blueye/commands', 10)
        
        # Set up logging
        self.logger = self.get_logger()
        
    def wait_for_services(self, timeout=10.0):
        """Wait for all services to be available."""
        services = [
            (self.depth_hold_client, 'depth_hold'),
            (self.heading_hold_client, 'heading_hold'),
            (self.stationkeep_client, 'stationkeep')
        ]
        
        for client, name in services:
            self.logger.info(f"Waiting for {name} service...")
            if not client.wait_for_service(timeout_sec=timeout):
                self.logger.error(f"{name} service not available after {timeout} seconds")
                return False
        
        self.logger.info("All services are available")
        return True
    
    def call_service(self, client, enable, service_name):
        """Call a SetBool service."""
        request = SetBool.Request()
        request.data = enable
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.logger.info(f"{service_name}: {response.message}")
                return True
            else:
                self.logger.error(f"{service_name} failed: {response.message}")
                return False
        else:
            self.logger.error(f"Service call to {service_name} failed")
            return False
    
    def send_command(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
        """Send movement command to drone."""
        cmd_msg = WrenchStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.wrench.force.x = surge
        cmd_msg.wrench.force.y = sway
        cmd_msg.wrench.force.z = heave
        cmd_msg.wrench.torque.z = yaw
        
        self.cmd_pub.publish(cmd_msg)
    
    def hold_position_and_reverse(self, reverse_duration=10, reverse_power=0.4):
        """Execute position hold and reverse sequence."""
        try:
            # Wait for services
            if not self.wait_for_services():
                return False
            
            # Enable station keeping
            self.logger.info("Activating station keeping mode...")
            if not self.call_service(self.stationkeep_client, True, "stationkeep"):
                self.logger.warning("Station keeping failed, using individual auto modes")
                # Fallback to individual modes
                self.call_service(self.depth_hold_client, True, "depth_hold")
                self.call_service(self.heading_hold_client, True, "heading_hold")
            
            # Hold position for a few seconds
            self.logger.info("Holding position for 5 seconds...")
            time.sleep(5)
            
            # Move backwards while maintaining position control
            self.logger.info(f"Moving backwards at {reverse_power} power for {reverse_duration} seconds...")
            
            # Disable station keeping but keep auto modes
            self.call_service(self.stationkeep_client, False, "stationkeep")
            time.sleep(0.5)
            self.call_service(self.depth_hold_client, True, "depth_hold")
            self.call_service(self.heading_hold_client, True, "heading_hold")
            time.sleep(1)
            
            # Move backwards
            start_time = time.time()
            while time.time() - start_time < reverse_duration:
                self.send_command(surge=-reverse_power)
                self.logger.info("Moving backwards...")
                time.sleep(2)
            
            # Stop movement
            self.send_command(surge=0.0)
            self.logger.info("Backwards movement complete - drone stopped")
            
            # Re-enable station keeping
            time.sleep(1)
            self.call_service(self.stationkeep_client, True, "stationkeep")
            self.logger.info("Station keeping re-activated - holding new position")
            
            # Hold new position briefly
            time.sleep(3)
            
            self.logger.info("Position hold and reverse sequence completed successfully!")
            return True
            
        except Exception as e:
            self.logger.error(f"Error during position hold and reverse: {str(e)}")
            return False

def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Hold position and move backwards via ROS')
    parser.add_argument('--reverse-duration', type=int, default=10,
                        help='Duration to move backwards in seconds')
    parser.add_argument('--reverse-power', type=float, default=0.4,
                        help='Power for backwards movement (0.1 to 1.0)')
    
    args = parser.parse_args()
    
    # Validate reverse power
    if not 0.1 <= args.reverse_power <= 1.0:
        print("Error: reverse-power must be between 0.1 and 1.0")
        return 1
    
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create client node
        client = PositionHoldClient()
        
        # Execute position hold sequence
        success = client.hold_position_and_reverse(
            reverse_duration=args.reverse_duration,
            reverse_power=args.reverse_power
        )
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 1
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main())