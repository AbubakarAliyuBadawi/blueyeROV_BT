#!/usr/bin/env python3
"""
ROS node for publishing Blueye drone position data.
This node connects to a Blueye drone using the SDK and publishes position data as ROS topics.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Bool
import numpy as np
import threading
import time
import blueye.protocol

class BlueyePositionNode(Node):
    """ROS Node for publishing Blueye drone position data."""
    
    def __init__(self):
        """Initialize the node."""
        super().__init__('blueye_position_node')
        
        # Declare parameters
        self.declare_parameter('drone_ip', '192.168.1.101')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        self.drone_ip = self.get_parameter('drone_ip').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'blueye/pose', 10)
        self.position_pub = self.create_publisher(Vector3, 'blueye/position', 10)
        self.depth_pub = self.create_publisher(Float32, 'blueye/depth', 10)
        self.altitude_pub = self.create_publisher(Float32, 'blueye/altitude', 10)
        self.position_valid_pub = self.create_publisher(Bool, 'blueye/position_valid', 10)
        
        # Initialize drone connection
        self.get_logger().info(f'Connecting to Blueye drone at {self.drone_ip}')
        
        try:
            from blueye.sdk import Drone
            self.drone = Drone(ip=self.drone_ip)
            self.connected = True
            self.get_logger().info(f'Connected to drone {self.drone.serial_number}')
            self.get_logger().info(f'Software version: {self.drone.software_version}')
            
            # Set up telemetry callback for position updates
            self.position_callback_id = self.drone.telemetry.add_msg_callback(
                [blueye.protocol.PositionEstimateTel],
                self.on_position_updated
            )
            
            # Store latest position data
            self.position_valid = False
            self.x = 0.0
            self.y = 0.0
            self.depth = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.altitude = 0.0
            
            # Create timer for publishing data
            self.timer = self.create_timer(1.0/self.publish_rate, self.publish_data)
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to drone: {str(e)}')
            self.connected = False
    
    def on_position_updated(self, msg_type, msg):
        """Handle position updates from drone telemetry."""
        # Extract position estimate from the message
        pos_estimate = msg.position_estimate
        
        # Update position validity
        self.position_valid = pos_estimate.is_valid
        
        # Only update coordinates if the position estimate is valid
        if pos_estimate.is_valid:
            # Get northing (Y) and easting (X) coordinates
            self.x = pos_estimate.easting
            self.y = pos_estimate.northing
            
            # Get orientation (convert to degrees for better readability)
            self.yaw = np.degrees(pos_estimate.heading) % 360
    
    def publish_data(self):
        """Publish drone position data to ROS topics."""
        if not self.connected:
            return
        
        # Get current timestamp
        current_time = self.get_clock().now().to_msg()
        
        # Get current depth from the drone
        self.depth = self.drone.depth if self.drone.depth is not None else 0.0
        
        # Get altitude if available
        self.altitude = self.drone.altitude if self.drone.altitude is not None else 0.0
        
        # Get attitude for roll and pitch
        pose = self.drone.pose
        if pose is not None:
            self.roll = pose['roll']
            self.pitch = pose['pitch']
        
        # Create and publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'map'
        
        # Position
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.depth
        
        # Orientation as quaternion
        # Convert roll, pitch, yaw (in degrees) to quaternion
        roll_rad = np.radians(self.roll)
        pitch_rad = np.radians(self.pitch)
        yaw_rad = np.radians(self.yaw)
        
        # Calculate quaternion (simplified)
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)
        
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Publish complete pose
        self.pose_pub.publish(pose_msg)
        
        # Also publish as separate components for easier use
        
        # Position vector
        position_msg = Vector3()
        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.depth
        self.position_pub.publish(position_msg)
        
        # Depth
        depth_msg = Float32()
        depth_msg.data = self.depth
        self.depth_pub.publish(depth_msg)
        
        # Altitude (if available)
        if self.altitude is not None:
            altitude_msg = Float32()
            altitude_msg.data = self.altitude
            self.altitude_pub.publish(altitude_msg)
        
        # Position validity
        valid_msg = Bool()
        valid_msg.data = self.position_valid
        self.position_valid_pub.publish(valid_msg)
        
        # Log occasionally
        if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:
            self.get_logger().info(
                f"Publishing position: X={self.x:.2f}, Y={self.y:.2f}, "
                f"Depth={self.depth:.2f}, Heading={self.yaw:.2f}"
            )
    
    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        if hasattr(self, 'drone') and self.connected:
            # Remove callback
            if hasattr(self, 'position_callback_id'):
                self.drone.telemetry.remove_msg_callback(self.position_callback_id)
            
            # Disconnect from drone
            self.drone.disconnect()
            self.get_logger().info("Disconnected from Blueye drone")
        
        super().destroy_node()


def main(args=None):
    """Main function."""
    # Initialize ROS
    rclpy.init(args=args)
    
    try:
        # Create the node
        node = BlueyePositionNode()
        
        # Spin the node (let ROS process in the background)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Shutdown
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()