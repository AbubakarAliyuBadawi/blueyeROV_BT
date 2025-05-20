#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from blueye.sdk import Drone
import blueye.protocol as bp

# Import custom message types
from blueye_msgs_real.msg import BatteryTel, DepthTel, PositionEstimateTel, DvlVelocityTel

class BlueyeRawBridge(Node):
    def __init__(self):
        super().__init__('blueye_raw_bridge')
        
        # Initialize drone connection
        self.get_logger().info('Connecting to Blueye drone...')
        self.drone = Drone(connect_as_observer=True)
        self.get_logger().info('Connected to Blueye drone')
        
        self.battery_pub = self.create_publisher(
            BatteryTel,
            '/blueye/protocol/battery_tel',
            10)
        self.depth_pub = self.create_publisher(
            DepthTel,
            '/blueye/protocol/depth_tel',
            10)
        self.position_pub = self.create_publisher(
            PositionEstimateTel,
            '/blueye/protocol/position_estimate_tel',
            10)
        self.velocity_pub = self.create_publisher(
            DvlVelocityTel,
            '/blueye/protocol/dvl_velocity_tel',
            10)
        
        # Register telemetry callbacks
        self.drone.telemetry.add_msg_callback([bp.BatteryTel], self.battery_callback)
        self.drone.telemetry.add_msg_callback([bp.DepthTel], self.depth_callback)
        self.drone.telemetry.add_msg_callback([bp.PositionEstimateTel], self.position_callback)
        self.drone.telemetry.add_msg_callback([bp.DvlVelocityTel], self.velocity_callback)
        
        # Set telemetry frequencies
        self.drone.telemetry.set_msg_publish_frequency(bp.BatteryTel, 1)
        self.drone.telemetry.set_msg_publish_frequency(bp.DepthTel, 10)
        self.drone.telemetry.set_msg_publish_frequency(bp.PositionEstimateTel, 10)
        self.drone.telemetry.set_msg_publish_frequency(bp.DvlVelocityTel, 10)
        
        self.get_logger().info('Blueye ROS2 raw bridge initialized')
    
    def battery_callback(self, msg_type, blueye_msg):
        """Forward BatteryTel messages to ROS2"""
        ros_msg = BatteryTel()
        
        # Map fields from Blueye message to ROS message
        ros_msg.voltage = blueye_msg.battery.voltage
        ros_msg.level = blueye_msg.battery.level
        ros_msg.temperature = blueye_msg.battery.temperature
        # Add more fields as needed
        
        self.battery_pub.publish(ros_msg)
    
    def depth_callback(self, msg_type, blueye_msg):
        """Forward DepthTel messages to ROS2"""
        ros_msg = DepthTel()
        
        # Map fields
        ros_msg.depth = blueye_msg.depth.value
        # Add more fields as needed
        
        self.depth_pub.publish(ros_msg)
    
    def position_callback(self, msg_type, blueye_msg):
        """Forward PositionEstimateTel messages to ROS2"""
        ros_msg = PositionEstimateTel()
        
        # Map fields
        ros_msg.northing = blueye_msg.position_estimate.northing
        ros_msg.easting = blueye_msg.position_estimate.easting
        ros_msg.heading = blueye_msg.position_estimate.heading
        ros_msg.surge_rate = blueye_msg.position_estimate.surge_rate
        ros_msg.sway_rate = blueye_msg.position_estimate.sway_rate
        ros_msg.yaw_rate = blueye_msg.position_estimate.yaw_rate
        ros_msg.ocean_current = blueye_msg.position_estimate.ocean_current
        ros_msg.odometer = blueye_msg.position_estimate.odometer
        ros_msg.is_valid = blueye_msg.position_estimate.is_valid
        # Map remaining fields...
        
        self.position_pub.publish(ros_msg)
    
    def velocity_callback(self, msg_type, blueye_msg):
        """Forward DvlVelocityTel messages to ROS2"""
        ros_msg = DvlVelocityTel()
        
        # Map fields (adjust field names based on actual message structure)
        # For example, if the structure is like:
        # ros_msg.vx = blueye_msg.vx
        # ros_msg.vy = blueye_msg.vy
        # ros_msg.vz = blueye_msg.vz
        
        # Print message structure for debugging
        self.get_logger().info(f"DVL message type: {type(blueye_msg).__name__}")
        self.get_logger().info(f"DVL attributes: {dir(blueye_msg)}")
        
        # Map all fields dynamically
        for attr in dir(blueye_msg):
            if not attr.startswith('_') and hasattr(ros_msg, attr):
                try:
                    setattr(ros_msg, attr, getattr(blueye_msg, attr))
                except:
                    pass
        
        self.velocity_pub.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BlueyeRawBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Blueye ROS2 raw bridge...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()