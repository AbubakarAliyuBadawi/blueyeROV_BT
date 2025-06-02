#!/usr/bin/env python3
"""
ROS-based mission navigation that uses existing telemetry connection
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Point, PoseStamped
from std_srvs.srv import SetBool
import math
import time
import logging

class RosMissionNode(Node):
    def __init__(self):
        super().__init__('ros_mission_node')
        
        # Publishers
        self.cmd_pub = self.create_publisher(WrenchStamped, '/blueye/commands', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/blueye/pose', self.pose_callback, 10)
        self.gps_sub = self.create_subscription(Point, '/blueye/gps', self.gps_callback, 10)
        
        # Service clients
        self.depth_hold_client = self.create_client(SetBool, '/blueye/depth_hold')
        self.heading_hold_client = self.create_client(SetBool, '/blueye/heading_hold')
        
        # Current state
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_depth = 0.0
        self.current_yaw = 0.0
        
        # Mission waypoints from your JSON coordinates
        self.waypoints = [
            {"lat": 63.4414001130402, "lon": 10.348227918148, "name": "Pipeline Point 1", "wait_time": 10.0},
            {"lat": 63.4413996633213, "lon": 10.3483355417848, "name": "Pipeline Point 2", "wait_time": 10.0},
            {"lat": 63.4414320430591, "lon": 10.3482845798135, "name": "Pipeline Point 3", "wait_time": 10.0},
            {"lat": 63.4414548287786, "lon": 10.3482882678509, "name": "Docking Station", "wait_time": 5.0}
        ]
        
        # Mission parameters from your JSON
        self.default_surge_speed = 0.2
        self.default_circle_of_acceptance = 0.5
        self.target_depth = 0.0  # Surface level
        
        self.current_waypoint = 0
        self.mission_active = False
        
        self.get_logger().info("ROS Mission Node initialized")
    
    def pose_callback(self, msg):
        self.current_depth = msg.pose.position.z
        # Extract yaw from quaternion if needed
        
    def gps_callback(self, msg):
        self.current_lat = msg.x
        self.current_lon = msg.y
        
    def call_service(self, client, enable):
        """Call a SetBool service."""
        request = SetBool.Request()
        request.data = enable
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False
    
    def send_command(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
        """Send movement command."""
        cmd_msg = WrenchStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.wrench.force.x = surge
        cmd_msg.wrench.force.y = sway
        cmd_msg.wrench.force.z = heave
        cmd_msg.wrench.torque.z = yaw
        self.cmd_pub.publish(cmd_msg)
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS coordinates."""
        # Simple approximation for small distances
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        return math.sqrt(dlat*dlat + dlon*dlon) * 111000  # Rough conversion to meters
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing to target."""
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        return math.atan2(dlon, dlat)
    
    def navigate_to_waypoint(self, waypoint):
        """Navigate to a single waypoint."""
        target_lat = waypoint["lat"]
        target_lon = waypoint["lon"]
        
        self.get_logger().info(f"Navigating to {waypoint['name']} ({target_lat}, {target_lon})")
        
        # Enable auto modes
        self.call_service(self.depth_hold_client, True)
        self.call_service(self.heading_hold_client, True)
        
        # Navigate until close enough
        while True:
            distance = self.calculate_distance(self.current_lat, self.current_lon, target_lat, target_lon)
            
            if distance < self.default_circle_of_acceptance:  # Within circle of acceptance
                self.get_logger().info(f"Reached {waypoint['name']}")
                break
            
            # Calculate bearing and send movement command
            bearing = self.calculate_bearing(self.current_lat, self.current_lon, target_lat, target_lon)
            
            # Use default surge speed with proportional control
            surge_speed = min(self.default_surge_speed, distance * 0.1)
            surge = surge_speed * math.cos(bearing)
            sway = surge_speed * math.sin(bearing)
            
            self.send_command(surge=surge, sway=sway)
            
            self.get_logger().info(f"Distance to {waypoint['name']}: {distance:.1f}m")
            time.sleep(1)
        
        # Stop movement
        self.send_command()
        
        # Wait at waypoint
        wait_time = waypoint.get('wait_time', 10.0)
        self.get_logger().info(f"Waiting at {waypoint['name']} for {wait_time} seconds")
        time.sleep(wait_time)
    
    def run_mission(self):
        """Execute the complete mission."""
        self.get_logger().info("Starting ROS-based mission")
        self.mission_active = True
        
        try:
            for waypoint in self.waypoints:
                if not self.mission_active:
                    break
                self.navigate_to_waypoint(waypoint)
            
            self.get_logger().info("Mission completed successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Mission failed: {str(e)}")
            return False
        finally:
            self.mission_active = False
            self.send_command()  # Stop all movement

def main():
    rclpy.init()
    node = RosMissionNode()
    
    # Wait for telemetry data
    time.sleep(5)
    
    # Run mission
    success = node.run_mission()
    
    node.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1

if __name__ == '__main__':
    exit(main())