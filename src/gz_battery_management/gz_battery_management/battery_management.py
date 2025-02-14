#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mundus_mir_msgs.msg import BatteryStatus, ReturnRecommendation
from rclpy.time import Time

class ReturnCalculator(Node):
    def __init__(self):
        super().__init__('return_calculator')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('safety_margin', 30.0),          # 30% safety margin
                ('update_frequency', 10.0),        # Update frequency in Hz
                ('average_speed', 0.5),           # Average speed in m/s for time estimation
            ]
        )
        
        # Get parameters
        self.safety_margin = self.get_parameter('safety_margin').value
        self.update_frequency = self.get_parameter('update_frequency').value
        self.average_speed = self.get_parameter('average_speed').value
        
        # Distance tracking
        self.total_distance = 0.0
        self.last_position = None
        self.start_time = None
        self.last_movement_time = None
        self.is_moving = False
        self.movement_threshold = 0.05  # m/s threshold to determine if ROV is moving
        
        # Battery tracking
        self.initial_battery = None
        self.current_battery = 100.0
        self.battery_per_meter = None
        self.hover_drain_rate = None  # Battery percentage per second while hovering
        
        # Hover drain calculation
        self.hover_start_battery = None
        self.hover_start_time = None
        self.hover_duration = 0.0
        self.battery_used_hovering = 0.0
        
        # Create subscriptions
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/blueye/battery',
            self.battery_callback,
            10
        )
        
        self.distance_sub = self.create_subscription(
            Float64,
            '/blueye/distance_to_dock',
            self.distance_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odom_callback,
            10
        )
        
        # Create publisher
        self.recommendation_pub = self.create_publisher(
            ReturnRecommendation,
            '/blueye/return_recommendation',
            10
        )
        
        # Initialize other variables
        self.current_distance = 0.0
        self.current_speed = 0.0
        
        # Create timer for periodic calculations
        timer_period = 0.1  # 10 Hz = 0.1 seconds
        self.timer = self.create_timer(timer_period, self.calculate_recommendation)
        
        # # Create timer for periodic calculations
        # self.timer = self.create_timer(
        #     1.0 / self.update_frequency, 
        #     self.calculate_recommendation
        # )
        
        self.get_logger().info('Return Calculator Node initialized')

    def battery_callback(self, msg):
        """Handle incoming battery status"""
        current_time = self.get_clock().now()
        
        # Initialize battery tracking
        if self.initial_battery is None:
            self.initial_battery = msg.state_of_charge * 100.0
            self.start_time = current_time
            
        self.current_battery = msg.state_of_charge * 100.0
        
        # Update hover drain calculations if not moving
        if not self.is_moving:
            if self.hover_start_battery is None:
                self.hover_start_battery = self.current_battery
                self.hover_start_time = current_time
            else:
                hover_time = (current_time - self.hover_start_time).nanoseconds / 1e9
                if hover_time > 0:
                    hover_drain = (self.hover_start_battery - self.current_battery) / hover_time
                    if self.hover_drain_rate is None:
                        self.hover_drain_rate = hover_drain
                    else:
                        # Running average of hover drain rate
                        self.hover_drain_rate = 0.9 * self.hover_drain_rate + 0.1 * hover_drain

    def distance_callback(self, msg):
        """Handle incoming distance to dock"""
        self.current_distance = msg.data

    def odom_callback(self, msg):
        """Handle incoming odometry data"""
        current_time = self.get_clock().now()
        current_pos = msg.pose.pose.position
        
        # Calculate speed
        vel = msg.twist.twist.linear
        self.current_speed = np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        
        # Determine if ROV is moving
        self.is_moving = self.current_speed > self.movement_threshold
        
        # Initialize position tracking
        if self.last_position is None:
            self.last_position = current_pos
            self.last_movement_time = current_time
            return
        
        # Calculate distance traveled
        distance = np.sqrt(
            (current_pos.x - self.last_position.x)**2 +
            (current_pos.y - self.last_position.y)**2 +
            (current_pos.z - self.last_position.z)**2
        )
        
        if self.is_moving:
            self.total_distance += distance
            # Calculate battery per meter if we have moved a significant distance
            if self.total_distance > 1.0 and self.initial_battery is not None:
                battery_used = self.initial_battery - self.current_battery
                self.battery_per_meter = battery_used / self.total_distance
        
        self.last_position = current_pos
        self.last_movement_time = current_time

    def estimate_return_energy(self):
        """Estimate energy needed to return to dock"""
        if self.battery_per_meter is None or self.hover_drain_rate is None:
            return 0.0
            
        # Add coefficient to reduce energy estimate
        consumption_coefficient = 0.4  # Adjust this value between 0 and 1
    
        # Calculate travel energy
        travel_energy = self.current_distance * self.battery_per_meter * consumption_coefficient
        
        # Calculate hover energy during return
        estimated_return_time = self.current_distance / self.average_speed
        hover_energy = self.hover_drain_rate * estimated_return_time * consumption_coefficient
        
        # Total energy needed
        total_energy = travel_energy + hover_energy
        
        return total_energy

    def calculate_recommendation(self):
        """Calculate and publish return recommendation"""
        msg = ReturnRecommendation()
        msg.stamp = self.get_clock().now().to_msg()
        
        # Current state
        msg.current_battery_level = self.current_battery
        msg.distance_to_dock = self.current_distance
        msg.current_speed = self.current_speed
        
        # Calculate consumption rates
        if self.battery_per_meter is not None:
            msg.current_consumption_rate = self.battery_per_meter
        
        # Calculate estimates
        msg.estimated_return_energy = self.estimate_return_energy()
        msg.estimated_time_to_return = (self.current_distance / self.average_speed
                                      if self.current_distance > 0 else 0.0)
        
        # Calculate minimum battery needed with safety margin
        safety_margin_decimal = self.safety_margin / 100.0
        msg.minimum_battery_needed = msg.estimated_return_energy * (1 + safety_margin_decimal)
        
        # Set safety margins
        msg.safety_margin_percent = self.safety_margin
        msg.battery_safety_threshold = 20.0  # Minimum 20% battery level
        
        msg.should_return = bool(
            float(msg.current_battery_level) <= float(msg.minimum_battery_needed) or
            float(msg.current_battery_level) <= float(msg.battery_safety_threshold)
        )
        
        # Publish recommendation
        self.recommendation_pub.publish(msg)
        
        # Log current status
        self.get_logger().info(
            f'Battery: {msg.current_battery_level:.1f}%, ' +
            f'Distance: {msg.distance_to_dock:.1f}m, ' +
            f'Return Energy: {msg.estimated_return_energy:.1f}%, ' +
            f'Minimum Needed: {msg.minimum_battery_needed:.1f}%'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ReturnCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()