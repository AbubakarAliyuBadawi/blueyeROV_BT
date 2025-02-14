# dock_distance_calc.py

#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class DockDistanceCalculator(Node):
    def __init__(self):
        super().__init__('dock_distance_calculator')
        
        # Hardcode dock station coordinates
        self.dock_position = np.array([-8.5, 8.54, 95.8])
        
        # Create subscription to odometry
        self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create publisher for distance
        self.distance_publisher = self.create_publisher(
            Float64,
            '/blueye/distance_to_dock',
            10
        )
        
        self.get_logger().info('Dock Distance Calculator Node started')
        self.get_logger().info(f'Dock position set to: {self.dock_position}')

    def odometry_callback(self, msg):
        """Calculate and publish distance to dock when new odometry data arrives"""
        # Extract current position
        current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Calculate Euclidean distance
        distance = np.linalg.norm(current_pos - self.dock_position)
        
        # Create and publish distance message
        distance_msg = Float64()
        distance_msg.data = float(distance)
        self.distance_publisher.publish(distance_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DockDistanceCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()