#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class RLExplanationPublisher(Node):
    def __init__(self):
        super().__init__('rl_explanation_publisher')
        
        # Create publisher for RL explanation
        self.explanation_pub = self.create_publisher(
            String,
            '/rl_explanation',
            10
        )
        
        # Set up timer to check for updates
        self.timer = self.create_timer(1.0, self.publish_explanation)
        
        # Path to the explanation file
        self.explanation_file = "/tmp/rl_explanation.txt"
        self.last_modification_time = 0
        
        # Create empty file if it doesn't exist
        if not os.path.exists(self.explanation_file):
            with open(self.explanation_file, 'w') as f:
                f.write("No RL data available yet")
        
        self.get_logger().info("RL Explanation Publisher started")
    
    def publish_explanation(self):
        """Check for new explanation and publish it"""
        try:
            # Check if file exists and has been modified
            if not os.path.exists(self.explanation_file):
                return
            
            mod_time = os.path.getmtime(self.explanation_file)
            
            # Only read if modified
            if mod_time > self.last_modification_time or self.last_modification_time == 0:
                self.last_modification_time = mod_time
                
                # Read explanation from file
                with open(self.explanation_file, 'r') as f:
                    explanation = f.read()
                
                # Publish explanation
                msg = String()
                msg.data = explanation
                self.explanation_pub.publish(msg)
                
                self.get_logger().info("Published RL explanation")
        
        except Exception as e:
            self.get_logger().error(f"Error publishing explanation: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    publisher = RLExplanationPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()