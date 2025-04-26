#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import subprocess
import time
import random

class MissionTester(Node):
    def __init__(self):
        super().__init__('mission_tester')
        self.battery_pub = self.create_publisher(Float64, '/blueye/battery_percentage', 10)
        self.distance_pub = self.create_publisher(Float64, '/blueye/distance_to_dock', 10)
        self.timer = self.create_timer(1.0, self.run_test)
        self.test_count = 0
        self.max_tests = 50
        
    def run_test(self):
        if self.test_count >= self.max_tests:
            self.get_logger().info("Testing complete")
            rclpy.shutdown()
            return
            
        # Randomize initial conditions
        battery = random.uniform(15.0, 95.0)
        distance = random.uniform(20.0, 200.0)
        
        # Publish test conditions
        battery_msg = Float64()
        battery_msg.data = battery
        self.battery_pub.publish(battery_msg)
        
        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)
        
        self.get_logger().info(f"Test {self.test_count}: Battery={battery:.1f}%, Distance={distance:.1f}m")
        
        # Run mission
        subprocess.run(["ros2", "run", "blueye_bt", "blueye_bt", 
                      "--ros-args", "-p", "behavior_tree_path:=/home/badawi/Desktop/blueyeROV_BT/src/blueye_bt/behavior_trees/TestLearningMission.xml"])
        
        self.test_count += 1
        time.sleep(2.0)  # Wait between tests

def main():
    rclpy.init()
    tester = MissionTester()
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main()