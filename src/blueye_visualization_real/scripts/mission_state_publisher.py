#!/usr/bin/env python3
"""
Simple Mission State Publisher for Blueye ROV visualization testing.
Automatically cycles through mission states with predefined durations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class MissionStatePublisher(Node):
    """Simple node that publishes mission states on a timer."""
    
    def __init__(self):
        super().__init__('mission_state_publisher')
        
        # Mission state definitions - Updated with Transit 1 and Transit 2
        self.state_names = {
            0: "Trajectory",
            1: "Undocking", 
            2: "Pipeline Inspection",
            3: "Transit 1",      # First transit
            4: "Docking",
            5: "Transit 2"       # Second transit
        }
        
        # Mission timeline (state_id: duration_in_seconds)
        # Updated to use Transit 1 (ID 3) and Transit 2 (ID 5)
        # self.mission_timeline = [
        #     (0, 1.0),   # Trajectory for 5 seconds
        #     (1, 20.0),   # Undocking for 5 seconds  
        #     (3, 38.0),  # Transit 1 for 10 seconds
        #     (2, 71.0),  # Pipeline Inspection for 15 seconds
        #     (5, 12.0),  # Transit 2 for 10 seconds
        #     (4, 24.0),  # Docking for 10 seconds
        #     (0, 1.0)    # Back to Trajectory for 5 seconds (then repeat)
        # ]
        
        # # Mission timeline (state_id: duration_in_seconds)
        # # Updated to use Transit 1 (ID 3) and Transit 2 (ID 5)
        self.mission_timeline = [
            (0, 1.0),   # Trajectory for 5 seconds
            (1, 26.16),   # Undocking for 5 seconds  
            (3, 39.17),  # Transit 1 for 10 seconds
            (2, 14.50),  # Pipeline Inspection for 15 seconds
            (5, 25.0),  # Transit 2 for 10 seconds
            (4, 30.0),  # Docking for 10 seconds
            (0, 5.0)    # Back to Trajectory for 5 seconds (then repeat)
        ]
        
        # Current mission state
        self.current_state = 0
        self.current_timeline_index = 0
        self.state_start_time = time.time()
        
        # Create publisher
        self.state_publisher = self.create_publisher(
            Int32, 
            '/mission_state', 
            10
        )
        
        # Create timer to check and update state (check every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Mission State Publisher initialized')
        self.get_logger().info('Mission Timeline:')
        for i, (state_id, duration) in enumerate(self.mission_timeline):
            self.get_logger().info(f'  Step {i+1}: {self.state_names[state_id]} for {duration} seconds')
        
        # Publish initial state
        self.publish_current_state()
    
    def timer_callback(self):
        """Timer callback to check if it's time to change state."""
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        # Get current state info from timeline
        current_state_id, duration = self.mission_timeline[self.current_timeline_index]
        
        # Check if it's time to move to next state
        if elapsed_time >= duration:
            # Move to next state in timeline
            self.current_timeline_index = (self.current_timeline_index + 1) % len(self.mission_timeline)
            next_state_id, next_duration = self.mission_timeline[self.current_timeline_index]
            
            # Update state
            old_state = self.current_state
            self.current_state = next_state_id
            self.state_start_time = current_time
            
            # Log the state change
            self.get_logger().info(
                f'State changed from {self.state_names[old_state]} to {self.state_names[self.current_state]} '
                f'(will last {next_duration} seconds)'
            )
            
            # Publish new state
            self.publish_current_state()
        else:
            # Calculate remaining time
            remaining_time = duration - elapsed_time
            
            # Log progress every 5 seconds (10 timer callbacks)
            if int(elapsed_time) % 5 == 0 and int(elapsed_time * 2) % 10 == 0:
                self.get_logger().info(
                    f'Current state: {self.state_names[current_state_id]} '
                    f'({remaining_time:.1f}s remaining)'
                )
    
    def publish_current_state(self):
        """Publish the current mission state."""
        msg = Int32()
        msg.data = self.current_state
        self.state_publisher.publish(msg)
        
        self.get_logger().info(f'Published mission state: {self.current_state} ({self.state_names[self.current_state]})')

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    mission_publisher = MissionStatePublisher()
    
    try:
        rclpy.spin(mission_publisher)
    except KeyboardInterrupt:
        mission_publisher.get_logger().info('Mission State Publisher stopped by user')
    finally:
        mission_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()