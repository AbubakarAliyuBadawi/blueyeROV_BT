#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class MissionStateVisualizer(Node):
    def __init__(self):
        super().__init__('mission_state_visualizer')
        
        # Initialize time tracking
        self.start_time = time.time()
        self.time_points = []
        
        # Thread safety
        self.plot_lock = threading.Lock()
        
        # Mission state data
        self.states = []
        
        # Create a mapping from state IDs to state names
        self.state_names = {
            1: "UnDocking",
            2: "Transit_1",
            3: "PipeLineInspection",
            4: "Transit_2",
            5: "WreckageInspection",
            6: "Homing",
            7: "Docking"
        }
        
        # Create the figure
        self.fig, self.ax = plt.subplots(figsize=(20, 6))
        
        # Configure the plot
        self.ax.set_title('Mission State over time')
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('State')
        
        # Set the y-ticks to be the state names
        self.state_labels = ["UnDocking", "Transit_1", "PipeLineInspection", 
                            "Transit_2", "WreckageInspection", "Homing", "Docking"]
        
        self.ax.set_yticks(range(1, 8))
        self.ax.set_yticklabels(self.state_labels)
        # self.ax.grid(True, linestyle='--', alpha=0.7)
        
        # Initialize the state line (empty at start)
        self.state_line, = self.ax.plot([], [], 'purple', linewidth=2, label='State')
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Set x and y limits
        self.ax.set_ylim(0.5, 7.5)  # From state 1 to 7 with some padding
        self.ax.set_xlim(0, 500)    # Initial x limit to 500 seconds
        
        # Subscribe to mission state topic
        self.subscription = self.create_subscription(
            Int32,
            '/mission_state',
            self.state_callback,
            10
        )
        
        # Animation to update the plot
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        self.get_logger().info("Mission State Visualizer initialized")
    
    def state_callback(self, msg):
        with self.plot_lock:
            # Get current time
            current_time = time.time() - self.start_time
            
            # Get the state ID
            state_id = msg.data
            
            # Check if the state is valid (1-7)
            if 1 <= state_id <= 7:
                # Get state name for logging
                state_name = self.state_names.get(state_id)
                
                # Log the state change
                if not self.states or state_id != self.states[-1]:
                    self.get_logger().info(f"State changed to: {state_name} (ID: {state_id})")
                
                # Append data
                self.time_points.append(current_time)
                self.states.append(state_id)  # Use the actual state ID as the y-value
            else:
                self.get_logger().warn(f"Received invalid state ID: {state_id}")
    
    def update_plot(self, frame):
        with self.plot_lock:
            if len(self.time_points) > 0:
                # Update the state line
                self.state_line.set_data(self.time_points, self.states)
                
                # Adjust x axis limit dynamically
                max_time = max(self.time_points)
                self.ax.set_xlim(0, max(500, max_time * 1.1))
        
        return self.state_line,

def main(args=None):
    rclpy.init(args=args)
    visualizer = MissionStateVisualizer()
    
    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()