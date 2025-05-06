#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        
        # History lists: time (in seconds) and actual depth (z)
        self.time_history = []
        self.trajectory_depth = []
        
        # Setup a 2D plot: x-axis = time, y-axis = depth (reversed)
        self.fig, self.ax = plt.subplots(figsize=(20, 6))
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Depth (m)')
        self.ax.set_title('ROV Depth Over Time')
        
        # Initialize the red depth line and blue marker for current depth
        self.depth_line, = self.ax.plot([], [], color='red', linewidth=2, label='Depth')
        self.current_marker, = self.ax.plot([], [], 'bo', markersize=8, label='Current Depth')
        self.ax.legend()
        
        # Subscribe to the odometry topic (using /blueye/odometry_frd/gt)
        self.subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create animation that updates the plot every 100 ms
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        plt.show(block=False)
    
    def odometry_callback(self, msg):
        # Extract timestamp from header (in seconds)
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract the depth (z coordinate) without reversing
        pos = msg.pose.pose.position
        depth = pos.z
        
        # Store data for the entire mission
        self.time_history.append(time_stamp)
        self.trajectory_depth.append(depth)
        
    def update_plot(self, frame):
        if self.time_history and self.trajectory_depth:
            t = np.array(self.time_history)
            d = np.array(self.trajectory_depth)
            
            # Update the red line for depth history and the blue marker for current depth
            self.depth_line.set_data(t, d)
            self.current_marker.set_data(t[-1], d[-1])
            
            # Adjust x-axis to span the recorded time and add a small buffer at the end
            self.ax.set_xlim(t[0], t[-1] + 1)
            
            # Adjust y-axis based on current data range with a small margin
            d_min = np.min(d)
            d_max = np.max(d)
            margin = (d_max - d_min) * 0.1 if d_max != d_min else 1.0
            self.ax.set_ylim(d_max + margin, d_min - margin)            
            return self.depth_line, self.current_marker
        return self.depth_line,

def main(args=None):
    rclpy.init(args=args)
    visualizer = DepthVisualizer()
    
    # Run the ROS spin loop in a separate thread
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