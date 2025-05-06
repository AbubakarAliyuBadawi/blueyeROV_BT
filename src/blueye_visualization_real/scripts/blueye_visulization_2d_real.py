#!/usr/bin/env python3
"""
2D visualization for Blueye ROV position using matplotlib.
Subscribes to the Blueye position topics published by the Blueye position node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Bool
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class BlueyeVisualizerNode(Node):
    """ROS Node for visualizing Blueye drone position in 2D."""
    
    def __init__(self):
        """Initialize the node."""
        super().__init__('blueye_visualizer_node')
        
        # ROV position and orientation
        self.rov_position = np.array([0.0, 0.0, 0.0])
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        
        # Docking station is at the origin
        self.docking_station = np.array([0.0, 0.0, 0.0])
        
        # Trajectory tracking
        self.trajectory = []
        self.movement_started = False
        self.movement_threshold = 0.1  # meters
        self.position_valid = False
        
        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/blueye/pose',
            self.pose_callback,
            10
        )
        
        self.position_valid_sub = self.create_subscription(
            Bool,
            '/blueye/position_valid',
            self.position_valid_callback,
            10
        )
        
        # Setup 2D plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111)
        
        self.ax.set_xlabel('X - East (m)')
        self.ax.set_ylabel('Y - North (m)')
        self.ax.set_title('Blueye ROV Position Visualization')
        
        # Equal aspect ratio to avoid distortion
        self.ax.set_aspect('equal')
        
        # Docking station
        self.ax.scatter(
            self.docking_station[0], self.docking_station[1],
            color='red', s=200, marker='s', label='Docking Station'
        )
        
        # Initialize trajectory line and ROV marker
        self.trajectory_line, = self.ax.plot(
            [], [], 
            color='blue', linewidth=1.5, label='Trajectory'
        )
        
        self.rov_marker, = self.ax.plot(
            self.rov_position[0], self.rov_position[1],
            'bo', markersize=10, label='ROV'
        )
        
        # Direction arrow (using quiver)
        yaw_rad = 0
        dx, dy = np.cos(yaw_rad), np.sin(yaw_rad)
        
        self.direction_arrow = self.ax.quiver(
            self.rov_position[0], self.rov_position[1],
            dx, dy, color='blue', scale=10, width=0.005
        )
        
        # Position text display
        self.position_text = self.ax.text(
            0.02, 0.02, 
            f'Position: X={self.rov_position[0]:.2f}m, Y={self.rov_position[1]:.2f}m',
            transform=self.ax.transAxes,
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Validity indicator
        self.validity_text = self.ax.text(
            0.02, 0.07, 
            f'Position Valid: {self.position_valid}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='red',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Set initial axis limits
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Create animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=True
        )
        
        self.get_logger().info('Blueye visualization node initialized')
    
    def pose_callback(self, msg):
        """Callback for handling pose updates."""
        # Extract position and orientation
        pos = msg.pose.position
        quat = msg.pose.orientation
        
        new_position = np.array([pos.x, pos.y, pos.z])
        self.rov_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
        
        # Only update if the position is valid
        if self.position_valid:
            # Check if ROV has actually started moving
            if not self.movement_started:
                # Calculate distance from docking station
                distance = np.linalg.norm(new_position[:2] - self.docking_station[:2])
                if distance > self.movement_threshold:
                    self.movement_started = True
                    # Add the first point to create a clean start
                    self.trajectory.append(self.rov_position.copy())
            
            # Update position after movement check
            self.rov_position = new_position
            
            # Only record trajectory after movement starts
            if self.movement_started:
                self.trajectory.append(self.rov_position.copy())
    
    def position_valid_callback(self, msg):
        """Callback for handling position validity updates."""
        self.position_valid = msg.data
    
    def update_plot(self, frame):
        """Update the visualization plot."""
        # Get current position
        x, y = self.rov_position[0], self.rov_position[1]
        
        # Update ROV marker
        self.rov_marker.set_data([x], [y])
        
        # Extract yaw from quaternion
        # Convert quaternion to Euler angles
        qx, qy, qz, qw = self.rov_orientation
        # Simplified calculation for yaw from quaternion
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Update direction arrow
        dx, dy = np.cos(yaw), np.sin(yaw)
        self.direction_arrow.set_offsets(np.array([[x, y]]))
        self.direction_arrow.set_UVC(dx, dy)
        
        # Update trajectory
        if len(self.trajectory) > 1:
            traj_array = np.array(self.trajectory)
            self.trajectory_line.set_data(traj_array[:, 0], traj_array[:, 1])
        
        # Update position text
        self.position_text.set_text(f'Position: X={x:.2f}m, Y={y:.2f}m, Z={self.rov_position[2]:.2f}m')
        
        # Update validity text
        valid_color = 'green' if self.position_valid else 'red'
        self.validity_text.set_text(f'Position Valid: {self.position_valid}')
        self.validity_text.set_color(valid_color)
        
        # Auto-adjust limits if needed to keep ROV in view
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        padding = 2.0
        
        need_update = False
        
        if x < x_min + padding:
            x_min = x - padding * 2
            need_update = True
        elif x > x_max - padding:
            x_max = x + padding * 2
            need_update = True
            
        if y < y_min + padding:
            y_min = y - padding * 2
            need_update = True
        elif y > y_max - padding:
            y_max = y + padding * 2
            need_update = True
        
        if need_update:
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
        
        # Return updated artists
        return self.rov_marker, self.trajectory_line, self.direction_arrow, self.position_text, self.validity_text

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    visualizer = BlueyeVisualizerNode()
    
    # Run ROS spin in a separate thread
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