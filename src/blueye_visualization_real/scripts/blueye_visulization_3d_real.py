#!/usr/bin/env python3
"""
3D visualization for Blueye ROV position using matplotlib.
Subscribes to the Blueye position topics published by the Blueye position node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Bool
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import threading

class Blueye3DVisualizerNode(Node):
    """ROS Node for visualizing Blueye drone position in 3D."""
    
    def __init__(self):
        """Initialize the node."""
        super().__init__('blueye_3d_visualizer_node')
        
        # ROV position and orientation
        self.rov_position = np.array([0.0, 0.0, 0.0])
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        
        # Docking station is at the origin
        self.docking_station = np.array([0.0, 0.0, 0.0])
        
        # Thread safety for plotting updates
        self.plot_lock = threading.Lock()
        
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
        
        # Setup 3D plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Remove grid for cleaner look
        self.ax.grid(False)
        
        # Set a good view angle
        self.ax.view_init(elev=30, azim=45)
        
        # Set axis labels and title
        self.ax.set_xlabel('X - East (m)')
        self.ax.set_ylabel('Y - North (m)')
        self.ax.set_zlabel('Z - Depth (m)')
        self.ax.set_title('Blueye ROV 3D Position Visualization')
        
        # Plot docking station
        self.dock_scatter = self.ax.scatter(
            self.docking_station[0], self.docking_station[1], self.docking_station[2],
            color='red', s=200, marker='s', label='Docking Station'
        )
        
        # For ROV position
        self.rov_scatter = self.ax.scatter(
            self.rov_position[0], self.rov_position[1], self.rov_position[2],
            color='blue', s=100, marker='o', label='ROV', zorder=10
        )
        
        # Initialize trajectory line - empty at start
        self.trajectory_line, = self.ax.plot(
            [], [], [], 
            color='cyan', linewidth=2, label='Trajectory'
        )
        
        # Add text displays
        self.position_text = self.ax.text2D(
            0.02, 0.02, 
            f'Position: X={self.rov_position[0]:.2f}, Y={self.rov_position[1]:.2f}, Z={self.rov_position[2]:.2f}',
            transform=self.ax.transAxes,
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        self.validity_text = self.ax.text2D(
            0.02, 0.07, 
            f'Position Valid: {self.position_valid}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='red',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Set initial axis limits with some padding
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_zlim(-5, 5)
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Create animation that calls update_plot periodically
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=False
        )
        
        self.get_logger().info('Blueye 3D visualization node initialized')
        
        # Show plot without blocking the ROS spin thread
        plt.show(block=False)
    
    def pose_callback(self, msg):
        """Callback for handling pose updates."""
        # Extract position and orientation
        pos = msg.pose.position
        quat = msg.pose.orientation
        
        new_position = np.array([pos.x, pos.y, pos.z])
        self.rov_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
        
        with self.plot_lock:
            # Only update if the position is valid
            if self.position_valid:
                # Check if ROV has actually started moving
                if not self.movement_started:
                    # Calculate distance from docking station
                    distance = np.linalg.norm(new_position - self.docking_station)
                    if distance > self.movement_threshold:
                        self.movement_started = True
                        # Add the first point to create a clean start
                        self.trajectory.append(self.rov_position.copy())
                
                # Update position after movement check
                self.rov_position = new_position
                
                # Only record trajectory after movement starts
                if self.movement_started:
                    # Only add if it's a new position
                    if len(self.trajectory) == 0 or not np.array_equal(new_position, self.trajectory[-1]):
                        self.trajectory.append(self.rov_position.copy())
                        
                        # Log position occasionally
                        if len(self.trajectory) % 10 == 0:
                            self.get_logger().info(
                                f"Position: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}, Points: {len(self.trajectory)}"
                            )
    
    def position_valid_callback(self, msg):
        """Callback for handling position validity updates."""
        self.position_valid = msg.data
    
    def update_plot(self, frame):
        """Update the visualization plot."""
        with self.plot_lock:
            # Update the ROV marker position
            if hasattr(self, 'rov_scatter'):
                self.rov_scatter._offsets3d = (
                    np.array([self.rov_position[0]]),
                    np.array([self.rov_position[1]]),
                    np.array([self.rov_position[2]])
                )
            
            # Update trajectory if there are points to plot
            if len(self.trajectory) >= 2:  # Need at least 2 points for a line
                # Remove old line and create new one (more reliable in 3D)
                try:
                    self.ax.lines.remove(self.trajectory_line)
                except ValueError:
                    # Line might have been removed already
                    pass
                
                # Plot a new line with all trajectory points
                traj_array = np.array(self.trajectory)
                self.trajectory_line, = self.ax.plot(
                    traj_array[:, 0], traj_array[:, 1], traj_array[:, 2],
                    color='cyan', linewidth=1.5, label='Trajectory'
                )
            
            # Update text displays
            self.position_text.set_text(
                f'Position: X={self.rov_position[0]:.2f}, Y={self.rov_position[1]:.2f}, Z={self.rov_position[2]:.2f}'
            )
            
            valid_color = 'green' if self.position_valid else 'red'
            self.validity_text.set_text(f'Position Valid: {self.position_valid}')
            self.validity_text.set_color(valid_color)
            
            # Auto-adjust limits to include all elements with proper zoom
            self.update_axis_limits()
    
    def update_axis_limits(self):
        """Update the plot limits to include all elements with proper padding"""
        # Get all points to include in limits
        all_points = []
        
        # Always include docking station and current position
        all_points.append(self.docking_station)
        all_points.append(self.rov_position)
        
        # Include trajectory points if available
        if len(self.trajectory) > 0:
            all_points.extend(self.trajectory)
        
        # Convert to numpy array
        all_points = np.array(all_points)
        
        # Check if we have enough points to set limits
        if len(all_points) >= 2:
            # Find min/max for all axes
            min_x, min_y, min_z = np.min(all_points, axis=0)
            max_x, max_y, max_z = np.max(all_points, axis=0)
            
            # Only adjust if the ROV is near the edge
            x_min, x_max = self.ax.get_xlim()
            y_min, y_max = self.ax.get_ylim()
            z_min, z_max = self.ax.get_zlim()
            
            padding = 2.0
            need_update = False
            
            if min_x < x_min + padding or max_x > x_max - padding:
                x_min = min_x - padding
                x_max = max_x + padding
                need_update = True
            
            if min_y < y_min + padding or max_y > y_max - padding:
                y_min = min_y - padding
                y_max = max_y + padding
                need_update = True
            
            if min_z < z_min + padding or max_z > z_max - padding:
                z_min = min_z - padding
                z_max = max_z + padding
                need_update = True
            
            if need_update:
                # Set new limits
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
                self.ax.set_zlim(z_min, z_max)

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    visualizer = Blueye3DVisualizerNode()
    
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