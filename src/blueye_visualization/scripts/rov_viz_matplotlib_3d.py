#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import threading
from matplotlib.patches import Circle, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.lines import Line2D

class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')
        
        # We'll set the initial position from the first odometry message
        self.initial_position_set = False
        
        # Initialize placeholders - these will be updated with actual values from odometry
        self.docking_station = np.array([-221.0, 59.0, 194.90])  
        self.rov_position = self.docking_station.copy()
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Thread safety for plotting updates
        self.plot_lock = threading.Lock()
        
        # Sunken ship position
        self.sunken_ship = np.array([-175.00, 180.00, 195.00])
        
        # Updated pipeline points extracted from the behavior tree XML
        self.pipeline = np.array([
            [-210.50, 108.00, 195.00],  # Point 1
            [-210.50, 134.50, 195.00],  # Point 2
            [-233.00, 134.50, 195.00],  # Point 3
            [-233.00, 130.50, 195.00],  # Point 4
            [-251.00, 130.50, 195.00],  # Point 5
        ])
        
        # Set waypoints as the pipeline points
        self.waypoints = self.pipeline.copy()
        
        self.obstacles = [
            {"position": np.array([-211.6, 91.1, 195.0]), "radius": 0.5, "height": 3.0},   # First new obstacle
            {"position": np.array([-215.5, 78.1, 195.0]), "radius": 1.0, "height": 3.0}    # Second new obstacle
        ]
        
        # Store positions over time 
        self.trajectory = []
        
        # Debug counters
        self.message_count = 0
        
        # Setup plot (create figure and 3D axes)
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Remove grid
        self.ax.grid(False)
        
        # Set a good view angle for the 3D plot
        self.ax.view_init(elev=30, azim=45)
        
        # Initialize static elements immediately
        
        # Plot docking station
        self.dock_scatter = self.ax.scatter(
            self.docking_station[0], self.docking_station[1], self.docking_station[2],
            color='red', s=200, marker='s', label='Docking Station'
        )
        
        # Pipeline
        self.pipeline_line, = self.ax.plot(
            self.pipeline[:, 0], self.pipeline[:, 1], self.pipeline[:, 2],
            color='orange', linewidth=3, label='Pipeline'
        )
        
        # Waypoints
        self.waypoints_scatter = self.ax.scatter(
            self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2],
            color='green', s=100, marker='^', label='Waypoints'
        )
        
        # Sunken ship
        self.ship_scatter = self.ax.scatter(
            self.sunken_ship[0], self.sunken_ship[1], self.sunken_ship[2],
            color='brown', s=250, marker='*', label='Sunken Ship'
        )
        
        self.obstacle_plots = []
        for obstacle in self.obstacles:
            X, Y, Z = self.create_cylinder(
                center=[obstacle["position"][0], obstacle["position"][1], obstacle["position"][2]],
                radius=obstacle["radius"],
                height=obstacle["height"]
            )
            obstacle_plot = self.ax.plot_surface(X, Y, Z, color='red', alpha=0.7)
            self.obstacle_plots.append(obstacle_plot)
        
        # For ROV position - start at docking station with larger marker
        self.rov_scatter = self.ax.scatter(
            self.rov_position[0], self.rov_position[1], self.rov_position[2],
            color='blue', s=50, marker='o', label='ROV', zorder=10
        )
        
        # Initialize trajectory line - empty at start
        self.trajectory_line, = self.ax.plot([], [], [], color='cyan', linewidth=2, label='Trajectory')
        
        # Custom legend entry for the obstacle
        custom_legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                markersize=10, label='Obstacle', markeredgecolor='black')
        ]
        
        # Get handles and labels
        handles, labels = self.ax.get_legend_handles_labels()
        handles.extend(custom_legend_elements)
        labels.append('Obstacle')
        self.ax.legend(handles, labels)
        
        # Update axis limits to include all elements
        self.update_axis_limits()
        
        # Set axis labels and title
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_zlabel('Depth (m)')
        self.ax.set_title('ROV 3D Mission Visualization')
        
        # Subscribe to ROV position topic
        self.subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create animation that calls update_plot periodically - with a slower refresh rate
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=False
        )
        
        # print("Visualization ready. Waiting for ROV position data...")
        
        # Show plot without blocking the ROS spin thread
        plt.show(block=False)
    
    def create_cylinder(self, center, radius, height, resolution=20):
        """Create a 3D cylinder for visualization"""
        x_center, y_center, z_bottom = center
        z_top = z_bottom - height  # Note: in the visualization, z increases downward
        
        # Create circles at the top and bottom of cylinder
        theta = np.linspace(0, 2*np.pi, resolution)
        x = radius * np.cos(theta) + x_center
        y = radius * np.sin(theta) + y_center
        
        # Create the cylinder sides
        X = np.vstack([x, x])
        Y = np.vstack([y, y])
        Z = np.vstack([np.ones(resolution)*z_top, np.ones(resolution)*z_bottom])
        
        return X, Y, Z
    
    def odometry_callback(self, msg):
        # Extract position and orientation data
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        
        # Convert to numpy array for easier handling
        new_position = np.array([pos.x, pos.y, pos.z])
        self.rov_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
        
        # Debug print all received positions
        # print(f"Received position: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]")
        
        # If this is our first position, set it as initial position
        with self.plot_lock:
            if not self.initial_position_set:
                self.initial_position_set = True
                # print(f"ROV initial position set to: {new_position}")
                # Add the starting point to trajectory
                self.trajectory.append(new_position.copy())
            else:
                # Only add points to trajectory if they're different from the last one
                if len(self.trajectory) == 0 or not np.array_equal(new_position, self.trajectory[-1]):
                    self.trajectory.append(new_position.copy())
                    
                    if len(self.trajectory) % 20 == 0:  # Print less frequently
                        print(f"Trajectory now has {len(self.trajectory)} points")
            
            # Update current position
            self.rov_position = new_position
    
    def update_plot(self, frame):
        with self.plot_lock:
            # Debug print the position being updated
            # print(f"Updating ROV position to: {self.rov_position}")
            
            # Update the ROV marker position directly
            if hasattr(self, 'rov_scatter'):
                self.rov_scatter._offsets3d = (
                    np.array([self.rov_position[0]]),
                    np.array([self.rov_position[1]]),
                    np.array([self.rov_position[2]])
                )
            
            # Draw trajectory if there are points to plot
            if len(self.trajectory) >= 2:  # Need at least 2 points for a line
                # Create new trajectory line each time (more reliable in 3D)
                try:
                    # This handles recreating the line completely
                    self.ax.lines.remove(self.trajectory_line)
                except ValueError:
                    # Line might have been removed already
                    pass
                
                # Plot a new line with all points
                traj_array = np.array(self.trajectory)
                self.trajectory_line, = self.ax.plot(
                    traj_array[:, 0], traj_array[:, 1], traj_array[:, 2],
                    color='cyan', linewidth=1, label='Trajectory'
                )
        
        # No need to return artists when blit=False
        return ()
    
    def update_axis_limits(self):
        """Update the plot limits to include all elements with proper zoom"""
        # Get all points to include in limits
        all_points = []
        
        # Always include docking station and current position
        all_points.append(self.docking_station)
        all_points.append(self.rov_position)
        
        # Include pipeline and other elements
        all_points.extend(self.pipeline)
        all_points.append(self.sunken_ship)
        for obstacle in self.obstacles:
            all_points.append(obstacle["position"])        
        # Convert to numpy array
        all_points = np.array(all_points)
        
        # Find min/max for all axes
        min_x, min_y, min_z = np.min(all_points, axis=0)
        max_x, max_y, max_z = np.max(all_points, axis=0)
        
        # Add padding
        padding_xy = 20
        padding_z = 10
        
        # Set limits with FLIPPED Z-AXIS (max first, then min)
        self.ax.set_xlim(min_x - padding_xy, max_x + padding_xy)
        self.ax.set_ylim(min_y - padding_xy, max_y + padding_xy)
        self.ax.set_zlim(max_z + padding_z, min_z - padding_z)

def main(args=None):
    rclpy.init(args=args)
    visualizer = MatplotlibVisualizer()
    
    # Start ROS spin loop in a separate thread so callbacks are processed
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