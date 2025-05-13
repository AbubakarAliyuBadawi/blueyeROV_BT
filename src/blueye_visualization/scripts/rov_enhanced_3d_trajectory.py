#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32  # Add this import for mission state
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
        self.docking_station = np.array([-223.0, 60.0, 194.90])  
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
        
        # Mission state tracking - add this section
        self.current_state = 0
        self.state_names = {
            1: "UnDocking",
            2: "Transit_1",
            3: "PipeLineInspection",
            4: "Transit_2",
            5: "WreckageInspection",
            6: "Homing",
            7: "Docking"
        }
        self.state_colors = {
            1: 'blue',
            2: 'green',
            3: 'red',
            4: 'purple',
            5: 'orange',
            6: 'brown',
            7: 'black'
        }
        
        # Store trajectory segments by state
        self.state_segments = {state_id: [] for state_id in range(1, 8)}
        self.state_segment_lines = {}
        self.state_transition_points = []
        self.state_labels = []
        
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
        
        # Initialize state transition markers
        self.state_transition_markers = self.ax.scatter(
            [], [], [], color='black', s=50, marker='o', zorder=9
        )
        
        # Custom legend entry for the obstacle
        custom_legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                markersize=10, label='Obstacle', markeredgecolor='black')
        ]
        
        # Add state colors to legend
        state_legend_elements = [
            Line2D([0], [0], color=color, lw=2, label=f"{self.state_names[state_id]}")
            for state_id, color in self.state_colors.items()
        ]
        
        # Get handles and labels
        handles, labels = self.ax.get_legend_handles_labels()
        handles.extend(custom_legend_elements + state_legend_elements)
        labels.extend(['Obstacle'] + [f"{self.state_names[state_id]}" for state_id in self.state_colors])
        self.ax.legend(handles, labels, loc='upper left')
        
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
        
        # Subscribe to mission state topic
        self.state_subscription = self.create_subscription(
            Int32,
            '/mission_state',
            self.state_callback,
            10
        )
        
        # Create animation that calls update_plot periodically - with a slower refresh rate
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=False
        )
        
        # Show plot without blocking the ROS spin thread
        plt.show(block=False)
    
    def state_callback(self, msg):
        """Callback for mission state updates"""
        state_id = msg.data
        if 1 <= state_id <= 7:  # Valid state range
            if state_id != self.current_state:
                self.get_logger().info(f"State changed to: {self.state_names.get(state_id)} (ID: {state_id})")
                
                # If we have a position, mark this as a transition point
                if len(self.trajectory) > 0:
                    transition_pos = self.trajectory[-1].copy()
                    self.state_transition_points.append((transition_pos, state_id))
                
                # Update current state
                self.current_state = state_id
        else:
            self.get_logger().warn(f"Received invalid state ID: {state_id}")
    
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
        
        # If this is our first position, set it as initial position
        with self.plot_lock:
            if not self.initial_position_set:
                self.initial_position_set = True
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
            
            # Add the position to the current state's segment
            if self.current_state > 0:
                self.state_segments[self.current_state].append(new_position.copy())
    
    def update_plot(self, frame):
        with self.plot_lock:
            # Update the ROV marker position directly
            if hasattr(self, 'rov_scatter'):
                self.rov_scatter._offsets3d = (
                    np.array([self.rov_position[0]]),
                    np.array([self.rov_position[1]]),
                    np.array([self.rov_position[2]])
                )
            
            # Update state segments 
            for state_id, segment in self.state_segments.items():
                if len(segment) >= 2:  # Need at least 2 points for a line
                    # Create or update line for this state segment
                    if state_id in self.state_segment_lines:
                        # Remove the old line
                        try:
                            self.ax.lines.remove(self.state_segment_lines[state_id])
                        except ValueError:
                            pass  # Line might have been removed already
                    
                    # Create a new line for this segment
                    segment_array = np.array(segment)
                    self.state_segment_lines[state_id], = self.ax.plot(
                        segment_array[:, 0], segment_array[:, 1], segment_array[:, 2],
                        color=self.state_colors[state_id], linewidth=2
                    )
            
            # Update state transition markers
            if self.state_transition_points:
                # Get only the positional part of the transition points
                points = np.array([p[0] for p in self.state_transition_points])
                
                # Update the markers
                self.state_transition_markers._offsets3d = (
                    points[:, 0],
                    points[:, 1],
                    points[:, 2]
                )
                
                # Update or add text labels for state transitions
                while len(self.state_labels) < len(self.state_transition_points):
                    idx = len(self.state_labels)
                    point, state_id = self.state_transition_points[idx]
                    
                    # In 3D, create a text annotation at the transition point
                    # Some 3D text positioning may require adjustments based on your view
                    label = self.ax.text(
                        point[0], point[1], point[2] + 3,  # Add some vertical offset in 3D
                        f"→ {self.state_names[state_id]}",
                        color=self.state_colors[state_id],
                        fontsize=9,
                        ha='center',
                        bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2')
                    )
                    self.state_labels.append(label)
        
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