#!/usr/bin/env python3
"""
3D visualization for Blueye ROV position with mission state tracking.
Subscribes to the Blueye position topics and mission state updates.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Bool, Int32
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import threading

class Blueye3DVisualizerNode(Node):
    """ROS Node for visualizing Blueye drone position with mission states in 3D."""
    
    def __init__(self):
        """Initialize the node."""
        super().__init__('blueye_3d_visualizer_node')
        
        # ROV position and orientation
        self.rov_position = np.array([0.0, 0.0, 2.2])
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        
        # Docking station is at the origin
        self.docking_station = np.array([0.0, 0.0, 2.2])
        
        # Thread safety for plotting updates
        self.plot_lock = threading.Lock()
        
        # Trajectory tracking
        self.trajectory = []
        self.movement_started = False
        self.movement_threshold = 0.1  # meters
        self.position_valid = False
        
        # Mission state tracking - Updated with Transit 1 and Transit 2
        self.current_state = 0  # 0 means standby/pre-mission
        self.state_names = {
            0: "Trajectory",  # Standby/Pre-Mission
            1: "Undocking",
            2: "Pipeline Inspection", 
            3: "Transit 1",      # First transit
            4: "Docking",
            5: "Transit 2"       # Second transit - new state ID
        }
        self.state_colors = {
            0: 'orange',     # Trajectory/Standby
            1: 'blue',       # Undocking
            2: 'red',        # Pipeline Inspection
            3: 'green',      # Transit 1
            4: 'black',      # Docking
            5: 'purple'      # Transit 2 - new color
        }
        
        # Store trajectory segments by state (including states 0-5)
        self.state_segments = {state_id: [] for state_id in range(0, 6)}  # Extended to 6 states
        self.state_segment_lines = {}
        self.state_transition_points = []
        self.state_labels = []
        
        # Track the last known position to detect movement
        self.last_position = np.array([0.0, 0.0, 0.0])
        self.position_change_threshold = 0.05  # meters - minimum change to record
        
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
        
        # Subscribe to mission state topic
        self.state_subscription = self.create_subscription(
            Int32,
            '/mission_state',
            self.state_callback,
            10
        )
        
        # Setup 3D plot
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Remove grid for cleaner look
        self.ax.grid(False)
        
        # Set a good view angle
        self.ax.view_init(elev=20, azim=45)
        
        # Set axis labels and title
        self.ax.set_xlabel('X - East (m)')
        self.ax.set_ylabel('Y - North (m)')
        self.ax.set_zlabel('Z - Depth (m)')
        self.ax.set_title('Blueye ROV 3D Mission Visualization')
        
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
        
        # Initialize state transition markers
        self.state_transition_scatter = self.ax.scatter(
            [], [], [], 
            color='black', s=80, marker='o', 
            facecolors='none', edgecolors='black', linewidth=2,
            label='State Transitions', zorder=8
        )
        
        # Add text displays
        self.position_text = self.ax.text2D(
            0.02, 0.28, 
            f'Position: X={self.rov_position[0]:.2f}, Y={self.rov_position[1]:.2f}, Z={self.rov_position[2]:.2f}',
            transform=self.ax.transAxes,
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        self.validity_text = self.ax.text2D(
            0.02, 0.23, 
            f'Position Valid: {self.position_valid}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='red',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Mission state indicator
        self.state_text = self.ax.text2D(
            0.02, 0.18, 
            f'Mission State: {self.state_names.get(self.current_state, "Trajectory")}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='blue',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Movement status indicator
        self.movement_text = self.ax.text2D(
            0.02, 0.13, 
            f'Movement Started: {self.movement_started}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='blue',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Trajectory points counter
        self.trajectory_text = self.ax.text2D(
            0.02, 0.08, 
            f'Trajectory Points: {len(self.trajectory)}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='blue',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Current state color indicator
        self.state_color_text = self.ax.text2D(
            0.02, 0.03, 
            f'Current State Color: {self.state_colors.get(self.current_state, "orange")}',
            transform=self.ax.transAxes,
            fontsize=10,
            color=self.state_colors.get(self.current_state, 'orange'),
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Set initial axis limits with some padding
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_zlim(5, -5)  # Inverted Z for depth
        
        # Create state legend elements (including all states)
        self.state_legend_elements = [
            Line2D([0], [0], color=color, lw=2, label=f"{self.state_names[state_id]}")
            for state_id, color in self.state_colors.items()
        ]
        
        # Get existing handles and labels
        handles, labels = self.ax.get_legend_handles_labels()
        
        # Add legend with state colors (including all states)
        self.ax.legend(
            handles + self.state_legend_elements, 
            labels + [f"{self.state_names[state_id]}" for state_id in self.state_colors],
            loc='upper right'
        )
        
        # Create animation that calls update_plot periodically
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=False
        )
        
        self.get_logger().info('Blueye 3D mission visualization node initialized')
        self.get_logger().info('Tracking will start in Trajectory state (orange)')
        self.get_logger().info('State Colors: Orange=Trajectory, Blue=Undocking, Green=Transit1, Red=Pipeline, Purple=Transit2, Black=Docking')
        
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
                        self.get_logger().info(f'Movement detected! Distance from docking: {distance:.2f}m')
                        # Add the initial position to create a clean start
                        self.trajectory.append(self.rov_position.copy())
                        # Also add it to the current state segment (which starts as state 0)
                        self.state_segments[self.current_state].append(self.rov_position.copy())
                
                # Update position after movement check
                self.rov_position = new_position
                
                # Only record trajectory after movement starts
                if self.movement_started:
                    # Check if the position has changed significantly to avoid cluttering
                    position_change = np.linalg.norm(self.rov_position - self.last_position)
                    
                    if position_change > self.position_change_threshold:
                        self.trajectory.append(self.rov_position.copy())
                        
                        # Add the position to the current state's segment
                        # This now includes state 0 (standby/pre-mission)
                        self.state_segments[self.current_state].append(self.rov_position.copy())
                        
                        # Update last position
                        self.last_position = self.rov_position.copy()
                        
                        # Log position occasionally
                        if len(self.trajectory) % 20 == 0:
                            self.get_logger().info(
                                f"Position: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}, "
                                f"State: {self.state_names.get(self.current_state, 'Unknown')}, "
                                f"Points: {len(self.trajectory)}"
                            )
    
    def position_valid_callback(self, msg):
        """Callback for handling position validity updates."""
        self.position_valid = msg.data
        
        # Log when position validity changes
        if self.position_valid:
            self.get_logger().info('Position is now valid - tracking enabled')
        else:
            self.get_logger().warn('Position is invalid - tracking disabled')
    
    def state_callback(self, msg):
        """Callback for handling mission state updates."""
        state_id = msg.data
        
        # Accept states 0-5 (including 0 for standby and 5 for Transit 2)
        if 0 <= state_id <= 5:
            if state_id != self.current_state:
                old_state = self.current_state
                self.get_logger().info(
                    f"State changed from {self.state_names.get(old_state, 'Unknown')} to "
                    f"{self.state_names.get(state_id, 'Unknown')} (ID: {state_id})"
                )
                
                # If we have a position and movement has started, mark this as a transition point
                if len(self.trajectory) > 0 and self.movement_started:
                    transition_pos = self.trajectory[-1].copy()
                    self.state_transition_points.append((transition_pos, state_id))
                
                # Update current state
                self.current_state = state_id
        else:
            self.get_logger().warn(f"Received invalid state ID: {state_id}")
    
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
            
            # Update state trajectory segments (now including states 0-5)
            if len(self.trajectory) > 0:
                # Clear old trajectory lines
                for line in list(self.state_segment_lines.values()):
                    try:
                        self.ax.lines.remove(line)
                    except ValueError:
                        pass
                self.state_segment_lines.clear()
                
                # Draw trajectory segments for each state (including states 0-5)
                for state_id, segment in self.state_segments.items():
                    if len(segment) >= 2:  # Need at least 2 points for a line
                        segment_array = np.array(segment)
                        
                        # Different line style for standby state
                        line_style = '-' if state_id > 0 else '--'  # Dashed for standby state
                        line_width = 2.5 if state_id > 0 else 2.0
                        line_alpha = 0.8 if state_id > 0 else 0.6
                        
                        line, = self.ax.plot(
                            segment_array[:, 0], segment_array[:, 1], segment_array[:, 2],
                            color=self.state_colors[state_id],
                            linestyle=line_style,
                            linewidth=line_width,
                            alpha=line_alpha
                        )
                        self.state_segment_lines[state_id] = line
                
                # Update transition markers
                if self.state_transition_points:
                    points = np.array([p[0] for p in self.state_transition_points])
                    self.state_transition_scatter._offsets3d = (
                        points[:, 0], points[:, 1], points[:, 2]
                    )
                    
                    # Update or add text labels for state transitions (3D text)
                    # Remove existing labels
                    for label in self.state_labels:
                        try:
                            label.remove()
                        except:
                            pass
                    self.state_labels.clear()
                    
                    # Add new labels
                    for idx, (point, state_id) in enumerate(self.state_transition_points):
                        # Add 3D text label with some vertical offset
                        z_offset = 0.5 if idx % 2 == 0 else -0.5
                        label = self.ax.text(
                            point[0], point[1], point[2] + z_offset,
                            f"→ {self.state_names[state_id]}",
                            fontsize=8,
                            ha='center'
                        )
                        self.state_labels.append(label)
            
            # Update text displays
            self.position_text.set_text(
                f'Position: X={self.rov_position[0]:.2f}, Y={self.rov_position[1]:.2f}, Z={self.rov_position[2]:.2f}'
            )
            
            valid_color = 'green' if self.position_valid else 'red'
            self.validity_text.set_text(f'Position Valid: {self.position_valid}')
            self.validity_text.set_color(valid_color)
            
            # Update state text
            current_state_name = self.state_names.get(self.current_state, "Unknown")
            state_color = self.state_colors.get(self.current_state, 'blue')
            self.state_text.set_text(f'Mission State: {current_state_name}')
            self.state_text.set_color(state_color)
            
            # Update movement status text
            movement_color = 'green' if self.movement_started else 'red'
            self.movement_text.set_text(f'Movement Started: {self.movement_started}')
            self.movement_text.set_color(movement_color)
            
            # Update trajectory points counter
            self.trajectory_text.set_text(f'Trajectory Points: {len(self.trajectory)}')
            
            # Update state color indicator
            current_color = self.state_colors.get(self.current_state, 'orange')
            self.state_color_text.set_text(f'Current State Color: {current_color}')
            self.state_color_text.set_color(current_color)
            
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
        
        # Include transition points if available
        if self.state_transition_points:
            transition_positions = [p[0] for p in self.state_transition_points]
            all_points.extend(transition_positions)
        
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
                # Set new limits (note: Z is inverted for depth visualization)
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
                self.ax.set_zlim(z_max, z_min)  # Inverted Z for depth

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