#!/usr/bin/env python3
"""
2D visualization for Blueye ROV position with mission state tracking.
Subscribes to the Blueye position topics and mission state updates.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Bool, Int32
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import threading

class BlueyeVisualizerNode(Node):
    """ROS Node for visualizing Blueye drone position with mission states in 2D."""
    
    def __init__(self):
        """Initialize the node."""
        super().__init__('blueye_visualizer_node')
        
        # ROV position and orientation
        self.rov_position = np.array([0.0, 0.0, 0.0])
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        
        # Docking station is at the origin
        self.docking_station = np.array([-0.1, 1.5, 1.0])
        
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
        
        # Setup 2D plot
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111)
        
        self.ax.set_xlabel('X - East (m)')
        self.ax.set_ylabel('Y - North (m)')
        self.ax.set_title('Blueye ROV Mission Visualization')
        
        # Equal aspect ratio to avoid distortion
        self.ax.set_aspect('equal')
        
        # Docking station
        self.ax.scatter(
            self.docking_station[0], self.docking_station[1],
            color='red', s=200, marker='s', label='Docking Station'
        )
        
        # Initialize ROV marker
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
        
        # Initialize state transition markers
        self.state_transition_markers, = self.ax.plot(
            [], [], 'ko', markersize=8, markerfacecolor='none', label='State Transitions'
        )
        
        # Position text display
        self.position_text = self.ax.text(
            0.02, 0.20, 
            f'Position: X={self.rov_position[0]:.2f}m, Y={self.rov_position[1]:.2f}m',
            transform=self.ax.transAxes,
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Validity indicator
        self.validity_text = self.ax.text(
            0.02, 0.15, 
            f'Position Valid: {self.position_valid}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='red',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Mission state indicator
        self.state_text = self.ax.text(
            0.02, 0.10, 
            f'Mission State: {self.state_names.get(self.current_state, "Standby")}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='blue',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Movement status indicator
        self.movement_text = self.ax.text(
            0.02, 0.05, 
            f'Movement Started: {self.movement_started}',
            transform=self.ax.transAxes,
            fontsize=10,
            color='blue',
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Set initial axis limits
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        
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
        
        # Create animation (non-blitting due to dynamic legend updates)
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        self.get_logger().info('Blueye mission visualization node initialized')
        self.get_logger().info('Tracking will start in Trajectory state (orange)')
    
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
        
        # Update state trajectory segments (now including states 0-5)
        if len(self.trajectory) > 0:
            for state_id, segment in self.state_segments.items():
                if len(segment) > 1:  # Need at least 2 points to draw a line
                    segment_array = np.array(segment)
                    
                    # Create line if it doesn't exist, otherwise update it
                    if state_id not in self.state_segment_lines:
                        line_style = '-' if state_id > 0 else '--'  # Dashed for standby state
                        line_width = 2.5 if state_id > 0 else 2.0
                        line_alpha = 0.8 if state_id > 0 else 0.6
                        
                        line, = self.ax.plot(
                            segment_array[:, 0], segment_array[:, 1],
                            color=self.state_colors[state_id],
                            linestyle=line_style,
                            linewidth=line_width,
                            alpha=line_alpha
                        )
                        self.state_segment_lines[state_id] = line
                    else:
                        self.state_segment_lines[state_id].set_data(
                            segment_array[:, 0], segment_array[:, 1]
                        )
            
            # Update transition markers
            if self.state_transition_points:
                points = np.array([p[0] for p in self.state_transition_points])
                self.state_transition_markers.set_data(points[:, 0], points[:, 1])
                
                # Update or add text labels for state transitions
                while len(self.state_labels) < len(self.state_transition_points):
                    idx = len(self.state_labels)
                    point, state_id = self.state_transition_points[idx]
                    
                    # Offset for label positioning
                    # vertical_offset = 3 if idx % 2 == 0 else -6
                    
                    # Add a text label
                    label = self.ax.text(
                        point[0], 
                        point[1],
                        f"→ {self.state_names[state_id]}",
                        fontsize=9,
                        ha='center',
                        bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2')
                    )
                    
                    self.state_labels.append(label)
        
        # Update position text
        self.position_text.set_text(f'Position: X={x:.2f}m, Y={y:.2f}m, Z={self.rov_position[2]:.2f}m')
        
        # Update validity text
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