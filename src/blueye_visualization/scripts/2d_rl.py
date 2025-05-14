#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import threading
import time

class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')
        
        # Updated docking station position
        self.docking_station = np.array([-223.0, 60.0, -196.40])
        self.sunken_ship = np.array([-175.00, 180.00, -197.00])
        
        self.undocking_vertical_offset = 0    # Default value, can be adjusted
        self.undocking_horizontal_offset = -5
        
        # ROV initial position at docking station and orientation
        self.rov_position = self.docking_station.copy()
        self.rov_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Trajectory: initialize as empty until ROV actually moves
        self.trajectory = []
        self.movement_started = False
        self.movement_threshold = 0.5

        # Updated pipeline points extracted from the behavior tree XML
        self.pipeline = np.array([
            [-210.50, 108.00, -195.00],  # Point 1
            [-210.50, 134.50, -195.00],  # Point 2
            [-233.00, 134.50, -195.00],  # Point 3
            [-233.00, 130.50, -195.00],  # Point 4
            [-251.00, 130.50, -195.00],  # Point 5
        ])
        
        # Add obstacle position - in front of docking station
        self.obstacles = [
            {"position": np.array([-211.6, 91.1, -195.0]), "radius": 0.5},  # First obstacle
            {"position": np.array([-215.5, 78.1, -195.0]), "radius": 1.0}   # Second obstacle
        ]
        
        # Set waypoints as the pipeline points
        self.waypoints = self.pipeline.copy()
        
        # Mission state tracking - UPDATED for RL states without Transit_2
        self.current_state = 0
        self.state_names = {
            1: "UnDocking",
            2: "Transit_1",
            3: "PipeLineInspection", 
            4: "WreckageInspection",
            5: "Homing",
            6: "Docking"
        }
        self.state_colors = {
            1: 'blue',
            2: 'green',
            3: 'red',
            4: 'orange',
            5: 'brown',
            6: 'black'
        }
        
        # Store trajectory segments by state - UPDATED for 6 states
        self.state_segments = {state_id: [] for state_id in range(1, 7)}
        self.state_segment_lines = {}
        self.state_transition_points = []
        self.state_labels = []
        
        # Setup 2D plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111)  # 2D axes
        
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_title('ROV Mission Visualization (2D)')
        
        # Docking station
        self.ax.scatter(
            self.docking_station[0], self.docking_station[1],
            color='red', s=200, marker='s', label='Docking Station'
        )
        
        # Sunken ship
        self.ax.scatter(
            self.sunken_ship[0], self.sunken_ship[1],
            color='brown', s=250, marker='*', label='Shipwreck'
        )
        
        # Pipeline (connects the waypoints)
        self.ax.plot(
            self.pipeline[:, 0], self.pipeline[:, 1],
            color='orange', linewidth=3, label='Pipeline'
        )
        
        # Waypoints
        if self.waypoints.size > 0:
            self.ax.scatter(
                self.waypoints[:, 0], self.waypoints[:, 1],
                color='green', s=100, marker='^', label='Waypoints'
            )
            
        for obstacle in self.obstacles:
            obstacle_circle = patches.Circle(
                (obstacle["position"][0], obstacle["position"][1]),
                radius=obstacle["radius"],
                color='red',
                alpha=0.8,
                edgecolor='black',
                linewidth=1.0,
                zorder=5
            )
            self.ax.add_patch(obstacle_circle)
        
        # Initialize ROV marker
        self.rov_marker, = self.ax.plot(
            self.rov_position[0], self.rov_position[1],
            'bo', markersize=6, label='ROV'
        )
        
        # Initialize state transition markers (will be populated during animation)
        self.state_transition_markers, = self.ax.plot(
            [], [], 'ko', markersize=8, markerfacecolor='none'
        )
        
        # Remove the state text annotation
        self.state_text = None
        
        obstacle_points = np.array([obstacle["position"][:2] for obstacle in self.obstacles])
        all_points = np.vstack((
            self.docking_station[:2],
            self.pipeline[:, :2],
            self.rov_position[:2],
            self.sunken_ship[:2],
            obstacle_points
        ))
        
        min_x, min_y = np.min(all_points, axis=0)
        max_x, max_y = np.max(all_points, axis=0)
        
        # Add some padding
        padding = 20
        self.ax.set_xlim(min_x - padding, max_x + padding)
        self.ax.set_ylim(min_y - padding, max_y + padding)
        
        # Create state legend elements
        self.state_legend_elements = [
            Line2D([0], [0], color=color, lw=2, label=f"{self.state_names[state_id]}")
            for state_id, color in self.state_colors.items()
        ]
        
        # Create a custom legend entry for the obstacle
        custom_legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                markersize=10, label='Obstacle', markeredgecolor='black')
        ]

        # Get the existing handles and labels
        handles, labels = self.ax.get_legend_handles_labels()
        
        # Set the legend to appear in the top left corner with state colors included
        self.ax.legend(handles + custom_legend_elements + self.state_legend_elements, 
                    labels + ['Obstacle'] + [f"{self.state_names[state_id]}" for state_id in self.state_colors],
                    loc='upper left')  # Set legend to top left
        
        # Subscribe to ROV odometry topic
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
        
        # Create animation: update_plot is called every 100 ms
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        # Show plot in non-blocking mode
        plt.show(block=False)
        
    def state_callback(self, msg):
        state_id = msg.data
        if 1 <= state_id <= 6:  # Valid state range - UPDATED for 6 states
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
    
    def odometry_callback(self, msg):
        # Update ROV position (x, y, z) and orientation
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        new_position = np.array([pos.x, pos.y, pos.z])
        self.rov_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
        
        # Check if ROV has actually started moving
        if not self.movement_started:
            # Calculate distance from docking station
            distance = np.linalg.norm(new_position - self.docking_station)
            if distance > self.movement_threshold:
                self.movement_started = True
                # Add the first point at docking station to create a clean start
                self.trajectory.append(self.docking_station.copy())
        
        # Update position after movement check
        self.rov_position = new_position
        
        # Only record trajectory after movement starts
        if self.movement_started:
            self.trajectory.append(self.rov_position.copy())
            
            # Add the position to the current state's segment
            if self.current_state > 0:
                self.state_segments[self.current_state].append(self.rov_position.copy())
    
    def update_plot(self, frame):
        # Update the ROV marker position (x and y only)
        self.rov_marker.set_data(self.rov_position[0], self.rov_position[1])
        
        # Only update trajectory segments if there are points to plot
        if len(self.trajectory) > 0:
            # Update state segments
            for state_id, segment in self.state_segments.items():
                if len(segment) > 1:  # Need at least 2 points to draw a line
                    segment_array = np.array(segment)
                    
                    # Create line if it doesn't exist, otherwise update it
                    if state_id not in self.state_segment_lines:
                        # Create the line without adding to legend (legend already set up)
                        line, = self.ax.plot(
                            segment_array[:, 0], segment_array[:, 1],
                            color=self.state_colors[state_id],
                            linewidth=2
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
                    
                    # Special handling for UnDocking state
                    if state_id == 1:  # UnDocking
                        # Use the configurable offsets
                        vertical_offset = self.undocking_vertical_offset
                        horizontal_offset = self.undocking_horizontal_offset
                        
                        # Add a text label with configurable positioning
                        label = self.ax.text(
                            point[0] + horizontal_offset, 
                            point[1] + vertical_offset,
                            f"→ {self.state_names[state_id]}",
                            fontsize=9,
                            ha='center',
                            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2')
                        )
                    else:
                        # For other states, keep the original positioning
                        vertical_offset = 3 if idx % 2 == 0 else -6
                        
                        # Add a text label with normal positioning
                        label = self.ax.text(
                            point[0], 
                            point[1] + vertical_offset,
                            f"→ {self.state_names[state_id]}",
                            fontsize=9,
                            ha='center',
                            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2')
                        )
                    
                    self.state_labels.append(label)
            
            # Return updated artists
            return self.rov_marker, self.state_transition_markers, *self.state_segment_lines.values()
        
def main(args=None):
    rclpy.init(args=args)
    visualizer = MatplotlibVisualizer()
    
    # Run ROS spin in a separate thread so the callbacks are processed
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