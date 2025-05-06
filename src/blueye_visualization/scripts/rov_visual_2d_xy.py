#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import threading

class MatplotlibVisualizer(Node):
    def __init__(self):
        super().__init__('matplotlib_visualizer')
        
        # Updated docking station position
        self.docking_station = np.array([-221.0, 59.0, -196.40])
        self.sunken_ship = np.array([-175.00, 180.00, -197.00])
        
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
        
        # Initialize trajectory (red line) and current ROV marker
        self.trajectory_line, = self.ax.plot(
            [], [],
            color='red', linewidth=1, label='Trajectory'
        )
        self.rov_marker, = self.ax.plot(
            self.rov_position[0], self.rov_position[1],
            'bo', markersize=6, label='ROV'
        )
        
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
        
        
        # Create a custom legend entry for the obstacle
        custom_legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                markersize=10, label='Obstacle', markeredgecolor='black')
        ]

        # Get the existing handles and labels
        handles, labels = self.ax.get_legend_handles_labels()
        self.ax.legend(handles + custom_legend_elements, labels + ['Obstacle'])

        # self.ax.legend()
        
        # Subscribe to ROV odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Create animation: update_plot is called every 100 ms
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        # Show plot in non-blocking mode
        plt.show(block=False)
    
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
    
    def update_plot(self, frame):
        # Update the ROV marker position (x and y only)
        self.rov_marker.set_data(self.rov_position[0], self.rov_position[1])
        
        # Only update trajectory line if there are points to plot
        if len(self.trajectory) > 0:
            traj_array = np.array(self.trajectory)
            # Update the trajectory line (x and y only)
            self.trajectory_line.set_data(traj_array[:, 0], traj_array[:, 1])
        else:
            # Empty trajectory - clear the line
            self.trajectory_line.set_data([], [])
        
        return self.trajectory_line, self.rov_marker

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