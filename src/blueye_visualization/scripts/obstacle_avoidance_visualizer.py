#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import threading
import time
from scipy.spatial.transform import Rotation as R

class ObstacleAvoidanceVisualizer(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_visualizer')
        
        # Thread safety
        self.plot_lock = threading.Lock()
        
        # Start time for plotting
        self.start_time = time.time()
        
        # Define fixed start and end points
        self.start_point_coords = np.array([-217.30, 71.90, 194.90])
        self.end_point_coords = np.array([-210.50, 97.00, 194.90])
        
        # Obstacles (positions from rov_visual_2d_xy.py)
        self.obstacles = [
            {"position": np.array([-211.6, 91.1, 194.90]), "radius": 0.5, "label": "Obstacle 2"},
            {"position": np.array([-215.5, 78.1, 194.90]), "radius": 1.0, "label": "Obstacle 1"}
        ]
        
        # ROV tracking
        self.rov_positions = []  # Store ROV positions for plotting trajectory
        self.current_position = None
        self.mission_started = False
        
        # Obstacle avoidance tracking
        self.detected_obstacles = []  # Store points where obstacles were detected
        self.oa_waypoints = []  # List to store obstacle avoidance waypoints
        self.oa_path = []  # For the OA path line
        
        # Setup the figure
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_title('Obstacle Avoidance')
        self.ax.set_xlabel('East position [m]')
        self.ax.set_ylabel('North position [m]')
        
        # Initialize plot elements with empty data
        self.rov_line, = self.ax.plot([], [], 'k-', linewidth=2, label='ROV estimated position')
        self.start_point = self.ax.scatter(self.start_point_coords[0], self.start_point_coords[1], 
                                          marker='*', color='k', s=150, label='Start point')
        self.end_point = self.ax.scatter(self.end_point_coords[0], self.end_point_coords[1], 
                                        marker='*', color='blue', s=150, label='Desired point')
        
        # Create the desired path - straight line from start to end
        self.desired_path_line, = self.ax.plot([self.start_point_coords[0], self.end_point_coords[0]], 
                                               [self.start_point_coords[1], self.end_point_coords[1]], 
                                               'g--', linewidth=1.5, label='Desired path of Transit')
        
        self.oa_path_line, = self.ax.plot([], [], 'm--', linewidth=1.5, label='Desired path of OA')
        self.oa_waypoints_scatter = self.ax.scatter([], [], marker='+', color='blue', s=100, label='Desired waypoints of OA')
        self.obstacle_detection_scatter = self.ax.scatter([], [], marker='x', color='orange', s=100, label='Obstacle detected')
        
        # Add annotations to start and end points
        self.ax.annotate(
            "η_start",
            (self.start_point_coords[0], self.start_point_coords[1]),
            xytext=(10, 10),
            textcoords='offset points',
            fontsize=10
        )
        
        self.ax.annotate(
            "η_end",
            (self.end_point_coords[0], self.end_point_coords[1]),
            xytext=(10, 10),
            textcoords='offset points',
            fontsize=10
        )
        
        # Add obstacles to the plot
        for i, obstacle in enumerate(self.obstacles):
            circle = patches.Circle(
                (obstacle["position"][0], obstacle["position"][1]),
                radius=obstacle["radius"],
                color='red',
                alpha=0.7,
                label=f'Obstacle {i+1}'
            )
            self.ax.add_patch(circle)
            
            # Add obstacle label
            self.ax.annotate(
                f"Obstacle {i+1}",
                (obstacle["position"][0], obstacle["position"][1] + obstacle["radius"] + 0.5),
                color='red',
                fontsize=10,
                ha='center'
            )
        
        # Legend
        self.ax.legend(loc='upper right')
        
        # Subscribe to odometry for ROV position
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odometry_callback,
            10
        )
        
        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        # Set initial axis limits based on start, end, and obstacles
        self.initial_axis_limits()
        
        # Generate obstacle avoidance waypoints
        # self.generate_oa_waypoints()
        
        self.get_logger().info("Obstacle Avoidance Visualizer initialized")
    
    def initial_axis_limits(self):
        """Set initial axis limits based on start, end points and obstacles"""
        points = [self.start_point_coords[:2], self.end_point_coords[:2]]
        for obstacle in self.obstacles:
            points.append(obstacle["position"][:2])
        
        points = np.array(points)
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        
        padding = 10.0  # meters
        self.ax.set_xlim(x_min - padding, x_max + padding)
        self.ax.set_ylim(y_min - padding, y_max + padding)
    
    def odometry_callback(self, msg):
        with self.plot_lock:
            # Extract ROV position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # Update current position
            self.current_position = np.array([x, y, z])
            
            # Initialize mission if not started
            if not self.mission_started:
                self.mission_started = True
            
            # Store position for trajectory
            self.rov_positions.append(self.current_position.copy())
            
            # Check distance to obstacles and detect if close
            for obstacle in self.obstacles:
                distance = np.linalg.norm(self.current_position[:2] - obstacle["position"][:2])
                detection_threshold = obstacle["radius"] + 3.0  # Detect within 3m of obstacle
                
                # If close to obstacle and in a new location, mark as detected
                if distance < detection_threshold:
                    # Only add detection point if we don't already have one nearby
                    if not self.detected_obstacles or np.min([np.linalg.norm(np.array(d) - self.current_position[:2]) for d in self.detected_obstacles]) > 1.0:
                        self.detected_obstacles.append(self.current_position[:2].copy())
                        self.get_logger().info(f"Obstacle detected near position {self.current_position[:2]}")
    
    def generate_oa_waypoints(self):
        """Generate obstacle avoidance waypoints for visualization"""
        # Direction vector from start to end
        direction = self.end_point_coords[:2] - self.start_point_coords[:2]
        distance = np.linalg.norm(direction)
        direction = direction / distance  # Normalize
        
        # Perpendicular vector for creating waypoints
        perp = np.array([-direction[1], direction[0]])
        
        # First obstacle
        obstacle1 = self.obstacles[0]  # Obstacle 1
        
        # Create detection point for obstacle 1
        detection_point1 = obstacle1["position"][:2] - direction * (obstacle1["radius"] + 2.0)
        self.detected_obstacles.append(detection_point1)
        
        # Create OA waypoints for first obstacle
        self.oa_waypoints = []
        self.oa_path = []
        
        # Starting point
        self.oa_path.append(self.start_point_coords[:2])
        
        # First waypoint - approach obstacle 1
        wp1 = detection_point1
        self.oa_waypoints.append(wp1)
        self.oa_path.append(wp1)
        
        # Second waypoint - go around obstacle 1
        wp2 = obstacle1["position"][:2] + perp * (obstacle1["radius"] + 1.5)
        self.oa_waypoints.append(wp2)
        self.oa_path.append(wp2)
        
        # Third waypoint - after obstacle 1
        wp3 = obstacle1["position"][:2] + direction * (obstacle1["radius"] + 3.0)
        self.oa_waypoints.append(wp3)
        self.oa_path.append(wp3)
        
        # Second obstacle
        obstacle2 = self.obstacles[1]  # Obstacle 2
        
        # Check if second obstacle is in the path after navigating around first obstacle
        # This is a simplified check - you'd use a more sophisticated algorithm in practice
        # Direction from wp3 to end
        direction_to_end = self.end_point_coords[:2] - wp3
        distance_to_end = np.linalg.norm(direction_to_end)
        direction_to_end = direction_to_end / distance_to_end
        
        # Project obstacle2 onto this path
        to_obstacle2 = obstacle2["position"][:2] - wp3
        projection_length = np.dot(to_obstacle2, direction_to_end)
        
        # If obstacle2 is on the way to the end and close enough to the path
        if 0 < projection_length < distance_to_end:
            # Calculate closest point on path to obstacle2
            closest_point = wp3 + direction_to_end * projection_length
            
            # Distance from obstacle2 to path
            perpendicular_distance = np.linalg.norm(obstacle2["position"][:2] - closest_point)
            
            # If the obstacle is close enough to interfere with the path
            if perpendicular_distance < obstacle2["radius"] + 2.0:
                # Create a detection point for obstacle 2
                detection_point2 = obstacle2["position"][:2] - direction_to_end * (obstacle2["radius"] + 2.0)
                self.detected_obstacles.append(detection_point2)
                
                # Fourth waypoint - approach obstacle 2
                wp4 = detection_point2
                self.oa_waypoints.append(wp4)
                self.oa_path.append(wp4)
                
                # Fifth waypoint - go around obstacle 2
                # Use the opposite direction of the first obstacle to make an interesting path
                wp5 = obstacle2["position"][:2] - perp * (obstacle2["radius"] + 1.5)
                self.oa_waypoints.append(wp5)
                self.oa_path.append(wp5)
                
                # Sixth waypoint - after obstacle 2
                wp6 = obstacle2["position"][:2] + direction_to_end * (obstacle2["radius"] + 2.0)
                self.oa_waypoints.append(wp6)
                self.oa_path.append(wp6)
        
        # Add the endpoint
        self.oa_path.append(self.end_point_coords[:2])
        
        # Add notations for the waypoints
        for i, wp in enumerate(self.oa_waypoints):
            self.ax.annotate(
                f"η_OA^{i}",
                (wp[0], wp[1]),
                xytext=(5, 5),
                textcoords='offset points',
                fontsize=10
            )
        
        # Update the plot with OA waypoints and path
        if self.oa_waypoints:
            self.oa_waypoints_scatter.set_offsets(np.array(self.oa_waypoints))
            self.oa_path_line.set_data(*zip(*self.oa_path))
    
    def update_plot(self, frame):
        with self.plot_lock:
            if not self.mission_started or not self.rov_positions:
                return
            
            # Update ROV trajectory
            rov_points = np.array(self.rov_positions)
            self.rov_line.set_data(rov_points[:, 0], rov_points[:, 1])
            
            # Update obstacle detection points
            if self.detected_obstacles:
                detected_points = np.array(self.detected_obstacles)
                self.obstacle_detection_scatter.set_offsets(detected_points)

def main(args=None):
    rclpy.init(args=args)
    visualizer = ObstacleAvoidanceVisualizer()
    
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