#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mundus_mir_msgs.srv import GetWaypointStatus
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from scipy.spatial.transform import Rotation
import time
import math
import re
from std_msgs.msg import Int32

class ROVTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('rov_trajectory_plotter')
        
        # Initialize time tracking
        self.start_time = time.time()
        self.time_points = []
        
        # Thread safety lock
        self.plot_lock = threading.Lock()
        
        # Arrays for storing position data over time
        self.north_actual = []
        self.east_actual = []
        self.depth_actual = []
        self.heading_actual = []
        
        # Arrays for desired values
        self.north_desired = []
        self.east_desired = []
        self.depth_desired = []
        self.heading_desired = []
        
        # Current desired values (to be updated by command)
        self.current_desired_north = None
        self.current_desired_east = None
        self.current_desired_depth = None
        self.current_desired_heading = None
        
        # Create the figure with 4 subplots
        # self.fig, self.axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        self.fig, self.axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
        
        # Configure subplots
        self.axs[0].set_ylabel('North [m]')
        self.axs[0].set_title('Estimated and desired position in north direction over time')
        self.axs[0].grid(False)  # Remove grid
        
        self.axs[1].set_ylabel('East [m]')
        self.axs[1].set_title('Estimated and desired position in east direction over time')
        self.axs[1].grid(False)  # Remove grid
        
        self.axs[2].set_ylabel('Depth [m]')
        self.axs[2].set_title('Estimated and desired depth over time')
        self.axs[2].grid(False)  # Remove grid
        
        self.axs[3].set_ylabel('Heading [rad]')
        self.axs[3].set_xlabel('Time [s]')
        self.axs[3].set_title('Estimated and desired heading over time')
        self.axs[3].grid(False)  # Remove grid
        
        # Create empty line objects for plotting
        self.north_actual_line, = self.axs[0].plot([], [], 'b-', label='$\\hat{x}$')
        self.north_desired_line, = self.axs[0].plot([], [], 'r-', label='$x_d$')
        
        self.east_actual_line, = self.axs[1].plot([], [], 'b-', label='$\\hat{y}$')
        self.east_desired_line, = self.axs[1].plot([], [], 'r-', label='$y_d$')
        
        self.depth_actual_line, = self.axs[2].plot([], [], 'b-', label='$\\hat{d}$')
        self.depth_desired_line, = self.axs[2].plot([], [], 'r-', label='$d_d$')
        
        self.heading_actual_line, = self.axs[3].plot([], [], 'b-', label='$\\hat{\\psi}$')
        self.heading_desired_line, = self.axs[3].plot([], [], 'r-', label='$\\psi_d$')
        
        # Add mission state tracking
        self.mission_states = []
        self.mission_times = []
        self.state_names = {
            1: "UnDocking",
            2: "Transit_1",
            3: "PipeLineInspection",
            4: "Transit_2",
            5: "WreckageInspection",
            6: "Homing",
            7: "Docking"
        }
        
        # Configure mission state subplot
        self.axs[4].set_ylabel('State')
        self.axs[4].set_xlabel('Time [s]')
        self.axs[4].set_title('Mission State over time')
        self.axs[4].set_yticks(range(1, 8))
        self.axs[4].set_yticklabels(list(self.state_names.values()))
        self.mission_state_line, = self.axs[4].plot([], [], 'purple', linewidth=2, label='State')
        self.axs[4].set_ylim(0.5, 7.5)
        
        # Add legends to each subplot
        for ax in self.axs:
            ax.legend(loc='upper right')
        
        # Adjust layout
        plt.tight_layout()
        
        # Subscribe to actual position from odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odom_callback,
            10
        )
        
        # Subscribe to velocity commands for desired values
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/blueye/ref_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to mission state
        self.mission_state_sub = self.create_subscription(
            Int32,
            '/mission_state',
            self.mission_state_callback,
            10
        )
        
        # Create client for waypoint status
        self.waypoint_status_client = self.create_client(
            GetWaypointStatus,
            '/blueye/get_waypoint_status'
        )
        
        # Separate timer for updating waypoint status (to avoid overwhelming the service)
        self.waypoint_timer = self.create_timer(0.5, self.update_waypoint_status)
        
        # Animation
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        self.get_logger().info("ROV Trajectory Plotter initialized")
    
    def odom_callback(self, msg):
        with self.plot_lock:
            # Record current time
            current_time = time.time() - self.start_time
            self.time_points.append(current_time)
            
            # Extract position from odometry message
            position = msg.pose.pose.position
            
            # Based on the data, your frame appears to be NED (North-East-Down)
            # So y is North, -x is East, z is Depth (positive z means deeper)
            self.north_actual.append(position.y)
            self.east_actual.append(-position.x)
            self.depth_actual.append(position.z)
            
            # Extract orientation (quaternion)
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # Convert quaternion to heading (yaw) - NED frame
            r = Rotation.from_quat([qx, qy, qz, qw])
            roll, pitch, yaw = r.as_euler('xyz')
            self.heading_actual.append(yaw)
            
            # If we don't have desired values yet, use actual as placeholder
            if self.current_desired_north is None:
                self.current_desired_north = position.y
            if self.current_desired_east is None:
                self.current_desired_east = -position.x
            if self.current_desired_depth is None:
                self.current_desired_depth = position.z
            if self.current_desired_heading is None:
                self.current_desired_heading = yaw
            
            # Append current desired values
            self.north_desired.append(self.current_desired_north)
            self.east_desired.append(self.current_desired_east)
            self.depth_desired.append(self.current_desired_depth)
            self.heading_desired.append(self.current_desired_heading)
            
    def mission_state_callback(self, msg):
        with self.plot_lock:
            # Get current time
            current_time = time.time() - self.start_time
            
            # Get the state ID
            state_id = msg.data
            
            # Check if the state is valid (1-7)
            if 1 <= state_id <= 7:
                # Get state name for logging
                state_name = self.state_names.get(state_id)
                
                # Log the state change
                if not self.mission_states or state_id != self.mission_states[-1]:
                    self.get_logger().info(f"State changed to: {state_name} (ID: {state_id})")
                
                # Append data
                self.mission_times.append(current_time)
                self.mission_states.append(state_id)
            else:
                self.get_logger().warn(f"Received invalid state ID: {state_id}")
                
    def cmd_vel_callback(self, msg):
        with self.plot_lock:
            # Extract velocity components - these are in the NED frame based on your data
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            vz = msg.twist.linear.z
            omega_z = msg.twist.angular.z
            
            # Check if the current_desired_heading is initialized
            if self.current_desired_heading is None:
                self.current_desired_heading = 0.0  # Default value
            
            # Calculate desired heading from angular velocity
            if abs(omega_z) > 0.001:
                # Integrate angular velocity for heading change
                if len(self.heading_desired) > 0:
                    dt = 0.1  # Assume 10Hz update rate
                    self.current_desired_heading = self.heading_desired[-1] + omega_z * dt
                
                # Normalize heading to [-pi, pi]
                self.current_desired_heading = ((self.current_desired_heading + math.pi) % (2 * math.pi)) - math.pi
            
            # For now, we'll just use the velocity direction for heading
            if abs(vx) > 0.01 or abs(vy) > 0.01:
                # In NED frame, heading is direction of velocity
                self.current_desired_heading = math.atan2(vy, vx)
    
    def update_waypoint_status(self):
        """Fetch current waypoint status from the controller service"""
        # Only call the service if it's available
        if not self.waypoint_status_client.service_is_ready():
            return
            
        # Create request
        request = GetWaypointStatus.Request()
        
        # Call service
        future = self.waypoint_status_client.call_async(request)
        future.add_done_callback(self.process_waypoint_status)
    
    def process_waypoint_status(self, future):
        """Process the waypoint status response"""
        try:
            response = future.result()
            if not response.accepted:
                return
                
            # Parse waypoint data from status string
            status = response.status_code
            
            # Check if the controller is actually running
            if "Controller running: 1" not in status:
                return
                
            # Update current desired position from waypoint
            if "Current waypoint:" in status:
                # Extract coordinates using regex
                match = re.search(r"Current waypoint: ([-\d.]+), ([-\d.]+), ([-\d.]+)", status)
                if match:
                    with self.plot_lock:
                        x = float(match.group(1))
                        y = float(match.group(2))
                        z = float(match.group(3))
                        self.current_desired_north = y           # Y coordinate is North
                        self.current_desired_east = -x           # Negative X coordinate is East
                        self.current_desired_depth = z
            elif "Station Keep On Position:" in status:
                # Extract station keeping position
                match = re.search(r"Station Keep On Position: ([-\d.]+), ([-\d.]+), ([-\d.]+)", status)
                if match:
                    with self.plot_lock:
                        x = float(match.group(1))
                        y = float(match.group(2))
                        z = float(match.group(3))
                        self.current_desired_north = y
                        self.current_desired_east = -x
                        self.current_desired_depth = z
                        
            # Log current waypoint info for debugging
            self.get_logger().debug(f"Current desired positions - N: {self.current_desired_north}, " +
                                   f"E: {self.current_desired_east}, D: {self.current_desired_depth}")
                        
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint status: {e}")
    
    def update_plot(self, frame):
        # Update all line data
        if len(self.time_points) > 0:
            with self.plot_lock:  # Use thread lock to prevent race conditions
                if self.mission_states and self.mission_times:
                    self.mission_state_line.set_data(self.mission_times, self.mission_states)
                # Make sure all arrays are the same length
                min_length = min(len(self.time_points), 
                            len(self.north_actual), 
                            len(self.east_actual),
                            len(self.depth_actual), 
                            len(self.heading_actual),
                            len(self.north_desired), 
                            len(self.east_desired),
                            len(self.depth_desired), 
                            len(self.heading_desired))
                
                if min_length == 0:
                    return
                    
                times = self.time_points[:min_length]
                
                # Update actual data
                self.north_actual_line.set_data(times, self.north_actual[:min_length])
                self.east_actual_line.set_data(times, self.east_actual[:min_length])
                self.depth_actual_line.set_data(times, self.depth_actual[:min_length])
                self.heading_actual_line.set_data(times, self.heading_actual[:min_length])
                
                # Update desired data
                self.north_desired_line.set_data(times, self.north_desired[:min_length])
                self.east_desired_line.set_data(times, self.east_desired[:min_length])
                self.depth_desired_line.set_data(times, self.depth_desired[:min_length])
                self.heading_desired_line.set_data(times, self.heading_desired[:min_length])
                
                # Update axis limits
                for i, ax in enumerate(self.axs):
                    ax.relim()
                    ax.autoscale_view()
                    
                # Set common x-axis limits
                if times:
                    max_time = max(times)
                    self.axs[3].set_xlim(0, max(500, max_time * 1.1))
                    
                    # Set Y-axis limits for heading to make it easier to read
                    self.axs[3].set_ylim(-3.5, 3.5)  # Range slightly larger than -pi to pi

def main(args=None):
    rclpy.init(args=args)
    plotter = ROVTrajectoryPlotter()
    
    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    spin_thread.start()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()