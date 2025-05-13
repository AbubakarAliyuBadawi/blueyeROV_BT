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
        
        # Last update time for heading integration
        self.last_heading_update_time = None
        self.last_cmd_vel_time = None
        self.last_heading_log_time = 0
        self.prev_cmd_vel_log_time = 0
        
        # Store last omega_z value to check if it's significant
        self.last_omega_z = 0.0
        
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
        
        # Create the figure with 5 subplots
        self.fig, self.axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
        
        # Configure mission state subplot (first - index 0)
        self.axs[0].set_ylabel('State')
        self.axs[0].set_title('Mission State over time')
        self.axs[0].set_yticks(range(1, 8))
        self.axs[0].set_yticklabels(list(self.state_names.values()))
        self.mission_state_line, = self.axs[0].plot([], [], 'purple', linewidth=2, label='State')
        self.axs[0].set_ylim(0.5, 7.5)
        self.axs[0].grid(False)  # Add grid for mission state
        
        # Configure north position subplot (second - index 1)
        self.axs[1].set_ylabel('North [m]')
        self.axs[1].set_title('Estimated and desired position in north direction over time')
        self.axs[1].grid(False)  # Remove grid
        
        # Configure east position subplot (third - index 2)
        self.axs[2].set_ylabel('East [m]')
        self.axs[2].set_title('Estimated and desired position in east direction over time')
        self.axs[2].grid(False)  # Remove grid
        
        # Configure depth subplot (fourth - index 3)
        self.axs[3].set_ylabel('Depth [m]')
        self.axs[3].set_title('Estimated and desired depth over time')
        self.axs[3].grid(False)  # Remove grid
        
        # Configure heading subplot (fifth - index 4)
        self.axs[4].set_ylabel('Heading [rad]')
        self.axs[4].set_xlabel('Time [s]')
        self.axs[4].set_title('Estimated and desired heading over time')
        self.axs[4].grid(False)  # Remove grid
        self.axs[4].set_ylim(-4.0, 4.0)  # Set initial y-axis limits to -4 to 4
        
        # Create empty line objects for plotting
        # Mission state line already created above
        
        # North position lines (index 1)
        self.north_actual_line, = self.axs[1].plot([], [], 'b-', label='$\\hat{x}$')
        self.north_desired_line, = self.axs[1].plot([], [], 'r-', label='$x_d$')
        
        # East position lines (index 2)
        self.east_actual_line, = self.axs[2].plot([], [], 'b-', label='$\\hat{y}$')
        self.east_desired_line, = self.axs[2].plot([], [], 'r-', label='$y_d$')
        
        # Depth lines (index 3)
        self.depth_actual_line, = self.axs[3].plot([], [], 'b-', label='$\\hat{d}$')
        self.depth_desired_line, = self.axs[3].plot([], [], 'r-', label='$d_d$')
        
        # Heading lines (index 4)
        self.heading_actual_line, = self.axs[4].plot([], [], 'b-', label='$\\hat{\\psi}$')
        self.heading_desired_line, = self.axs[4].plot([], [], 'r-', label='$\\psi_d$')
        
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
    
    def quaternion_to_heading(self, qx, qy, qz, qw):
        """
        Convert quaternion to heading angle.
        
        Since we're in an FRD frame:
        - 0 rad = Forward direction
        - π/2 rad = Right
        - π rad = Backward
        - -π/2 rad = Left
        """
        # Extract yaw (heading) from quaternion
        heading = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Ensure the heading is in the range [-π, π]
        heading = ((heading + math.pi) % (2 * math.pi)) - math.pi
        
        return heading
    
    def angle_diff(self, angle1, angle2):
        """
        Calculate the shortest difference between two angles, accounting for wrap-around.
        Returns the difference in the range [-π, π]
        """
        diff = angle1 - angle2
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
    
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
            
            # Convert quaternion to heading (yaw) - FRD frame
            heading = self.quaternion_to_heading(qx, qy, qz, qw)
            
            # Handle discontinuities by checking for large jumps from previous heading
            if len(self.heading_actual) > 0:
                prev_heading = self.heading_actual[-1]
                diff = self.angle_diff(heading, prev_heading)
                
                # If we have a large jump (over 3 radians) that's close to 2π, 
                # it's likely a wrap-around that we should eliminate
                if abs(diff) > 3.0 and abs(abs(diff) - 2*math.pi) < 0.2:
                    # Adjust the new heading to be on the same side as the previous one
                    if diff > 0:
                        heading = heading - 2*math.pi
                    else:
                        heading = heading + 2*math.pi
                    
            # Log actual heading periodically
            if current_time - self.last_heading_log_time > 5.0:
                self.last_heading_log_time = current_time
                # self.get_logger().info(f"Actual heading: {heading:.3f} rad, quat: [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]")
            
            self.heading_actual.append(heading)
            
            # If we don't have desired values yet, use actual as placeholder
            if self.current_desired_north is None:
                self.current_desired_north = position.y
            if self.current_desired_east is None:
                self.current_desired_east = -position.x
            if self.current_desired_depth is None:
                self.current_desired_depth = position.z
                
            # MODIFIED: Always ensure desired heading matches actual heading if not explicitly set
            if self.current_desired_heading is None:
                self.current_desired_heading = heading
                # self.get_logger().info(f"Initialized desired heading to match actual: {heading:.3f} rad")
            
            # MODIFIED: Check if the last received angular velocity was significant
            # If not, sync the desired heading with actual
            significant_angular_vel = False
            if hasattr(self, 'last_omega_z') and abs(self.last_omega_z) > 0.1:
                significant_angular_vel = True
                
            # Reset desired heading to match actual if no significant rotation commands
            if not significant_angular_vel:
                # Use angle_diff to properly compare angles with wrap-around
                if abs(self.angle_diff(self.current_desired_heading, heading)) > 0.2:
                    # self.get_logger().info(f"Syncing desired heading to actual: {self.current_desired_heading:.3f} -> {heading:.3f} rad")
                    self.current_desired_heading = heading
            
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
            # Get current time for dt calculation
            current_time = time.time() - self.start_time
            
            # Log the incoming velocity command periodically
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            vz = msg.twist.linear.z
            omega_z = msg.twist.angular.z
            
            # Store the latest angular velocity for the odom_callback to check
            self.last_omega_z = omega_z
            
            if current_time - self.prev_cmd_vel_log_time > 5.0:
                self.prev_cmd_vel_log_time = current_time
                # self.get_logger().info(f"Received velocity command: vx={vx:.3f}, vy={vy:.3f}, omega_z={omega_z:.3f}")
                if self.current_desired_heading is not None:
                    self.get_logger().info(f"Current desired heading: {self.current_desired_heading:.3f} rad")
            
            # ADDED: Update last command velocity time
            self.last_cmd_vel_time = current_time
            
            # Save previous heading for comparison
            prev_heading = self.current_desired_heading if self.current_desired_heading is not None else 0.0
            
            # Calculate dt since last heading update
            dt = 0.1  # Default if this is the first update
            if self.last_heading_update_time is not None:
                dt = current_time - self.last_heading_update_time
                dt = min(dt, 0.5)  # Limit dt to prevent large jumps
            
            self.last_heading_update_time = current_time
            
            # Check if the current_desired_heading is initialized
            if self.current_desired_heading is None:
                # If we get here, we should have an actual heading already
                if len(self.heading_actual) > 0:
                    self.current_desired_heading = self.heading_actual[-1]
                    self.get_logger().info(f"Initializing desired heading to actual: {self.current_desired_heading:.3f} rad")
                else:
                    self.current_desired_heading = 0.0  # Fallback default value
                    self.get_logger().info("Initialized desired heading to default 0.0 rad")
            
            # MODIFIED: ONLY update heading when there's a significant angular velocity
            # We no longer use linear velocity direction for heading
            if abs(omega_z) > 0.1:  # Only update heading for significant angular velocity (increased from 0.001)
                # Small angle change using angular velocity
                angle_change = omega_z * dt
                new_heading = self.current_desired_heading + angle_change
                
                # Normalize to [-pi, pi] maintaining continuity
                while new_heading > math.pi:
                    new_heading -= 2.0 * math.pi
                while new_heading < -math.pi:
                    new_heading += 2.0 * math.pi
                    
                self.current_desired_heading = new_heading
                
                # Log heading update
                if abs(self.angle_diff(new_heading, prev_heading)) > 0.1:  # Only log significant changes
                    self.get_logger().info(f"Heading updated from angular velocity: {prev_heading:.3f} -> {new_heading:.3f} rad, omega_z: {omega_z:.3f}")
    
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
                        
                        # Log updated waypoint (for debugging)
                        self.get_logger().debug(f"Updated waypoint: N={y}, E={-x}, D={z}")
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
                        
                        # Log station keeping (for debugging)
                        # self.get_logger().debug(f"Station keeping: N={y}, E={-x}, D={z}")
                        
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint status: {e}")
    
    def update_plot(self, frame):
        # Update all line data
        if len(self.time_points) > 0:
            with self.plot_lock:  # Use thread lock to prevent race conditions
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
                
                # Update mission state plot (now first)
                if self.mission_states and self.mission_times:
                    self.mission_state_line.set_data(self.mission_times, self.mission_states)
                
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
                
                # Update axis limits for position, depth, and heading plots
                for i in range(1, 4):  # Skip mission state plot and heading plot
                    self.axs[i].relim()
                    self.axs[i].autoscale_view()
                
                # Update mission state plot limits if needed
                if self.mission_times:
                    self.axs[0].set_xlim(0, max(max(self.mission_times) * 1.1, 10))
                
                # Set common x-axis limits
                if times:
                    max_time = max(times)
                    for ax in self.axs:
                        ax.set_xlim(0, max(800, max_time * 1.1))
                    
                    # Set Y-axis limits for heading to make it easier to read
                    self.axs[4].set_ylim(-4.0, 4.0)  # Fixed range of -4 to 4 for heading

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