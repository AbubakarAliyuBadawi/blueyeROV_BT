#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class SystemHealthVisualizer(Node):
    def __init__(self):
        super().__init__('system_health_visualizer')
        
        # Initialize time tracking
        self.start_time = time.time()
        self.time_points = []
        
        # Thread safety
        self.plot_lock = threading.Lock()
        
        # Timeouts for determining if topics are active
        self.timeout_seconds = 5.0  # Consider a topic inactive after 5 seconds
        
        # Status tracking (1 = working, 0 = not working)
        self.camera_status = []
        self.sonar_status = []
        self.system_status = []  # Combined status of odometry and cmd_force
        self.battery_percentage = []
        
        # Last message times
        self.last_camera_time = self.get_clock().now()
        self.last_sonar_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.last_cmd_force_time = self.get_clock().now()
        
        # Current battery percentage
        self.current_battery = 100.0
        
        # Create figure with subplots
        self.fig, self.axs = plt.subplots(4, 1, figsize=(12, 8), sharex=True)
        
        # Configure the plots
        titles = ['Camera Status', 'Sonar Status', 'System Status', 'Battery Percentage']
        colors = ['green', 'blue', 'purple', 'orange']
        self.lines = []
        
        for i, (ax, title, color) in enumerate(zip(self.axs, titles, colors)):
            ax.set_title(title)
            if i < 3:  # Status plots
                ax.set_ylabel('Status')
                ax.set_ylim(-0.1, 1.1)
                ax.set_yticks([0, 1])
                ax.set_yticklabels(['Offline', 'Online'])
                line, = ax.plot([], [], color=color, linewidth=2)
            else: # Battery plot
                ax.set_ylabel('Battery %')
                ax.set_ylim(0, 100)
                line, = ax.plot([], [], color=color, linewidth=2)
            
            self.lines.append(line)
        
        self.axs[-1].set_xlabel('Time [s]')
        
        # Create subscriptions to monitor topics
        self.camera_sub = self.create_subscription(
            Image,
            '/blueye/camera_1/image_raw',
            self.camera_callback,
            10)
        
        self.sonar_sub = self.create_subscription(
            ProjectedSonarImage,
            '/mundus_mir/sonar',
            self.sonar_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/blueye/odometry_frd/gt',
            self.odom_callback,
            10)
        
        self.cmd_force_sub = self.create_subscription(
            WrenchStamped,
            '/blueye/cmd_force',
            self.cmd_force_callback,
            10)
        
        self.battery_sub = self.create_subscription(
            Float64,
            '/blueye/battery_percentage',
            self.battery_callback,
            10)
        
        # Timer for updating status based on timeouts
        self.timer = self.create_timer(0.5, self.update_status)
        
        # Animation for plotting
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        self.get_logger().info("System Health Visualizer initialized")
    
    def camera_callback(self, msg):
        self.last_camera_time = self.get_clock().now()
    
    def sonar_callback(self, msg):
        self.last_sonar_time = self.get_clock().now()
    
    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
    
    def cmd_force_callback(self, msg):
        self.last_cmd_force_time = self.get_clock().now()
    
    def battery_callback(self, msg):
        self.current_battery = msg.data
    
    def mission_state_callback(self, msg):
        self.current_state = msg.data
    
    def update_status(self):
        current_time = self.get_clock().now()
        
        with self.plot_lock:
            # Record time
            time_point = time.time() - self.start_time
            self.time_points.append(time_point)
            
            # Check camera status
            camera_active = (current_time - self.last_camera_time).nanoseconds / 1e9 < self.timeout_seconds
            self.camera_status.append(1 if camera_active else 0)
            
            # Check sonar status
            sonar_active = (current_time - self.last_sonar_time).nanoseconds / 1e9 < self.timeout_seconds
            self.sonar_status.append(1 if sonar_active else 0)
            
            # Check system status (both odometry and cmd_force)
            odom_active = (current_time - self.last_odom_time).nanoseconds / 1e9 < self.timeout_seconds
            cmd_force_active = (current_time - self.last_cmd_force_time).nanoseconds / 1e9 < self.timeout_seconds
            system_active = odom_active and cmd_force_active
            self.system_status.append(1 if system_active else 0)
            
            # Record battery percentage
            self.battery_percentage.append(self.current_battery)
    
    def update_plot(self, frame):
        with self.plot_lock:
            if len(self.time_points) > 0:
                # Update each line with its data
                self.lines[0].set_data(self.time_points, self.camera_status)
                self.lines[1].set_data(self.time_points, self.sonar_status)
                self.lines[2].set_data(self.time_points, self.system_status)
                self.lines[3].set_data(self.time_points, self.battery_percentage)
                
                # Adjust x-axis limits
                max_time = max(self.time_points)
                for ax in self.axs:
                    ax.set_xlim(0, max(500, max_time * 1.1))
        
        return self.lines

def main(args=None):
    rclpy.init(args=args)
    visualizer = SystemHealthVisualizer()
    
    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()
    
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()