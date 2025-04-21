#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mundus_mir_msgs.msg import ReturnRecommendation, BatteryStatus
import matplotlib.pyplot as plt
import numpy as np
import time
import csv
import os
from datetime import datetime
import threading
from matplotlib.ticker import MaxNLocator

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Create subscriptions for battery and return recommendation
        self.battery_sub = self.create_subscription(
            Float64, '/blueye/battery_percentage', self.battery_callback, 10)
        
        self.battery_status_sub = self.create_subscription(
            BatteryStatus, '/blueye/battery', self.battery_status_callback, 10)
        
        self.return_sub = self.create_subscription(
            ReturnRecommendation, '/blueye/return_recommendation', self.return_callback, 10)
        
        # Initialize state tracking variables
        self.battery_percentage = 100.0
        self.should_return = False
        self.minimum_battery_needed = 0.0
        self.distance_to_dock = 0.0
        self.last_update_time = None
        
        # Initialize data for plotting
        self.time_history = []
        self.battery_history = []
        self.return_history = []
        self.return_threshold_history = []
        self.distance_history = []
        self.event_markers = []
        
        # For logging
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"/tmp/battery_log_{self.timestamp}.csv"
        self.initialize_csv()
        
        # Setup visualization
        self.setup_plot()
        
        # Start time reference
        self.start_time = time.time()
        
        # Create timer for updating the plot
        self.timer = self.create_timer(1.0, self.update_plot)
        
        self.get_logger().info('Battery Monitor Node initialized')
        self.get_logger().info(f'Logging data to: {self.csv_file}')
    
    def initialize_csv(self):
        """Initialize the CSV log file"""
        with open(self.csv_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Timestamp', 
                'Elapsed Time', 
                'Battery Percentage', 
                'Should Return', 
                'Minimum Battery Needed',
                'Distance to Dock',
                'Event'
            ])
        
    def log_data(self, event=None):
        """Log data to CSV file"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        
        with open(self.csv_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                timestamp,
                f"{elapsed:.2f}",
                f"{self.battery_percentage:.2f}",
                "Yes" if self.should_return else "No",
                f"{self.minimum_battery_needed:.2f}",
                f"{self.distance_to_dock:.2f}",
                event if event else ""
            ])
    
    def setup_plot(self):
        """Initialize the matplotlib plot"""
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.canvas.manager.set_window_title('Battery Monitor')
        
        # Create lines for plotting
        self.battery_line, = self.ax.plot([], [], 'g-', linewidth=2, label='Battery %')
        self.return_threshold_line, = self.ax.plot([], [], 'r--', linewidth=1, label='Min Battery Needed')
        
        # Setup the axes
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Battery Level (%)')
        self.ax.set_ylim(0, 105)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.legend(loc='upper right')
        self.ax.set_title('Battery Status and Return Condition')
        
        # Add text annotations for current status
        self.battery_text = self.ax.text(0.02, 0.15, '', transform=self.ax.transAxes, 
                                        fontsize=10, fontweight='bold')
        self.return_text = self.ax.text(0.02, 0.08, '', transform=self.ax.transAxes,
                                       fontsize=10, color='red')
        self.distance_text = self.ax.text(0.02, 0.01, '', transform=self.ax.transAxes,
                                        fontsize=10)
        
        # Draw the plot
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Show the plot
        plt.show(block=False)
    
    def battery_callback(self, msg):
        """Handle battery percentage updates"""
        previous_percentage = self.battery_percentage
        self.battery_percentage = msg.data
        self.last_update_time = self.now()
        
        # Log significant changes
        if abs(self.battery_percentage - previous_percentage) > 2.0:
            self.log_data(f"Battery changed: {previous_percentage:.1f}% -> {self.battery_percentage:.1f}%")
    
    def battery_status_callback(self, msg):
        """Handle full battery status updates"""
        self.battery_percentage = msg.state_of_charge * 100.0
        self.last_update_time = self.now()
    
    def return_callback(self, msg):
        """Handle return recommendation updates"""
        previous_return = self.should_return
        self.should_return = msg.should_return
        self.minimum_battery_needed = msg.minimum_battery_needed
        self.distance_to_dock = msg.distance_to_dock
        self.last_update_time = self.now()
        
        # Add event marker and log if return condition changes
        if self.should_return != previous_return:
            elapsed_time = time.time() - self.start_time
            if self.should_return:
                self.event_markers.append((elapsed_time, "RETURN TRIGGERED"))
                self.log_data("RETURN TRIGGERED")
                self.get_logger().warn(f"Return condition activated at {self.battery_percentage:.1f}% battery")
            else:
                self.event_markers.append((elapsed_time, "RETURN CANCELED"))
                self.log_data("RETURN CANCELED")
                self.get_logger().info("Return condition deactivated")
    
    def update_plot(self):
        """Update the plot with current data"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Add current data to history
        self.time_history.append(elapsed_time)
        self.battery_history.append(self.battery_percentage)
        self.return_history.append(1 if self.should_return else 0)
        self.return_threshold_history.append(self.minimum_battery_needed)
        self.distance_history.append(self.distance_to_dock)
        
        # Limit history length (last 10 minutes)
        max_history = 10 * 60  # 10 minutes at 1 second intervals
        if len(self.time_history) > max_history:
            self.time_history = self.time_history[-max_history:]
            self.battery_history = self.battery_history[-max_history:]
            self.return_history = self.return_history[-max_history:]
            self.return_threshold_history = self.return_threshold_history[-max_history:]
            self.distance_history = self.distance_history[-max_history:]
        
        # Update the plot
        self.battery_line.set_data(self.time_history, self.battery_history)
        self.return_threshold_line.set_data(self.time_history, self.return_threshold_history)
        
        # Update text annotations
        self.battery_text.set_text(f"Battery: {self.battery_percentage:.1f}%")
        self.return_text.set_text("RETURN TO DOCK ACTIVATED" if self.should_return else "")
        self.distance_text.set_text(f"Distance to dock: {self.distance_to_dock:.1f}m")
        
        # Add or update return condition shading
        self.draw_return_condition()
        
        # Add event markers
        self.draw_event_markers()
        
        # Adjust x-axis limits to show recent data
        if self.time_history:
            x_max = self.time_history[-1]
            # Show the last 5 minutes or all data if less than 5 minutes
            x_min = max(0, x_max - 300)
            self.ax.set_xlim(x_min, x_max + 10)
            
            # Make sure y-axis shows all battery data
            y_min = max(0, min(self.battery_history[-100:]) - 5)
            y_max = min(105, max(self.battery_history[-100:]) + 5)
            self.ax.set_ylim(y_min, y_max)
        
        # Force integer x-axis ticks
        self.ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        
        # Redraw the figure
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Log data periodically (every minute)
        if int(elapsed_time) % 60 == 0:
            self.log_data()
    
    def draw_return_condition(self):
        """Draw shaded areas when return condition is active"""
        # Remove existing patches
        for patch in self.ax.patches:
            patch.remove()
        
        # Create shaded areas for return condition
        active_ranges = []
        current_start = None
        
        for i, return_val in enumerate(self.return_history):
            if return_val == 1 and current_start is None:
                current_start = self.time_history[i]
            elif return_val == 0 and current_start is not None:
                active_ranges.append((current_start, self.time_history[i]))
                current_start = None
        
        # Add final range if still active
        if current_start is not None:
            active_ranges.append((current_start, self.time_history[-1]))
        
        # Draw shaded rectangles for each active range
        for start, end in active_ranges:
            rect = plt.Rectangle((start, 0), end - start, 105, 
                               color='red', alpha=0.2, label='Return Active')
            self.ax.add_patch(rect)
    
    def draw_event_markers(self):
        """Draw markers for significant events"""
        visible_range = self.ax.get_xlim()
        
        for time_val, event_text in self.event_markers:
            if visible_range[0] <= time_val <= visible_range[1]:
                # Draw vertical line
                self.ax.axvline(x=time_val, color='r', linestyle='-', alpha=0.7)
                
                # Calculate y position for text (near top of plot)
                y_pos = self.ax.get_ylim()[1] * 0.9
                
                # Add text
                self.ax.text(time_val, y_pos, event_text, 
                           fontsize=9, color='red', 
                           rotation=90, ha='center', va='top',
                           bbox=dict(facecolor='white', alpha=0.7))
    
    def save_data(self):
        """Save the plot and final report when shutting down"""
        # Save the current plot
        plot_file = f"/tmp/battery_plot_{self.timestamp}.png"
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Battery plot saved to {plot_file}")
        
        # Generate a simple report
        report_file = f"/tmp/battery_report_{self.timestamp}.txt"
        with open(report_file, 'w') as f:
            f.write("===== Battery Monitor Report =====\n\n")
            f.write(f"Report generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total monitoring time: {(time.time() - self.start_time)/60:.1f} minutes\n\n")
            
            # Count return events
            return_events = [e for e in self.event_markers if "RETURN TRIGGERED" in e[1]]
            f.write(f"Return condition triggered: {len(return_events)} times\n")
            
            # Calculate battery discharge rate if enough data
            if len(self.battery_history) > 10:
                start_battery = self.battery_history[0]
                end_battery = self.battery_history[-1]
                total_time = (self.time_history[-1] - self.time_history[0]) / 60  # minutes
                if total_time > 0:
                    discharge_rate = (start_battery - end_battery) / total_time
                    f.write(f"Average battery discharge rate: {discharge_rate:.2f}% per minute\n")
            
            f.write("\n===== Return Events =====\n")
            for i, (time_val, event_text) in enumerate(self.event_markers):
                if "RETURN" in event_text:
                    elapsed_minutes = time_val / 60
                    f.write(f"{i+1}. [{elapsed_minutes:.1f} min] {event_text}\n")
        
        self.get_logger().info(f"Battery report saved to {report_file}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    
    # Handle graceful shutdown to save data
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, saving data before shutdown...")
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()