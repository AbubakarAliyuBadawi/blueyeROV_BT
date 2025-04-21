#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import csv
import os
from matplotlib.patches import Rectangle
import datetime
from threading import Lock

class SensorVisualizerNode(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')
        
        # Create subscriptions for sensors and system state
        self.camera_sub = self.create_subscription(
            Image, '/blueye/camera_1/image_raw', self.camera_callback, 10)
        
        self.sonar_sub = self.create_subscription(
            ProjectedSonarImage, '/mundus_mir/sonar', self.sonar_callback, 10)
        
        self.mission_state_sub = self.create_subscription(
            Int32, '/mission_state', self.mission_state_callback, 10)
        
        # Initialize state tracking variables
        self.camera_active = False
        self.sonar_active = False
        self.last_camera_time = None
        self.last_sonar_time = None
        self.camera_timeout = 5.0  # seconds
        self.sonar_timeout = 5.0   # seconds
        self.current_mission_state = 0
        
        # State names for mission states
        self.mission_states = {
            0: "Not started",
            1: "Undocking",
            2: "Transit_1",
            3: "Pipeline Inspection",
            4: "Transit_2",
            5: "Wreckage Inspection",
            6: "Homing/Return to Dock",
            7: "Docking"
        }
        
        # Sensor failure tracking
        self.camera_failure_detected = False
        self.sonar_failure_detected = False
        self.emergency_triggered = False
        self.failure_start_times = {
            'camera': None,
            'sonar': None,
            'emergency': None
        }
        
        # Setup visualization
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.fig.canvas.manager.set_window_title('Sensor Status Timeline')
        
        # Initialize data structures for timeline
        self.time_history = []
        self.camera_history = []
        self.sonar_history = []
        self.mission_state_history = []
        self.event_markers = []  # To store when emergency behaviors triggered
        
        # For data export
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"/tmp/sensor_log_{self.timestamp}.csv"
        self.csv_mutex = Lock()
        self.initialize_csv()
        
        # Setup the plot
        self.setup_plot()
        
        # Create timers
        self.plot_timer = self.create_timer(0.5, self.update_visualization)
        self.status_timer = self.create_timer(0.1, self.check_sensor_status)
        
        # Start time reference
        self.start_time = time.time()
        
        self.get_logger().info('Sensor Visualizer Node initialized')
        self.get_logger().info(f'Logging data to: {self.csv_file}')
    
    def initialize_csv(self):
        with open(self.csv_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Elapsed Time', 'Event Type', 'Details', 'Camera Status', 'Sonar Status', 'Mission State'])
    
    def log_event(self, event_type, details):
        with self.csv_mutex:
            current_time = time.time()
            elapsed = current_time - self.start_time
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            
            with open(self.csv_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    timestamp, 
                    f"{elapsed:.2f}", 
                    event_type, 
                    details,
                    "Active" if self.camera_active else "Inactive",
                    "Active" if self.sonar_active else "Inactive",
                    self.mission_states.get(self.current_mission_state, f"Unknown ({self.current_mission_state})")
                ])
            
            self.get_logger().info(f"Logged {event_type}: {details}")
    
    def setup_plot(self):
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Status')
        self.ax.set_ylim(-0.5, 2.5)
        self.ax.set_yticks([0, 1, 2])
        self.ax.set_yticklabels(['Mission State', 'Sonar', 'Camera'])
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_title('Sensor Status and Emergency Timeline')
        
        # Create lines for sensor status
        self.camera_line, = self.ax.plot([], [], 'g-', linewidth=2, label='Camera Active')
        self.sonar_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Sonar Active')
        
        # Create text annotation for current state
        self.state_text = self.ax.text(0.02, 0.1, '', transform=self.ax.transAxes, 
                                      fontsize=12, fontweight='bold')
        
        # Emergency indicator
        self.emergency_text = self.ax.text(0.02, 0.05, '', transform=self.ax.transAxes,
                                         fontsize=12, color='red')
        
        self.ax.legend(loc='upper right')
        
        # Enable interactive mode
        plt.ion()
        plt.show()
    
    def camera_callback(self, msg):
        was_active = self.camera_active
        self.camera_active = True
        self.last_camera_time = time.time()
        
        # If camera was previously inactive, log recovery
        if not was_active:
            if self.failure_start_times['camera'] is not None:
                duration = time.time() - self.failure_start_times['camera']
                self.log_event("CAMERA_RECOVERY", f"Camera active after {duration:.2f}s failure")
                self.failure_start_times['camera'] = None
                self.camera_failure_detected = False
    
    def sonar_callback(self, msg):
        was_active = self.sonar_active
        self.sonar_active = True
        self.last_sonar_time = time.time()
        
        # If sonar was previously inactive, log recovery
        if not was_active:
            if self.failure_start_times['sonar'] is not None:
                duration = time.time() - self.failure_start_times['sonar']
                self.log_event("SONAR_RECOVERY", f"Sonar active after {duration:.2f}s failure")
                self.failure_start_times['sonar'] = None
                self.sonar_failure_detected = False
    
    def mission_state_callback(self, msg):
        old_state = self.current_mission_state
        self.current_mission_state = msg.data
        
        # If state changed, log the transition
        if old_state != self.current_mission_state:
            old_state_name = self.mission_states.get(old_state, f"Unknown ({old_state})")
            new_state_name = self.mission_states.get(self.current_mission_state, f"Unknown ({self.current_mission_state})")
            
            self.log_event("STATE_CHANGE", f"From {old_state_name} to {new_state_name}")
            
            # If changing to state 6 (Homing), check if it might be emergency return
            if self.current_mission_state == 6 and (not self.camera_active or not self.sonar_active):
                self.log_event("EMERGENCY_RETURN", "Mission state changed to Homing with sensor failures")
                self.emergency_triggered = True
                self.failure_start_times['emergency'] = time.time()
                
                # Add to event markers for visualization
                elapsed_time = time.time() - self.start_time
                self.event_markers.append((elapsed_time, "EMERGENCY RETURN"))
            
            # Add to event markers for visualization
            elapsed_time = time.time() - self.start_time
            self.event_markers.append((elapsed_time, f"State: {new_state_name}"))
    
    def check_sensor_status(self):
        current_time = time.time()
        
        # Check camera timeout
        if self.last_camera_time is not None:
            camera_elapsed = current_time - self.last_camera_time
            was_active = self.camera_active
            self.camera_active = camera_elapsed < self.camera_timeout
            
            # Detect camera failure
            if was_active and not self.camera_active and not self.camera_failure_detected:
                self.log_event("CAMERA_FAILURE", f"No camera data for {self.camera_timeout}s")
                self.failure_start_times['camera'] = current_time
                self.camera_failure_detected = True
                
                # Add to event markers
                elapsed_time = current_time - self.start_time
                self.event_markers.append((elapsed_time, "CAMERA FAILURE"))
        
        # Check sonar timeout
        if self.last_sonar_time is not None:
            sonar_elapsed = current_time - self.last_sonar_time
            was_active = self.sonar_active
            self.sonar_active = sonar_elapsed < self.sonar_timeout
            
            # Detect sonar failure
            if was_active and not self.sonar_active and not self.sonar_failure_detected:
                self.log_event("SONAR_FAILURE", f"No sonar data for {self.sonar_timeout}s")
                self.failure_start_times['sonar'] = current_time
                self.sonar_failure_detected = True
                
                # Add to event markers
                elapsed_time = current_time - self.start_time
                self.event_markers.append((elapsed_time, "SONAR FAILURE"))
    
    def update_visualization(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Add current data to history
        self.time_history.append(elapsed_time)
        self.camera_history.append(2 if self.camera_active else float('nan'))
        self.sonar_history.append(1 if self.sonar_active else float('nan'))
        self.mission_state_history.append(self.current_mission_state)
        
        # Limit history length (last 5 minutes)
        max_history = 5 * 60 * 2  # 5 minutes at 0.5 second intervals
        if len(self.time_history) > max_history:
            self.time_history = self.time_history[-max_history:]
            self.camera_history = self.camera_history[-max_history:]
            self.sonar_history = self.sonar_history[-max_history:]
            self.mission_state_history = self.mission_state_history[-max_history:]
        
        # Update the plot
        self.camera_line.set_data(self.time_history, self.camera_history)
        self.sonar_line.set_data(self.time_history, self.sonar_history)
        
        # Update mission state text
        state_name = self.mission_states.get(self.current_mission_state, f"Unknown ({self.current_mission_state})")
        self.state_text.set_text(f"Current State: {state_name}")
        
        # Update emergency text
        if self.emergency_triggered:
            self.emergency_text.set_text("EMERGENCY RETURN ACTIVE")
        elif not self.camera_active or not self.sonar_active:
            self.emergency_text.set_text("SENSOR FAILURE DETECTED")
        else:
            self.emergency_text.set_text("")
        
        # Add mission state background colors
        self.draw_mission_state_backgrounds()
        
        # Adjust x-axis limits to show all data plus some margin
        if self.time_history:
            self.ax.set_xlim(max(0, self.time_history[-1] - 120), self.time_history[-1] + 5)
        
        # Add markers for events
        for time_mark, event_text in self.event_markers:
            if time_mark >= self.ax.get_xlim()[0] and time_mark <= self.ax.get_xlim()[1]:
                self.ax.axvline(x=time_mark, color='r', linestyle='--', alpha=0.7)
                self.ax.text(time_mark, 2.3, event_text, rotation=90, fontsize=8)
        
        # Draw the updated figure
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def draw_mission_state_backgrounds(self):
        # Clear previous backgrounds
        for artist in self.ax.get_children():
            if isinstance(artist, Rectangle) and getattr(artist, 'is_mission_state', False):
                artist.remove()
        
        # Don't draw if history is empty
        if not self.time_history:
            return
        
        # Create colored background regions for different mission states
        current_state = self.mission_state_history[0]
        start_idx = 0
        
        colors = {
            0: 'white',
            1: 'lightyellow',
            2: 'lightblue',
            3: 'lightgreen',
            4: 'lightblue',
            5: 'lightgreen',
            6: 'lightsalmon',
            7: 'lightcoral'
        }
        
        for i, state in enumerate(self.mission_state_history[1:], 1):
            if state != current_state:
                # Create a rectangle for the previous state
                rect = Rectangle((self.time_history[start_idx], -0.5), 
                                self.time_history[i-1] - self.time_history[start_idx],
                                3, facecolor=colors.get(current_state, 'white'), alpha=0.3)
                rect.is_mission_state = True
                self.ax.add_patch(rect)
                
                # Update for the new state
                current_state = state
                start_idx = i
        
        # Add the final state rectangle
        rect = Rectangle((self.time_history[start_idx], -0.5), 
                        self.time_history[-1] - self.time_history[start_idx],
                        3, facecolor=colors.get(current_state, 'white'), alpha=0.3)
        rect.is_mission_state = True
        self.ax.add_patch(rect)
    
    def save_timeline(self, filename=None):
        if filename is None:
            filename = f"/tmp/sensor_timeline_{self.timestamp}.png"
        
        self.fig.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"Timeline saved to {filename}")
        
        # Also save a CSV report summary
        self.generate_report_summary()
    
    def generate_report_summary(self):
        summary_file = f"/tmp/sensor_report_{self.timestamp}.txt"
        
        with open(summary_file, 'w') as f:
            f.write("===== Sensor Status and Emergency Events Summary =====\n\n")
            f.write(f"Report generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total runtime: {(time.time() - self.start_time)/60:.2f} minutes\n\n")
            
            # Count events from CSV
            camera_failures = 0
            sonar_failures = 0
            emergency_returns = 0
            
            with open(self.csv_file, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)  # Skip header
                
                for row in reader:
                    if len(row) >= 3:  # Ensure row has enough elements
                        event_type = row[2]
                        if event_type == "CAMERA_FAILURE":
                            camera_failures += 1
                        elif event_type == "SONAR_FAILURE":
                            sonar_failures += 1
                        elif event_type == "EMERGENCY_RETURN":
                            emergency_returns += 1
            
            f.write(f"Camera failures detected: {camera_failures}\n")
            f.write(f"Sonar failures detected: {sonar_failures}\n")
            f.write(f"Emergency returns triggered: {emergency_returns}\n\n")
            
            f.write("===== Mission Timeline =====\n")
            f.write("See the visual timeline saved as sensor_timeline_*.png\n")
            f.write("Full event log available in sensor_log_*.csv\n")
        
        self.get_logger().info(f"Report summary saved to {summary_file}")

def main(args=None):
    rclpy.init(args=args)
    visualizer = SensorVisualizerNode()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        # Save the timeline on exit
        visualizer.save_timeline()
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()