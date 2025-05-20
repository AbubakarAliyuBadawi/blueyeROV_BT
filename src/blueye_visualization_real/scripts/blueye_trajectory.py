#!/usr/bin/env python3
"""
Real-time visualizer for Blueye drone using the SDK and DVL data.
This script plots the drone's position (X, Y), depth, and heading over time.
"""

import threading
import time
import math
import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from blueye.sdk import Drone

class DVLData:
    """Class for storing and processing DVL data."""
    
    def __init__(self):
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.velocity = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.orientation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.altitude = 0.0
        self.timestamp = 0.0


class DVLDataReceiver:
    """Class for receiving DVL data from a socket connection."""
    
    def __init__(self, host="192.168.1.99", port=16171):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.connected = False
        self.dvl_data = DVLData()
        
        # Callback functions
        self.position_callback = None
        self.velocity_callback = None
        self.connection_callback = None
    
    def connect(self):
        """Connect to the DVL socket server."""
        if self.connected:
            return True
            
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.host, self.port))
            self.connected = True
            
            if self.connection_callback:
                self.connection_callback(True, f"Connected to DVL at {self.host}:{self.port}")
            return True
        except Exception as e:
            self.connected = False
            print(f"Failed to connect to DVL: {str(e)}")
            if self.connection_callback:
                self.connection_callback(False, f"Connection failed: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from the DVL socket server."""
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
        
        if self.connection_callback:
            self.connection_callback(False, "Disconnected from DVL system")
    
    def start_receiving(self):
        """Start receiving data in a separate thread."""
        if not self.connected:
            if not self.connect():
                return False
                
        self.running = True
        receiver_thread = threading.Thread(target=self._receive_data)
        receiver_thread.daemon = True
        receiver_thread.start()
        return True
    
    def _receive_data(self):
        """Receive and process DVL data."""
        if not self.socket:
            return
            
        buffer = b""
        
        while self.running:
            try:
                data = self.socket.recv(2048)
                if not data:
                    # Connection closed
                    if self.connection_callback:
                        self.connection_callback(False, "Connection closed by server")
                    break
                    
                buffer += data
                
                # Process complete JSON messages
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        self._process_message(line.decode('utf-8'))
                    except Exception as e:
                        print(f"Error processing message: {e}")
                        pass
                        
            except socket.timeout:
                # Socket timeout, continue
                continue
                
            except socket.error as e:
                print(f"Socket error: {str(e)}")
                if self.connection_callback:
                    self.connection_callback(False, f"Socket error: {str(e)}")
                break
                
            except Exception as e:
                print(f"Error: {str(e)}")
                if self.connection_callback:
                    self.connection_callback(False, f"Error: {str(e)}")
                break
                
        # Clean up
        self.disconnect()
    
    def _process_message(self, message):
        """Process a DVL message."""
        try:
            data = json.loads(message)
            data_type = data.get("type")
            
            if data_type == "velocity":
                # Process velocity data
                vx = data.get("vx", 0.0)
                vy = data.get("vy", 0.0)
                vz = data.get("vz", 0.0)
                altitude = data.get("altitude", 0.0)
                
                # Update internal data
                self.dvl_data.velocity["vx"] = vx
                self.dvl_data.velocity["vy"] = vy
                self.dvl_data.velocity["vz"] = vz
                self.dvl_data.altitude = altitude
                
                # Call velocity callback if set
                if self.velocity_callback:
                    self.velocity_callback(vx, vy, vz, altitude)
                
            elif data_type == "position_local":
                # Process position data
                x = data.get("x", 0.0)
                y = data.get("y", 0.0)
                z = data.get("z", 0.0)
                roll = data.get("roll", 0.0)
                pitch = data.get("pitch", 0.0)
                yaw = data.get("yaw", 0.0)
                
                # Update internal data
                self.dvl_data.position["x"] = x
                self.dvl_data.position["y"] = y
                self.dvl_data.position["z"] = z
                self.dvl_data.orientation["roll"] = roll
                self.dvl_data.orientation["pitch"] = pitch
                self.dvl_data.orientation["yaw"] = yaw
                
                # Call position callback if set
                if self.position_callback:
                    self.position_callback(x, y, z, roll, pitch, yaw)
                
        except json.JSONDecodeError:
            # Invalid JSON, ignore
            pass
        except Exception as e:
            print(f"Error processing message: {str(e)}")


class BlueyeRealVisualizer:
    """Visualizer for Blueye drone using the SDK and DVL data."""
    
    def __init__(self, drone_ip="192.168.1.101", dvl_ip="192.168.1.99", dvl_port=16171):
        """Initialize the visualizer.
        
        Args:
            drone_ip (str): IP address of the Blueye drone
            dvl_ip (str): IP address of the DVL system
            dvl_port (int): Port number of the DVL system
        """
        # Initialize data storage arrays
        self.max_data_points = 100000  # Maximum number of points to keep
        self.time_points = deque(maxlen=self.max_data_points)
        self.north_actual = deque(maxlen=self.max_data_points)  # X position from DVL
        self.east_actual = deque(maxlen=self.max_data_points)   # Y position from DVL
        self.depth_actual = deque(maxlen=self.max_data_points)  # Depth from drone.depth
        self.heading_actual = deque(maxlen=self.max_data_points)  # Heading from drone.pose['yaw']
        
        # Thread safety
        self.plot_lock = threading.Lock()
        
        # Starting flag to know when to start recording data
        self.start_time = time.time()
        self.connected_to_drone = False
        self.connected_to_dvl = False
        self.dvl_has_valid_position = False
        
        # Status messages
        self.status_messages = []
        
        # Connect to Blueye drone
        print(f"Connecting to Blueye drone at {drone_ip}...")
        try:
            self.drone = Drone(ip=drone_ip, auto_connect=True, timeout=10)
            self.connected_to_drone = True
            print(f"Connected to drone: {self.drone.serial_number}")
            print(f"Software version: {self.drone.software_version}")
            self.status_messages.append(f"Connected to drone: {self.drone.serial_number}")
        except Exception as e:
            print(f"Failed to connect to drone: {str(e)}")
            self.status_messages.append(f"Failed to connect to drone: {str(e)}")
            self.drone = None
        
        # Connect to DVL for position
        print(f"Connecting to DVL at {dvl_ip}:{dvl_port}...")
        self.dvl_receiver = DVLDataReceiver(host=dvl_ip, port=dvl_port)
        self.dvl_receiver.position_callback = self.on_position_updated
        self.dvl_receiver.velocity_callback = self.on_velocity_updated
        self.dvl_receiver.connection_callback = self.on_connection_status
        
        if self.dvl_receiver.connect():
            self.connected_to_dvl = True
            self.dvl_receiver.start_receiving()
            print("Connected to DVL system")
            self.status_messages.append("Connected to DVL system")
        else:
            print("Failed to connect to DVL system")
            self.status_messages.append("Failed to connect to DVL system")
        
        # Create subplots
        self.fig, self.axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        
        # Configure north position subplot (first - index 0)
        self.axs[0].set_ylabel('North [m]')
        self.axs[0].set_title('Position in north direction over time')
        self.axs[0].grid(False)  # Remove grid
        
        # Configure east position subplot (second - index 1)
        self.axs[1].set_ylabel('East [m]')
        self.axs[1].set_title('Position in east direction over time')
        self.axs[1].grid(False)  # Remove grid
        
        # Configure depth subplot (third - index 2)
        self.axs[2].set_ylabel('Depth [m]')
        self.axs[2].set_title('Depth over time')
        self.axs[2].grid(False)  # Remove grid
        self.axs[2].invert_yaxis() 
        
        # Configure heading subplot (fourth - index 3)
        self.axs[3].set_ylabel('Heading [rad]')
        self.axs[3].set_xlabel('Time [s]')
        self.axs[3].set_title('Heading over time')
        self.axs[3].grid(False)  # Remove grid
        self.axs[3].set_ylim(-4.0, 4.0)  # Set initial y-axis limits to -4 to 4
        
        # Create empty line objects for plotting
        self.north_line, = self.axs[0].plot([], [], 'b-', label='North')
        self.east_line, = self.axs[1].plot([], [], 'b-', label='East')
        self.depth_line, = self.axs[2].plot([], [], 'b-', label='Depth')
        self.heading_line, = self.axs[3].plot([], [], 'b-', label='Heading')
        
        # Add legends to each subplot
        for ax in self.axs:
            ax.legend(loc='upper right')
        
        # Add status text
        self.status_text = self.fig.text(
            0.5, 0.01, 
            self._get_status_text(),
            ha='center', fontsize=10,
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        # Adjust layout
        plt.tight_layout()
        
        # Start data collection thread
        self.running = True
        self.collection_thread = threading.Thread(target=self.collect_data)
        self.collection_thread.daemon = True
        self.collection_thread.start()
        
        # Set up animation
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
    
    def _get_status_text(self):
        """Get a formatted status text."""
        status = []
        
        if self.connected_to_drone:
            status.append("✓ Connected to Blueye drone")
        else:
            status.append("✗ Not connected to Blueye drone")
            
        if self.connected_to_dvl:
            status.append("✓ Connected to DVL")
        else:
            status.append("✗ Not connected to DVL")
            
        if self.dvl_has_valid_position:
            status.append("✓ Valid position data")
        else:
            status.append("✗ No valid position data")
            
        return " | ".join(status)
    
    def collect_data(self):
        """Collect data from Blueye drone at regular intervals."""
        last_log_time = 0
        
        while self.running:
            try:
                if self.connected_to_drone and self.drone:
                    with self.plot_lock:
                        # Get current time
                        current_time = time.time() - self.start_time
                        
                        # Get depth from drone
                        depth = self.drone.depth
                        if depth is None:
                            depth = float('nan')  # Use NaN for missing data
                        
                        # Get heading from drone
                        pose = self.drone.pose
                        if pose:
                            # Convert from degrees (0-359) to radians (-pi to pi)
                            yaw_deg = pose['yaw']
                            heading_rad = math.radians(yaw_deg)
                            # Normalize to (-pi, pi)
                            heading_rad = ((heading_rad + math.pi) % (2 * math.pi)) - math.pi
                        else:
                            heading_rad = float('nan')
                        
                        # Store data (if we have valid DVL position data)
                        if self.dvl_has_valid_position:
                            self.time_points.append(current_time)
                            self.depth_actual.append(depth)
                            self.heading_actual.append(heading_rad)
                        
                        # Log data periodically
                        if current_time - last_log_time > 5.0:
                            last_log_time = current_time
                            print(f"Time: {current_time:.1f}s, Depth: {depth:.2f}m, Heading: {math.degrees(heading_rad):.1f}°")
            except Exception as e:
                print(f"Error collecting data: {str(e)}")
            
            time.sleep(0.1)  # 10 Hz update rate
    
    def on_position_updated(self, x, y, z, roll, pitch, yaw):
        """Callback for DVL position updates."""
        with self.plot_lock:
            # Flag that we have valid position data
            self.dvl_has_valid_position = True
            
            # Store position data if we're already collecting
            if len(self.time_points) > 0:
                self.north_actual.append(x)  # Use x directly from DVL
                self.east_actual.append(y)   # Use y directly from DVL
    
    def on_velocity_updated(self, vx, vy, vz, altitude):
        """Callback for DVL velocity updates."""
        # We don't use velocity data for now
        pass
    
    def on_connection_status(self, connected, message):
        """Callback for DVL connection status updates."""
        self.connected_to_dvl = connected
        print(f"DVL connection status: {message}")
        self.status_messages.append(message)
    
    def update_plot(self, frame):
        """Update the plot with the latest data."""
        with self.plot_lock:
            # Update status text
            self.status_text.set_text(self._get_status_text())
            
            # Check if we have data to plot
            if len(self.time_points) == 0:
                return
            
            # Make sure all arrays are the same length
            # This might happen during data collection when arrays are updated at slightly different times
            min_length = min(len(self.time_points), 
                        len(self.north_actual), 
                        len(self.east_actual),
                        len(self.depth_actual), 
                        len(self.heading_actual))
            
            if min_length == 0:
                return
                
            times = list(self.time_points)[:min_length]
            
            # Update data lines
            self.north_line.set_data(times, list(self.north_actual)[:min_length])
            self.east_line.set_data(times, list(self.east_actual)[:min_length])
            self.depth_line.set_data(times, list(self.depth_actual)[:min_length])
            self.heading_line.set_data(times, list(self.heading_actual)[:min_length])
            
            # Update axis limits automatically for position and depth
            for i in range(3):  # First three plots (skip heading)
                self.axs[i].relim()
                self.axs[i].autoscale_view()
            
            # Set a fixed range for heading
            self.axs[3].set_ylim(-4.0, 4.0)
            
            # Set common x-axis limits
            if times:
                max_time = times[-1]
                for ax in self.axs:
                    ax.set_xlim(0, max(10, max_time * 1.1))  # Always show at least 10 seconds
    
    def stop(self):
        """Stop data collection and close connections."""
        self.running = False
        
        # Disconnect from DVL
        if hasattr(self, 'dvl_receiver'):
            self.dvl_receiver.disconnect()
        
        # Disconnect from drone
        if self.connected_to_drone and self.drone:
            try:
                self.drone.disconnect()
                print("Disconnected from drone")
            except Exception as e:
                print(f"Error disconnecting from drone: {str(e)}")
        
        # Close the plot
        plt.close(self.fig)


def main():
    # Parse command line arguments if needed
    import argparse
    parser = argparse.ArgumentParser(description='Blueye Real Visualizer')
    parser.add_argument('--drone-ip', type=str, default='192.168.1.101',
                        help='IP address of the Blueye drone')
    parser.add_argument('--dvl-ip', type=str, default='192.168.1.99',
                        help='IP address of the DVL system')
    parser.add_argument('--dvl-port', type=int, default=16171,
                        help='Port number of the DVL system')
    args = parser.parse_args()
    
    # Create and start visualizer
    visualizer = BlueyeRealVisualizer(
        drone_ip=args.drone_ip,
        dvl_ip=args.dvl_ip,
        dvl_port=args.dvl_port
    )
    
    try:
        print("Starting visualization. Press Ctrl+C to exit.")
        plt.show()
    except KeyboardInterrupt:
        print("Stopping visualization...")
    finally:
        visualizer.stop()
        print("Visualization stopped.")


if __name__ == '__main__':
    main()