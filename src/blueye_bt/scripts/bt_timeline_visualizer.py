#!/usr/bin/env python3
# scripts/bt_realtime_visualizer.py

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import os

# Define your states in order (bottom to top)
STATES = [
    "Station Keeping",
    "Launch",
    "Descent",
    "Transit",
    "Operation",
    "Sonar Tracking",
    "Camera Tracking",
    "Cable Tension",
    "Obstacle Avoidance"
]

# Mapping from node names to states
NODE_TO_STATE = {
    "NavigateToWaypoint": "Transit",
    "StationKeeping": "Station Keeping",
    "check_camera": "Camera Tracking",
    "check_sonar": "Sonar Tracking",
    # Add more mappings as needed
}

# Default state mapping - for nodes not explicitly mapped
DEFAULT_STATE = "Operation"

class RealTimeVisualizer:
    def __init__(self, csv_file='/tmp/bt_timeline_data.csv', update_interval=1000):
        self.csv_file = csv_file
        self.update_interval = update_interval  # milliseconds
        self.state_history = []
        self.time_history = []
        self.current_state = None
        self.last_timestamp = 0
        
        # Create the figure and axis
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.fig.canvas.manager.set_window_title('Behavior Tree State Visualization')
        
        # Set up the plot
        self.setup_plot()
        
        # Initialize the animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=update_interval, save_count=100
        )
    
    def setup_plot(self):
        # Configure the axis
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('State')
        self.ax.set_yticks(range(len(STATES)))
        self.ax.set_yticklabels(STATES)
        self.ax.set_xlim(0, 500)  # Adjust as needed
        self.ax.set_ylim(-0.5, len(STATES) - 0.5)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        
        # Set title
        self.ax.set_title('States over time')
        
        # Add a line that will be updated
        self.line, = self.ax.plot([], [], 'purple', linewidth=2, label='State')
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Adjust layout
        plt.tight_layout()
    
    def update_plot(self, frame):
        if not os.path.exists(self.csv_file):
            return self.line,
        
        try:
            # Read new data
            df = pd.read_csv(self.csv_file)
            if df.empty:
                return self.line,
            
            # Filter for RUNNING status entries
            running_df = df[df['status'] == 'RUNNING']
            if running_df.empty:
                return self.line,
            
            # Process each state change
            for idx, row in running_df.iterrows():
                time_val = row['timestamp']
                node_name = row['node_name']
                
                # Skip if we've already processed this timestamp
                if time_val <= self.last_timestamp:
                    continue
                
                self.last_timestamp = time_val
                
                # Determine state from node name
                state = self.node_to_state(node_name)
                
                # If the state has changed
                if state != self.current_state:
                    # If this is not the first state change
                    if self.current_state is not None:
                        # Add a data point to maintain the previous state until this time
                        self.time_history.append(time_val)
                        self.state_history.append(self.state_to_index(self.current_state))
                    
                    # Add the new state
                    self.current_state = state
                    self.time_history.append(time_val)
                    self.state_history.append(self.state_to_index(state))
            
            # Update line data
            if self.time_history and self.state_history:
                self.line.set_data(self.time_history, self.state_history)
                
                # Adjust x limit to show all data plus some margin
                if self.time_history[-1] > self.ax.get_xlim()[1]:
                    self.ax.set_xlim(0, self.time_history[-1] * 1.1)
        
        except Exception as e:
            print(f"Error updating plot: {e}")
        
        return self.line,
    
    def node_to_state(self, node_name):
        # Extract base node name (remove numeric identifiers if present)
        base_name = node_name.split('_')[0] if '_' in node_name else node_name
        
        # Try to match with known node types
        for pattern, state in NODE_TO_STATE.items():
            if pattern in node_name:
                return state
        
        return DEFAULT_STATE
    
    def state_to_index(self, state):
        # Convert state name to y-axis index
        try:
            return STATES.index(state)
        except ValueError:
            # If state not found, default to Operation
            return STATES.index(DEFAULT_STATE)
    
    def show(self):
        plt.show()
    
    def save(self, filename='/tmp/bt_realtime_timeline.png'):
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Timeline saved to {filename}")

if __name__ == "__main__":
    visualizer = RealTimeVisualizer()
    visualizer.show()