#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
import argparse
import os
from datetime import datetime

def plot_sensor_timeline(csv_file, output_dir=None):
    """
    Create visualizations from sensor log data.
    
    Args:
        csv_file: Path to the sensor log CSV file
        output_dir: Directory to save output files (default: same directory as CSV)
    """
    print(f"Analyzing log file: {csv_file}")
    
    # Set up output directory
    if output_dir is None:
        output_dir = os.path.dirname(csv_file)
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract timestamp from filename for output naming
    timestamp = os.path.basename(csv_file).replace('sensor_log_', '').replace('.csv', '')
    
    # Read the CSV file
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded {len(df)} log entries")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Convert timestamp to datetime
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])
    
    # Create a base filename for output files
    base_filename = f"sensor_analysis_{timestamp}"
    
    # 1. Create a timeline of sensor status
    create_sensor_status_timeline(df, os.path.join(output_dir, f"{base_filename}_status.png"))
    
    # 2. Create event occurrence chart
    create_event_occurrence_chart(df, os.path.join(output_dir, f"{base_filename}_events.png"))
    
    # 3. Create mission state transitions chart
    create_mission_state_chart(df, os.path.join(output_dir, f"{base_filename}_states.png"))
    
    # 4. Calculate and plot sensor failure statistics
    create_failure_stats_report(df, os.path.join(output_dir, f"{base_filename}_stats.txt"))
    
    # 5. Create time-to-emergency-response analysis
    create_emergency_response_analysis(df, os.path.join(output_dir, f"{base_filename}_response.png"))
    
    print(f"Analysis complete. Files saved to {output_dir}")

def create_sensor_status_timeline(df, output_file):
    """Create a timeline showing camera and sonar status."""
    print("Creating sensor status timeline...")
    
    # Extract camera and sonar status changes
    camera_status = df[['Timestamp', 'Camera Status']].drop_duplicates()
    sonar_status = df[['Timestamp', 'Sonar Status']].drop_duplicates()
    
    # Create a figure
    plt.figure(figsize=(24, 10))
    
    # Plot camera status (1 = Active, 0 = Inactive)
    camera_y = [1 if status == 'Active' else 0 for status in camera_status['Camera Status']]
    plt.step(camera_status['Timestamp'], camera_y, where='post', label='Camera', color='green', linewidth=2)
    
    # Plot sonar status
    sonar_y = [1 if status == 'Active' else 0 for status in sonar_status['Sonar Status']]
    plt.step(sonar_status['Timestamp'], sonar_y, where='post', label='Sonar', color='blue', linewidth=2)
    
    # Highlight failure events
    failure_events = df[df['Event Type'].str.contains('FAILURE', na=False)]
    for _, event in failure_events.iterrows():
        plt.axvline(x=event['Timestamp'], color='red', linestyle='--', alpha=0.7)
        plt.text(event['Timestamp'], 0.5, event['Event Type'].replace('_FAILURE', ''), 
                 rotation=90, verticalalignment='center')
    
    # Highlight emergency events
    emergency_events = df[df['Event Type'].str.contains('EMERGENCY', na=False)]
    for _, event in emergency_events.iterrows():
        plt.axvline(x=event['Timestamp'], color='darkred', linestyle='-', alpha=0.8, linewidth=2)
        plt.text(event['Timestamp'], 0.5, 'EMERGENCY', 
                 rotation=90, verticalalignment='center', color='darkred', fontweight='bold')
    
    # Format the plot
    plt.ylim(-0.1, 1.1)
    plt.yticks([0, 1], ['Inactive', 'Active'])
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='upper right')
    plt.title('Sensor Status Timeline')
    plt.ylabel('Status')
    
    # Format x-axis with times
    plt.gcf().autofmt_xdate()
    date_format = mdates.DateFormatter('%H:%M:%S')
    plt.gca().xaxis.set_major_formatter(date_format)
    
    # Save the figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Saved sensor timeline to {output_file}")

def create_event_occurrence_chart(df, output_file):
    """Create a bar chart of event occurrences."""
    print("Creating event occurrence chart...")
    
    # Count event types
    event_counts = df['Event Type'].value_counts()
    
    # Create the plot
    plt.figure(figsize=(24, 10))
    bars = plt.bar(event_counts.index, event_counts.values, color='steelblue')
    
    # Add count labels on top of bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 0.1,
                f'{height:.0f}', ha='center', va='bottom')
    
    # Format the plot
    plt.title('Event Occurrences')
    plt.ylabel('Count')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.xticks(rotation=45, ha='right')
    
    # Save the figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Saved event occurrence chart to {output_file}")

def create_mission_state_chart(df, output_file):
    """Create a timeline of mission states."""
    print("Creating mission state timeline...")
    
    # Extract state changes
    state_changes = df[df['Event Type'] == 'STATE_CHANGE']
    
    if len(state_changes) == 0:
        print("No state changes found in log")
        return
    
    # Extract state from details (format: "From X to Y")
    states = []
    times = []
    
    for _, row in state_changes.iterrows():
        detail = row['Details']
        parts = detail.split(' to ')
        if len(parts) > 1:
            to_state = parts[1]
            states.append(to_state)
            times.append(row['Timestamp'])
    
    if len(states) == 0:
        print("Could not parse state changes")
        return
    
    # Create a figure
    plt.figure(figsize=(24, 10))
    
    # Convert states to numerical values for plotting
    unique_states = list(dict.fromkeys(states))
    state_values = [unique_states.index(state) for state in states]
    
    # Create step plot
    plt.step(times, state_values, where='post', marker='o', color='purple', linewidth=2)
    
    # Format the plot
    plt.yticks(range(len(unique_states)), unique_states)
    plt.title('Mission State Timeline')
    plt.ylabel('Mission State')
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Format x-axis with times
    plt.gcf().autofmt_xdate()
    date_format = mdates.DateFormatter('%H:%M:%S')
    plt.gca().xaxis.set_major_formatter(date_format)
    
    # Highlight emergency return transitions
    emergency_events = df[df['Event Type'].str.contains('EMERGENCY', na=False)]
    for _, event in emergency_events.iterrows():
        plt.axvline(x=event['Timestamp'], color='red', linestyle='--', alpha=0.7)
        plt.text(event['Timestamp'], len(unique_states)/2, 'EMERGENCY', 
                 rotation=90, verticalalignment='center', color='red')
    
    # Save the figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Saved mission state chart to {output_file}")

def create_failure_stats_report(df, output_file):
    """Create a text report with failure statistics."""
    print("Creating failure statistics report...")
    
    # Calculate statistics
    camera_failures = df[df['Event Type'] == 'CAMERA_FAILURE']
    sonar_failures = df[df['Event Type'] == 'SONAR_FAILURE']
    emergency_returns = df[df['Event Type'] == 'EMERGENCY_RETURN']
    
    camera_recoveries = df[df['Event Type'] == 'CAMERA_RECOVERY']
    sonar_recoveries = df[df['Event Type'] == 'SONAR_RECOVERY']
    
    # Calculate failure durations if recoveries exist
    camera_durations = []
    sonar_durations = []
    
    if len(camera_failures) > 0 and len(camera_recoveries) > 0:
        for _, failure in camera_failures.iterrows():
            failure_time = failure['Timestamp']
            # Find the next recovery after this failure
            next_recoveries = camera_recoveries[camera_recoveries['Timestamp'] > failure_time]
            if len(next_recoveries) > 0:
                recovery_time = next_recoveries.iloc[0]['Timestamp']
                duration = (recovery_time - failure_time).total_seconds()
                camera_durations.append(duration)
    
    if len(sonar_failures) > 0 and len(sonar_recoveries) > 0:
        for _, failure in sonar_failures.iterrows():
            failure_time = failure['Timestamp']
            # Find the next recovery after this failure
            next_recoveries = sonar_recoveries[sonar_recoveries['Timestamp'] > failure_time]
            if len(next_recoveries) > 0:
                recovery_time = next_recoveries.iloc[0]['Timestamp']
                duration = (recovery_time - failure_time).total_seconds()
                sonar_durations.append(duration)
    
    # Calculate time to emergency response
    emergency_response_times = []
    if len(emergency_returns) > 0:
        for _, emergency in emergency_returns.iterrows():
            emergency_time = emergency['Timestamp']
            # Find the most recent failure before this emergency
            recent_failures = df[(df['Event Type'].str.contains('FAILURE', na=False)) & 
                                (df['Timestamp'] < emergency_time)]
            if len(recent_failures) > 0:
                latest_failure = recent_failures.iloc[-1]
                failure_time = latest_failure['Timestamp']
                response_time = (emergency_time - failure_time).total_seconds()
                emergency_response_times.append(response_time)
    
    # Write the report
    with open(output_file, 'w') as f:
        f.write("===== Sensor Failure Statistics =====\n\n")
        f.write(f"Report generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        
        f.write("== Event Counts ==\n")
        f.write(f"Camera failures: {len(camera_failures)}\n")
        f.write(f"Sonar failures: {len(sonar_failures)}\n")
        f.write(f"Camera recoveries: {len(camera_recoveries)}\n")
        f.write(f"Sonar recoveries: {len(sonar_recoveries)}\n")
        f.write(f"Emergency returns triggered: {len(emergency_returns)}\n\n")
        
        f.write("== Failure Durations ==\n")
        if camera_durations:
            f.write(f"Camera failure average duration: {np.mean(camera_durations):.2f} seconds\n")
            f.write(f"Camera failure min duration: {np.min(camera_durations):.2f} seconds\n")
            f.write(f"Camera failure max duration: {np.max(camera_durations):.2f} seconds\n")
        else:
            f.write("No camera failure durations calculated\n")
            
        if sonar_durations:
            f.write(f"Sonar failure average duration: {np.mean(sonar_durations):.2f} seconds\n")
            f.write(f"Sonar failure min duration: {np.min(sonar_durations):.2f} seconds\n")
            f.write(f"Sonar failure max duration: {np.max(sonar_durations):.2f} seconds\n")
        else:
            f.write("No sonar failure durations calculated\n")
        f.write("\n")
        
        f.write("== Emergency Response ==\n")
        if emergency_response_times:
            f.write(f"Average time to emergency response: {np.mean(emergency_response_times):.2f} seconds\n")
            f.write(f"Min time to emergency response: {np.min(emergency_response_times):.2f} seconds\n")
            f.write(f"Max time to emergency response: {np.max(emergency_response_times):.2f} seconds\n")
        else:
            f.write("No emergency response times calculated\n")
    
    print(f"Saved failure statistics to {output_file}")

def create_emergency_response_analysis(df, output_file):
    """Create an analysis of time between failures and emergency responses."""
    print("Creating emergency response analysis...")
    
    # Find failures followed by emergency returns
    emergency_returns = df[df['Event Type'] == 'EMERGENCY_RETURN']
    if len(emergency_returns) == 0:
        print("No emergency returns found in log")
        return
    
    # For each emergency, find the preceding failure
    response_times = []
    failure_types = []
    
    for _, emergency in emergency_returns.iterrows():
        emergency_time = emergency['Timestamp']
        # Find all failures before this emergency
        failures = df[(df['Event Type'].str.contains('FAILURE', na=False)) & 
                    (df['Timestamp'] < emergency_time)]
        
        if len(failures) > 0:
            # Get the most recent failure
            latest_failure = failures.iloc[-1]
            failure_time = latest_failure['Timestamp']
            failure_type = latest_failure['Event Type']
            
            # Calculate response time
            response_time = (emergency_time - failure_time).total_seconds()
            response_times.append(response_time)
            failure_types.append(failure_type)
    
    if not response_times:
        print("No failure-to-emergency sequences found")
        return
    
    # Create plot
    plt.figure(figsize=(24, 10))
    
    # Separate by failure type
    camera_times = [t for t, f in zip(response_times, failure_types) if 'CAMERA' in f]
    sonar_times = [t for t, f in zip(response_times, failure_types) if 'SONAR' in f]
    
    # Create bar chart
    bars = plt.bar(['Camera Failure', 'Sonar Failure'], 
                  [np.mean(camera_times) if camera_times else 0, 
                   np.mean(sonar_times) if sonar_times else 0],
                  yerr=[np.std(camera_times) if len(camera_times) > 1 else 0,
                        np.std(sonar_times) if len(sonar_times) > 1 else 0],
                  capsize=10,
                  color=['green', 'blue'])
    
    # Add value labels
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 0.1,
                f'{height:.2f}s', ha='center', va='bottom')
    
    # Format plot
    plt.title('Average Time to Emergency Response')
    plt.ylabel('Response Time (seconds)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Save the figure
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Saved emergency response analysis to {output_file}")

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Analyze sensor log data.')
    parser.add_argument('csv_file', help='Path to the sensor log CSV file')
    parser.add_argument('--output-dir', help='Directory to save output files')
    args = parser.parse_args()
    
    # Run the analysis
    plot_sensor_timeline(args.csv_file, args.output_dir)