"""
Position Data Visualization Script

This script visualizes position estimate data (North, East, Heading) from CSV files.

IMPORTANT NOTE ABOUT HEADING DATA:
The heading data in your files appears to be "unwrapped" - meaning it goes beyond 
the typical ±π (-3.14 to 3.14) range. This is common in navigation systems where:
- The system tracks continuous rotation without wrapping back to ±π
- Multiple full rotations need to be distinguished
- The data represents cumulative angular change

This script provides both wrapped (±π) and unwrapped versions for comparison.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi] range
    """
    return np.arctan2(np.sin(angle), np.cos(angle))

def load_and_parse_csv(filename, wrap_heading=False):
    """
    Load and parse the CSV files that have data concatenated in the value column
    """
    try:
        # Read the CSV file
        df = pd.read_csv(filename)
        
        # Initialize lists to store parsed data
        elapsed_times = []
        timestamps = []
        values = []
        
        # Parse each row
        for _, row in df.iterrows():
            elapsed_time = row['elapsed time']
            timestamp = row['timestamp']
            
            # The 'value' column contains the actual value we need
            value = row['value']
            
            # Wrap heading angles if requested
            if wrap_heading and 'heading' in filename.lower():
                value = wrap_angle(value)
            
            elapsed_times.append(elapsed_time)
            timestamps.append(timestamp)
            values.append(value)
        
        # Create a clean DataFrame
        parsed_df = pd.DataFrame({
            'elapsed_time': elapsed_times,
            'timestamp': timestamps,
            'value': values
        })
        
        return parsed_df
        
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None

def create_position_subplots(wrap_heading_angles=True):
    """
    Create subplot visualization with North, East, and Heading data
    
    Parameters:
    wrap_heading_angles (bool): If True, wrap heading angles to [-pi, pi] range
    """
    # Load the three datasets
    north_df = load_and_parse_csv('north_3.csv')
    east_df = load_and_parse_csv('east_3.csv')
    heading_df = load_and_parse_csv('heading_3.csv', wrap_heading=wrap_heading_angles)
    
    # Also load unwrapped version for comparison
    heading_unwrapped_df = load_and_parse_csv('heading_3.csv', wrap_heading=False)
    
    # Check if all files loaded successfully
    if north_df is None or east_df is None or heading_df is None:
        print("Error: Could not load one or more CSV files")
        return
    
    # Create figure with 4 subplots (4 rows, 1 column) to show both heading versions
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle('Position Estimate Data Over Time', fontsize=16, fontweight='bold')
    
    # Plot North data (top subplot)
    axes[0].plot(north_df['elapsed_time'], north_df['value'], 'b-', linewidth=1, alpha=0.8)
    axes[0].set_ylabel('Northing (m)', fontsize=12, fontweight='bold')
    axes[0].set_title('North Position', fontsize=12, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_facecolor('#f8f9fa')
    
    # Add statistics text
    north_stats = f'Min: {north_df["value"].min():.3f}m, Max: {north_df["value"].max():.3f}m, Mean: {north_df["value"].mean():.3f}m'
    axes[0].text(0.02, 0.95, north_stats, transform=axes[0].transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                fontsize=9, verticalalignment='top')
    
    # Plot East data (second subplot)
    axes[1].plot(east_df['elapsed_time'], east_df['value'], 'r-', linewidth=1, alpha=0.8)
    axes[1].set_ylabel('Easting (m)', fontsize=12, fontweight='bold')
    axes[1].set_title('East Position', fontsize=12, fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_facecolor('#f8f9fa')
    
    # Add statistics text
    east_stats = f'Min: {east_df["value"].min():.3f}m, Max: {east_df["value"].max():.3f}m, Mean: {east_df["value"].mean():.3f}m'
    axes[1].text(0.02, 0.95, east_stats, transform=axes[1].transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                fontsize=9, verticalalignment='top')
    
    # Plot Wrapped Heading data (third subplot)
    axes[2].plot(heading_df['elapsed_time'], heading_df['value'], 'g-', linewidth=1, alpha=0.8)
    axes[2].set_ylabel('Heading (rad)', fontsize=12, fontweight='bold')
    axes[2].set_title('Heading (Wrapped to ±π)', fontsize=12, fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_facecolor('#f8f9fa')
    axes[2].axhline(y=np.pi, color='k', linestyle='--', alpha=0.5, label='±π bounds')
    axes[2].axhline(y=-np.pi, color='k', linestyle='--', alpha=0.5)
    axes[2].set_ylim(-np.pi - 0.5, np.pi + 0.5)
    
    # Add statistics text
    heading_stats = f'Min: {heading_df["value"].min():.3f}rad, Max: {heading_df["value"].max():.3f}rad, Mean: {heading_df["value"].mean():.3f}rad'
    axes[2].text(0.02, 0.95, heading_stats, transform=axes[2].transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                fontsize=9, verticalalignment='top')
    
    # Plot Unwrapped Heading data (fourth subplot)
    axes[3].plot(heading_unwrapped_df['elapsed_time'], heading_unwrapped_df['value'], 'orange', linewidth=1, alpha=0.8)
    axes[3].set_ylabel('Heading (rad)', fontsize=12, fontweight='bold')
    axes[3].set_xlabel('Elapsed Time (s)', fontsize=12, fontweight='bold')
    axes[3].set_title('Heading (Original/Unwrapped)', fontsize=12, fontweight='bold')
    axes[3].grid(True, alpha=0.3)
    axes[3].set_facecolor('#f8f9fa')
    
    # Add statistics text for unwrapped heading
    unwrapped_stats = f'Min: {heading_unwrapped_df["value"].min():.3f}rad, Max: {heading_unwrapped_df["value"].max():.3f}rad'
    axes[3].text(0.02, 0.95, unwrapped_stats, transform=axes[3].transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                fontsize=9, verticalalignment='top')
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Add some spacing between subplots
    plt.subplots_adjust(hspace=0.4)
    
    # Display the plot
    plt.show()
    
    # Print data summary
    print("Data Summary:")
    print("=" * 50)
    print(f"North data points: {len(north_df)}")
    print(f"East data points: {len(east_df)}")
    print(f"Heading data points: {len(heading_df)}")
    print(f"\nTime range: {north_df['elapsed_time'].min():.1f}s to {north_df['elapsed_time'].max():.1f}s")
    
    print(f"\nHeading Analysis:")
    print(f"Original (unwrapped) range: {heading_unwrapped_df['value'].min():.3f} to {heading_unwrapped_df['value'].max():.3f} rad")
    print(f"Wrapped range: {heading_df['value'].min():.3f} to {heading_df['value'].max():.3f} rad")
    print(f"Total rotation: {heading_unwrapped_df['value'].max() - heading_unwrapped_df['value'].min():.3f} rad")
    print(f"Equivalent in degrees: {np.degrees(heading_unwrapped_df['value'].max() - heading_unwrapped_df['value'].min()):.1f}°")
    
def create_combined_plot(wrap_heading_angles=True):
    """
    Alternative visualization: All three datasets on a single plot with different y-axes
    """
    # Load the datasets
    north_df = load_and_parse_csv('north_3.csv')
    east_df = load_and_parse_csv('east_3.csv')
    heading_df = load_and_parse_csv('heading_3.csv', wrap_heading=wrap_heading_angles)
    
    if north_df is None or east_df is None or heading_df is None:
        print("Error: Could not load one or more CSV files")
        return
    
    fig, ax1 = plt.subplots(figsize=(12, 8))
    
    # Plot North and East on the primary y-axis
    ax1.plot(north_df['elapsed_time'], north_df['value'], 'b-', label='North (m)', linewidth=1.5, alpha=0.8)
    ax1.plot(east_df['elapsed_time'], east_df['value'], 'r-', label='East (m)', linewidth=1.5, alpha=0.8)
    ax1.set_xlabel('Elapsed Time (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Position (m)', fontsize=12, fontweight='bold', color='black')
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor('#f8f9fa')
    
    # Create secondary y-axis for heading
    ax2 = ax1.twinx()
    ax2.plot(heading_df['elapsed_time'], heading_df['value'], 'g-', label='Heading (rad)', linewidth=1.5, alpha=0.8)
    ax2.set_ylabel('Heading (rad)', fontsize=12, fontweight='bold', color='green')
    ax2.tick_params(axis='y', labelcolor='green')

    # Add legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    plt.title('Position Estimate Data - Combined View', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Create the main subplot visualization
    print("Creating subplot visualization...")
    print("Note: The heading data appears to be unwrapped (beyond ±π range).")
    print("This likely indicates continuous rotation tracking without angle wrapping.")
    print("The visualization will show both wrapped and unwrapped versions for comparison.\n")
    
    create_position_subplots(wrap_heading_angles=True)
    
    # # Optionally create the combined plot
    # print("\nCreating combined visualization...")
    # create_combined_plot(wrap_heading_angles=True)