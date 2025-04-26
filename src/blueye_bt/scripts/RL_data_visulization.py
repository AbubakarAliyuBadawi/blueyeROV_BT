#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from collections import defaultdict
import os
import argparse

# Define state and action mappings for readability
BATTERY_LEVELS = {0: "Low", 1: "Medium", 2: "High"}
DISTANCE_LEVELS = {0: "Near", 1: "Medium", 2: "Far"}
TASK_NAMES = {0: "Pipeline Inspection", 1: "Wreckage Inspection", 2: "Return to Dock"}

def parse_qtable(filename):
    """Parse the Q-table file saved by the C++ code."""
    data = []
    try:
        with open(filename, 'r') as f:
            version = f.readline().strip()
            if version != 'v1':
                print(f"Unknown format: {version}")
                return []
                
            entries = int(f.readline().strip())
            for _ in range(entries):
                line = f.readline().strip().split()
                
                # Parse state
                battery = int(line[0])
                camera = int(line[1]) == 1
                sonar = int(line[2]) == 1
                distance = int(line[3])
                
                # Parse order
                order_size = int(line[4])
                order = tuple(int(line[5+i]) for i in range(order_size))
                
                # Parse Q-value
                q_value = float(line[5+order_size])
                
                data.append({
                    'battery': battery,
                    'camera': camera,
                    'sonar': sonar,
                    'distance': distance,
                    'order': order,
                    'q_value': q_value
                })
        print(f"Successfully loaded {len(data)} Q-value entries")
        return data
    except FileNotFoundError:
        print(f"File not found: {filename}")
        return []
    except Exception as e:
        print(f"Error parsing Q-table: {e}")
        return []

def order_to_string(order):
    """Convert a task order tuple to a readable string."""
    return " → ".join([TASK_NAMES.get(task, f"Task{task}") for task in order])

def create_qtable_heatmap(data):
    """Create a heatmap of Q-values for different states and actions."""
    if not data:
        print("No data to visualize")
        return
    
    # Group data by state
    state_data = defaultdict(list)
    for entry in data:
        state = (entry['battery'], entry['camera'], entry['sonar'], entry['distance'])
        state_data[state].append((entry['order'], entry['q_value']))
    
    # Prepare data for heatmap
    states = []
    actions = []
    q_values = []
    
    for state, action_values in state_data.items():
        battery, camera, sonar, distance = state
        state_str = f"B:{BATTERY_LEVELS[battery]}, C:{'✓' if camera else '✗'}, S:{'✓' if sonar else '✗'}, D:{DISTANCE_LEVELS[distance]}"
        
        for action, value in action_values:
            action_str = order_to_string(action)
            states.append(state_str)
            actions.append(action_str)
            q_values.append(value)
    
    # Create DataFrame for heatmap
    df = pd.DataFrame({'State': states, 'Action': actions, 'Q-Value': q_values})
    pivot_df = df.pivot_table(index='State', columns='Action', values='Q-Value', aggfunc='mean')
    
    # Plot heatmap
    plt.figure(figsize=(15, 10))
    ax = sns.heatmap(pivot_df, annot=True, cmap="YlGnBu", linewidths=.5, cbar_kws={'label': 'Q-Value'})
    plt.title("Q-Values Heatmap by State and Action")
    plt.tight_layout()
    plt.savefig("q_value_heatmap.png", dpi=300, bbox_inches='tight')
    print("Saved Q-value heatmap to q_value_heatmap.png")
    plt.close()

def create_best_action_plot(data):
    """Create a visualization of the best action for each state."""
    if not data:
        print("No data to visualize")
        return
    
    # Group data by state and find best action
    best_actions = {}
    for entry in data:
        state = (entry['battery'], entry['camera'], entry['sonar'], entry['distance'])
        if state not in best_actions or entry['q_value'] > best_actions[state][1]:
            best_actions[state] = (entry['order'], entry['q_value'])
    
    # Organize states by battery and distance
    states_by_battery_distance = defaultdict(list)
    for state, (action, value) in best_actions.items():
        battery, camera, sonar, distance = state
        key = (battery, distance)
        states_by_battery_distance[key].append((camera, sonar, action, value))
    
    # Create a plot grid for different battery/distance combinations
    fig, axes = plt.subplots(3, 3, figsize=(15, 15), sharex=True, sharey=True)
    plt.subplots_adjust(hspace=0.3, wspace=0.3)
    
    for i, battery in enumerate(range(3)):  # Low, Medium, High
        for j, distance in enumerate(range(3)):  # Near, Medium, Far
            key = (battery, distance)
            ax = axes[i][j]
            
            if key in states_by_battery_distance:
                # Prepare data for this battery/distance combination
                sensor_states = []
                actions = []
                values = []
                
                for camera, sonar, action, value in states_by_battery_distance[key]:
                    sensor_str = f"Camera: {'✓' if camera else '✗'}, Sonar: {'✓' if sonar else '✗'}"
                    action_str = order_to_string(action)
                    sensor_states.append(sensor_str)
                    actions.append(action_str)
                    values.append(value)
                
                # Create a small table for this subplot
                table_data = []
                for s, a, v in zip(sensor_states, actions, values):
                    table_data.append([s, a, f"{v:.2f}"])
                
                ax.axis('tight')
                ax.axis('off')
                table = ax.table(cellText=table_data, 
                                colLabels=['Sensors', 'Best Action', 'Q-Value'],
                                loc='center',
                                cellLoc='center')
                table.auto_set_font_size(False)
                table.set_fontsize(8)
                table.scale(1, 1.5)
            
            ax.set_title(f"Battery: {BATTERY_LEVELS[battery]}, Distance: {DISTANCE_LEVELS[distance]}")
    
    plt.suptitle("Best Action by State", fontsize=16)
    plt.savefig("best_actions.png", dpi=300, bbox_inches='tight')
    print("Saved best actions visualization to best_actions.png")
    plt.close()

def create_reward_function_visualization():
    """Create a visualization of the reward function components."""
    # Define components of the reward function
    components = ['Base Success', 'Base Failure', 'Low Battery Efficiency', 
                  'Camera for Pipeline', 'Sonar for Wreckage', 'Low Battery Return Penalty']
    values = [10, -5, 5, 3, 3, -4]
    colors = ['green', 'red', 'blue', 'purple', 'orange', 'brown']
    
    # Create bar chart
    plt.figure(figsize=(12, 6))
    bars = plt.bar(components, values, color=colors)
    
    # Add value labels on top of bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                f'{height:+.0f}', ha='center', va='bottom')
    
    plt.axhline(y=0, color='black', linestyle='-', alpha=0.3)
    plt.title('Reward Function Components')
    plt.ylabel('Reward Value')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig("reward_function.png", dpi=300)
    print("Saved reward function visualization to reward_function.png")
    plt.close()

def create_exploration_rate_plot():
    """Create a visualization of how exploration rate affects learning."""
    # Simulate learning progress with different exploration rates
    episodes = range(1, 101)
    high_exploration = [max(0.5 * (0.99 ** i), 0.01) for i in episodes]
    medium_exploration = [max(0.3 * (0.97 ** i), 0.01) for i in episodes]
    low_exploration = [max(0.1 * (0.95 ** i), 0.01) for i in episodes]
    
    plt.figure(figsize=(10, 6))
    plt.plot(episodes, high_exploration, label='High Initial (ε=0.5)', linewidth=2)
    plt.plot(episodes, medium_exploration, label='Medium Initial (ε=0.3)', linewidth=2)
    plt.plot(episodes, low_exploration, label='Low Initial (ε=0.1)', linewidth=2)
    
    plt.axhline(y=0.01, color='gray', linestyle='--', alpha=0.7, label='Minimum Exploration (ε=0.01)')
    
    plt.title('Exploration Rate Decay Over Episodes')
    plt.xlabel('Episode')
    plt.ylabel('Exploration Rate (ε)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("exploration_rate.png", dpi=300)
    print("Saved exploration rate visualization to exploration_rate.png")
    plt.close()

def create_learning_curve():
    """Create a simulated learning curve."""
    # Simulate average reward per episode
    episodes = range(1, 101)
    
    # Create a noisy learning curve
    np.random.seed(42)  # For reproducibility
    base_curve = -5 + 15 * (1 - np.exp(-0.05 * np.array(episodes)))
    noise = np.random.normal(0, 2, len(episodes))
    rewards = base_curve + noise
    
    # Calculate moving average
    window_size = 10
    moving_avg = np.convolve(rewards, np.ones(window_size)/window_size, mode='valid')
    moving_avg_x = episodes[window_size-1:]
    
    plt.figure(figsize=(10, 6))
    plt.plot(episodes, rewards, 'o-', alpha=0.4, label='Episode Reward')
    plt.plot(moving_avg_x, moving_avg, 'r-', linewidth=2, label=f'{window_size}-Episode Moving Average')
    
    plt.axhline(y=10, color='green', linestyle='--', alpha=0.7, label='Maximum Task Reward')
    plt.axhline(y=0, color='gray', linestyle='-', alpha=0.5)
    plt.axhline(y=-5, color='red', linestyle='--', alpha=0.7, label='Task Failure Penalty')
    
    plt.title('Simulated Learning Curve for ROV Mission Planning')
    plt.xlabel('Episode')
    plt.ylabel('Average Reward')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("learning_curve.png", dpi=300)
    print("Saved simulated learning curve to learning_curve.png")
    plt.close()

def create_sensor_impact_visualization():
    """Create a visualization of how sensor availability impacts mission success."""
    # Define scenarios
    scenarios = ['All Sensors\nWorking', 'Camera\nOffline', 'Sonar\nOffline', 'Both Sensors\nOffline']
    
    # Simulated success rates for each scenario
    success_rates = [0.95, 0.60, 0.70, 0.30]
    
    # Simulated best actions for each scenario
    best_actions = [
        "Pipeline → Wreckage → Return",
        "Return → Wreckage → Pipeline",
        "Pipeline → Return → Wreckage",
        "Return → Pipeline → Wreckage"
    ]
    
    # Create bar chart
    plt.figure(figsize=(12, 7))
    bars = plt.bar(scenarios, success_rates, color=['green', 'orange', 'orange', 'red'])
    
    # Add percentage labels on top of bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                f'{height:.0%}', ha='center', va='bottom')
    
    # Add best actions as text under each bar
    for i, action in enumerate(best_actions):
        plt.text(i, -0.07, f"Best: {action}", ha='center', va='top', 
                 rotation=0, fontsize=9, color='blue')
    
    plt.title('Impact of Sensor Availability on Mission Success')
    plt.ylabel('Simulated Success Rate')
    plt.ylim(0, 1.1)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig("sensor_impact.png", dpi=300)
    print("Saved sensor impact visualization to sensor_impact.png")
    plt.close()

def main():
    parser = argparse.ArgumentParser(description='Visualize RL data from Q-table')
    parser.add_argument('--qtable', type=str, default='/tmp/mission_q_table.txt',
                       help='Path to the Q-table file (default: /tmp/mission_q_table.txt)')
    args = parser.parse_args()
    
    # Create output directory
    output_dir = "rl_visualizations"
    os.makedirs(output_dir, exist_ok=True)
    os.chdir(output_dir)
    
    # Parse Q-table and create visualizations
    data = parse_qtable(args.qtable)
    
    # Create visualizations based on actual Q-table data (if available)
    if data:
        create_qtable_heatmap(data)
        create_best_action_plot(data)
    else:
        print("No Q-table data found. Creating conceptual visualizations only.")
    
    # Create conceptual visualizations (don't require actual data)
    create_reward_function_visualization()
    create_exploration_rate_plot()
    create_learning_curve()
    create_sensor_impact_visualization()
    
    print(f"All visualizations saved to {os.path.abspath(output_dir)}")

if __name__ == "__main__":
    main()