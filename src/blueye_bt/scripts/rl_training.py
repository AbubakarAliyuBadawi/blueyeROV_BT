import numpy as np
import random
import matplotlib.pyplot as plt
from collections import defaultdict
import os
import time
import itertools
import pandas as pd
import seaborn as sns
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap

class EnhancedROVEnvironment:
    def __init__(self):
        # State space definitions
        self.battery_levels = [0, 1, 2]  # Low, Medium, High
        self.camera_states = [False, True]  # Not working, Working
        self.sonar_states = [False, True]  # Not working, Working
        self.distance_levels = [0, 1, 2]  # Near, Medium, Far
        
        # Tasks:
        # 0: Transit_1
        # 1: Pipeline Inspection
        # 2: Transit_2
        # 3: Wreckage Inspection
        # 4: Return to Dock
        
        # Generate valid permutations that respect constraints:
        # - Transit_1 must come before Pipeline
        # - Transit_2 must come before Wreckage
        self.actions = []
        
        # Generate all possible permutations of the 5 tasks
        all_permutations = list(itertools.permutations([0, 1, 2, 3, 4]))
        
        # Filter for valid permutations (respecting constraints)
        for perm in all_permutations:
            valid = True
            # Check Transit_1 (0) comes before Pipeline (1)
            if list(perm).index(0) > list(perm).index(1):
                valid = False
            # Check Transit_2 (2) comes before Wreckage (3)
            if list(perm).index(2) > list(perm).index(3):
                valid = False
            
            if valid:
                self.actions.append(perm)
        
        print(f"Generated {len(self.actions)} valid mission orderings")
        
        # Current state
        self.reset()
        
    def reset(self):
        # Randomly initialize state
        self.battery = random.choice(self.battery_levels)
        self.camera = random.choice(self.camera_states)
        self.sonar = random.choice(self.sonar_states)
        self.distance = random.choice(self.distance_levels)
        return self.get_state()
    
    def get_state(self):
        return (self.battery, self.camera, self.sonar, self.distance)
        
    def step(self, action_idx):
        action = self.actions[action_idx]
        reward = 0
        tasks_completed = []
        
        # Simulate mission execution and calculate reward
        for task_idx, task in enumerate(action):
            # Transit_1
            if task == 0:
                tasks_completed.append(task)
                # Small reward for completing transit
                reward += 1
                # Bonus for doing transit early with high battery
                if task_idx < 2 and self.battery == 2:
                    reward += 2
            
            # Pipeline inspection
            elif task == 1:
                if self.camera:
                    reward += 5  # Success with camera
                    tasks_completed.append(task)
                else:
                    reward -= 2  # Reduced success without camera
                    tasks_completed.append(task)  # Still complete but with penalty
            
            # Transit_2
            elif task == 2:
                tasks_completed.append(task)
                # Small reward for completing transit
                reward += 1
                # Bonus for doing transit early with high battery
                if task_idx < 2 and self.battery == 2:
                    reward += 2
            
            # Wreckage inspection
            elif task == 3:
                if self.sonar:
                    reward += 5  # Success with sonar
                    tasks_completed.append(task)
                else:
                    reward -= 2  # Reduced success without sonar
                    tasks_completed.append(task)  # Still complete but with penalty
            
            # Return to dock
            elif task == 4:
                tasks_completed.append(task)
                # Higher reward for returning when battery is low
                if self.battery == 0:
                    reward += 8
                elif self.battery == 1:
                    reward += 3
                else:
                    reward += 1
        
        # Penalty for not returning to dock when battery is low
        if self.battery == 0 and action[-1] != 4:
            reward -= 5
        
        # Extra reward for using appropriate sensors for specific tasks
        if 1 in tasks_completed and self.camera:
            reward += 3  # Using camera for pipeline inspection
        if 3 in tasks_completed and self.sonar:
            reward += 3  # Using sonar for wreckage inspection
        
        # Efficiency bonus for completing multiple missions with low battery
        if len(tasks_completed) > 3 and self.battery == 0:
            reward += 5
        
        # Generate next state (for episodic training, we'll reset)
        next_state = self.reset()
        
        return next_state, reward, True  # terminal=True for episodic


def train_q_table(env, num_episodes=500, alpha=0.1, gamma=0.9, epsilon=0.3):
    # Initialize Q-table
    q_table = defaultdict(lambda: defaultdict(float))
    
    # For plotting metrics
    all_rewards = []
    avg_rewards = []
    
    # Modified: Track multiple mission orderings
    q_value_evolution = {}  # Format: {action_idx: {(episode, state): q_value}}
    
    # Choose specific state-action pairs to track for different scenarios
    track_states = [
        (2, True, True, 0),   # High battery, all sensors working, near dock
        (0, True, True, 0),   # Low battery, all sensors working, near dock
        (2, False, True, 2),  # High battery, camera offline, far dock
        (2, True, False, 0),  # High battery, sonar offline, near dock
        (0, True, True, 2),   # Low battery, all sensors working, far dock
    ]
    
    # Define multiple mission orderings to track
    mission_orderings = [
        (0, 1, 2, 3, 4),  # Transit_1 → Pipeline → Transit_2 → Wreckage → Return (preferred)
        (0, 1, 4, 2, 3),  # Transit_1 → Pipeline → Return → Transit_2 → Wreckage
        (0, 2, 3, 1, 4),  # Transit_1 → Transit_2 → Wreckage → Pipeline → Return 
        (0, 4, 1, 2, 3),  # Transit_1 → Return → Pipeline → Transit_2 → Wreckage
        (4, 0, 1, 2, 3),  # Return → Transit_1 → Pipeline → Transit_2 → Wreckage
    ]
    
    # Find indices for these orderings
    track_action_indices = {}
    for ordering in mission_orderings:
        for idx, action in enumerate(env.actions):
            if action == ordering:
                track_action_indices[ordering] = idx
                print(f"Found mission ordering {ordering} at index {idx}")
                break
    
    # Initialize separate tracking for each ordering
    for ordering in mission_orderings:
        if ordering in track_action_indices:
            q_value_evolution[ordering] = {}
    
    for episode in range(num_episodes):
        state = env.reset()
        total_reward = 0
        done = False
        
        # Choose action using epsilon-greedy
        if random.uniform(0, 1) < epsilon:
            # Exploration
            action_idx = random.randrange(len(env.actions))
        else:
            # Exploitation
            q_values = [q_table[state][a] for a in range(len(env.actions))]
            action_idx = np.argmax(q_values) if q_values else random.randrange(len(env.actions))
        
        # Take action
        next_state, reward, done = env.step(action_idx)
        total_reward += reward
        
        # Q-learning update
        old_q = q_table[state][action_idx]
        next_max_q = max([q_table[next_state][a] for a in range(len(env.actions))]) if not done else 0
        new_q = (1 - alpha) * old_q + alpha * (reward + gamma * next_max_q)
        
        q_table[state][action_idx] = new_q
        
        # Track metrics
        all_rewards.append(total_reward)
        
        # Track Q-values for all specified orderings across all states
        for state_to_track in track_states:
            for ordering, idx in track_action_indices.items():
                if state == state_to_track:
                    # Track Q-value for this ordering regardless of chosen action
                    # Get current Q-value for this ordering in this state
                    current_q = q_table[state][idx] if idx in q_table[state] else 0
                    q_value_evolution[ordering][(episode, state)] = current_q
        
        # Calculate running average reward
        if episode % 100 == 0:
            avg_reward = np.mean(all_rewards[-100:])
            avg_rewards.append(avg_reward)
            print(f"Episode {episode}, Average reward: {avg_reward:.2f}")
    
    return q_table, all_rewards, avg_rewards, q_value_evolution

def export_q_table(q_table, env, filename="/tmp/mission_q_table.txt"):
    # Count entries
    count = sum(len(actions) for actions in q_table.values())
    
    with open(filename, 'w') as f:
        # Write format version
        f.write("v1\n")
        
        # Write number of entries
        f.write(f"{count}\n")
        
        # Write each entry
        for state, actions in q_table.items():
            battery, camera, sonar, distance = state
            
            for action_idx, q_value in actions.items():
                action = env.actions[action_idx]
                
                # Convert booleans to 0/1
                camera_int = 1 if camera else 0
                sonar_int = 1 if sonar else 0
                
                # Write state
                f.write(f"{battery} {camera_int} {sonar_int} {distance} ")
                
                # Write action
                f.write(f"{len(action)} ")
                for task in action:
                    f.write(f"{task} ")
                
                # Write Q-value
                f.write(f"{q_value:.6f}\n")
    
    print(f"Q-table exported to {filename}")


def generate_learning_curve_plot(all_rewards, avg_rewards, output_dir):
    plt.figure(figsize=(10, 6))
    plt.plot(all_rewards, alpha=0.3, color='blue')
    
    # Plot average rewards
    x_values = np.arange(0, len(all_rewards), 100)
    plt.plot(x_values, avg_rewards, linewidth=2, color='red')
    
    plt.title('ROV Mission Planning Learning Curve')
    plt.xlabel('Episodes')
    plt.ylabel('Reward')
    plt.grid(False)
    plt.legend(['Episode Reward', 'Average Reward (100 episodes)'])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'learning_curve.png'), dpi=300)
    print(f"Learning curve saved to {os.path.join(output_dir, 'learning_curve.png')}")

def generate_q_value_evolution_plot(q_value_evolution, env, output_dir):
    # Create a subplot for each mission ordering
    num_orderings = len(q_value_evolution)
    
    if num_orderings == 0:
        print("No Q-value evolution data to plot")
        return
    
    # Define ordering names for titles
    ordering_names = {
        (0, 1, 2, 3, 4): "Transit_1 → Pipeline → Transit_2 → Wreckage → Return",
        (0, 1, 4, 2, 3): "Transit_1 → Pipeline → Return → Transit_2 → Wreckage",
        (0, 2, 3, 1, 4): "Transit_1 → Transit_2 → Wreckage → Pipeline → Return",
        (0, 4, 1, 2, 3): "Transit_1 → Return → Pipeline → Transit_2 → Wreckage",
        (4, 0, 1, 2, 3): "Return → Transit_1 → Pipeline → Transit_2 → Wreckage",
    }
    
    # Create a separate figure for each mission ordering
    for ordering, state_data in q_value_evolution.items():
        plt.figure(figsize=(10, 6))
        
        # Group by state
        grouped_data = defaultdict(list)
        for (episode, state), value in state_data.items():
            grouped_data[state].append((episode, value))
        
        # Plot each state's evolution
        colors = ['blue', 'green', 'red', 'purple', 'orange']
        labels = []
        
        for i, (state, data) in enumerate(grouped_data.items()):
            battery, camera, sonar, distance = state
            color = colors[i % len(colors)]
            
            # Sort by episode
            data.sort(key=lambda x: x[0])
            episodes = [x[0] for x in data]
            values = [x[1] for x in data]
            
            plt.plot(episodes, values, marker='o', markersize=3, linewidth=1, color=color)
            
            # Create label for legend
            battery_str = ["Low", "Medium", "High"][battery]
            camera_str = "Camera ON" if camera else "Camera OFF"
            sonar_str = "Sonar ON" if sonar else "Sonar OFF"
            distance_str = ["Near", "Medium", "Far"][distance]
            
            label = f"{battery_str} Battery, {camera_str}, {sonar_str}, {distance_str} Dock"
            labels.append(label)
        
        # Set title to show the specific mission ordering
        plt.title(f'Q-Value Evolution for {ordering_names.get(ordering, str(ordering))}')
        plt.xlabel('Episode')
        plt.ylabel('Q-Value')
        plt.grid(False)
        plt.legend(labels, loc='best')
        
        plt.tight_layout()
        
        # Create a filename based on the ordering
        ordering_str = "_".join(str(x) for x in ordering)
        plt.savefig(os.path.join(output_dir, f'q_value_evolution_{ordering_str}.png'), dpi=300)
        print(f"Q-value evolution for ordering {ordering} saved to {os.path.join(output_dir, f'q_value_evolution_{ordering_str}.png')}")
    
    # Create a combined plot with the most interesting states
    plt.figure(figsize=(12, 8))
    
    # Choose one state to show across all orderings
    highlight_state = (2, True, True, 0)  # High battery, all sensors working, near dock
    
    for i, (ordering, state_data) in enumerate(q_value_evolution.items()):
        data_for_state = [(episode, value) for (episode, state), value in state_data.items() if state == highlight_state]
        
        if data_for_state:
            data_for_state.sort(key=lambda x: x[0])
            episodes = [x[0] for x in data_for_state]
            values = [x[1] for x in data_for_state]
            
            plt.plot(episodes, values, marker='o', markersize=3, linewidth=1.5, color=colors[i % len(colors)], 
                    label=f"{ordering_names.get(ordering, str(ordering))}")
    
    plt.title(f'Q-Value Evolution Comparison for Different Mission Orderings\n(High Battery, All Sensors Working, Near Dock)')
    plt.xlabel('Episode')
    plt.ylabel('Q-Value')
    plt.grid(False)
    plt.legend(loc='best')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'q_value_ordering_comparison.png'), dpi=300)
    print(f"Q-value ordering comparison saved to {os.path.join(output_dir, 'q_value_ordering_comparison.png')}")
    
def analyze_optimal_policies(q_table, env, output_dir):
    # Define meaningful state combinations to analyze
    key_states = [
        ((0, True, True, 0), "Low Battery, All Sensors Working, Near Dock"),
        ((0, True, True, 2), "Low Battery, All Sensors Working, Far from Dock"),
        ((2, True, True, 2), "High Battery, All Sensors Working, Far from Dock"),
        ((2, False, True, 0), "High Battery, Camera Offline, Near Dock"),
        ((2, True, False, 0), "High Battery, Sonar Offline, Near Dock"),
        ((2, False, False, 0), "High Battery, All Sensors Offline, Near Dock")
    ]
    
    plt.figure(figsize=(15, 10))
    
    for i, (state, title) in enumerate(key_states):
        plt.subplot(3, 2, i+1)
        
        if state in q_table:
            # Get action values for this state
            action_values = [(env.actions[action_idx], value) for action_idx, value in q_table[state].items()]
            
            # Sort by value
            action_values.sort(key=lambda x: x[1], reverse=True)
            
            # Take top 5 or fewer if less are available
            action_values = action_values[:min(5, len(action_values))]
            
            # Plot
            actions = [format_action(action) for action, _ in action_values]
            values = [value for _, value in action_values]
            
            plt.bar(range(len(actions)), values, color='blue', alpha=0.7)
            plt.xticks(range(len(actions)), actions, rotation=45, ha='right', fontsize=8)
            plt.title(title)
            plt.ylabel('Q-Value')
            # plt.grid(axis='y', linestyle='--', alpha=0.3)
        else:
            plt.text(0.5, 0.5, "No data for this state", 
                     horizontalalignment='center',
                     verticalalignment='center',
                     transform=plt.gca().transAxes)
            plt.title(title)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'optimal_policies_by_state.png'), dpi=300)
    print(f"Optimal policies plot saved to {os.path.join(output_dir, 'optimal_policies_by_state.png')}")


def format_action(action):
    action_map = {
        0: "Transit_1",
        1: "Pipeline",
        2: "Transit_2",
        3: "Wreckage", 
        4: "Return"
    }
    return " → ".join([action_map[a] for a in action])


def create_heatmap(q_table, env, output_dir):
    # Prepare data for heatmap
    data = []
    
    # Get all unique states in the Q-table
    states = list(q_table.keys())
    
    for state in states:
        battery, camera, sonar, distance = state
        
        # Create state string
        battery_str = ["Low", "Medium", "High"][battery]
        camera_str = "Cam✓" if camera else "Cam✗"
        sonar_str = "Son✓" if sonar else "Son✗"
        distance_str = ["Near", "Med", "Far"][distance]
        
        state_str = f"{battery_str},{camera_str},{sonar_str},{distance_str}"
        
        # Find best action for this state
        if state in q_table and q_table[state]:
            best_action_idx = max(q_table[state].items(), key=lambda x: x[1])[0]
            best_action = env.actions[best_action_idx]
            best_q = q_table[state][best_action_idx]
            
            # Convert action to string
            action_str = format_action(best_action)
            
            # Add to data
            data.append({
                'State': state_str,
                'Best Action': action_str,
                'Q-Value': best_q
            })
    
    # If no data, return
    if not data:
        print("No data for heatmap")
        return
    
    # Convert to DataFrame
    df = pd.DataFrame(data)
    
    # Create a more compact heatmap for visualization
    # Group by state and get best action and Q-value
    state_action_map = {}
    for _, row in df.iterrows():
        state = row['State']
        action = row['Best Action']
        q_value = row['Q-Value']
        
        if state not in state_action_map or q_value > state_action_map[state][1]:
            state_action_map[state] = (action, q_value)
    
    # Create a new dataframe with just state and best action
    simple_df = pd.DataFrame([
        {'State': state, 'Best Action': action, 'Q-Value': q_value}
        for state, (action, q_value) in state_action_map.items()
    ])
    
    # Create heatmap
    plt.figure(figsize=(15, 10))
    
    # Reshape data for heatmap
    # For this many actions, better to use a table-like visualization
    ax = plt.subplot(1, 1, 1)
    ax.axis('off')
    
    # Sort by battery level, then distance
    simple_df['Battery'] = simple_df['State'].apply(lambda x: x.split(',')[0])
    simple_df['Distance'] = simple_df['State'].apply(lambda x: x.split(',')[3])
    simple_df = simple_df.sort_values(['Battery', 'Distance'])
    
    # Create table
    cell_text = []
    for _, row in simple_df.iterrows():
        cell_text.append([row['State'], row['Best Action'], f"{row['Q-Value']:.2f}"])
    
    table = ax.table(cellText=cell_text,
                    colLabels=['State', 'Best Mission Order', 'Q-Value'],
                    loc='center',
                    cellLoc='center')
    
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.5)
    
    plt.title('Best Mission Order by ROV State')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'q_value_heatmap.png'), dpi=300, bbox_inches='tight')
    print(f"Q-value heatmap saved to {os.path.join(output_dir, 'q_value_heatmap.png')}")


def create_optimal_policy_visualization(q_table, env, output_dir):
    """Create visualization showing optimal policies for different states"""
    
    # Collect optimal policies for each state
    optimal_policies = {}
    best_action_counts = defaultdict(int)
    
    for state, actions in q_table.items():
        if actions:
            best_action_idx = max(actions.items(), key=lambda x: x[1])[0]
            best_action = env.actions[best_action_idx]
            optimal_policies[state] = best_action
            
            # Count occurrences of each best action
            action_str = format_action(best_action)
            best_action_counts[action_str] += 1
    
    # Visualization 1: Pie chart of most common optimal policies
    plt.figure(figsize=(12, 8))
    
    # Sort by count (descending)
    sorted_counts = sorted(best_action_counts.items(), key=lambda x: x[1], reverse=True)
    
    # Take top 5 or fewer
    top_n = min(5, len(sorted_counts))
    labels = [item[0] for item in sorted_counts[:top_n]]
    sizes = [item[1] for item in sorted_counts[:top_n]]
    
    # Add "Others" category if needed
    if len(sorted_counts) > top_n:
        labels.append("Others")
        sizes.append(sum(item[1] for item in sorted_counts[top_n:]))
    
    # Plot
    plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90, shadow=True)
    plt.axis('equal')
    plt.title('Distribution of Optimal Mission Orders Across All States')
    
    plt.savefig(os.path.join(output_dir, 'optimal_policy_distribution.png'), dpi=300)
    print(f"Optimal policy distribution saved to {os.path.join(output_dir, 'optimal_policy_distribution.png')}")
    
    # Visualization 2: Table of optimal policies by state category
    state_categories = [
        ("High Battery, All Sensors", [(2, True, True, d) for d in [0, 1, 2]]),
        ("Low Battery, All Sensors", [(0, True, True, d) for d in [0, 1, 2]]),
        ("Camera Offline", [(b, False, True, d) for b in [0, 1, 2] for d in [0, 1, 2]]),
        ("Sonar Offline", [(b, True, False, d) for b in [0, 1, 2] for d in [0, 1, 2]]),
        ("Both Sensors Offline", [(b, False, False, d) for b in [0, 1, 2] for d in [0, 1, 2]])
    ]
    
    fig, ax = plt.subplots(figsize=(15, 10))
    ax.axis('off')
    
    table_data = []
    for category_name, states in state_categories:
        category_policies = {}
        
        for state in states:
            if state in optimal_policies:
                policy_str = format_action(optimal_policies[state])
                category_policies[policy_str] = category_policies.get(policy_str, 0) + 1
        
        if category_policies:
            most_common = max(category_policies.items(), key=lambda x: x[1])[0]
            total = sum(category_policies.values())
            percentage = category_policies[most_common] / total * 100
            
            table_data.append([category_name, most_common, f"{percentage:.1f}%"])
        else:
            table_data.append([category_name, "N/A", "0%"])
    
    table = ax.table(cellText=table_data,
                    colLabels=['State Category', 'Most Common Optimal Order', 'Percentage'],
                    loc='center',
                    cellLoc='center')
    
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1, 1.8)
    
    plt.title('Most Common Optimal Mission Orders by State Category', fontsize=16, pad=20)
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'optimal_policy_by_category.png'), dpi=300, bbox_inches='tight')
    print(f"Optimal policy by category saved to {os.path.join(output_dir, 'optimal_policy_by_category.png')}")


def visualize_sensor_impact(q_table, env, output_dir):
    """Visualize how sensor availability impacts mission planning"""
    
    # Define test states with different sensor configurations
    test_states = [
        (2, True, True, 1),   # High battery, all sensors working, medium distance
        (2, False, True, 1),  # High battery, camera offline, medium distance
        (2, True, False, 1),  # High battery, sonar offline, medium distance
        (2, False, False, 1), # High battery, both sensors offline, medium distance
    ]
    
    # Get optimal policy for each state
    policies = []
    values = []
    labels = []
    
    for state in test_states:
        if state in q_table and q_table[state]:
            best_action_idx = max(q_table[state].items(), key=lambda x: x[1])[0]
            best_action = env.actions[best_action_idx]
            best_value = max(q_table[state].values())
            
            battery, camera, sonar, distance = state
            state_str = f"Battery: {['Low', 'Med', 'High'][battery]}, "
            state_str += f"Camera: {'ON' if camera else 'OFF'}, "
            state_str += f"Sonar: {'ON' if sonar else 'OFF'}, "
            state_str += f"Dist: {['Near', 'Med', 'Far'][distance]}"
            
            policies.append(best_action)
            values.append(best_value)
            labels.append(state_str)
    
    # Skip if no data
    if not policies:
        print("No data for sensor impact visualization")
        return
    
    # Visualization
    plt.figure(figsize=(14, 10))
    
    # Top: Bar chart of Q-values
    plt.subplot(2, 1, 1)
    colors = ['green', 'orange', 'orange', 'red'][:len(values)]
    bars = plt.bar(range(len(values)), values, color=colors)
    plt.xticks(range(len(labels)), labels, rotation=45, ha='right')
    plt.title('Impact of Sensor Availability on Expected Mission Value')
    plt.ylabel('Optimal Q-Value')
    # plt.grid(axis='y', linestyle='--', alpha=0.3)
    
    # Bottom: Task prioritization visualization
    plt.subplot(2, 1, 2)
    
    for i, (policy, label) in enumerate(zip(policies, labels)):
        y_pos = len(policies) - i - 0.5
        
        # Track position for drawing
        x_pos = 0
        for j, task in enumerate(policy):
            # Set color based on task type
            if task == 0:  # Transit_1
                color = 'lightblue'
                name = 'T1'
            elif task == 1:  # Pipeline
                color = 'royalblue' 
                name = 'P'
            elif task == 2:  # Transit_2
                color = 'skyblue'
                name = 'T2'
            elif task == 3:  # Wreckage
                color = 'navy'
                name = 'W'
            elif task == 4:  # Return
                color = 'red'
                name = 'R'
                
            width = 1.0  # Fixed width for each task
            rect = plt.Rectangle((x_pos, y_pos-0.4), width, 0.8, color=color)
            plt.gca().add_patch(rect)
            plt.text(x_pos + width/2, y_pos, name, ha='center', va='center', color='white', fontweight='bold')
            
            x_pos += width
    
    # Add legend
    legend_elements = [
        mpatches.Patch(color='lightblue', label='Transit_1'),
        mpatches.Patch(color='royalblue', label='Pipeline'),
        mpatches.Patch(color='skyblue', label='Transit_2'),
        mpatches.Patch(color='navy', label='Wreckage'),
        mpatches.Patch(color='red', label='Return')
    ]
    plt.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=5)
    
    plt.yticks(range(len(labels)), labels)
    plt.xlim(0, 5)
    plt.title('Optimal Task Order by Sensor Availability')
    plt.xlabel('Execution Order')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'sensor_impact_analysis.png'), dpi=300)
    print(f"Sensor impact analysis saved to {os.path.join(output_dir, 'sensor_impact_analysis.png')}")


def visualize_battery_impact(q_table, env, output_dir):
    """Visualize how battery level impacts mission planning"""
    
    # Define test states with different battery levels
    test_states = [
        (2, True, True, 1),  # High battery, all sensors
        (1, True, True, 1),  # Medium battery, all sensors
        (0, True, True, 1),  # Low battery, all sensors
    ]
    
    # Get optimal policy for each state
    policies = []
    values = []
    labels = []
    
    for state in test_states:
        if state in q_table and q_table[state]:
            best_action_idx = max(q_table[state].items(), key=lambda x: x[1])[0]
            best_action = env.actions[best_action_idx]
            best_value = max(q_table[state].values())
            
            battery, camera, sonar, distance = state
            state_str = f"Battery: {['Low', 'Med', 'High'][battery]}"
            
            policies.append(best_action)
            values.append(best_value)
            labels.append(state_str)
    
    # Skip if no data
    if not policies:
        print("No data for battery impact visualization")
        return
    
    # Visualization: Mission sequence modification based on battery level
    plt.figure(figsize=(12, 8))
    
    task_labels = {
        0: "Transit_1", 
        1: "Pipeline", 
        2: "Transit_2", 
        3: "Wreckage", 
        4: "Return"
    }
    
    # Create grid for comparison
    colors = {
        0: 'lightblue',  # Transit_1
        1: 'royalblue',  # Pipeline
        2: 'skyblue',    # Transit_2
        3: 'navy',       # Wreckage
        4: 'red'         # Return
    }
    
    for i, (policy, label) in enumerate(zip(policies, labels)):
        for j, task in enumerate(policy):
            plt.scatter(j+1, i+1, color=colors[task], s=500, marker='s')
            plt.text(j+1, i+1, task_labels[task][:1], fontsize=12, 
                    ha='center', va='center', color='white', fontweight='bold')
    
    plt.yticks(range(1, len(labels)+1), labels)
    plt.xticks(range(1, 6), ['1st', '2nd', '3rd', '4th', '5th'])
    plt.grid(False, linestyle='--', alpha=0.3)
    plt.title('Optimal Mission Sequence Based on Battery Level')
    plt.xlabel('Task Order')
    
    # Add arrows showing shifts in task priorities
    if len(policies) > 1:
        for i in range(len(policies)-1):
            top_policy = policies[i]
            bottom_policy = policies[i+1]
            
            # Find shifts in tasks between consecutive battery levels
            for task in range(5):  # 0-4 for all tasks
                if task in top_policy and task in bottom_policy:
                    top_idx = top_policy.index(task)
                    bottom_idx = bottom_policy.index(task)
                    if top_idx != bottom_idx:
                        # Draw arrow showing task shifting positions
                        plt.annotate('', 
                                    xy=(bottom_idx+1, i+2), xytext=(top_idx+1, i+1),
                                    arrowprops=dict(arrowstyle='->', lw=1.5, color='gray', alpha=0.7))
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'battery_impact_analysis.png'), dpi=300)
    print(f"Battery impact analysis saved to {os.path.join(output_dir, 'battery_impact_analysis.png')}")


def visualize_distance_impact(q_table, env, output_dir):
    """Visualize how distance to dock impacts mission planning"""
    
    # Define test states with different distances to dock
    test_states = [
        (1, True, True, 0),  # Medium battery, Near dock
        (1, True, True, 1),  # Medium battery, Medium distance
        (1, True, True, 2),  # Medium battery, Far from dock
    ]
    
    # Get optimal policy for each state and the top 3 alternatives
    fig, axes = plt.subplots(3, 1, figsize=(12, 15))
    
    for i, state in enumerate(test_states):
        if state not in q_table:
            continue
            
        # Get all actions sorted by Q-value
        action_values = [(env.actions[idx], q_val) for idx, q_val in q_table[state].items()]
        action_values.sort(key=lambda x: x[1], reverse=True)
        
        # Take top 4 or fewer
        top_actions = action_values[:min(4, len(action_values))]
        
        # Get distance label
        distance_labels = ["Near Dock", "Medium Distance", "Far from Dock"]
        battery, camera, sonar, distance = state
        distance_label = distance_labels[distance]
        
        # Plot
        ax = axes[i]
        
        # Bar chart of Q-values for alternative actions
        x_vals = range(len(top_actions))
        y_vals = [val for _, val in top_actions]
        bars = ax.bar(x_vals, y_vals, color='skyblue', alpha=0.7)
        
        # Add action labels
        ax.set_xticks(x_vals)
        action_labels = [format_action(action) for action, _ in top_actions]
        ax.set_xticklabels(action_labels, rotation=30, ha='right', fontsize=8)
        
        # Add value labels
        for j, (_, val) in enumerate(top_actions):
            ax.text(j, val + 0.5, f"{val:.1f}", ha='center')
        
        ax.set_title(f"Top Mission Orders - {distance_label}")
        ax.set_ylabel("Q-Value")
        # ax.grid(axis='y', linestyle='--', alpha=0.3)
    
    plt.suptitle("Impact of Distance to Dock on Mission Ordering", fontsize=16)
    plt.tight_layout()
    plt.subplots_adjust(top=0.95)
    
    plt.savefig(os.path.join(output_dir, 'distance_impact_analysis.png'), dpi=300)
    print(f"Distance impact analysis saved to {os.path.join(output_dir, 'distance_impact_analysis.png')}")


def simulate_missions(env, q_table, output_dir, num_simulations=100):
    """Simulate mission outcomes for different initial states"""
    import pandas as pd
    
    # Define initial states to test
    test_states = [
        (2, True, True, 2),   # Optimal: High battery, all sensors, far
        (0, True, True, 2),   # Challenge: Low battery, all sensors, far
        (2, False, True, 2),  # Challenge: High battery, camera offline, far
        (2, True, False, 2),  # Challenge: High battery, sonar offline, far
    ]
    
    state_names = [
        "High Battery, All Sensors",
        "Low Battery, All Sensors",
        "Camera Offline",
        "Sonar Offline"
    ]
    
    # Simulate missions for each state
    all_results = []
    
    for state, state_name in zip(test_states, state_names):
        # Skip if state not in Q-table
        if state not in q_table:
            continue
            
        # Get best action for this state
        best_action_idx = max(q_table[state].items(), key=lambda x: x[1])[0] if q_table[state] else 0
        best_action = env.actions[best_action_idx]
        
        # Run simulation
        env.battery, env.camera, env.sonar, env.distance = state
        _, reward, _ = env.step(best_action_idx)
        
        # Record results
        all_results.append({
            'State': state_name,
            'Mission Order': format_action(best_action),
            'Reward': reward,
            'Success': reward > 0
        })
    
    # Create dataframe
    results_df = pd.DataFrame(all_results)
    
    # Visualization: Results table
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.axis('off')
    
    table = ax.table(
        cellText=results_df.values,
        colLabels=results_df.columns,
        loc='center',
        cellLoc='center'
    )
    
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1, 1.8)
    
    # Color code success
    for i in range(len(results_df)):
        success = results_df.iloc[i]['Success']
        cell = table[i+1, 3]  # Success column
        cell.set_facecolor('lightgreen' if success else 'lightcoral')
    
    plt.title('Mission Performance Simulation Results', fontsize=16, pad=20)
    
    plt.savefig(os.path.join(output_dir, 'mission_simulation_results.png'), dpi=300, bbox_inches='tight')
    print(f"Mission simulation results saved to {os.path.join(output_dir, 'mission_simulation_results.png')}")


def enhanced_learning_metrics(all_rewards, output_dir):
    """Create enhanced learning metrics visualization"""
    
    # Calculate rolling statistics
    window_sizes = [10, 100, 1000]
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Raw rewards with rolling averages
    plt.subplot(2, 1, 1)
    plt.plot(all_rewards, alpha=0.2, color='gray', label='Individual Episodes')
    
    for window in window_sizes:
        rolling_avg = pd.Series(all_rewards).rolling(window=window).mean().values
        plt.plot(rolling_avg, label=f'{window}-Episode Rolling Average')
    
    plt.title('Reward Evolution During Training')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.grid(False)
    plt.legend()
    
    # Plot 2: Performance improvement trends
    plt.subplot(2, 1, 2)
    
    # Calculate percentage of positive rewards over time
    chunk_size = len(all_rewards) // 20  # 20 points on the graph
    positive_pcts = []
    x_vals = []
    
    for i in range(0, len(all_rewards), chunk_size):
        chunk = all_rewards[i:i+chunk_size]
        positive_pct = sum(r > 0 for r in chunk) / len(chunk) * 100
        positive_pcts.append(positive_pct)
        x_vals.append(i + chunk_size//2)
    
    plt.plot(x_vals, positive_pcts, 'go-', label='% Positive Rewards')
    
    # Calculate exponential trend
    import numpy as np
    from scipy.optimize import curve_fit
    
    def exp_func(x, a, b, c):
        return a * np.exp(b * x) + c
    
    try:
        x_array = np.array(x_vals)
        popt, _ = curve_fit(exp_func, x_array, positive_pcts)
        
        x_smooth = np.linspace(x_array[0], x_array[-1], 100)
        y_smooth = exp_func(x_smooth, *popt)
        
        plt.plot(x_smooth, y_smooth, 'r--', label='Performance Trend')
    except:
        # Fallback if curve_fit fails
        pass
    
    plt.title('Learning Performance Trends')
    plt.xlabel('Episode')
    plt.ylabel('% of Positive Rewards')
    plt.grid(False)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'enhanced_learning_metrics.png'), dpi=300)
    print(f"Enhanced learning metrics saved to {os.path.join(output_dir, 'enhanced_learning_metrics.png')}")


def visualize_reward_components(output_dir):
    """Visualize the components of the reward function"""
    
    # Define the reward components
    components = [
        'Base Success', 
        'Pipeline with Camera',
        'Wreckage with Sonar',
        'Transit with High Battery',
        'Low Battery Return',
        'Penalty: No Return',
        'Penalty: Task without Sensor'
    ]
    
    values = [10, 3, 3, 2, 8, -5, -2]
    colors = ['green', 'blue', 'blue', 'lightblue', 'orange', 'red', 'red']
    
    plt.figure(figsize=(12, 8))
    bars = plt.bar(components, values, color=colors)
    
    # Add value labels
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., 
                height + 0.5 if height > 0 else height - 1.5,
                f'{height:+d}', 
                ha='center', va='center')
    
    plt.axhline(y=0, color='black', linestyle='-', alpha=0.3)
    plt.title('Reward Function Components')
    plt.ylabel('Reward Value')
    plt.xticks(rotation=30, ha='right')
    # plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'reward_components.png'), dpi=300)
    print(f"Reward components visualization saved to {os.path.join(output_dir, 'reward_components.png')}")


def visualize_q_table(q_table, env, output_dir):
    """Create a comprehensive visualization of the Q-table"""
    
    # Get statistics about Q-table
    num_states = len(q_table)
    num_entries = sum(len(actions) for actions in q_table.values())
    
    # If empty, skip
    if num_states == 0:
        print("Q-table is empty, skipping visualization")
        return
    
    # Get all q-values
    all_q_values = []
    for state, actions in q_table.items():
        for action_idx, q_value in actions.items():
            all_q_values.append(q_value)
    
    # Calculate statistics
    min_q = min(all_q_values) if all_q_values else 0
    max_q = max(all_q_values) if all_q_values else 0
    avg_q = sum(all_q_values) / len(all_q_values) if all_q_values else 0
    
    # Aggregate state information
    battery_levels = [0, 1, 2]
    distance_levels = [0, 1, 2]
    
    # Create a matrix for visualizing Q-values
    # We'll create a grid of heatmaps, one for each sensor configuration
    sensor_configs = [
        (True, True, "All Sensors Working"),
        (False, True, "Camera Offline"),
        (True, False, "Sonar Offline"),
        (False, False, "Both Sensors Offline")
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()
    
    for i, (camera, sonar, title) in enumerate(sensor_configs):
        # Create a matrix for this sensor configuration
        matrix = np.zeros((len(battery_levels), len(distance_levels)))
        matrix_counts = np.zeros((len(battery_levels), len(distance_levels)))
        
        # Fill the matrix with average Q-values for each state
        for state, actions in q_table.items():
            battery, state_camera, state_sonar, distance = state
            
            # Only include states with this sensor configuration
            if state_camera == camera and state_sonar == sonar:
                # Get max Q-value for this state
                if actions:
                    max_q = max(actions.values())
                    matrix[battery][distance] += max_q
                    matrix_counts[battery][distance] += 1
        
        # Calculate averages
        for b in range(len(battery_levels)):
            for d in range(len(distance_levels)):
                if matrix_counts[b][d] > 0:
                    matrix[b][d] /= matrix_counts[b][d]
        
        # Create heatmap
        ax = axes[i]
        im = ax.imshow(matrix, cmap='viridis', aspect='auto')
        
        # Add labels
        ax.set_xticks(range(len(distance_levels)))
        ax.set_yticks(range(len(battery_levels)))
        ax.set_xticklabels(['Near', 'Medium', 'Far'])
        ax.set_yticklabels(['Low', 'Medium', 'High'])
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Avg Max Q-Value')
        
        # Add values to cells
        for b in range(len(battery_levels)):
            for d in range(len(distance_levels)):
                if matrix_counts[b][d] > 0:
                    ax.text(d, b, f"{matrix[b][d]:.1f}", ha="center", va="center", 
                           color="white" if matrix[b][d] < 10 else "black")
                else:
                    ax.text(d, b, "N/A", ha="center", va="center", color="white")
        
        ax.set_title(f"{title}")
        ax.set_xlabel('Distance to Dock')
        ax.set_ylabel('Battery Level')
    
    plt.suptitle(f'Q-Table Overview: {num_states} States, {num_entries} Entries\nAvg Q-Value: {avg_q:.2f}, Range: [{min_q:.2f}, {max_q:.2f}]', 
                fontsize=16, y=0.99)
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    
    plt.savefig(os.path.join(output_dir, 'q_table_overview.png'), dpi=300)
    print(f"Q-table overview saved to {os.path.join(output_dir, 'q_table_overview.png')}")
    
    # Create a detailed Q-values distribution
    plt.figure(figsize=(10, 6))
    plt.hist(all_q_values, bins=30, alpha=0.7, color='blue')
    plt.axvline(x=avg_q, color='red', linestyle='--', label=f'Average: {avg_q:.2f}')
    
    plt.title('Distribution of Q-Values')
    plt.xlabel('Q-Value')
    plt.ylabel('Count')
    plt.grid(False)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'q_value_distribution.png'), dpi=300)
    print(f"Q-value distribution saved to {os.path.join(output_dir, 'q_value_distribution.png')}")


def main():
    # Create output directory
    output_dir = "rov_rl_results"
    os.makedirs(output_dir, exist_ok=True)
    plt.rcParams['axes.grid'] = False
    
    # Set random seed for reproducibility
    random.seed(42)
    np.random.seed(42)
    
    # Create environment
    env = EnhancedROVEnvironment()
    
    # Training parameters
    num_episodes = 1000  # Increase for more stable results
    learning_rate = 0.1
    discount_factor = 0.9
    exploration_rate = 0.3
    
    print(f"Starting training with {num_episodes} episodes...")
    print(f"Learning rate: {learning_rate}, Discount factor: {discount_factor}, Exploration rate: {exploration_rate}")
    
    start_time = time.time()
    
    # Train Q-table
    q_table, all_rewards, avg_rewards, q_value_evolution = train_q_table(
        env, 
        num_episodes=num_episodes, 
        alpha=learning_rate, 
        gamma=discount_factor, 
        epsilon=exploration_rate
    )
    
    training_time = time.time() - start_time
    print(f"Training completed in {training_time:.2f} seconds")
    
    # Export Q-table
    export_q_table(q_table, env)
    
    # Generate visualizations
    print("Generating visualizations...")
    generate_learning_curve_plot(all_rewards, avg_rewards, output_dir)
    generate_q_value_evolution_plot(q_value_evolution, env, output_dir)
    analyze_optimal_policies(q_table, env, output_dir)
    visualize_q_table(q_table, env, output_dir)  # Added new Q-table visualization
    
    # Generate advanced visualizations
    print("Generating advanced visualizations...")
    create_optimal_policy_visualization(q_table, env, output_dir)
    visualize_sensor_impact(q_table, env, output_dir)
    visualize_battery_impact(q_table, env, output_dir)
    visualize_distance_impact(q_table, env, output_dir)
    simulate_missions(env, q_table, output_dir)
    enhanced_learning_metrics(all_rewards, output_dir)
    visualize_reward_components(output_dir)
    
    # Generate heatmap
    try:
        create_heatmap(q_table, env, output_dir)
    except Exception as e:
        print(f"Error creating heatmap: {e}")
    
    print(f"All results saved to {output_dir} directory")
    print(f"Q-table exported to /tmp/mission_q_table.txt")


if __name__ == "__main__":
    main()