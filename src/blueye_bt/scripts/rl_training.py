import numpy as np
import random
import matplotlib.pyplot as plt
from collections import defaultdict
import os
import time

class SimplifiedROVEnvironment:
    def __init__(self):
        # State space definitions
        self.battery_levels = [0, 1, 2]  # Low, Medium, High
        self.camera_states = [False, True]  # Not working, Working
        self.sonar_states = [False, True]  # Not working, Working
        self.distance_levels = [0, 1, 2]  # Near, Medium, Far
        
        # Action space - permutations of [0, 1, 2] (Pipeline, Wreckage, Return)
        self.actions = [
            (0, 1, 2),  # Pipeline → Wreckage → Return
            (0, 2, 1),  # Pipeline → Return → Wreckage
            (1, 0, 2),  # Wreckage → Pipeline → Return
            (1, 2, 0),  # Wreckage → Return → Pipeline
            (2, 0, 1),  # Return → Pipeline → Wreckage
            (2, 1, 0)   # Return → Wreckage → Pipeline
        ]
        
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
        for task in action:
            # Pipeline inspection
            if task == 0:
                if self.camera:
                    reward += 5  # Success with camera
                    tasks_completed.append(task)
                else:
                    reward -= 2  # Failure without camera
            
            # Wreckage inspection
            elif task == 1:
                if self.sonar:
                    reward += 5  # Success with sonar
                    tasks_completed.append(task)
                else:
                    reward -= 2  # Failure without sonar
            
            # Return to dock
            elif task == 2:
                tasks_completed.append(task)
                # Higher reward for returning when battery is low
                if self.battery == 0:
                    reward += 8
                elif self.battery == 1:
                    reward += 3
                else:
                    reward += 1
        
        # Penalty for not returning to dock when battery is low
        if self.battery == 0 and action[-1] != 2:
            reward -= 5
        
        # Extra reward for using appropriate sensors for specific tasks
        if 0 in tasks_completed and self.camera:
            reward += 3  # Using camera for pipeline inspection
        if 1 in tasks_completed and self.sonar:
            reward += 3  # Using sonar for wreckage inspection
        
        # Efficiency bonus for completing missions with low battery
        if len(tasks_completed) > 1 and self.battery == 0:
            reward += 5
        
        # Generate next state (for episodic training, we'll reset)
        next_state = self.reset()
        
        return next_state, reward, True  # terminal=True for episodic


def train_q_table(env, num_episodes=10000, alpha=0.1, gamma=0.9, epsilon=0.3):
    # Initialize Q-table
    q_table = defaultdict(lambda: defaultdict(float))
    
    # For plotting metrics
    all_rewards = []
    avg_rewards = []
    q_value_evolution = {}  # Track specific state-action pair over time
    
    # Choose specific state-action pairs to track for different scenarios
    track_states = [
        (2, True, True, 0),   # High battery, all sensors working, near dock
        (0, True, True, 0),   # Low battery, all sensors working, near dock
        (2, False, True, 2),  # High battery, camera offline, far dock
    ]
    
    track_action_idx = 2  # Wreckage → Pipeline → Return
    
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
        
        # Track specific state-action pairs
        for track_state in track_states:
            if state == track_state and action_idx == track_action_idx:
                key = (episode, track_state)
                q_value_evolution[key] = new_q
        
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
                f.write(f"{q_value:.2f}\n")
    
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
    plt.grid(True, alpha=0.3)
    plt.legend(['Episode Reward', 'Average Reward (100 episodes)'])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'learning_curve.png'), dpi=300)
    print(f"Learning curve saved to {os.path.join(output_dir, 'learning_curve.png')}")


def generate_q_value_evolution_plot(q_value_evolution, env, output_dir):
    plt.figure(figsize=(10, 6))
    
    # Group by state
    state_data = defaultdict(list)
    for (episode, state), value in q_value_evolution.items():
        state_data[state].append((episode, value))
    
    # Plot each state's evolution
    colors = ['blue', 'green', 'red', 'purple', 'orange']
    labels = []
    
    for i, (state, data) in enumerate(state_data.items()):
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
    
    plt.title('Q-Value Evolution for Wreckage → Pipeline → Return Mission Order')
    plt.xlabel('Episode')
    plt.ylabel('Q-Value')
    plt.grid(True, alpha=0.3)
    plt.legend(labels)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'q_value_evolution.png'), dpi=300)
    print(f"Q-value evolution plot saved to {os.path.join(output_dir, 'q_value_evolution.png')}")


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
            
            # Plot
            actions = [format_action(action) for action, _ in action_values]
            values = [value for _, value in action_values]
            
            plt.bar(range(len(actions)), values, color='blue', alpha=0.7)
            plt.xticks(range(len(actions)), actions, rotation=45, ha='right')
            plt.title(title)
            plt.ylabel('Q-Value')
            plt.grid(axis='y', linestyle='--', alpha=0.3)
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
    action_map = {0: "Pipeline", 1: "Wreckage", 2: "Return"}
    return " → ".join([action_map[a] for a in action])


def create_heatmap(q_table, env, output_dir):
    import seaborn as sns
    
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
        if state in q_table:
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
    
    # Convert to DataFrame
    import pandas as pd
    df = pd.DataFrame(data)
    
    # Create heatmap
    plt.figure(figsize=(12, 10))
    pivot_df = df.pivot_table(index='State', columns='Best Action', values='Q-Value')
    
    # Fill missing values with NaN
    pivot_df = pivot_df.fillna(-100)  # -100 will be a distinct color
    
    # Set up color map that shows missing values distinctly
    cmap = sns.diverging_palette(20, 220, as_cmap=True)
    cmap.set_bad('white')
    
    # Create heatmap
    ax = sns.heatmap(pivot_df, annot=True, cmap=cmap, linewidths=.5, cbar_kws={'label': 'Q-Value'})
    plt.title('Best Mission Order by ROV State')
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'q_value_heatmap.png'), dpi=300)
    print(f"Q-value heatmap saved to {os.path.join(output_dir, 'q_value_heatmap.png')}")


def main():
    # Create output directory
    output_dir = "rov_rl_results"
    os.makedirs(output_dir, exist_ok=True)
    
    # Set random seed for reproducibility
    random.seed(42)
    np.random.seed(42)
    
    # Create environment
    env = SimplifiedROVEnvironment()
    
    # Training parameters
    num_episodes = 50000  # Increase for more stable results
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
    
    # Generate heatmap
    try:
        import seaborn as sns
        import pandas as pd
        create_heatmap(q_table, env, output_dir)
    except ImportError:
        print("Seaborn or pandas not installed. Skipping heatmap generation.")
    
    print(f"All results saved to {output_dir} directory")
    print(f"Q-table exported to /tmp/mission_q_table.txt")


if __name__ == "__main__":
    main()