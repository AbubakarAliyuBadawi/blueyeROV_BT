import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import os

def read_qtable(filename):
    """Parse the Q-table file and return as a structured format."""
    data = []
    try:
        with open(filename, 'r') as f:
            version = f.readline().strip()
            if version != 'v1':
                print(f"Unknown format: {version}")
                return []
                
            entries = int(f.readline().strip())
            print(f"Found {entries} entries in Q-table")
            
            for _ in range(entries):
                line = f.readline().strip().split()
                
                # Parse state
                battery = int(line[0])
                camera = bool(int(line[1]))
                sonar = bool(int(line[2]))
                distance = int(line[3])
                
                # Parse order
                order_size = int(line[4])
                order = tuple(int(line[5+i]) for i in range(order_size))
                
                # Parse Q-value
                q_value = float(line[5+order_size])
                
                # Add to data
                data.append({
                    'battery': battery,
                    'camera': camera,
                    'sonar': sonar,
                    'distance': distance,
                    'order': order,
                    'q_value': q_value
                })
        
        return data
    except Exception as e:
        print(f"Error parsing Q-table: {e}")
        return []

def state_to_string(state):
    """Convert a state tuple to a readable string."""
    battery, camera, sonar, distance = state
    battery_str = ["Low", "Medium", "High"][battery]
    camera_str = "Camera ON" if camera else "Camera OFF" 
    sonar_str = "Sonar ON" if sonar else "Sonar OFF"
    distance_str = ["Near", "Medium", "Far"][distance]
    
    return f"{battery_str} Battery, {camera_str}, {sonar_str}, {distance_str} Dock"

def order_to_string(order):
    """Convert an order tuple to a readable string."""
    action_map = {0: "Pipeline", 1: "Wreckage", 2: "Return"}
    return " → ".join([action_map[a] for a in order])

def create_heatmap(q_data, output_dir):
    """Create a heatmap of best actions for each state."""
    # Group by state to find best action for each state
    state_best_action = {}
    for entry in q_data:
        state = (entry['battery'], entry['camera'], entry['sonar'], entry['distance'])
        if state not in state_best_action or entry['q_value'] > state_best_action[state]['q_value']:
            state_best_action[state] = {
                'order': entry['order'],
                'q_value': entry['q_value']
            }
    
    # Create a meaningful representation for the heatmap
    rows = []
    for state, action in state_best_action.items():
        battery, camera, sonar, distance = state
        battery_str = ["Low", "Medium", "High"][battery]
        cam_son_str = f"{'Cam✓' if camera else 'Cam✗'},{'Son✓' if sonar else 'Son✗'}"
        distance_str = ["Near", "Medium", "Far"][distance]
        
        state_str = f"{battery_str},{cam_son_str}"
        action_str = order_to_string(action['order'])
        
        rows.append({
            'Battery-Sensors': state_str,
            'Distance': distance_str,
            'Best Action': action_str,
            'Q-Value': action['q_value']
        })
    
    # Convert to DataFrame
    df = pd.DataFrame(rows)
    
    # Create pivot table for heatmap
    pivot_df = df.pivot_table(
        index='Battery-Sensors', 
        columns='Distance',
        values='Q-Value', 
        aggfunc='mean'
    )
    
    # Create heatmap
    plt.figure(figsize=(12, 8))
    ax = sns.heatmap(pivot_df, annot=True, fmt=".1f", cmap="YlGnBu", 
                    linewidths=.5, cbar_kws={'label': 'Q-Value'})
    plt.title('Q-Values by State (Battery, Sensors, Distance)')
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'q_value_heatmap.png'), dpi=300)
    print(f"Q-value heatmap saved to {os.path.join(output_dir, 'q_value_heatmap.png')}")
    
    # Create a second heatmap for best actions
    action_pivot = df.pivot_table(
        index='Battery-Sensors', 
        columns='Distance',
        values='Best Action', 
        aggfunc=lambda x: ', '.join(x)
    )
    
    plt.figure(figsize=(15, 10))
    
    # Create a custom table-like visualization
    plt.axis('tight')
    plt.axis('off')
    
    cell_text = []
    for idx in action_pivot.index:
        row_data = []
        for col in action_pivot.columns:
            if pd.notna(action_pivot.loc[idx, col]):
                # Find Q-value for this state/action
                q_val = df[
                    (df['Battery-Sensors'] == idx) & 
                    (df['Distance'] == col)
                ]['Q-Value'].values[0]
                
                row_data.append(f"{action_pivot.loc[idx, col]}\n(Q={q_val:.1f})")
            else:
                row_data.append("N/A")
        cell_text.append(row_data)
    
    table = plt.table(
        cellText=cell_text,
        rowLabels=action_pivot.index,
        colLabels=action_pivot.columns,
        cellLoc='center',
        loc='center',
        bbox=[0, 0, 1, 1]
    )
    
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2)
    
    plt.title('Best Mission Order by ROV State', y=1.08, fontsize=14)
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'best_actions.png'), dpi=300)
    print(f"Best actions visualization saved to {os.path.join(output_dir, 'best_actions.png')}")

def create_state_action_bars(q_data, output_dir):
    """Create bar charts showing Q-values for different actions in key states."""
    # Define interesting state combinations to visualize
    key_states = [
        # Low battery scenarios
        ((0, True, True, 0), "Low Battery, All Sensors Working, Near Dock"),
        ((0, True, True, 2), "Low Battery, All Sensors Working, Far from Dock"),
        
        # Sensor failure scenarios
        ((2, False, True, 0), "High Battery, Camera Offline, Near Dock"),
        ((2, True, False, 0), "High Battery, Sonar Offline, Near Dock"),
        
        # High battery, different distances
        ((2, True, True, 0), "High Battery, All Sensors Working, Near Dock"),
        ((2, True, True, 2), "High Battery, All Sensors Working, Far from Dock")
    ]
    
    # Create a figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 18))
    axes = axes.flatten()
    
    for i, ((battery, camera, sonar, distance), title) in enumerate(key_states):
        # Get Q-values for this state
        state_data = [entry for entry in q_data if 
                      entry['battery'] == battery and
                      entry['camera'] == camera and
                      entry['sonar'] == sonar and
                      entry['distance'] == distance]
        
        if state_data:
            # Sort by Q-value
            state_data.sort(key=lambda x: x['q_value'], reverse=True)
            
            # Extract action strings and Q-values
            actions = [order_to_string(entry['order']) for entry in state_data]
            values = [entry['q_value'] for entry in state_data]
            
            # Create color map based on Q-values
            colors = plt.cm.YlGnBu(np.linspace(0.2, 0.8, len(values)))
            
            # Plot
            axes[i].bar(range(len(actions)), values, color=colors)
            axes[i].set_xticks(range(len(actions)))
            axes[i].set_xticklabels(actions, rotation=45, ha='right')
            axes[i].set_title(title)
            axes[i].set_ylabel('Q-Value')
            axes[i].grid(axis='y', linestyle='--', alpha=0.3)
            
            # Add value labels
            for j, v in enumerate(values):
                axes[i].text(j, v + 0.5, f"{v:.1f}", ha='center', va='bottom', fontweight='bold')
        else:
            axes[i].text(0.5, 0.5, "No data for this state", 
                        horizontalalignment='center',
                        verticalalignment='center',
                        transform=axes[i].transAxes)
            axes[i].set_title(title)
    
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'state_action_bars.png'), dpi=300)
    print(f"State-action bar charts saved to {os.path.join(output_dir, 'state_action_bars.png')}")

def create_sensor_impact_visualization(q_data, output_dir):
    """Create visualization showing impact of sensor availability on mission planning."""
    # Define scenarios to compare
    scenarios = [
        ((2, True, True, 0), "All Sensors Working"),
        ((2, False, True, 0), "Camera Offline"),
        ((2, True, False, 0), "Sonar Offline"),
        ((2, False, False, 0), "Both Sensors Offline")
    ]
    
    # For each scenario, find the best mission order and its Q-value
    scenario_results = []
    
    for (battery, camera, sonar, distance), label in scenarios:
        # Get entries for this state
        state_data = [entry for entry in q_data if 
                      entry['battery'] == battery and
                      entry['camera'] == camera and
                      entry['sonar'] == sonar and
                      entry['distance'] == distance]
        
        if state_data:
            # Find best action
            best_entry = max(state_data, key=lambda x: x['q_value'])
            
            # Calculate "success probability" (normalized Q-value)
            q_value = best_entry['q_value']
            
            scenario_results.append({
                'Scenario': label,
                'Best Action': order_to_string(best_entry['order']),
                'Q-Value': q_value
            })
    
    # Create the visualization
    if scenario_results:
        df = pd.DataFrame(scenario_results)
        
        plt.figure(figsize=(12, 8))
        
        # Use different colors based on scenario
        colors = ['green', 'orange', 'orange', 'red'][:len(df)]
        
        # Create bar chart
        bars = plt.bar(df['Scenario'], df['Q-Value'], color=colors)
        
        # Add value labels
        for bar, value in zip(bars, df['Q-Value']):
            plt.text(bar.get_x() + bar.get_width()/2., value + 0.5,
                    f'Q={value:.1f}', ha='center', va='bottom')
        
        # Add best actions below bars
        for i, (scenario, action) in enumerate(zip(df['Scenario'], df['Best Action'])):
            plt.text(i, -3, f"Best: {action}", ha='center', va='top', 
                    rotation=0, fontsize=10, color='blue')
        
        plt.title('Impact of Sensor Availability on Mission Planning')
        plt.ylabel('Q-Value')
        plt.ylim(min(df['Q-Value'])-5, max(df['Q-Value'])+3)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        plt.tight_layout()
        
        plt.savefig(os.path.join(output_dir, 'sensor_impact.png'), dpi=300)
        print(f"Sensor impact visualization saved to {os.path.join(output_dir, 'sensor_impact.png')}")

def create_battery_distance_grid(q_data, output_dir):
    """Create a grid visualization showing optimal mission order for different battery/distance combinations."""
    battery_levels = [0, 1, 2]  # Low, Medium, High
    distance_levels = [0, 1, 2]  # Near, Medium, Far
    
    # Create a 3x3 grid for different battery/distance combinations
    fig, axes = plt.subplots(3, 3, figsize=(15, 15))
    
    # For each battery/distance combination
    for i, battery in enumerate(battery_levels):
        for j, distance in enumerate(distance_levels):
            # Get data for this battery/distance with all sensors working
            state_data = [entry for entry in q_data if 
                         entry['battery'] == battery and
                         entry['camera'] == True and
                         entry['sonar'] == True and
                         entry['distance'] == distance]
            
            if state_data:
                # Find best action
                best_entry = max(state_data, key=lambda x: x['q_value'])
                
                # Create a formatted mini-table
                ax = axes[i][j]
                ax.axis('off')
                
                battery_str = ["Low", "Medium", "High"][battery]
                distance_str = ["Near", "Medium", "Far"][distance]
                
                # Format text for display
                text = (f"Best Mission Order:\n{order_to_string(best_entry['order'])}\n\n"
                       f"Q-Value: {best_entry['q_value']:.1f}")
                
                # Add text to the plot
                ax.text(0.5, 0.5, text, ha='center', va='center', 
                       bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="gray", alpha=0.8))
                
                # Add title
                ax.set_title(f"Battery: {battery_str}, Distance: {distance_str}")
            else:
                # No data for this state
                ax = axes[i][j]
                ax.axis('off')
                ax.text(0.5, 0.5, "No data available", ha='center', va='center')
                ax.set_title(f"Battery: {battery_str}, Distance: {distance_str}")
    
    plt.tight_layout()
    
    plt.savefig(os.path.join(output_dir, 'battery_distance_grid.png'), dpi=300)
    print(f"Battery/distance grid saved to {os.path.join(output_dir, 'battery_distance_grid.png')}")

def main():
    # Create output directory
    output_dir = "rov_ql_visualizations"
    os.makedirs(output_dir, exist_ok=True)
    
    # Read the Q-table
    q_data = read_qtable("/tmp/mission_q_table.txt")
    
    if not q_data:
        print("Failed to read Q-table or Q-table is empty.")
        return
    
    # Create visualizations
    create_heatmap(q_data, output_dir)
    create_state_action_bars(q_data, output_dir)
    create_sensor_impact_visualization(q_data, output_dir)
    create_battery_distance_grid(q_data, output_dir)
    
    print(f"All visualizations saved to {output_dir} directory")
    
    # Additional analysis
    total_states = len(set((entry['battery'], entry['camera'], entry['sonar'], entry['distance']) 
                           for entry in q_data))
    print(f"Found data for {total_states} unique states")
    
    # Find the best action overall for each state
    state_best_action = {}
    for entry in q_data:
        state = (entry['battery'], entry['camera'], entry['sonar'], entry['distance'])
        if state not in state_best_action or entry['q_value'] > state_best_action[state]['q_value']:
            state_best_action[state] = {
                'order': entry['order'],
                'q_value': entry['q_value']
            }
    
    # Print the top 5 highest Q-values
    print("\nTop 5 highest Q-values:")
    sorted_states = sorted(state_best_action.items(), key=lambda x: x[1]['q_value'], reverse=True)
    for i, (state, action) in enumerate(sorted_states[:5]):
        state_str = state_to_string(state)
        action_str = order_to_string(action['order'])
        print(f"{i+1}. {state_str} → {action_str} (Q={action['q_value']:.1f})")
    
    # Print a summary for use in the thesis
    print("\nSummary for Thesis:")
    print(f"The Q-table contains {len(q_data)} total entries representing {total_states} unique states.")
    
    # Analyze best mission orders
    best_orders = {}
    for state, action in state_best_action.items():
        order = action['order']
        if order not in best_orders:
            best_orders[order] = 0
        best_orders[order] += 1
    
    print("\nMost common optimal mission orders:")
    sorted_orders = sorted(best_orders.items(), key=lambda x: x[1], reverse=True)
    for order, count in sorted_orders[:3]:
        print(f"- {order_to_string(order)}: optimal for {count} states ({count/total_states*100:.1f}% of states)")

if __name__ == "__main__":
    main()