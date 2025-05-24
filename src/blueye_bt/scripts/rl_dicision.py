import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import networkx as nx
from matplotlib.patches import Rectangle, FancyArrowPatch
import seaborn as sns
from matplotlib.colors import LinearSegmentedColormap

def visualize_mission_decision(output_file="mission_decision_visualization.png"):
    """Create a visualization of the RL mission decision process"""
    
    # Define the mission data from the logs
    current_state = {
        "battery": "High",  # Level 2
        "camera": "Working",
        "sonar": "Working",
        "distance": "Near"  # 1.55m
    }
    
    chosen_mission = "Transit_1 → Pipeline Inspection → Transit_2 → Wreckage Inspection → Homing"
    chosen_value = 12.8642
    
    alternative_missions = [
        {"order": "Transit_2 → Transit_1 → Wreckage Inspection → Pipeline Inspection → Homing", "value": 2.3}
    ]
    
    # Create figure with multiple plots
    fig = plt.figure(figsize=(18, 14))
    fig.suptitle("Reinforcement Learning Mission Decision Analysis", fontsize=20, y=0.98)
    
    # Plot 1: Decision Tree Visualization
    ax1 = plt.subplot2grid((2, 2), (0, 0), colspan=2)
    create_decision_tree(ax1, current_state, chosen_mission, chosen_value, alternative_missions)
    
    # Plot 2: State Factor Influence
    ax2 = plt.subplot2grid((2, 2), (1, 0))
    create_state_influence_diagram(ax2, current_state)
    
    # Plot 3: Mission Sequence Timeline
    ax3 = plt.subplot2grid((2, 2), (1, 1))
    create_mission_sequence(ax3, chosen_mission)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Mission decision visualization saved to {output_file}")
    plt.close()

def create_decision_tree(ax, current_state, chosen_mission, chosen_value, alternative_missions):
    """Create a decision tree visualization showing chosen and alternative paths"""
    
    # Create a directed graph
    G = nx.DiGraph()
    
    # Add nodes
    G.add_node("Current State", pos=(0, 0))
    
    # Add leaf nodes
    G.add_node(f"Chosen:\n{chosen_mission}", pos=(1, 0.5))
    
    # Add alternative nodes
    for i, alt in enumerate(alternative_missions):
        G.add_node(f"Alternative {i+1}:\n{alt['order']}", pos=(1, -0.5*(i+1)))
    
    # Add edges
    G.add_edge("Current State", f"Chosen:\n{chosen_mission}", weight=chosen_value)
    
    for i, alt in enumerate(alternative_missions):
        G.add_edge("Current State", f"Alternative {i+1}:\n{alt['order']}", weight=alt['value'])
    
    # Get positions
    pos = nx.get_node_attributes(G, 'pos')
    
    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=3000, node_color='lightblue', node_shape='o', alpha=0.8, ax=ax)
    nx.draw_networkx_nodes(G, pos, nodelist=[f"Chosen:\n{chosen_mission}"], node_size=3000, node_color='lightgreen', alpha=0.8, ax=ax)
    
    # Draw edges with weights
    for u, v, data in G.edges(data=True):
        weight = data.get('weight', 1.0)
        # Draw the edge
        edge = FancyArrowPatch(pos[u], pos[v], arrowstyle='->', connectionstyle='arc3,rad=0.1', 
                              mutation_scale=20, lw=2, alpha=0.7, 
                              color='green' if v.startswith('Chosen') else 'gray')
        ax.add_patch(edge)
        
        # Add the weight as a label
        midpoint = ((pos[u][0] + pos[v][0])/2, (pos[u][1] + pos[v][1])/2)
        ax.text(midpoint[0], midpoint[1]+0.1, f"Q={weight:.1f}", fontsize=10, 
               ha='center', va='center', bbox=dict(boxstyle="round", fc="white", alpha=0.7))
    
    # Draw node labels
    nx.draw_networkx_labels(G, pos, font_size=10, ax=ax)
    
    # Add state information
    state_text = f"State:\n• Battery: {current_state['battery']}\n• Camera: {current_state['camera']}\n• Sonar: {current_state['sonar']}\n• Distance: {current_state['distance']}"
    ax.text(-0.1, -0.1, state_text, fontsize=10, bbox=dict(boxstyle="round", fc="lightgray", alpha=0.7))
    
    # Configure plot
    ax.set_title("Mission Decision Tree", fontsize=16)
    ax.axis('off')

def create_state_influence_diagram(ax, current_state):
    """Create a diagram showing how each state factor influences mission decisions"""
    
    # Define influence weights (hypothetical based on the Q-values)
    influences = {
        "battery": {
            "High": {"Transit_1 → Pipeline": 0.9, "Pipeline → Transit_2": 0.7, "Transit_2 → Wreckage": 0.8, "Wreckage → Homing": 0.6},
            "Medium": {"Transit_1 → Pipeline": 0.7, "Pipeline → Transit_2": 0.6, "Transit_2 → Wreckage": 0.7, "Wreckage → Homing": 0.8},
            "Low": {"Transit_1 → Pipeline": 0.3, "Pipeline → Transit_2": 0.2, "Transit_2 → Wreckage": 0.3, "Wreckage → Homing": 0.9}
        },
        "camera": {
            "Working": {"Transit_1 → Pipeline": 0.9, "Pipeline → Transit_2": 0.8, "Transit_2 → Wreckage": 0.7, "Wreckage → Homing": 0.6},
            "Offline": {"Transit_1 → Pipeline": 0.2, "Pipeline → Transit_2": 0.3, "Transit_2 → Wreckage": 0.8, "Wreckage → Homing": 0.7}
        },
        "sonar": {
            "Working": {"Transit_1 → Pipeline": 0.7, "Pipeline → Transit_2": 0.7, "Transit_2 → Wreckage": 0.9, "Wreckage → Homing": 0.7},
            "Offline": {"Transit_1 → Pipeline": 0.6, "Pipeline → Transit_2": 0.6, "Transit_2 → Wreckage": 0.3, "Wreckage → Homing": 0.6}
        },
        "distance": {
            "Near": {"Transit_1 → Pipeline": 0.8, "Pipeline → Transit_2": 0.7, "Transit_2 → Wreckage": 0.7, "Wreckage → Homing": 0.9},
            "Medium": {"Transit_1 → Pipeline": 0.7, "Pipeline → Transit_2": 0.7, "Transit_2 → Wreckage": 0.7, "Wreckage → Homing": 0.7},
            "Far": {"Transit_1 → Pipeline": 0.6, "Pipeline → Transit_2": 0.7, "Transit_2 → Wreckage": 0.7, "Wreckage → Homing": 0.5}
        }
    }
    
    # Create the heatmap data
    transitions = ["Transit_1 → Pipeline", "Pipeline → Transit_2", "Transit_2 → Wreckage", "Wreckage → Homing"]
    factors = ["Battery", "Camera", "Sonar", "Distance"]
    
    data = np.zeros((4, 4))
    
    # Fill the data matrix
    for i, factor in enumerate(factors):
        factor_lower = factor.lower()
        for j, transition in enumerate(transitions):
            data[i, j] = influences[factor_lower][current_state[factor_lower]][transition]
    
    # Create a custom colormap (green for positive influence)
    cmap = LinearSegmentedColormap.from_list("gr", ["white", "darkgreen"])
    
    # Create heatmap
    sns.heatmap(data, annot=True, cmap=cmap, vmin=0, vmax=1, linewidths=.5, 
               xticklabels=transitions, yticklabels=factors, ax=ax)
    
    ax.set_title("State Factor Influence on Mission Transitions", fontsize=16)
    ax.set_ylabel("State Factors")
    ax.set_xlabel("Mission Transitions")
    
    # Rotate x-axis labels for better readability
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")

def create_mission_sequence(ax, chosen_mission):
    """Create a timeline visualization of the chosen mission sequence"""
    
    # Parse mission sequence
    mission_steps = chosen_mission.split(" → ")
    
    # Define colors and estimated durations (in minutes)
    colors = {
        "Transit_1": "lightblue",
        "Pipeline Inspection": "royalblue",
        "Transit_2": "skyblue",
        "Wreckage Inspection": "navy",
        "Homing": "red"
    }
    
    durations = {
        "Transit_1": 10,
        "Pipeline Inspection": 30,
        "Transit_2": 15,
        "Wreckage Inspection": 25,
        "Homing": 20
    }
    
    # Calculate cumulative times
    cumulative_time = 0
    start_times = []
    for step in mission_steps:
        start_times.append(cumulative_time)
        cumulative_time += durations[step]
    
    # Plot timeline
    for i, step in enumerate(mission_steps):
        width = durations[step]
        ax.barh(0, width, left=start_times[i], height=0.5, color=colors[step], alpha=0.8)
        
        # Add task label
        ax.text(start_times[i] + width/2, 0, step, ha='center', va='center', fontsize=10, 
               color='white', fontweight='bold')
    
    # Set y-axis limits and hide y-axis
    ax.set_ylim(-0.5, 0.5)
    ax.set_yticks([])
    
    # Set x-axis as time
    ax.set_xlabel("Mission Duration (minutes)")
    
    # Add grid for time reference
    ax.grid(axis='x', linestyle='--', alpha=0.3)
    
    # Calculate total duration
    total_duration = sum(durations[step] for step in mission_steps)
    
    ax.set_title(f"Selected Mission Sequence Timeline\nTotal Duration: {total_duration} minutes", fontsize=16)

if __name__ == "__main__":
    visualize_mission_decision()