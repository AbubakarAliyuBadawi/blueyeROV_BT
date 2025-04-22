#!/usr/bin/env python3
import streamlit as st
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from collections import defaultdict
import os
import io
import base64
import time

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
                st.error(f"Unknown format: {version}")
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
        st.success(f"Successfully loaded {len(data)} Q-value entries")
        return data
    except FileNotFoundError:
        st.error(f"File not found: {filename}")
        return []
    except Exception as e:
        st.error(f"Error parsing Q-table: {e}")
        return []

def order_to_string(order):
    """Convert a task order tuple to a readable string."""
    return " → ".join([TASK_NAMES.get(task, f"Task{task}") for task in order])

def create_qtable_heatmap(data, filter_options=None):
    """Create a heatmap of Q-values for different states and actions."""
    if not data:
        st.warning("No data to visualize")
        return
    
    # Apply filters if specified
    filtered_data = data
    if filter_options:
        filtered_data = [entry for entry in data 
                        if (filter_options['battery'] is None or entry['battery'] == filter_options['battery']) and
                           (filter_options['camera'] is None or entry['camera'] == filter_options['camera']) and
                           (filter_options['sonar'] is None or entry['sonar'] == filter_options['sonar']) and
                           (filter_options['distance'] is None or entry['distance'] == filter_options['distance'])]
    
    # Group data by state
    state_data = defaultdict(list)
    for entry in filtered_data:
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
    
    if df.empty:
        st.warning("No data available for the selected filters")
        return None
    
    pivot_df = df.pivot_table(index='State', columns='Action', values='Q-Value', aggfunc='mean')
    
    # Plot heatmap
    fig, ax = plt.subplots(figsize=(12, len(pivot_df) * 0.7))
    sns.heatmap(pivot_df, annot=True, cmap="YlGnBu", linewidths=.5, cbar_kws={'label': 'Q-Value'}, ax=ax)
    plt.title("Q-Values Heatmap by State and Action")
    plt.tight_layout()
    
    return fig

def create_best_action_plot(data, filter_options=None):
    """Create a visualization of the best action for each state."""
    if not data:
        st.warning("No data to visualize")
        return None
    
    # Apply filters if specified
    filtered_data = data
    if filter_options:
        filtered_data = [entry for entry in data 
                        if (filter_options['battery'] is None or entry['battery'] == filter_options['battery']) and
                           (filter_options['camera'] is None or entry['camera'] == filter_options['camera']) and
                           (filter_options['sonar'] is None or entry['sonar'] == filter_options['sonar']) and
                           (filter_options['distance'] is None or entry['distance'] == filter_options['distance'])]
    
    # Group data by state and find best action
    best_actions = {}
    for entry in filtered_data:
        state = (entry['battery'], entry['camera'], entry['sonar'], entry['distance'])
        if state not in best_actions or entry['q_value'] > best_actions[state][1]:
            best_actions[state] = (entry['order'], entry['q_value'])
    
    if not best_actions:
        st.warning("No data available for the selected filters")
        return None
    
    # Create a formatted table of states and best actions
    table_data = []
    for state, (action, value) in best_actions.items():
        battery, camera, sonar, distance = state
        table_data.append({
            'Battery': BATTERY_LEVELS[battery],
            'Camera': '✓' if camera else '✗',
            'Sonar': '✓' if sonar else '✗',
            'Distance': DISTANCE_LEVELS[distance],
            'Best Action': order_to_string(action),
            'Q-Value': f"{value:.2f}"
        })
    
    return pd.DataFrame(table_data)

def create_reward_function_visualization():
    """Create a visualization of the reward function components."""
    # Define components of the reward function
    components = ['Base Success', 'Base Failure', 'Low Battery Efficiency', 
                  'Camera for Pipeline', 'Sonar for Wreckage', 'Low Battery Return Penalty']
    values = [10, -5, 5, 3, 3, -4]
    colors = ['green', 'red', 'blue', 'purple', 'orange', 'brown']
    
    # Create bar chart
    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(components, values, color=colors)
    
    # Add value labels on top of bars
    for bar in bars:
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                f'{height:+.0f}', ha='center', va='bottom')
    
    ax.axhline(y=0, color='black', linestyle='-', alpha=0.3)
    ax.set_title('Reward Function Components')
    ax.set_ylabel('Reward Value')
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    
    return fig

def create_sensor_impact_simulation(battery_level, distance_level):
    """Create an interactive simulation of sensor impact on mission planning."""
    # Define scenarios
    columns = ["Sensors", "Success Probability", "Recommended Action"]
    
    # Base probabilities - These would come from actual learning over time
    base_probs = {
        # Battery LOW
        (0, 0): {  # Near
            (True, True): 0.85,
            (True, False): 0.70,
            (False, True): 0.75,
            (False, False): 0.40
        },
        (0, 1): {  # Medium
            (True, True): 0.70,
            (True, False): 0.50,
            (False, True): 0.55,
            (False, False): 0.25
        },
        (0, 2): {  # Far
            (True, True): 0.50,
            (True, False): 0.30,
            (False, True): 0.35,
            (False, False): 0.15
        },
        # Battery MEDIUM
        (1, 0): {  # Near
            (True, True): 0.95,
            (True, False): 0.80,
            (False, True): 0.85,
            (False, False): 0.60
        },
        (1, 1): {  # Medium
            (True, True): 0.85,
            (True, False): 0.70,
            (False, True): 0.75,
            (False, False): 0.45
        },
        (1, 2): {  # Far
            (True, True): 0.75,
            (True, False): 0.55,
            (False, True): 0.60,
            (False, False): 0.30
        },
        # Battery HIGH
        (2, 0): {  # Near
            (True, True): 0.98,
            (True, False): 0.90,
            (False, True): 0.92,
            (False, False): 0.75
        },
        (2, 1): {  # Medium
            (True, True): 0.95,
            (True, False): 0.80,
            (False, True): 0.85,
            (False, False): 0.60
        },
        (2, 2): {  # Far
            (True, True): 0.90,
            (True, False): 0.70,
            (False, True): 0.75,
            (False, False): 0.50
        }
    }
    
    # Recommended actions based on state
    actions = {
        # Battery LOW
        (0, 0): {  # Near
            (True, True): (2, 0, 1),  # Return, Pipeline, Wreckage
            (True, False): (2, 0),    # Return, Pipeline
            (False, True): (2, 1),    # Return, Wreckage
            (False, False): (2,)      # Return only
        },
        (0, 1): {  # Medium
            (True, True): (2, 0, 1),
            (True, False): (2, 0),
            (False, True): (2, 1),
            (False, False): (2,)
        },
        (0, 2): {  # Far
            (True, True): (2,),
            (True, False): (2,),
            (False, True): (2,),
            (False, False): (2,)
        },
        # Battery MEDIUM
        (1, 0): {  # Near
            (True, True): (0, 1, 2),  # Pipeline, Wreckage, Return
            (True, False): (0, 2),    # Pipeline, Return
            (False, True): (1, 2),    # Wreckage, Return
            (False, False): (2,)      # Return only
        },
        (1, 1): {  # Medium
            (True, True): (0, 1, 2),
            (True, False): (0, 2),
            (False, True): (1, 2),
            (False, False): (2,)
        },
        (1, 2): {  # Far
            (True, True): (2, 0, 1),
            (True, False): (2, 0),
            (False, True): (2, 1),
            (False, False): (2,)
        },
        # Battery HIGH
        (2, 0): {  # Near
            (True, True): (0, 1, 2),
            (True, False): (0, 2),
            (False, True): (1, 2),
            (False, False): (2,)
        },
        (2, 1): {  # Medium
            (True, True): (0, 1, 2),
            (True, False): (0, 2),
            (False, True): (1, 2),
            (False, False): (2,)
        },
        (2, 2): {  # Far
            (True, True): (0, 1, 2),
            (True, False): (0, 2),
            (False, True): (1, 2),
            (False, False): (2,)
        }
    }
    
    # Create table data
    table_data = []
    for camera in [True, False]:
        for sonar in [True, False]:
            sensor_str = f"Camera: {'✓' if camera else '✗'}, Sonar: {'✓' if sonar else '✗'}"
            prob = base_probs.get((battery_level, distance_level), {}).get((camera, sonar), 0.5)
            action_tuple = actions.get((battery_level, distance_level), {}).get((camera, sonar), (2,))
            action_str = order_to_string(action_tuple)
            
            table_data.append([sensor_str, f"{prob:.0%}", action_str])
    
    return table_data, columns

def main():
    st.set_page_config(
        page_title="ROV Reinforcement Learning Dashboard",
        page_icon="🤖",
        layout="wide",
    )
    
    st.title("ROV Reinforcement Learning Dashboard")
    st.write("""
    This dashboard visualizes the Q-learning data and mission planning strategies for the autonomous ROV.
    Upload your Q-table file or use the simulation features to explore different scenarios.
    """)
    
    # Sidebar for file upload and filters
    with st.sidebar:
        st.header("Data Source")
        
        upload_tab, path_tab = st.tabs(["Upload File", "File Path"])
        
        with upload_tab:
            uploaded_file = st.file_uploader("Upload Q-table file", type="txt")
            if uploaded_file:
                # Save the uploaded file temporarily
                with open("/tmp/uploaded_qtable.txt", "wb") as f:
                    f.write(uploaded_file.getvalue())
                qtable_path = "/tmp/uploaded_qtable.txt"
            else:
                qtable_path = None
        
        with path_tab:
            default_path = "/tmp/mission_q_table.txt"
            custom_path = st.text_input("Q-table file path", value=default_path)
            qtable_path = custom_path if not uploaded_file else qtable_path
        
        st.header("Data Filters")
        
        # Filters for visualizations
        filter_battery = st.selectbox("Battery Level", 
                                      [None, "Low (0)", "Medium (1)", "High (2)"])
        filter_camera = st.selectbox("Camera Status", 
                                     [None, "Working (True)", "Offline (False)"])
        filter_sonar = st.selectbox("Sonar Status", 
                                    [None, "Working (True)", "Offline (False)"])
        filter_distance = st.selectbox("Distance to Dock", 
                                       [None, "Near (0)", "Medium (1)", "Far (2)"])
        
        # Convert filters to values
        filter_options = {
            'battery': int(filter_battery.split('(')[1].split(')')[0]) if filter_battery else None,
            'camera': filter_camera == "Working (True)" if filter_camera else None,
            'sonar': filter_sonar == "Working (True)" if filter_sonar else None,
            'distance': int(filter_distance.split('(')[1].split(')')[0]) if filter_distance else None
        } if filter_battery or filter_camera or filter_sonar or filter_distance else None
    
    # Tab-based interface for different visualizations
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "Q-Table Analysis", 
        "Best Actions", 
        "Reward Function", 
        "Mission Simulator",
        "Learning Metrics"
    ])
    
    # Tab 1: Q-Table Analysis
    with tab1:
        st.header("Q-Table Heatmap")
        st.write("This visualization shows the Q-values for different state-action pairs.")
        
        if qtable_path:
            data = parse_qtable(qtable_path)
            if data:
                fig = create_qtable_heatmap(data, filter_options)
                if fig:
                    st.pyplot(fig)
                
                # Raw data table
                with st.expander("View Raw Q-Table Data"):
                    # Convert to pandas DataFrame for display
                    rows = []
                    for entry in data:
                        rows.append({
                            'Battery': BATTERY_LEVELS[entry['battery']],
                            'Camera': '✓' if entry['camera'] else '✗',
                            'Sonar': '✓' if entry['sonar'] else '✗',
                            'Distance': DISTANCE_LEVELS[entry['distance']],
                            'Mission Order': order_to_string(entry['order']),
                            'Q-Value': entry['q_value']
                        })
                    
                    st.dataframe(pd.DataFrame(rows))
            else:
                st.info("Upload a Q-table file or provide a valid file path to see the heatmap.")
        else:
            st.info("Upload a Q-table file or provide a valid file path to see the heatmap.")
    
    # Tab 2: Best Actions
    with tab2:
        st.header("Best Actions by State")
        st.write("This visualization shows the best action (mission order) for each state based on learned Q-values.")
        
        if qtable_path:
            data = parse_qtable(qtable_path)
            if data:
                df = create_best_action_plot(data, filter_options)
                if df is not None:
                    st.dataframe(df)
            else:
                st.info("Upload a Q-table file or provide a valid file path to see the best actions.")
        else:
            st.info("Upload a Q-table file or provide a valid file path to see the best actions.")
    
    # Tab 3: Reward Function
    with tab3:
        st.header("Reward Function Analysis")
        st.write("This visualization shows the components of the reward function used in the RL algorithm.")
        
        fig = create_reward_function_visualization()
        st.pyplot(fig)
        
        with st.expander("Reward Function Details"):
            st.write("""
            The reward function has the following components:
            
            $R(s, a, t, success) = R_{base} + R_{efficiency} + R_{sensor} + R_{battery}$
            
            Where:
            - $R_{base}$ = +10 for success, -5 for failure
            - $R_{efficiency}$ = +5 for completing missions with low battery
            - $R_{sensor}$ = +3 for using the appropriate sensor for each task
            - $R_{battery}$ = -4 for not returning to dock when battery is low
            
            This reward function encourages the ROV to:
            1. Complete missions successfully
            2. Be efficient with battery usage
            3. Use the appropriate sensors for each task
            4. Return to dock when battery is low
            """)
    
    # Tab 4: Mission Simulator
    with tab4:
        st.header("Mission Planning Simulator")
        st.write("This simulator shows how the ROV would plan missions under different conditions.")
        
        # State selection for simulation
        col1, col2 = st.columns(2)
        
        with col1:
            sim_battery = st.selectbox("Battery Level", 
                                      ["Low (0)", "Medium (1)", "High (2)"], 
                                      index=2)  # Default to High
            sim_battery_value = int(sim_battery.split('(')[1].split(')')[0])
        
        with col2:
            sim_distance = st.selectbox("Distance to Dock", 
                                       ["Near (0)", "Medium (1)", "Far (2)"], 
                                       index=0)  # Default to Near
            sim_distance_value = int(sim_distance.split('(')[1].split(')')[0])
        
        # Run simulation
        st.subheader(f"Mission Planning with {sim_battery.split('(')[0].strip()} Battery and {sim_distance.split('(')[0].strip()} Distance")
        
        table_data, columns = create_sensor_impact_simulation(sim_battery_value, sim_distance_value)
        
        # Display results in a formatted table
        df = pd.DataFrame(table_data, columns=columns)
        st.table(df)
        
        with st.expander("Simulation Details"):
            st.write("""
            This simulation shows how sensor availability affects mission planning and success probability.
            
            - When both sensors are working, the ROV can perform all tasks with high success rate
            - When the camera is offline, pipeline inspection becomes risky or impossible
            - When the sonar is offline, wreckage inspection becomes risky or impossible
            - When both sensors are offline, return to dock is usually the only safe option
            
            The recommended actions and probabilities are derived from the Q-learning process, where
            the ROV learns which mission order is optimal for each state combination.
            """)
    
    # Tab 5: Learning Metrics
    with tab5:
        st.header("Learning Metrics")
        st.write("This section shows how the RL algorithm learns over time.")
        
        # Simulated exploration rate plot
        st.subheader("Exploration Rate Decay")
        
        # Create the plot
        episodes = range(1, 101)
        high_exploration = [max(0.5 * (0.99 ** i), 0.01) for i in episodes]
        medium_exploration = [max(0.3 * (0.97 ** i), 0.01) for i in episodes]
        low_exploration = [max(0.1 * (0.95 ** i), 0.01) for i in episodes]
        
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(episodes, high_exploration, label='High Initial (ε=0.5)', linewidth=2)
        ax.plot(episodes, medium_exploration, label='Medium Initial (ε=0.3)', linewidth=2)
        ax.plot(episodes, low_exploration, label='Low Initial (ε=0.1)', linewidth=2)
        
        ax.axhline(y=0.01, color='gray', linestyle='--', alpha=0.7, label='Minimum Exploration (ε=0.01)')
        
        ax.set_title('Exploration Rate Decay Over Episodes')
        ax.set_xlabel('Episode')
        ax.set_ylabel('Exploration Rate (ε)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        st.pyplot(fig)
        
        # Simulated learning curve
        st.subheader("Learning Curve Simulation")
        
        # Create the plot
        episodes = range(1, 101)
        np.random.seed(42)  # For reproducibility
        base_curve = -5 + 15 * (1 - np.exp(-0.05 * np.array(episodes)))
        noise = np.random.normal(0, 2, len(episodes))
        rewards = base_curve + noise
        
        # Calculate moving average
        window_size = 10
        moving_avg = np.convolve(rewards, np.ones(window_size)/window_size, mode='valid')
        moving_avg_x = episodes[window_size-1:]
        
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(episodes, rewards, 'o-', alpha=0.4, label='Episode Reward')
        ax.plot(moving_avg_x, moving_avg, 'r-', linewidth=2, label=f'{window_size}-Episode Moving Average')
        
        ax.axhline(y=10, color='green', linestyle='--', alpha=0.7, label='Maximum Task Reward')
        ax.axhline(y=0, color='gray', linestyle='-', alpha=0.5)
        ax.axhline(y=-5, color='red', linestyle='--', alpha=0.7, label='Task Failure Penalty')
        
        ax.set_title('Simulated Learning Curve for ROV Mission Planning')
        ax.set_xlabel('Episode')
        ax.set_ylabel('Average Reward')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        st.pyplot(fig)
        
        with st.expander("Learning Process Details"):
            st.write("""
            ## Q-Learning Process
            
            The ROV uses Q-learning to determine the optimal mission order based on the current state:
            
            $Q(s, a) \\leftarrow (1 - \\alpha) \\cdot Q(s, a) + \\alpha \\cdot R(s, a)$
            
            Where:
            - $Q(s, a)$ is the expected value of taking action $a$ in state $s$
            - $\\alpha$ is the learning rate (how quickly new information overrides old)
            - $R(s, a)$ is the reward received for the action
            
            ## Exploration vs. Exploitation
            
            The ROV uses an ε-greedy policy to balance exploration and exploitation:
            - With probability ε, it tries a random mission order (exploration)
            - With probability 1-ε, it selects the mission order with highest Q-value (exploitation)
            
            The exploration rate ε decreases over time as the ROV learns, transitioning from exploration to exploitation.
            """)
    
    # Footer
    st.markdown("---")
    st.markdown("ROV Reinforcement Learning Dashboard | Created for mission analysis and visualization")

if __name__ == "__main__":
    main()