#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

def parse_qtable(filename):
    data = []
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
            order = [int(line[5+i]) for i in range(order_size)]
            
            # Parse Q-value
            q_value = float(line[5+order_size])
            
            data.append({
                'state': (battery, camera, sonar, distance),
                'order': tuple(order),
                'q_value': q_value
            })
    return data

def analyze_qtable(data):
    # Group by state and find best action for each state
    best_actions = {}
    for entry in data:
        state = entry['state']
        if state not in best_actions or entry['q_value'] > best_actions[state]['q_value']:
            best_actions[state] = {
                'order': entry['order'],
                'q_value': entry['q_value']
            }
    
    # Print findings
    print(f"Found {len(data)} Q-value entries across {len(best_actions)} states")
    print("\nTop learned strategies:")
    
    task_names = {0: "Pipeline Inspection", 1: "Wreckage Inspection", 2: "Return to Dock"}
    
    for state, action in best_actions.items():
        battery, camera, sonar, distance = state
        battery_level = ["Low", "Medium", "High"][battery]
        distance_level = ["Near", "Medium", "Far"][distance]
        
        print(f"\nState: Battery={battery_level}, Camera={'Working' if camera else 'Offline'}, " +
              f"Sonar={'Working' if sonar else 'Offline'}, Distance={distance_level}")
        
        print(f"  Best action: {[task_names[t] for t in action['order']]} (Q={action['q_value']:.2f})")

# Main analysis
data = parse_qtable("/tmp/mission_q_table.txt")
analyze_qtable(data)