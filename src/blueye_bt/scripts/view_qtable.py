#!/usr/bin/env python3

def read_qtable(filename):
    try:
        with open(filename, 'r') as f:
            version = f.readline().strip()
            if version != 'v1':
                print(f"Unknown format: {version}")
                return
                
            entries = int(f.readline().strip())
            print(f"Found {entries} entries in Q-table")
            
            for i in range(entries):
                line = f.readline().strip().split()
                
                # Parse state
                battery = int(line[0])
                camera = bool(int(line[1]))
                sonar = bool(int(line[2]))
                distance = int(line[3])
                
                # Parse order
                order_size = int(line[4])
                order = [int(line[5+i]) for i in range(order_size)]
                
                # Parse Q-value
                q_value = float(line[5+order_size])
                
                # Convert to readable format
                battery_str = ["Low", "Medium", "High"][battery]
                distance_str = ["Near", "Medium", "Far"][distance]
                task_names = {0: "Undocking", 1: "Pipeline Inspection", 2: "Wreckage Inspection", 3: "Return to Dock"}
                order_str = " -> ".join([task_names.get(t, f"Task{t}") for t in order])
                
                print(f"State: Battery={battery_str}, Camera={'✓' if camera else '✗'}, " +
                      f"Sonar={'✓' if sonar else '✗'}, Distance={distance_str}")
                print(f"  Action: {order_str}")
                print(f"  Q-value: {q_value:.2f}")
                print()
    except FileNotFoundError:
        print(f"File not found: {filename}")
    except Exception as e:
        print(f"Error reading Q-table: {e}")

if __name__ == "__main__":
    import sys
    filename = sys.argv[1] if len(sys.argv) > 1 else "/tmp/mission_q_table.txt"
    read_qtable(filename)