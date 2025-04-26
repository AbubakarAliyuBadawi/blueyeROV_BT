import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file
df = pd.read_csv('states.csv')

# Create a mapping for state values to names
state_names = {
    0: 'Not Started',
    1: 'Undocking',
    2: 'Pipeline Inspection',
    3: 'Wreckage Inspection',
    4: 'Return to Dock',
    5: 'Docking',
    99: 'Emergency'
}

# Extract just the time and mission state columns
mission_data = df[['__time', '/mission_state/data']].copy()
mission_data.columns = ['time', 'state']

# Drop rows with NaN values in state column
mission_data = mission_data.dropna(subset=['state'])

# If the time column isn't starting from 0, you might want to normalize it
mission_data['time'] = mission_data['time'] - mission_data['time'].min()

# Create the plot
plt.figure(figsize=(20, 6))
plt.step(mission_data['time'], mission_data['state'], where='post', linewidth=2.5)

# Find the unique states in your actual data
states = sorted(mission_data['state'].unique())

# Create custom y-ticks with state names
plt.yticks(states, [state_names.get(int(s), f'State {s}') for s in states])

# Add grid, labels and title
plt.grid(True, alpha=0.3)
plt.xlabel('Time (seconds)', fontsize=12)
plt.ylabel('Mission State', fontsize=12)
plt.title('ROV Mission State Timeline', fontsize=14)

# Add some styling
plt.tight_layout()
plt.savefig('mission_timeline.png', dpi=300)
plt.show()