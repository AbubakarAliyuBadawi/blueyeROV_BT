# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# import folium
# import webbrowser
# import os
# from datetime import datetime
# import json

# class GPSRealMapVisualizer(Node):
#     def __init__(self):
#         super().__init__('gps_real_map_visualizer')
        
#         # Store GPS trajectory
#         self.trajectory = []
#         self.map_file = 'blueye_rov_trajectory.html'
        
#         # Subscribe to GPS topic
#         self.subscription = self.create_subscription(
#             Point,
#             '/blueye/gps',
#             self.gps_callback,
#             10
#         )
        
#         # Create initial map centered on Trondheim area
#         self.initialize_map()
        
#         # Timer to update map every 5 seconds
#         self.timer = self.create_timer(5.0, self.update_map)
        
#         self.get_logger().info('GPS Real Map Visualizer started - Map will be saved as blueye_rov_trajectory.html')

#     def initialize_map(self):
#         """Initialize the folium map"""
#         # Center on Trondheim fjord area based on your coordinates
#         self.map = folium.Map(
#             location=[63.44145, 10.3483],  # Your GPS coordinates
#             zoom_start=14,
#             tiles='OpenStreetMap'
#         )
        
#         # Add satellite imagery option
#         folium.TileLayer(
#             'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
#             attr='Esri',
#             name='Satellite',
#             overlay=False,
#             control=True
#         ).add_to(self.map)
        
#         # Add marine/nautical chart layer
#         folium.TileLayer(
#             'https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png',
#             attr='OpenSeaMap',
#             name='Nautical Chart',
#             overlay=True,
#             control=True
#         ).add_to(self.map)
        
#         # Add layer control
#         folium.LayerControl().add_to(self.map)

#     def gps_callback(self, msg):
#         """Store GPS points as they arrive"""
#         timestamp = datetime.now().strftime("%H:%M:%S")
        
#         point = {
#             'lat': msg.x,
#             'lon': msg.y,
#             'alt': msg.z,
#             'timestamp': timestamp
#         }
        
#         self.trajectory.append(point)
        
#         self.get_logger().info(f'GPS Point: {msg.x:.6f}, {msg.y:.6f}, Alt: {msg.z:.2f}m at {timestamp}')

#     def update_map(self):
#         """Update the map with new trajectory points"""
#         if len(self.trajectory) < 1:
#             return
            
#         # Clear the map and reinitialize
#         self.initialize_map()
        
#         # Add trajectory line if we have multiple points
#         if len(self.trajectory) > 1:
#             coords = [[point['lat'], point['lon']] for point in self.trajectory]
            
#             folium.PolyLine(
#                 coords,
#                 color='red',
#                 weight=4,
#                 opacity=0.8,
#                 popup='ROV Trajectory'
#             ).add_to(self.map)
        
#         # Add start point (green)
#         if len(self.trajectory) > 0:
#             start = self.trajectory[0]
#             folium.Marker(
#                 [start['lat'], start['lon']],
#                 popup=f"Start Point<br>Time: {start['timestamp']}<br>Alt: {start['alt']:.2f}m",
#                 icon=folium.Icon(color='green', icon='play')
#             ).add_to(self.map)
        
#         # Add current position (red)
#         if len(self.trajectory) > 0:
#             current = self.trajectory[-1]
#             folium.Marker(
#                 [current['lat'], current['lon']],
#                 popup=f"Current Position<br>Time: {current['timestamp']}<br>Lat: {current['lat']:.6f}<br>Lon: {current['lon']:.6f}<br>Alt: {current['alt']:.2f}m",
#                 icon=folium.Icon(color='red', icon='stop')
#             ).add_to(self.map)
        
#         # Add waypoint markers every 10th point
#         for i, point in enumerate(self.trajectory[::10]):
#             if i > 0:  # Skip start point
#                 folium.CircleMarker(
#                     [point['lat'], point['lon']],
#                     radius=3,
#                     popup=f"Waypoint {i}<br>Time: {point['timestamp']}<br>Alt: {point['alt']:.2f}m",
#                     color='blue',
#                     fillColor='lightblue',
#                     fillOpacity=0.7
#                 ).add_to(self.map)
        
#         # Add trajectory statistics
#         if len(self.trajectory) > 1:
#             # Calculate total distance (approximate)
#             total_distance = 0
#             for i in range(1, len(self.trajectory)):
#                 lat1, lon1 = self.trajectory[i-1]['lat'], self.trajectory[i-1]['lon']
#                 lat2, lon2 = self.trajectory[i]['lat'], self.trajectory[i]['lon']
                
#                 # Haversine distance approximation for short distances
#                 dlat = lat2 - lat1
#                 dlon = lon2 - lon1
#                 distance = ((dlat * 111000)**2 + (dlon * 111000 * abs(lat1/90))**2)**0.5
#                 total_distance += distance
            
#             # Add info box
#             info_html = f"""
#             <div style="position: fixed; top: 10px; right: 10px; width: 200px; background: white; 
#                         border: 2px solid grey; z-index: 9999; font-size: 12px; padding: 10px;">
#             <h4>ROV Mission Stats</h4>
#             <b>Points:</b> {len(self.trajectory)}<br>
#             <b>Distance:</b> {total_distance:.1f}m<br>
#             <b>Duration:</b> {self.trajectory[0]['timestamp']} - {self.trajectory[-1]['timestamp']}<br>
#             <b>Current Depth:</b> {self.trajectory[-1]['alt']:.2f}m
#             </div>
#             """
#             self.map.get_root().html.add_child(folium.Element(info_html))
        
#         # Save the map
#         self.map.save(self.map_file)
        
#         self.get_logger().info(f'Map updated with {len(self.trajectory)} points')

#     def save_trajectory_data(self):
#         """Save trajectory data as JSON for later analysis"""
#         with open('rov_trajectory_data.json', 'w') as f:
#             json.dump(self.trajectory, f, indent=2)

# def main():
#     rclpy.init()
    
#     try:
#         visualizer = GPSRealMapVisualizer()
#         print(f"\n🗺️  Real-world map visualization started!")
#         print(f"📍 Open 'blueye_rov_trajectory.html' in your browser to see the live map")
#         print(f"🚁 ROV trajectory will be plotted in real-time on the map")
#         print(f"⚓ Based on coordinates, you're operating near Trondheim, Norway")
#         print(f"\nPress Ctrl+C to stop...\n")
        
#         rclpy.spin(visualizer)
        
#     except KeyboardInterrupt:
#         print("\n🛑 Stopping GPS visualization...")
#         if hasattr(visualizer, 'trajectory') and visualizer.trajectory:
#             visualizer.save_trajectory_data()
#             print(f"💾 Saved {len(visualizer.trajectory)} GPS points to 'rov_trajectory_data.json'")
        
#     finally:
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# import numpy as np
# import pandas as pd
# from datetime import datetime, timedelta

# # Define the waypoints from the table
# waypoints = {
#     'P1': {'lat': 63.441461, 'lon': 10.348320, 'phase': 'Undocking'},
#     'P2': {'lat': 63.441461, 'lon': 10.348370, 'phase': 'Transit 1'},
#     'P3': {'lat': 63.441390, 'lon': 10.348361, 'phase': 'Pipeline Inspection'},
#     'P4': {'lat': 63.441394, 'lon': 10.348239, 'phase': 'Pipeline Inspection'},
#     'P5': {'lat': 63.441420, 'lon': 10.348241, 'phase': 'Transit 2'},
#     'P6': {'lat': 63.441427, 'lon': 10.348363, 'phase': 'Docking'}
# }

# def interpolate_path(start_point, end_point, num_points):
#     """Generate interpolated points between two waypoints"""
#     lat_start, lon_start = start_point
#     lat_end, lon_end = end_point
    
#     # Linear interpolation
#     lats = np.linspace(lat_start, lat_end, num_points)
#     lons = np.linspace(lon_start, lon_end, num_points)
    
#     return list(zip(lats, lons))

# def generate_rov_mission_data():
#     """Generate continuous GPS data for the ROV mission"""
    
#     # Define the mission sequence based on the diagram
#     mission_sequence = [
#         ('P1', 'P2', 'Transit 1', 20),      # Undocking to Transit 1
#         ('P2', 'P3', 'Pipeline Inspection', 30),  # Transit 1 to Inspection start
#         ('P3', 'P4', 'Pipeline Inspection', 50),  # Along pipeline inspection
#         ('P4', 'P5', 'Transit 2', 25),     # Inspection end to Transit 2
#         ('P5', 'P6', 'Docking', 20),       # Transit 2 to Docking
#         ('P6', 'P6', 'Docking Station', 10) # At docking station
#     ]
    
#     gps_data = []
#     timestamp = datetime(2024, 6, 4, 10, 0, 0)  # Start time
#     point_id = 1
    
#     for start_wp, end_wp, phase, num_points in mission_sequence:
#         start_coords = (waypoints[start_wp]['lat'], waypoints[start_wp]['lon'])
#         end_coords = (waypoints[end_wp]['lat'], waypoints[end_wp]['lon'])
        
#         # Generate interpolated points
#         if start_wp == end_wp:  # Stationary at docking station
#             points = [start_coords] * num_points
#         else:
#             points = interpolate_path(start_coords, end_coords, num_points)
        
#         # Add points to dataset
#         for i, (lat, lon) in enumerate(points):
#             # Add small random variations to simulate real GPS noise
#             lat_noise = np.random.normal(0, 0.000002)  # ~0.2m variation
#             lon_noise = np.random.normal(0, 0.000002)
            
#             gps_data.append({
#                 'Point_ID': point_id,
#                 'Timestamp': timestamp.strftime('%Y-%m-%d %H:%M:%S'),
#                 'Latitude': round(lat + lat_noise, 6),
#                 'Longitude': round(lon + lon_noise, 6),
#                 'Mission_Phase': phase,
#                 'Waypoint_Nearest': start_wp if i < num_points//2 else end_wp,
#                 'Depth_m': round(np.random.uniform(15, 25), 1),  # Simulated depth
#                 'Speed_ms': round(np.random.uniform(0.5, 2.0), 1) if phase != 'Docking Station' else 0.0
#             })
            
#             point_id += 1
#             timestamp += timedelta(seconds=10)  # 10-second intervals
    
#     return pd.DataFrame(gps_data)

# # Generate the GPS data
# df = generate_rov_mission_data()

# # Display sample data
# print("ROV Mission GPS Data Sample:")
# print("="*80)
# print(df.head(15).to_string(index=False))
# print("\n" + "="*80)
# print(f"Total data points: {len(df)}")
# print(f"Mission duration: {len(df) * 10 / 60:.1f} minutes")

# # Show waypoint summary
# print("\nWaypoint Summary:")
# print("-"*50)
# for wp, data in waypoints.items():
#     print(f"{wp}: {data['lat']:.6f}, {data['lon']:.6f} - {data['phase']}")

# # Phase distribution
# print("\nPhase Distribution:")
# print("-"*30)
# phase_counts = df['Mission_Phase'].value_counts()
# for phase, count in phase_counts.items():
#     print(f"{phase}: {count} points ({count*10/60:.1f} min)")

# # Save to CSV (uncomment to save)
# # df.to_csv('rov_mission_gps_data.csv', index=False)
# # print("\nData saved to 'rov_mission_gps_data.csv'")

# # Additional analysis - distance calculations
# def haversine_distance(lat1, lon1, lat2, lon2):
#     """Calculate distance between two GPS points in meters"""
#     R = 6371000  # Earth's radius in meters
    
#     lat1_rad = np.radians(lat1)
#     lat2_rad = np.radians(lat2)
#     delta_lat = np.radians(lat2 - lat1)
#     delta_lon = np.radians(lon2 - lon1)
    
#     a = np.sin(delta_lat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(delta_lon/2)**2
#     c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
#     return R * c

# # Calculate total mission distance
# total_distance = 0
# for i in range(1, len(df)):
#     dist = haversine_distance(
#         df.iloc[i-1]['Latitude'], df.iloc[i-1]['Longitude'],
#         df.iloc[i]['Latitude'], df.iloc[i]['Longitude']
#     )
#     total_distance += dist

# print(f"\nMission Statistics:")
# print(f"Total distance traveled: {total_distance:.1f} meters")
# print(f"Average speed: {total_distance / (len(df) * 10):.2f} m/s")

# # Show last few points to verify return to docking
# print(f"\nFinal positions (return to docking):")
# print(df.tail(5)[['Timestamp', 'Latitude', 'Longitude', 'Mission_Phase']].to_string(index=False))