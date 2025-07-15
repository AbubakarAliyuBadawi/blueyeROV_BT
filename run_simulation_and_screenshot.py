import json
import folium
from folium import plugins
import numpy as np

# Load your JSON file
json_path = "/home/badawi/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts/rov_trajectory_data.json"
with open(json_path, "r") as f:
    data = json.load(f)

# Extract trajectory points (remove duplicates for cleaner visualization)
coordinates = []
seen_coords = set()
for entry in data:
    # coord_key = (round(entry["lat"], 8), round(entry["lon"], 8))  # Round to avoid tiny differences
    # if coord_key not in seen_coords:
    coordinates.append([entry["lat"], entry["lon"]])
        # seen_coords.add(coord_key)

print(f"Total unique coordinates: {len(coordinates)}")

# Calculate the bounding box for your trajectory
lats = [coord[0] for coord in coordinates]
lons = [coord[1] for coord in coordinates]
min_lat, max_lat = min(lats), max(lats)
min_lon, max_lon = min(lons), max(lons)

# Calculate center point
center_lat = (min_lat + max_lat) / 2
center_lon = (min_lon + max_lon) / 2

print(f"Trajectory bounds: Lat({min_lat:.8f} to {max_lat:.8f}), Lon({min_lon:.8f} to {max_lon:.8f})")

# Initialize folium map with very high zoom
m = folium.Map(
    location=[center_lat, center_lon],
    zoom_start=22,  # Maximum zoom level
    tiles=None
)

# Add multiple high-resolution satellite tile sources
# Esri World Imagery (your current choice)
folium.TileLayer(
    tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    attr='Esri World Imagery',
    name='Esri Satellite',
    overlay=False,
    control=True,
    max_zoom=22
).add_to(m)

# Google Satellite (often higher resolution)
folium.TileLayer(
    tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
    attr='Google Satellite',
    name='Google Satellite',
    overlay=False,
    control=True,
    max_zoom=22
).add_to(m)

# Bing Satellite (alternative high-res option)
folium.TileLayer(
    tiles='https://ecn.t3.tiles.virtualearth.net/tiles/a{q}.jpeg?g=1',
    attr='Bing Satellite',
    name='Bing Satellite',
    overlay=False,
    control=True,
    max_zoom=21,
    subdomains='0123'
).add_to(m)

# Plot trajectory with enhanced styling
if len(coordinates) > 1:
    folium.PolyLine(
        coordinates, 
        color="red", 
        weight=4, 
        opacity=0.8,
        popup="ROV Trajectory"
    ).add_to(m)
    
    # Add direction markers at regular intervals instead of arrows
    num_markers = min(10, len(coordinates)//20)  # Add up to 10 direction markers
    if num_markers > 0:
        step = len(coordinates) // num_markers
        for i in range(0, len(coordinates), step):
            if i < len(coordinates):
                folium.CircleMarker(
                    location=coordinates[i],
                    radius=3,
                    popup=f"Point {i+1}",
                    color="yellow",
                    fillColor="orange",
                    fillOpacity=0.8
                ).add_to(m)

# Add markers for start and end points
if coordinates:
    # Start point
    folium.Marker(
        location=coordinates[0],
        popup=f"Start Point<br>Lat: {coordinates[0][0]:.8f}<br>Lon: {coordinates[0][1]:.8f}",
        icon=folium.Icon(color="green", icon="play")
    ).add_to(m)
    
    # End point (if different from start)
    if len(coordinates) > 1:
        folium.Marker(
            location=coordinates[-1],
            popup=f"End Point<br>Lat: {coordinates[-1][0]:.8f}<br>Lon: {coordinates[-1][1]:.8f}",
            icon=folium.Icon(color="red", icon="stop")
        ).add_to(m)

# Fit the map to show all trajectory points with some padding
if len(coordinates) > 1:
    # Add small padding to the bounds
    lat_padding = (max_lat - min_lat) * 0.1 if max_lat != min_lat else 0.0001
    lon_padding = (max_lon - min_lon) * 0.1 if max_lon != min_lon else 0.0001
    
    southwest = [min_lat - lat_padding, min_lon - lon_padding]
    northeast = [max_lat + lat_padding, max_lon + lon_padding]
    
    m.fit_bounds([southwest, northeast])

# Add layer control to switch between satellite providers
folium.LayerControl().add_to(m)

# Add a scale bar
plugins.MeasureControl().add_to(m)

# Save map
output_path = "/home/badawi/Desktop/rov_trajectory_satellite_enhanced.html"
m.save(output_path)
print(f"Enhanced satellite map saved to: {output_path}")

# Print some statistics
if len(coordinates) > 1:
    # Calculate approximate trajectory length
    total_distance = 0
    for i in range(len(coordinates)-1):
        lat1, lon1 = coordinates[i]
        lat2, lon2 = coordinates[i+1]
        
        # Haversine distance (approximate for small distances)
        dlat = np.radians(lat2 - lat1)
        dlon = np.radians(lon2 - lon1)
        a = np.sin(dlat/2)**2 + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        distance = 6371000 * c  # Earth radius in meters
        total_distance += distance
    
    print(f"Approximate trajectory length: {total_distance:.2f} meters")
    print(f"Trajectory spans: {(max_lat-min_lat)*111000:.2f}m N-S, {(max_lon-min_lon)*111000*np.cos(np.radians(center_lat)):.2f}m E-W")