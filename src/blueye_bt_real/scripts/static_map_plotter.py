#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import contextily as ctx
import geopandas as gpd
from shapely.geometry import Point as ShapelyPoint, LineString
import pandas as pd
from datetime import datetime
import numpy as np

class StaticMapPlotter(Node):
    def __init__(self):
        super().__init__('static_map_plotter')
        
        self.gps_points = []
        
        # Subscribe to GPS topic
        self.subscription = self.create_subscription(
            Point,
            '/blueye/gps',
            self.gps_callback,
            10
        )
        
        # Timer to update plot every 10 seconds
        self.timer = self.create_timer(10.0, self.create_static_map)
        
        self.get_logger().info('Static Map Plotter started')

    def gps_callback(self, msg):
        """Store GPS points"""
        self.gps_points.append({
            'lat': msg.x,
            'lon': msg.y,
            'alt': msg.z,
            'timestamp': datetime.now()
        })

    def create_static_map(self):
        """Create a static map with satellite imagery background"""
        if len(self.gps_points) < 2:
            return
            
        # Create DataFrame
        df = pd.DataFrame(self.gps_points)
        
        # Create GeoDataFrame
        geometry = [ShapelyPoint(xy) for xy in zip(df['lon'], df['lat'])]
        gdf = gpd.GeoDataFrame(df, geometry=geometry, crs='EPSG:4326')
        
        # Convert to Web Mercator for contextily
        gdf = gdf.to_crs('EPSG:3857')
        
        # Create the plot
        fig, ax = plt.subplots(figsize=(15, 10))
        
        # Plot trajectory line
        if len(self.gps_points) > 1:
            coords = [(point['lon'], point['lat']) for point in self.gps_points]
            line = LineString(coords)
            line_gdf = gpd.GeoDataFrame([1], geometry=[line], crs='EPSG:4326').to_crs('EPSG:3857')
            line_gdf.plot(ax=ax, color='red', linewidth=3, alpha=0.8, label='ROV Track')
        
        # Plot points
        gdf.plot(ax=ax, color='blue', markersize=30, alpha=0.7, label='GPS Points')
        
        # Highlight start and end points
        if len(self.gps_points) > 0:
            start_gdf = gdf.iloc[:1]
            start_gdf.plot(ax=ax, color='green', markersize=100, marker='^', label='Start')
            
            end_gdf = gdf.iloc[-1:]
            end_gdf.plot(ax=ax, color='red', markersize=100, marker='v', label='Current')
        
        # Add satellite basemap
        try:
            ctx.add_basemap(ax, 
                          crs=gdf.crs, 
                          source=ctx.providers.Esri.WorldImagery,
                          zoom=16)
        except:
            # Fallback to OpenStreetMap if satellite fails
            ctx.add_basemap(ax, crs=gdf.crs, source=ctx.providers.OpenStreetMap.Mapnik)
        
        # Styling
        ax.set_title(f'BlueEye ROV Trajectory - {len(self.gps_points)} GPS Points\n'
                    f'Location: Trondheim Area, Norway', fontsize=16, fontweight='bold')
        ax.legend(loc='upper right')
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        
        # Remove axis ticks for cleaner look
        ax.tick_params(axis='both', which='both', length=0)
        
        # Add north arrow and scale
        ax.annotate('N', xy=(0.05, 0.95), xycoords='axes fraction', 
                   fontsize=20, fontweight='bold', ha='center')
        ax.annotate('↑', xy=(0.05, 0.92), xycoords='axes fraction', 
                   fontsize=16, ha='center')
        
        plt.tight_layout()
        plt.savefig('rov_trajectory_map.png', dpi=300, bbox_inches='tight')
        plt.savefig('rov_trajectory_map.pdf', bbox_inches='tight')
        
        self.get_logger().info(f'Static map saved with {len(self.gps_points)} points')
        
        plt.show(block=False)
        plt.pause(0.1)

def main():
    rclpy.init()
    
    try:
        plotter = StaticMapPlotter()
        rclpy.spin(plotter)
    except KeywordInterrupt:
        print("Shutting down static map plotter...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()