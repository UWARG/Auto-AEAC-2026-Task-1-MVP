#!/usr/bin/env python3
"""
WARG Drone Competition 2025 - Ground Station
Enhanced ground control station for target visualization and building mapping
"""

import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import re
import simplekml
import threading
from decimal import Decimal
import os
import subprocess
import math
import json
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from tkintermapview import TkinterMapView
from dataclasses import dataclass

@dataclass
class Target:
    """Represents a detected target"""
    lat: float
    lon: float
    alt: float
    color: str
    location_type: str  # 'Ground', 'Wall', or 'Roof'
    timestamp: datetime
    id: int
    
    def __str__(self):
        return f"{self.color.title()} {self.location_type} Target #{self.id}"

@dataclass
class BuildingCorner:
    """Represents a building corner"""
    lat: float
    lon: float
    corner_id: int
    timestamp: datetime

class GroundStation:
    """Enhanced ground station for drone competition"""
    
    def __init__(self):
        # Data storage
        self.targets: List[Target] = []
        self.building_corners: List[BuildingCorner] = []
        self.target_counter = 1
        self.corner_counter = 1
        
        # KML export
        self.kml = simplekml.Kml()
        self.target_folder = self.kml.newfolder(name="Targets")
        self.building_folder = self.kml.newfolder(name="Building")
        
        # Communication
        self.mavlink_connection: Optional[mavutil.mavlink_connection] = None
        self.stop_flag = threading.Event()
        
        # Regex patterns for different message types
        self.target_pattern = re.compile(
            r"Target-([a-zA-Z]+):\s*(-?\d+\.\d+),\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)m\s*\(([^)]+)\)"
        )
        self.corner_pattern = re.compile(
            r"Corner\s+(\d+)/4\s+recorded:\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)"
        )
        
        # GUI components
        self.root = None
        self.setup_gui()
    
    def setup_gui(self):
        """Setup the graphical user interface"""
        self.root = tk.Tk()
        self.root.title("WARG Drone Competition 2025 - Ground Station")
        self.root.state("zoomed")
        
        # Create main panels
        self.create_control_panel()
        self.create_map_panel()
        
        # Setup map
        self.setup_map()
    
    def create_control_panel(self):
        """Create left control panel"""
        control_frame = tk.Frame(self.root, width=400)
        control_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)
        control_frame.pack_propagate(False)
        
        # Title
        title_label = tk.Label(control_frame, text="WARG Ground Station 2025", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=(0, 10))
        
        # Status frame
        status_frame = tk.LabelFrame(control_frame, text="System Status", font=("Arial", 10, "bold"))
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.status_label = tk.Label(status_frame, text="Initializing...", fg="orange")
        self.status_label.pack(pady=5)
        
        # Building corners frame
        corners_frame = tk.LabelFrame(control_frame, text="Building Corners", font=("Arial", 10, "bold"))
        corners_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.corners_listbox = tk.Listbox(corners_frame, height=4)
        self.corners_listbox.pack(fill=tk.X, padx=5, pady=5)
        
        corners_button_frame = tk.Frame(corners_frame)
        corners_button_frame.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        self.clear_corners_button = tk.Button(corners_button_frame, text="Clear Corners",
                                            command=self.clear_building_corners)
        self.clear_corners_button.pack(side=tk.LEFT)
        
        # Targets frame  
        targets_frame = tk.LabelFrame(control_frame, text="Detected Targets", font=("Arial", 10, "bold"))
        targets_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Create treeview for targets
        columns = ("ID", "Color", "Type", "Coordinates")
        self.targets_tree = ttk.Treeview(targets_frame, columns=columns, show="headings", height=15)
        
        # Define column headings
        for col in columns:
            self.targets_tree.heading(col, text=col)
            if col == "Coordinates":
                self.targets_tree.column(col, width=200)
            else:
                self.targets_tree.column(col, width=80)
        
        # Scrollbar for treeview
        scrollbar = ttk.Scrollbar(targets_frame, orient=tk.VERTICAL, command=self.targets_tree.yview)
        self.targets_tree.configure(yscrollcommand=scrollbar.set)
        
        # Pack treeview and scrollbar
        self.targets_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0), pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 5), pady=5)
        
        # Target controls
        targets_button_frame = tk.Frame(targets_frame)
        targets_button_frame.pack(fill=tk.X, padx=5, pady=(0, 5))
        
        self.delete_target_button = tk.Button(targets_button_frame, text="Delete Selected",
                                            command=self.delete_selected_target)
        self.delete_target_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.clear_targets_button = tk.Button(targets_button_frame, text="Clear All",
                                            command=self.clear_all_targets)
        self.clear_targets_button.pack(side=tk.LEFT)
        
        # Export/Save controls
        export_frame = tk.LabelFrame(control_frame, text="Export & Save", font=("Arial", 10, "bold"))
        export_frame.pack(fill=tk.X, pady=(0, 10))
        
        export_button_frame = tk.Frame(export_frame)
        export_button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.export_kml_button = tk.Button(export_button_frame, text="Export KML",
                                         command=self.export_kml, bg="lightgreen")
        self.export_kml_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.export_json_button = tk.Button(export_button_frame, text="Export JSON",
                                          command=self.export_json)
        self.export_json_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.stop_button = tk.Button(export_button_frame, text="Stop & Exit",
                                   command=self.stop_program, bg="lightcoral")
        self.stop_button.pack(side=tk.RIGHT)
    
    def create_map_panel(self):
        """Create right map panel"""
        map_frame = tk.Frame(self.root)
        map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(0, 10), pady=10)
        
        # Map controls
        map_control_frame = tk.Frame(map_frame)
        map_control_frame.pack(fill=tk.X, pady=(0, 5))
        
        tk.Label(map_control_frame, text="Map View", font=("Arial", 12, "bold")).pack(side=tk.LEFT)
        
        self.show_building_var = tk.BooleanVar(value=True)
        tk.Checkbutton(map_control_frame, text="Show Building", 
                      variable=self.show_building_var,
                      command=self.update_building_display).pack(side=tk.RIGHT)
        
        # Map widget
        self.map_widget = TkinterMapView(map_frame, width=600, height=700, corner_radius=0)
        self.map_widget.pack(fill=tk.BOTH, expand=True)
        
    def setup_map(self):
        """Initialize map settings"""
        # Default position (can be updated when first coordinate received)
        self.map_widget.set_position(43.4725, -80.5448)  # University of Waterloo area
        self.map_widget.set_zoom(16)
        
        # Use satellite imagery
        self.map_widget.set_tile_server(
            "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            max_zoom=19
        )
        
        # Storage for map markers
        self.target_markers = []
        self.corner_markers = []
        self.building_polygon = None
    
    def start_mavlink_listener(self):
        """Start MAVLink message listener in background thread"""
        listener_thread = threading.Thread(target=self._mavlink_listener, daemon=True)
        listener_thread.start()
        self.status_label.config(text="Connecting to MAVLink...", fg="orange")
    
    def _mavlink_listener(self):
        """MAVLink message listener thread"""
        try:
            self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:14550')
            self.root.after(0, lambda: self.status_label.config(text="Connected to MAVLink", fg="green"))
            
            while not self.stop_flag.is_set():
                try:
                    msg = self.mavlink_connection.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
                    if msg and msg.get_srcComponent() == 191:
                        self.root.after(0, self._process_message, msg.text)
                        
                except Exception as e:
                    print(f"Receive error: {e}")
                    
        except Exception as e:
            print(f"MAVLink connection failed: {e}")
            self.root.after(0, lambda: self.status_label.config(
                text="MAVLink connection failed", fg="red"))
    
    def _process_message(self, text: str):
        """Process incoming MAVLink message"""
        # Try to match target message
        target_match = self.target_pattern.match(text)
        if target_match:
            color = target_match.group(1)
            lat = float(target_match.group(2))
            lon = float(target_match.group(3))
            alt = float(target_match.group(4))
            location_type = target_match.group(5)
            
            self._add_target(lat, lon, alt, color, location_type)
            return
        
        # Try to match corner message
        corner_match = self.corner_pattern.match(text)
        if corner_match:
            corner_id = int(corner_match.group(1))
            lat = float(corner_match.group(2))
            lon = float(corner_match.group(3))
            
            self._add_building_corner(lat, lon, corner_id)
            return
        
        print(f"Unmatched message: {text}")
    
    def _add_target(self, lat: float, lon: float, alt: float, 
                   color: str, location_type: str):
        """Add a new target to the system"""
        # Check for duplicate targets (within 5 meters)
        for existing_target in self.targets:
            distance = self._haversine_distance(lat, lon, existing_target.lat, existing_target.lon)
            if distance < 5.0:
                print(f"Updating existing {existing_target.color} target")
                # Update existing target
                existing_target.lat = lat
                existing_target.lon = lon
                existing_target.alt = alt
                existing_target.timestamp = datetime.now()
                
                # Update display and map
                self._update_target_display()
                self._update_map_display()
                return
        
        # Create new target
        target = Target(
            lat=lat, lon=lon, alt=alt, color=color,
            location_type=location_type, timestamp=datetime.now(),
            id=self.target_counter
        )
        
        self.targets.append(target)
        self.target_counter += 1
        
        print(f"Added new target: {target}")
        
        # Update displays
        self._update_target_display()
        self._update_map_display()
        self._update_kml()
    
    def _add_building_corner(self, lat: float, lon: float, corner_id: int):
        """Add a building corner"""
        corner = BuildingCorner(
            lat=lat, lon=lon, corner_id=corner_id, 
            timestamp=datetime.now()
        )
        
        # Remove existing corner with same ID
        self.building_corners = [c for c in self.building_corners if c.corner_id != corner_id]
        
        self.building_corners.append(corner)
        self.building_corners.sort(key=lambda c: c.corner_id)
        
        print(f"Added building corner {corner_id}: {lat:.7f}, {lon:.7f}")
        
        # Update displays
        self._update_corners_display()
        self._update_map_display()
        self._update_kml()
    
    def _update_target_display(self):
        """Update the targets treeview"""
        # Clear existing items
        for item in self.targets_tree.get_children():
            self.targets_tree.delete(item)
        
        # Add all targets
        for target in self.targets:
            coord_str = f"{target.lat:.7f}, {target.lon:.7f}, {target.alt:.1f}m"
            self.targets_tree.insert("", tk.END, values=(
                target.id, target.color.title(), target.location_type, coord_str
            ))
    
    def _update_corners_display(self):
        """Update the corners listbox"""
        self.corners_listbox.delete(0, tk.END)
        
        for corner in sorted(self.building_corners, key=lambda c: c.corner_id):
            coord_str = f"Corner {corner.corner_id}: {corner.lat:.7f}, {corner.lon:.7f}"
            self.corners_listbox.insert(tk.END, coord_str)
    
    def _update_map_display(self):
        """Update map markers and building outline"""
        # Clear existing markers
        for marker in self.target_markers:
            marker.delete()
        for marker in self.corner_markers:
            marker.delete()
        self.target_markers.clear()
        self.corner_markers.clear()
        
        # Add target markers
        for target in self.targets:
            color_map = {
                'red': 'red', 'blue': 'blue', 'green': 'green',
                'yellow': 'orange', 'black': 'gray', 'white': 'lightgray'
            }
            marker_color = color_map.get(target.color.lower(), 'purple')
            
            marker_text = f"{target.color[:1].upper()}{target.id}"
            marker = self.map_widget.set_marker(
                target.lat, target.lon, 
                text=marker_text,
                marker_color_circle=marker_color,
                marker_color_outside=marker_color
            )
            self.target_markers.append(marker)
        
        # Add corner markers
        for corner in self.building_corners:
            marker = self.map_widget.set_marker(
                corner.lat, corner.lon,
                text=str(corner.corner_id),
                marker_color_circle='yellow',
                marker_color_outside='orange'
            )
            self.corner_markers.append(marker)
        
        # Update building outline
        self._update_building_outline()
        
        # Auto-fit map view
        self._auto_fit_map()
    
    def _update_building_outline(self):
        """Update building outline on map"""
        if self.building_polygon:
            self.building_polygon.delete()
            self.building_polygon = None
        
        if len(self.building_corners) >= 3 and self.show_building_var.get():
            # Sort corners by ID to create proper polygon
            sorted_corners = sorted(self.building_corners, key=lambda c: c.corner_id)
            
            if len(sorted_corners) >= 4:
                # Create closed polygon for complete building
                coords = [(c.lat, c.lon) for c in sorted_corners]
                coords.append(coords[0])  # Close the polygon
                
                self.building_polygon = self.map_widget.set_polygon(
                    coords, fill_color=None, outline_color="yellow", width=3
                )
    
    def _auto_fit_map(self):
        """Auto-fit map to show all markers"""
        all_coords = []
        
        # Add target coordinates
        for target in self.targets:
            all_coords.append((target.lat, target.lon))
        
        # Add corner coordinates
        for corner in self.building_corners:
            all_coords.append((corner.lat, corner.lon))
        
        if len(all_coords) == 1:
            lat, lon = all_coords[0]
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(18)
        elif len(all_coords) > 1:
            lats = [coord[0] for coord in all_coords]
            lons = [coord[1] for coord in all_coords]
            
            min_lat, max_lat = min(lats), max(lats)
            min_lon, max_lon = min(lons), max(lons)
            
            center_lat = (min_lat + max_lat) / 2
            center_lon = (min_lon + max_lon) / 2
            
            self.map_widget.set_position(center_lat, center_lon)
            
            # Calculate appropriate zoom level
            lat_range = max_lat - min_lat
            lon_range = max_lon - min_lon
            max_range = max(lat_range, lon_range)
            
            if max_range < 0.0005:
                zoom = 19
            elif max_range < 0.002:
                zoom = 18
            elif max_range < 0.005:
                zoom = 17
            elif max_range < 0.01:
                zoom = 16
            else:
                zoom = 15
            
            self.map_widget.set_zoom(zoom)
    
    def update_building_display(self):
        """Toggle building outline display"""
        self._update_building_outline()
    
    def delete_selected_target(self):
        """Delete selected target from treeview"""
        selected_items = self.targets_tree.selection()
        if not selected_items:
            messagebox.showwarning("No Selection", "Please select a target to delete.")
            return
        
        for item in selected_items:
            values = self.targets_tree.item(item, 'values')
            target_id = int(values[0])
            
            # Remove from targets list
            self.targets = [t for t in self.targets if t.id != target_id]
            
        # Update displays
        self._update_target_display()
        self._update_map_display()
        self._update_kml()
    
    def clear_all_targets(self):
        """Clear all targets"""
        if messagebox.askyesno("Confirm", "Are you sure you want to clear all targets?"):
            self.targets.clear()
            self.target_counter = 1
            
            self._update_target_display()
            self._update_map_display()
            self._update_kml()
    
    def clear_building_corners(self):
        """Clear all building corners"""
        if messagebox.askyesno("Confirm", "Are you sure you want to clear all building corners?"):
            self.building_corners.clear()
            self.corner_counter = 1
            
            self._update_corners_display()
            self._update_map_display()
            self._update_kml()
    
    def _update_kml(self):
        """Update KML with current data"""
        # Clear existing KML data
        self.kml = simplekml.Kml()
        self.target_folder = self.kml.newfolder(name="Targets")
        self.building_folder = self.kml.newfolder(name="Building")
        
        # Add targets
        for target in self.targets:
            point = self.target_folder.newpoint(
                name=f"{target.color.title()} {target.location_type} Target #{target.id}",
                coords=[(target.lon, target.lat, target.alt)]
            )
            point.description = (f"Color: {target.color}\n"
                               f"Type: {target.location_type}\n"
                               f"Altitude: {target.alt:.1f}m\n"
                               f"Detected: {target.timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
        
        # Add building corners
        for corner in self.building_corners:
            point = self.building_folder.newpoint(
                name=f"Building Corner {corner.corner_id}",
                coords=[(corner.lon, corner.lat, 0)]
            )
            point.description = f"Corner ID: {corner.corner_id}\nRecorded: {corner.timestamp.strftime('%Y-%m-%d %H:%M:%S')}"
        
        # Add building outline if complete
        if len(self.building_corners) >= 4:
            sorted_corners = sorted(self.building_corners, key=lambda c: c.corner_id)
            coords = [(c.lon, c.lat, 0) for c in sorted_corners]
            coords.append(coords[0])  # Close polygon
            
            polygon = self.building_folder.newpolygon(
                name="Building Outline",
                outerboundaryis=coords
            )
            polygon.style.polystyle.fill = 0
            polygon.style.polystyle.outline = 1
            polygon.style.linestyle.color = simplekml.Color.yellow
            polygon.style.linestyle.width = 3
    
    def export_kml(self):
        """Export data to KML file"""
        filename = f"warg_competition_{datetime.now().strftime('%Y%m%d_%H%M%S')}.kml"
        try:
            self.kml.save(filename)
            messagebox.showinfo("Export Successful", f"Data exported to {filename}")
            
            # Open folder containing the file
            folder_path = os.path.abspath(os.path.dirname(filename))
            if os.name == 'nt':  # Windows
                subprocess.Popen(f'explorer "{folder_path}"')
            elif os.name == 'posix':  # Linux/Mac
                subprocess.Popen(['xdg-open', folder_path])
                
        except Exception as e:
            messagebox.showerror("Export Failed", f"Failed to export KML: {e}")
    
    def export_json(self):
        """Export data to JSON file"""
        filename = f"warg_competition_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        data = {
            'targets': [
                {
                    'id': t.id, 'color': t.color, 'location_type': t.location_type,
                    'lat': t.lat, 'lon': t.lon, 'alt': t.alt,
                    'timestamp': t.timestamp.isoformat()
                } for t in self.targets
            ],
            'building_corners': [
                {
                    'corner_id': c.corner_id, 'lat': c.lat, 'lon': c.lon,
                    'timestamp': c.timestamp.isoformat()
                } for c in self.building_corners
            ],
            'export_timestamp': datetime.now().isoformat(),
            'total_targets': len(self.targets),
            'building_complete': len(self.building_corners) >= 4
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Export Successful", f"Data exported to {filename}")
        except Exception as e:
            messagebox.showerror("Export Failed", f"Failed to export JSON: {e}")
    
    def stop_program(self):
        """Stop the program and cleanup"""
        self.stop_flag.set()
        
        if self.mavlink_connection:
            self.mavlink_connection.close()
        
        # Auto-export on exit
        if self.targets or self.building_corners:
            self.export_kml()
        
        self.root.quit()
        self.root.destroy()
    
    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points in meters"""
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def run(self):
        """Start the ground station application"""
        self.start_mavlink_listener()
        self.root.mainloop()


def main():
    """Main entry point"""
    print("WARG Drone Competition 2025 - Ground Station")
    print("Enhanced for colored target detection and building mapping")
    print("Connecting to MAVLink on tcp:127.0.0.1:14550...\n")
    
    ground_station = GroundStation()
    ground_station.run()


if __name__ == "__main__":
    main()

