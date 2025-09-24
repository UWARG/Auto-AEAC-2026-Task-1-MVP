"""
Building Mapper Module
Handles building corner detection, mapping, and geometric calculations
"""

import math
import logging
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from config import MIN_BUILDING_SIZE, MAX_BUILDING_SIZE, CORNER_VALIDATION_TOLERANCE

logger = logging.getLogger(__name__)

@dataclass
class Corner:
    """Represents a building corner with GPS coordinates"""
    lat: float
    lon: float
    corner_id: int  # 0=NW, 1=NE, 2=SE, 3=SW
    
    def to_dict(self) -> Dict:
        return {
            'lat': self.lat,
            'lon': self.lon,
            'corner_id': self.corner_id
        }

@dataclass
class BuildingDimensions:
    """Represents building dimensions and orientation"""
    width: float  # East-West dimension (meters)
    height: float  # North-South dimension (meters)
    center_lat: float
    center_lon: float
    bearing: float  # Building orientation in degrees (0 = North)

class BuildingMapper:
    """
    Manages building corner mapping and geometric calculations
    """
    
    def __init__(self):
        self.corners: List[Corner] = []
        self.building_dimensions: Optional[BuildingDimensions] = None
        self.is_complete = False
    
    def add_corner(self, lat: float, lon: float) -> bool:
        """
        Add a new corner to the building map
        Returns True if corner was added successfully
        """
        if len(self.corners) >= 4:
            logger.warning("Building already has 4 corners, cannot add more")
            return False
        
        corner_id = len(self.corners)
        new_corner = Corner(lat, lon, corner_id)
        self.corners.append(new_corner)
        
        logger.info(f"Added corner {corner_id + 1}/4: {lat:.7f}, {lon:.7f}")
        
        # If we have all 4 corners, calculate building dimensions
        if len(self.corners) == 4:
            self._calculate_building_dimensions()
            
        return True
    
    def reset_mapping(self):
        """Reset the building mapping"""
        self.corners.clear()
        self.building_dimensions = None
        self.is_complete = False
        logger.info("Building mapping reset")
    
    def _calculate_building_dimensions(self) -> bool:
        """
        Calculate building dimensions from the 4 corners
        Assumes corners are provided in order: NW, NE, SE, SW
        """
        if len(self.corners) != 4:
            logger.error("Need exactly 4 corners to calculate dimensions")
            return False
        
        try:
            # Calculate center point
            center_lat = sum(corner.lat for corner in self.corners) / 4
            center_lon = sum(corner.lon for corner in self.corners) / 4
            
            # Calculate distances between adjacent corners
            distances = []
            for i in range(4):
                corner1 = self.corners[i]
                corner2 = self.corners[(i + 1) % 4]  # Next corner, wrapping around
                distance = self._haversine_distance(
                    corner1.lat, corner1.lon, 
                    corner2.lat, corner2.lon
                )
                distances.append(distance)
            
            # For a rectangle, opposite sides should be equal
            width = (distances[0] + distances[2]) / 2  # North and South sides
            height = (distances[1] + distances[3]) / 2  # East and West sides
            
            # Validate building dimensions
            if not self._validate_building_dimensions(width, height, distances):
                logger.error("Invalid building dimensions detected")
                return False
            
            # Calculate building orientation (bearing of north side)
            north_corner1 = self.corners[0]  # NW
            north_corner2 = self.corners[1]  # NE
            bearing = self._calculate_bearing(
                north_corner1.lat, north_corner1.lon,
                north_corner2.lat, north_corner2.lon
            )
            
            self.building_dimensions = BuildingDimensions(
                width=width,
                height=height,
                center_lat=center_lat,
                center_lon=center_lon,
                bearing=bearing
            )
            
            self.is_complete = True
            logger.info(f"Building mapped successfully: {width:.1f}m x {height:.1f}m, "
                       f"center: {center_lat:.7f}, {center_lon:.7f}, "
                       f"bearing: {bearing:.1f}°")
            
            return True
            
        except Exception as e:
            logger.error(f"Error calculating building dimensions: {e}")
            return False
    
    def _validate_building_dimensions(self, width: float, height: float, 
                                    distances: List[float]) -> bool:
        """Validate that the building dimensions are reasonable"""
        
        # Check size limits
        if width < MIN_BUILDING_SIZE or height < MIN_BUILDING_SIZE:
            logger.error(f"Building too small: {width:.1f}m x {height:.1f}m")
            return False
            
        if width > MAX_BUILDING_SIZE or height > MAX_BUILDING_SIZE:
            logger.error(f"Building too large: {width:.1f}m x {height:.1f}m")
            return False
        
        # Check that opposite sides are approximately equal (rectangular validation)
        side1_diff = abs(distances[0] - distances[2])  # North vs South
        side2_diff = abs(distances[1] - distances[3])  # East vs West
        
        if side1_diff > CORNER_VALIDATION_TOLERANCE or side2_diff > CORNER_VALIDATION_TOLERANCE:
            logger.error(f"Building not rectangular: side differences {side1_diff:.1f}m, {side2_diff:.1f}m")
            return False
        
        return True
    
    def estimate_wall_target_position(self, drone_lat: float, drone_lon: float, 
                                    drone_alt: float, wall_side: str, 
                                    relative_position: Tuple[float, float]) -> Optional[Tuple[float, float, float]]:
        """
        Estimate GPS position of a wall target
        
        Args:
            drone_lat, drone_lon: Current drone position
            drone_alt: Current drone altitude
            wall_side: Which wall ('north', 'south', 'east', 'west')
            relative_position: (horizontal_offset, vertical_offset) in meters from wall center
            
        Returns:
            (target_lat, target_lon, target_alt) or None if estimation fails
        """
        if not self.is_complete or not self.building_dimensions:
            logger.error("Building mapping not complete, cannot estimate wall position")
            return None
        
        try:
            building = self.building_dimensions
            
            # Calculate which wall to project to
            wall_corners = self._get_wall_corners(wall_side)
            if not wall_corners:
                return None
            
            # Calculate wall center
            wall_center_lat = (wall_corners[0].lat + wall_corners[1].lat) / 2
            wall_center_lon = (wall_corners[0].lon + wall_corners[1].lon) / 2
            
            # Calculate wall direction vector
            wall_vector = self._calculate_wall_vector(wall_corners[0], wall_corners[1])
            
            # Apply relative position offset
            h_offset, v_offset = relative_position
            
            # Calculate target position on wall
            target_lat = wall_center_lat + (h_offset * wall_vector['lat_per_meter'])
            target_lon = wall_center_lon + (h_offset * wall_vector['lon_per_meter'])
            target_alt = drone_alt + v_offset  # Assuming wall base is at drone altitude
            
            logger.info(f"Estimated wall target position: {target_lat:.7f}, {target_lon:.7f}, {target_alt:.1f}m")
            
            return target_lat, target_lon, target_alt
            
        except Exception as e:
            logger.error(f"Error estimating wall target position: {e}")
            return None
    
    def _get_wall_corners(self, wall_side: str) -> Optional[List[Corner]]:
        """Get the two corners that define a wall"""
        if len(self.corners) != 4:
            return None
            
        wall_map = {
            'north': [0, 1],  # NW to NE
            'east': [1, 2],   # NE to SE  
            'south': [2, 3],  # SE to SW
            'west': [3, 0]    # SW to NW
        }
        
        if wall_side.lower() not in wall_map:
            logger.error(f"Invalid wall side: {wall_side}")
            return None
            
        indices = wall_map[wall_side.lower()]
        return [self.corners[indices[0]], self.corners[indices[1]]]
    
    def _calculate_wall_vector(self, corner1: Corner, corner2: Corner) -> Dict[str, float]:
        """Calculate unit vector along wall direction"""
        distance = self._haversine_distance(corner1.lat, corner1.lon, corner2.lat, corner2.lon)
        
        lat_diff = corner2.lat - corner1.lat
        lon_diff = corner2.lon - corner1.lon
        
        return {
            'lat_per_meter': lat_diff / distance,
            'lon_per_meter': lon_diff / distance
        }
    
    def _haversine_distance(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points in meters"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = (math.sin(dphi / 2) ** 2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
        
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def _calculate_bearing(self, lat1: float, lon1: float, 
                         lat2: float, lon2: float) -> float:
        """Calculate bearing from point 1 to point 2 in degrees"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon_rad = math.radians(lon2 - lon1)
        
        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))
        
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)
        
        return (bearing_deg + 360) % 360  # Normalize to 0-360
    
    def get_building_info(self) -> Dict:
        """Get building information as dictionary"""
        info = {
            'corners': [corner.to_dict() for corner in self.corners],
            'corner_count': len(self.corners),
            'is_complete': self.is_complete
        }
        
        if self.building_dimensions:
            info['dimensions'] = {
                'width': self.building_dimensions.width,
                'height': self.building_dimensions.height,
                'center_lat': self.building_dimensions.center_lat,
                'center_lon': self.building_dimensions.center_lon,
                'bearing': self.building_dimensions.bearing
            }
        
        return info