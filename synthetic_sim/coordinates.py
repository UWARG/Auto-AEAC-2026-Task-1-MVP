"""
Coordinate transformation system for synthetic simulation.

This module handles conversions between:
- GPS coordinates (lat, lon, alt)
- Local Cartesian coordinates (x_north, y_east, z_up) in meters
- Alignment of simulated world to SITL's actual GPS location
"""

import numpy as np
from typing import Tuple
from dataclasses import dataclass


@dataclass
class GPSCoord:
    """GPS coordinate (latitude, longitude, altitude)."""
    lat: float  # degrees
    lon: float  # degrees
    alt: float  # meters above sea level


@dataclass
class LocalCoord:
    """Local Cartesian coordinate (meters from origin)."""
    x: float  # meters North
    y: float  # meters East
    z: float  # meters Up


class CoordinateTransform:
    """
    Handles coordinate transformations between GPS and local Cartesian.

    Usage:
        # Start with relative world coordinates
        transform = CoordinateTransform()

        # When SITL starts, lock a reference point to its GPS
        sitl_gps = GPSCoord(43.473146, -80.540103, 100.0)
        transform.set_origin(sitl_gps, LocalCoord(0, 0, 0))

        # Now convert between systems
        world_point = LocalCoord(10, 5, 2)  # 10m north, 5m east, 2m up
        gps_point = transform.local_to_gps(world_point)
    """

    # Earth radius in meters (WGS84 approximation)
    EARTH_RADIUS = 6378137.0

    def __init__(self):
        """Initialize with no origin set."""
        self.origin_gps: GPSCoord | None = None
        self.origin_local: LocalCoord | None = None
        self.meters_per_deg_lat: float | None = None
        self.meters_per_deg_lon: float | None = None

    def set_origin(self, gps_origin: GPSCoord, local_origin: LocalCoord = None):
        """
        Set the origin point that aligns GPS and local coordinates.

        Args:
            gps_origin: GPS coordinates of the origin
            local_origin: Local coordinates of the origin (default: 0,0,0)
        """
        self.origin_gps = gps_origin
        self.origin_local = local_origin or LocalCoord(0, 0, 0)

        # Calculate meters per degree at this latitude
        lat_rad = np.radians(gps_origin.lat)

        # Meters per degree latitude (constant)
        self.meters_per_deg_lat = 111319.9

        # Meters per degree longitude (varies with latitude)
        self.meters_per_deg_lon = 111319.9 * np.cos(lat_rad)

    def is_initialized(self) -> bool:
        """Check if origin has been set."""
        return self.origin_gps is not None

    def local_to_gps(self, local: LocalCoord) -> GPSCoord:
        """
        Convert local Cartesian coordinates to GPS.

        Args:
            local: Local coordinates (x=North, y=East, z=Up in meters)

        Returns:
            GPS coordinates

        Raises:
            ValueError: If origin not set
        """
        if not self.is_initialized():
            raise ValueError("Origin not set. Call set_origin() first.")

        # Calculate offset from origin in meters
        dx = local.x - self.origin_local.x  # North offset
        dy = local.y - self.origin_local.y  # East offset
        dz = local.z - self.origin_local.z  # Up offset

        # Convert to GPS offset
        dlat = dx / self.meters_per_deg_lat
        dlon = dy / self.meters_per_deg_lon

        return GPSCoord(
            lat=self.origin_gps.lat + dlat,
            lon=self.origin_gps.lon + dlon,
            alt=self.origin_gps.alt + dz
        )

    def gps_to_local(self, gps: GPSCoord) -> LocalCoord:
        """
        Convert GPS coordinates to local Cartesian.

        Args:
            gps: GPS coordinates

        Returns:
            Local coordinates (x=North, y=East, z=Up in meters)

        Raises:
            ValueError: If origin not set
        """
        if not self.is_initialized():
            raise ValueError("Origin not set. Call set_origin() first.")

        # Calculate GPS offset from origin
        dlat = gps.lat - self.origin_gps.lat
        dlon = gps.lon - self.origin_gps.lon
        dalt = gps.alt - self.origin_gps.alt

        # Convert to meters
        dx = dlat * self.meters_per_deg_lat  # North
        dy = dlon * self.meters_per_deg_lon  # East
        dz = dalt  # Up

        return LocalCoord(
            x=self.origin_local.x + dx,
            y=self.origin_local.y + dy,
            z=self.origin_local.z + dz
        )

    def get_distance(self, local1: LocalCoord, local2: LocalCoord) -> float:
        """Calculate Euclidean distance between two local points."""
        dx = local2.x - local1.x
        dy = local2.y - local1.y
        dz = local2.z - local1.z
        return np.sqrt(dx**2 + dy**2 + dz**2)


# Convenience functions for quick conversions
def haversine_distance(gps1: GPSCoord, gps2: GPSCoord) -> float:
    """
    Calculate great-circle distance between two GPS points (meters).
    Uses Haversine formula.
    """
    lat1, lon1 = np.radians(gps1.lat), np.radians(gps1.lon)
    lat2, lon2 = np.radians(gps2.lat), np.radians(gps2.lon)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))

    distance = 6378137.0 * c  # Earth radius in meters

    # Add altitude difference
    dalt = gps2.alt - gps1.alt
    return np.sqrt(distance**2 + dalt**2)
