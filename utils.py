"""
Utility functions for the drone competition system
Common functions used across multiple modules
"""

import math
import logging
import time
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass
import cv2

# import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class GPS:
    """GPS coordinate representation"""

    lat: float
    lon: float
    alt: float = 0.0

    def to_dict(self) -> Dict:
        return {"lat": self.lat, "lon": self.lon, "alt": self.alt}

    def __str__(self) -> str:
        return f"GPS({self.lat:.7f}, {self.lon:.7f}, {self.alt:.1f}m)"


@dataclass
class Velocity:
    """Velocity vector in NED frame"""

    x: float  # North (m/s)
    y: float  # East (m/s)
    z: float  # Down (m/s)

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


class MovingAverage:
    """Simple moving average filter"""

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.values = []

    def update(self, value: float) -> float:
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

    def reset(self):
        self.values.clear()


class RateLimiter:
    """Rate limiter for controlling message/action frequency"""

    def __init__(self, max_rate_hz: float):
        self.min_interval = 1.0 / max_rate_hz
        self.last_time = 0.0

    def is_ready(self) -> bool:
        current_time = time.time()
        if current_time - self.last_time >= self.min_interval:
            self.last_time = current_time
            return True
        return False


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great circle distance between two points on Earth
    Returns distance in meters
    """
    R = 6371000  # Earth radius in meters

    # Convert to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate bearing from point 1 to point 2
    Returns bearing in degrees (0-360, where 0 is North)
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)

    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
        lat2_rad
    ) * math.cos(dlon_rad)

    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)

    return (bearing_deg + 360) % 360  # Normalize to 0-360


def gps_offset(
    lat: float, lon: float, offset_north: float, offset_east: float
) -> Tuple[float, float]:
    """
    Calculate new GPS coordinates given offsets in meters

    Args:
        lat, lon: Original coordinates
        offset_north: Offset in meters (positive = north)
        offset_east: Offset in meters (positive = east)

    Returns:
        (new_lat, new_lon)
    """
    R = 6371000  # Earth radius in meters

    # Calculate new latitude
    new_lat = lat + math.degrees(offset_north / R)

    # Calculate new longitude (accounting for latitude)
    new_lon = lon + math.degrees(offset_east / (R * math.cos(math.radians(lat))))

    return new_lat, new_lon


def pixel_to_velocity(
    offset_x: int, offset_y: int, scale_factor: float = 0.00002
) -> Velocity:
    """
    Convert pixel offsets to velocity commands in NED frame

    Args:
        offset_x: Horizontal pixel offset (positive = right)
        offset_y: Vertical pixel offset (positive = down)
        scale_factor: Conversion factor from pixels to m/s

    Returns:
        Velocity command in NED frame
    """
    # Convert image coordinates to NED velocity commands
    vel_north = -offset_y * scale_factor  # Image Y down -> NED North
    vel_east = offset_x * scale_factor  # Image X right -> NED East
    vel_down = 0.0  # No vertical velocity for target tracking

    return Velocity(vel_north, vel_east, vel_down)


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate Euclidean distance between two points"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def angle_difference(angle1: float, angle2: float) -> float:
    """
    Calculate smallest angle difference between two angles (in degrees)
    Returns value between -180 and 180
    """
    diff = angle2 - angle1
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


def constrain_value(value: float, min_val: float, max_val: float) -> float:
    """Constrain value within min/max bounds"""
    return max(min_val, min(max_val, value))


def format_gps_string(lat: float, lon: float, precision: int = 7) -> str:
    """Format GPS coordinates as string"""
    return f"{lat:.{precision}f}, {lon:.{precision}f}"


def parse_gps_string(gps_str: str) -> Optional[Tuple[float, float]]:
    """Parse GPS string back to coordinates"""
    try:
        parts = gps_str.split(",")
        if len(parts) == 2:
            lat = float(parts[0].strip())
            lon = float(parts[1].strip())
            return lat, lon
    except Exception as e:
        logger.error(f"Error parsing GPS string '{gps_str}': {e}")
    return None


def setup_logging(level: str = "INFO", format_str: str = None) -> logging.Logger:
    """Setup logging configuration"""
    if format_str is None:
        format_str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format=format_str,
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    return logging.getLogger(__name__)


def validate_gps_coordinates(lat: float, lon: float) -> bool:
    """Validate GPS coordinates are within reasonable bounds"""
    return (-90.0 <= lat <= 90.0) and (-180.0 <= lon <= 180.0)


def calculate_image_center(frame) -> Tuple[int, int]:
    """Calculate center coordinates of image frame"""
    if frame is None:
        return 0, 0

    height, width = frame.shape[:2]
    return width // 2, height // 2


def add_text_overlay(
    frame,
    text: str,
    position: Tuple[int, int],
    color: Tuple[int, int, int] = (0, 255, 0),
    font_scale: float = 0.7,
    thickness: int = 2,
):
    """Add text overlay to frame"""
    if frame is None:
        return frame

    cv2.putText(
        frame,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        thickness,
        cv2.LINE_AA,
    )
    return frame


def draw_crosshairs(
    frame,
    center_x: int,
    center_y: int,
    size: int = 50,
    color: Tuple[int, int, int] = (255, 255, 255),
    thickness: int = 2,
):
    """Draw crosshairs on frame"""
    if frame is None:
        return frame

    # Draw horizontal line
    cv2.line(
        frame,
        (center_x - size, center_y),
        (center_x + size, center_y),
        color,
        thickness,
    )

    # Draw vertical line
    cv2.line(
        frame,
        (center_x, center_y - size),
        (center_x, center_y + size),
        color,
        thickness,
    )

    # Draw center dot
    cv2.circle(frame, (center_x, center_y), 3, color, -1)

    return frame


def calculate_fps(frame_times: List[float], max_samples: int = 30) -> float:
    """Calculate FPS from frame timestamps"""
    if len(frame_times) < 2:
        return 0.0

    # Keep only recent samples
    recent_times = frame_times[-max_samples:]

    # Calculate average time between frames
    time_diffs = [
        recent_times[i] - recent_times[i - 1] for i in range(1, len(recent_times))
    ]

    if not time_diffs:
        return 0.0

    avg_frame_time = sum(time_diffs) / len(time_diffs)

    if avg_frame_time <= 0:
        return 0.0

    return 1.0 / avg_frame_time


def create_status_text(
    gps: Optional[GPS], target_info: str = "", camera_name: str = "", fps: float = 0.0
) -> List[str]:
    """Create status text lines for display"""
    status_lines = []

    # GPS information
    if gps:
        status_lines.append(f"GPS: {gps.lat:.7f}, {gps.lon:.7f}")
        if gps.alt > 0:
            status_lines.append(f"Alt: {gps.alt:.1f}m")
    else:
        status_lines.append("GPS: Waiting for fix...")

    # Camera information
    if camera_name:
        status_lines.append(f"Camera: {camera_name}")

    # Target information
    if target_info:
        status_lines.append(target_info)

    # FPS information
    if fps > 0:
        status_lines.append(f"FPS: {fps:.1f}")

    return status_lines


def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    """Safely divide two numbers, returning default if denominator is zero"""
    if abs(denominator) < 1e-10:  # Very small number check
        return default
    return numerator / denominator
