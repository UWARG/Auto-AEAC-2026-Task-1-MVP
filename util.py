"""
Utility classes and constants for drone communication and data structures.

This module provides core data structures and enums used throughout the drone control system,
including coordinate representation, MAVLink message types, and RC channel data.
"""

from enum import Enum

UINT16_MAX = 65535

# MAVLink communication constants
AIRSIDE_COMPONENT_ID = 191
MAVLINK_TCP_HOST = "127.0.0.1"
MAVLINK_TCP_PORT = 14550
MAVLINK_RECEIVE_TIMEOUT_SEC = 1

# Geometric calculation constants
ALTITUDE_TOLERANCE_M = 0.5  # Tolerance for ground/roof level detection (meters)


class MavlinkMessageType(Enum):
    """MAVLink message types used in drone communication"""

    GLOBAL_POSITION_INT = "GLOBAL_POSITION_INT"
    RC_CHANNELS = "RC_CHANNELS"


class Coordinate:
    """Represents a geographic coordinate with latitude, longitude, and altitude."""

    def __init__(self, lat: float, lon: float, alt: float = 0.0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def __str__(self):
        return f"({self.lat}, {self.lon}, {self.alt})"

    def __repr__(self):
        return f"Coordinate(lat={self.lat}, lon={self.lon}, alt={self.alt})"


class RCChannel:
    """Represents a single RC channel with raw value and activity status."""

    def __init__(self, channel: int, raw: int = 0, is_active: bool = False):
        self.channel = channel
        self.raw = raw
        self.is_active = is_active

    def __str__(self):
        return f"({self.channel}, {self.raw}, {self.is_active})"

    def __repr__(self):
        return f"RCChannel(channel={self.channel}, raw={self.raw}, is_active={self.is_active})"


class Vector3d:
    """Represents a 3D vector with x, y, and z components."""

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z})"


class Colour:
    def __init__(
        self,
        name: str,
        lower_hsv: tuple[int, int, int],
        upper_hsv: tuple[int, int, int],
    ):
        self.name = name
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv

    def __str__(self):
        return f"({self.name}, {self.lower_hsv}, {self.upper_hsv})"

    def __repr__(self):
        return f"Colour(name={self.name}, lower_hsv={self.lower_hsv}, upper_hsv={self.upper_hsv})"


class Colours(Enum):
    # HSV ranges tightened for more precise detection
    # H (Hue): 0-180, S (Saturation): 0-255, V (Value): 0-255
    RED = Colour("Red", (0, 120, 120), (10, 255, 255))
    RED2 = Colour("Red", (168, 120, 120), (180, 255, 255))
    GREEN = Colour("Green", (40, 120, 120), (80, 255, 255))
    BLUE = Colour("Blue", (93, 120, 120), (120, 255, 255))
    YELLOW = Colour("Yellow", (26, 120, 120), (36, 255, 255))
    WHITE = Colour("White", (0, 0, 225), (255, 30, 255))
    BLACK = Colour("Black", (0, 0, 0), (255, 255, 30))
    # WHITE = Colour("White", (0, 0, 200), (180, 255, 255))
