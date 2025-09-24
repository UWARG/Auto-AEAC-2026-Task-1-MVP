"""
Configuration file for WARG Drone Competition 2025
Contains all constants, thresholds, and settings for the autonomous drone system
"""

import numpy as np

# === CAMERA CONSTANTS ===
CAMERA_EXPOSURE_TIME = 15
CAMERA_ANALOG_GAIN = 16.0
CAMERA_RESOLUTION = (1920, 1080)
DOWNWARD_CAMERA_ID = 0
FORWARD_CAMERA_ID = 1

# === DETECTION CONSTANTS ===
LOCKED_RADIUS = 40  # max radius to lock target (pixels)
PX_TO_MS = 0.00002  # pixel-to-velocity scale factor
MIN_TARGET_AREA = 500  # minimum area for target detection (pixels)
MAX_TARGET_AREA = 50000  # maximum area for target detection (pixels)

# === RC CHANNEL CONSTANTS ===
RC_TARGET_DETECTION_CHANNEL = 7  # Channel for target detection mode
RC_CORNER_MAPPING_CHANNEL = 8  # Channel for building corner mapping
RC_THRESHOLD = 1200  # Threshold for RC channel activation

# === TARGET COLORS (HSV ranges) ===
TARGET_COLORS = {
    "black": {"lower": np.array([0, 0, 0]), "upper": np.array([179, 255, 50])},
    "white": {"lower": np.array([0, 0, 200]), "upper": np.array([179, 30, 255])},
    "red": {"lower": np.array([0, 120, 70]), "upper": np.array([10, 255, 255])},
    "red_upper": {  # Red wraps around in HSV
        "lower": np.array([170, 120, 70]),
        "upper": np.array([179, 255, 255]),
    },
    "yellow": {"lower": np.array([15, 120, 70]), "upper": np.array([35, 255, 255])},
    "blue": {"lower": np.array([100, 120, 70]), "upper": np.array([130, 255, 255])},
    "green": {"lower": np.array([40, 120, 70]), "upper": np.array([80, 255, 255])},
}

# === BUILDING MAPPING ===
MIN_BUILDING_SIZE = 10.0  # minimum building dimension in meters
MAX_BUILDING_SIZE = 200.0  # maximum building dimension in meters
CORNER_VALIDATION_TOLERANCE = 5.0  # tolerance for rectangular validation (meters)

# === MAVLINK CONSTANTS ===
MAVLINK_PORT = "/dev/ttyAMA0"
MAVLINK_BAUD = 57600
MAVLINK_SOURCE_COMPONENT = 191
MAVLINK_SOURCE_SYSTEM = 1

# === WALL TARGET ESTIMATION ===
DRONE_CAMERA_HEIGHT_OFFSET = 0.1  # height offset of camera from drone center (meters)
WALL_HEIGHT_ESTIMATE = 3.0  # estimated wall height (meters)
GPS_ACCURACY_BUFFER = 2.0  # buffer for GPS accuracy (meters)

# === DISPLAY CONSTANTS ===
FONT_SCALE = 1.0
FONT_THICKNESS = 2
TEXT_COLOR = (0, 255, 0)  # Green in BGR
TARGET_LOCKED_COLOR = (0, 0, 255)  # Red in BGR
CROSSHAIR_COLOR = (255, 255, 255)  # White in BGR

# === LOGGING ===
LOG_LEVEL = "INFO"
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
