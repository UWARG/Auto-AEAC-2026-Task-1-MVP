"""
HUD overlay module for camera feeds.

This module provides functions to draw heads-up display elements on camera frames,
including crosshairs, target indicators, status panels, and metrics.
"""

import cv2
import numpy as np
from util import Colour, Vector3d


class HudState:
    """Encapsulates HUD state for a single camera feed."""

    def __init__(self):
        """Initialize HUD state with default values."""
        self.target_center: tuple[int, int] | None = None
        self.target_colour: Colour | None = None
        self.offset_x: float | None = None
        self.offset_y: float | None = None
        self.error: float | None = None
        self.locked: bool = False

    def reset(self) -> None:
        """Reset all HUD state to default values."""
        self.target_center = None
        self.target_colour = None
        self.offset_x = None
        self.offset_y = None
        self.error = None
        self.locked = False

    def reset_target(self) -> None:
        """Reset only target-related state, keeping lock status."""
        self.target_center = None
        self.offset_x = None
        self.offset_y = None
        self.error = None

    def update_target(
        self,
        center: tuple[int, int],
        colour: Colour,
        offset_x: float,
        offset_y: float,
        error: float,
    ) -> None:
        """Update target tracking data."""
        self.target_center = center
        self.target_colour = colour
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.error = error

    def set_locked(self, locked: bool) -> None:
        """Set lock status."""
        self.locked = locked


# Color constants (BGR format for OpenCV)
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (0, 0, 255)
COLOR_CYAN = (255, 255, 0)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

# HUD constants
CROSSHAIR_SIZE = 20
CROSSHAIR_THICKNESS = 2
TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX
TEXT_SCALE = 0.5
TEXT_THICKNESS = 1
TEXT_LINE_HEIGHT = 20


def draw_crosshairs(frame: np.ndarray, color: tuple = COLOR_GREEN) -> None:
    """Draw center crosshairs on the frame."""
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    # Horizontal line
    cv2.line(
        frame,
        (center_x - CROSSHAIR_SIZE, center_y),
        (center_x + CROSSHAIR_SIZE, center_y),
        color,
        CROSSHAIR_THICKNESS,
    )

    # Vertical line
    cv2.line(
        frame,
        (center_x, center_y - CROSSHAIR_SIZE),
        (center_x, center_y + CROSSHAIR_SIZE),
        color,
        CROSSHAIR_THICKNESS,
    )


def draw_error_circle(
    frame: np.ndarray, threshold_px: int, color: tuple = COLOR_YELLOW
) -> None:
    """Draw the error threshold circle at the center of the frame."""
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    cv2.circle(frame, (center_x, center_y), threshold_px, color, 2)


def draw_target_box(
    frame: np.ndarray,
    target_center: tuple[int, int] | None,
    color: tuple = COLOR_CYAN,
    box_size: int = 30,
) -> None:
    """Draw a bounding box around the detected target."""
    if target_center is None:
        return

    cx, cy = target_center

    # Draw box
    cv2.rectangle(
        frame,
        (cx - box_size, cy - box_size),
        (cx + box_size, cy + box_size),
        color,
        2,
    )

    # Draw center dot
    cv2.circle(frame, (cx, cy), 5, color, -1)


def draw_offset_lines(
    frame: np.ndarray,
    target_center: tuple[int, int] | None,
    error: float | None,
    color: tuple = COLOR_CYAN,
) -> None:
    """Draw lines showing offset from center to target."""
    if target_center is None:
        return

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    target_x, target_y = target_center

    # Draw line from center to target
    cv2.line(frame, (center_x, center_y), (target_x, target_y), color, 2)

    # Draw error distance text at midpoint
    if error is not None:
        mid_x = (center_x + target_x) // 2
        mid_y = (center_y + target_y) // 2
        text = f"{error:.1f}px"
        _draw_text_with_background(frame, text, (mid_x + 10, mid_y), color)


def draw_status_panel(
    frame: np.ndarray,
    camera_label: str,
    mode: str,
    corner_count: int | None = None,
    locked: bool = False,
) -> None:
    """Draw status panel in top-left corner."""
    x, y = 10, 20

    # Camera label
    _draw_text_with_background(frame, f"CAMERA: {camera_label}", (x, y), COLOR_WHITE)
    y += TEXT_LINE_HEIGHT

    # Mode
    mode_color = COLOR_GREEN if mode == "BUILDING_RECORD" else COLOR_CYAN
    _draw_text_with_background(frame, f"MODE: {mode}", (x, y), mode_color)
    y += TEXT_LINE_HEIGHT

    # Corner count (if in building record mode)
    if corner_count is not None:
        _draw_text_with_background(
            frame, f"CORNERS: {corner_count}/4", (x, y), COLOR_WHITE
        )
        y += TEXT_LINE_HEIGHT

    # Lock status
    if locked:
        _draw_text_with_background(frame, "STATUS: LOCKED", (x, y), COLOR_YELLOW)
    else:
        _draw_text_with_background(frame, "STATUS: TRACKING", (x, y), COLOR_GREEN)


def draw_metrics_panel(
    frame: np.ndarray,
    offset_x: float | None = None,
    offset_y: float | None = None,
    error: float | None = None,
    colour: Colour | None = None,
) -> None:
    """Draw metrics panel in bottom-left corner."""
    height = frame.shape[0]
    x, y = 10, height - 120

    # Offset
    if offset_x is not None and offset_y is not None:
        _draw_text_with_background(
            frame, f"OFFSET X: {offset_x:.1f}px", (x, y), COLOR_WHITE
        )
        y += TEXT_LINE_HEIGHT
        _draw_text_with_background(
            frame, f"OFFSET Y: {offset_y:.1f}px", (x, y), COLOR_WHITE
        )
        y += TEXT_LINE_HEIGHT

    # Error
    if error is not None:
        error_color = COLOR_GREEN if error <= 40 else COLOR_RED
        _draw_text_with_background(frame, f"ERROR: {error:.1f}px", (x, y), error_color)
        y += TEXT_LINE_HEIGHT


    # Color
    if colour is not None:
        _draw_text_with_background(
            frame, f"COLOR: {colour.name}", (x, y), COLOR_YELLOW
        )


def overlay_hud(
    frame: np.ndarray,
    camera_label: str,
    mode: str,
    hud_state: HudState,
    corner_count: int | None = None,
    error_threshold_px: int = 40,
) -> np.ndarray:
    """
    Apply full HUD overlay to a frame.

    Args:
        frame: Input frame to draw on
        camera_label: "DOWN" or "FORWARD"
        mode: "BUILDING_RECORD" or "TARGET_DETECT"
        hud_state: HudState object containing tracking data
        corner_count: Number of corners recorded (if in building mode)
        error_threshold_px: Lock threshold radius

    Returns:
        Frame with HUD overlay applied
    """
    # Create a copy to avoid modifying original
    display_frame = frame.copy()

    # Draw center reference
    crosshair_color = COLOR_YELLOW if hud_state.locked else COLOR_GREEN
    draw_crosshairs(display_frame, crosshair_color)

    # Draw error threshold circle
    draw_error_circle(display_frame, error_threshold_px)

    # Draw target indicators
    if hud_state.target_center is not None:
        draw_target_box(display_frame, hud_state.target_center, COLOR_CYAN)
        draw_offset_lines(
            display_frame, hud_state.target_center, hud_state.error, COLOR_CYAN
        )

    # Draw status panel
    draw_status_panel(
        display_frame, camera_label, mode, corner_count, hud_state.locked
    )

    # Draw metrics panel
    draw_metrics_panel(
        display_frame,
        hud_state.offset_x,
        hud_state.offset_y,
        hud_state.error,
        hud_state.target_colour,
    )

    return display_frame


def _draw_text_with_background(
    frame: np.ndarray, text: str, position: tuple[int, int], color: tuple
) -> None:
    """Draw text with a semi-transparent black background for readability."""
    x, y = position

    # Get text size
    (text_width, text_height), baseline = cv2.getTextSize(
        text, TEXT_FONT, TEXT_SCALE, TEXT_THICKNESS
    )

    # Draw background rectangle
    cv2.rectangle(
        frame,
        (x - 2, y - text_height - 2),
        (x + text_width + 2, y + baseline + 2),
        COLOR_BLACK,
        -1,
    )

    # Draw text
    cv2.putText(frame, text, (x, y), TEXT_FONT, TEXT_SCALE, color, TEXT_THICKNESS)
