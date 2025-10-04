"""
Camera module for handling Picamera2 operations.

This module provides a Camera class that handles Raspberry Pi Camera Module 2
operations, including initialization and frame capture.
"""

from picamera2 import Picamera2
import numpy as np
import cv2
import logging
import time

from util import Colour, Colours


class Camera:
    """
    Handles Raspberry Pi Camera Module 2 operations
    """

    # Default camera settings optimized for IR beacon detection
    DEFAULT_EXPOSURE_TIME = 15  # microseconds - very fast to reduce motion blur
    DEFAULT_ANALOGUE_GAIN = 16.0  # high gain to amplify IR beacon signal
    DEFAULT_AUTO_EXPOSURE = False  # disable to maintain consistent brightness

    def __init__(
        self,
        camera_index: int = 0,
        exposure_time: int = DEFAULT_EXPOSURE_TIME,
        analogue_gain: float = DEFAULT_ANALOGUE_GAIN,
        auto_exposure: bool = DEFAULT_AUTO_EXPOSURE,
    ) -> None:
        """
        Initialize and configure the Raspberry Pi Camera Module 2.

        Args:
            camera_index: Camera index (0 for first camera, 1 for second camera)
            exposure_time: Exposure time in microseconds
            analogue_gain: Analogue gain value
            auto_exposure: Enable/disable auto exposure
        """
        self.camera_index = camera_index
        self.exposure_time = exposure_time
        self.analogue_gain = analogue_gain
        self.auto_exposure_enabled = auto_exposure
        self.picam2: Picamera2 | None = None

        # Retry camera initialization
        while not self._initialize_camera():
            logging.error(
                f"Failed to initialize camera {camera_index}, retrying in 1 second..."
            )
            time.sleep(1)

    def _initialize_camera(self) -> bool:
        """
        Initialize camera hardware and apply settings.
        """
        try:
            # Initialize Raspberry Pi Camera Module 2 with specified camera index
            self.picam2 = Picamera2(self.camera_index)

            # Configure camera with preview mode (faster than still mode)
            self.picam2.configure(self.picam2.create_preview_configuration())

            # Apply IR detection settings
            self._apply_camera_settings()

            # Start camera capture
            self.picam2.start()

            logging.info(f"Camera {self.camera_index} initialized successfully")
            return True

        except Exception as e:
            logging.error(f"Failed to initialize camera {self.camera_index}: {e}")
            self.picam2 = None
            return False

    def _apply_camera_settings(self) -> None:
        """
        Apply camera control settings.
        """
        if self.picam2 is None:
            return

        try:
            self.picam2.set_controls(
                {
                    "AeEnable": self.auto_exposure_enabled,
                    "ExposureTime": self.exposure_time,
                    "AnalogueGain": self.analogue_gain,
                }
            )
        except Exception as e:
            logging.error(f"Failed to apply camera settings: {e}")

    def capture_frame(self) -> np.ndarray | None:
        """
        Capture a single frame from the camera.
        """
        if self.picam2 is None:
            logging.warning("Camera not initialized, cannot capture frame")
            return None

        try:
            return self.picam2.capture_array()
        except Exception as e:
            logging.error(f"Failed to capture frame: {e}")
            return None

    def stop(self) -> None:
        """
        Stop camera capture and release resources.

        Should be called when done using the camera to properly clean up.
        """
        if self.picam2 is None:
            return

        try:
            self.picam2.stop()
            logging.info("Camera stopped successfully")
        except Exception as e:
            logging.error(f"Error stopping camera: {e}")

    def is_initialized(self) -> bool:
        """
        Check if camera is initialized.
        """
        return self.picam2 is not None

    def __enter__(self) -> "Camera":
        """Context manager entry - allows use with 'with' statement."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        """Context manager exit - ensures camera is stopped."""
        self.stop()
        return False

    def colour_in_frame(self, frame: np.ndarray) -> Colour | None:
        """
        Check if a colour is in the frame.
        """
        if frame is None or frame.size == 0:
            logging.warning("Invalid frame provided to colour_in_frame method")
            return None

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for colour in [c.value for c in Colours]:
            mask = cv2.inRange(frame_hsv, colour.lower_hsv, colour.upper_hsv)
            if cv2.countNonZero(mask) > 0:
                return colour
        return None

    def center_of_target_in_frame(
        self,
        frame: np.ndarray,
        colour: Colour = Colours.WHITE.value,
    ) -> tuple[int, int] | None:
        """
        Detect target in frame and return its center coordinates.
        """
        if frame is None or frame.size == 0:
            logging.warning(
                "Invalid frame provided to center_of_target_in_frame method"
            )
            return None

        try:
            # Convert frame from BGR to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create binary mask of pixels within HSV thresholds
            mask = cv2.inRange(hsv, colour.lower_hsv, colour.upper_hsv)

            # Find contours in the binary mask
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if not contours:
                logging.warning("No contours found in frame")
                return None

            # Find the largest contour (assumed to be the target)
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the centroid using image moments
            moments = cv2.moments(largest_contour)

            # Avoid division by zero
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                logging.info(f"Target detected at ({center_x}, {center_y})")
                return (center_x, center_y)
            else:
                logging.warning("Contour found but moments calculation failed")
                return None

        except Exception as e:
            logging.error(f"Error in center_of_target_in_frame: {e}")
            return None
