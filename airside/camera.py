"""
Camera module for handling Picamera2 operations.

This module provides a Camera class that handles Raspberry Pi Camera Module 2
operations, including initialization and frame capture.
"""

from warg_common.camera import BaseCameraDevice, create_camera, CameraOption
import numpy as np
import cv2
import logging
import time
from typing import Literal

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
        mode: Literal["rpi", "webcam"] = "rpi",
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
        self.mode = mode
        self._camera: BaseCameraDevice | None = None
        print("mode", self.mode)

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
        # Initialize Raspberry Pi Camera Module 2 with specified camera index
        if self.mode == "rpi":
            print("Attempting to initialize rpi camera")
            from warg_common.camera.camera_picamera2 import ConfigPiCamera2

            config = ConfigPiCamera2(
                exposure_time=self.exposure_time, analogue_gain=self.analogue_gain
            )
            status, obj = create_camera(CameraOption.PICAM2, 500, 500, config)
            self._camera = obj
            if status:
                logging.info(
                    f"picam2 camera {self.camera_index} initialized successfully"
                )
            else:
                logging.error(f"picam2 camera {self.camera_index} failed to initialize")

            return status
        else:
            print("Attempting to annitialize webcam camera")
            from warg_common.camera.camera_opencv import ConfigOpenCV

            # index0 = webcam
            config = ConfigOpenCV(device_index=0)
            status, obj = create_camera(
                camera_option=CameraOption.OPENCV, width=640, height=480, config=config
            )
            self._camera = obj
            if status:
                logging.info(
                    f"opencv camera {self.camera_index} initialized successfully"
                )
            else:
                logging.error(f"opencv camera {self.camera_index} failed to initialize")
            return status

    def capture_frame(self) -> np.ndarray | None:
        """
        Capture a single frame from the camera.
        """
        if self._camera is not None:
            status, frame = self._camera.run()
            if not status:
                logging.warning(
                    "Failed frame capture due to camera implementation or timeout"
                )
            return frame
        logging.warning("Attempted to capture frame with initialized camera device")
        return None

    def is_initialized(self) -> bool:
        """
        Check if camera is initialized.
        """
        return self._camera is not None

    def __enter__(self) -> "Camera":
        """Context manager entry - allows use with 'with' statement."""
        return self

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
