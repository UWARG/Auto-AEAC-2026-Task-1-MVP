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

from warg_common.simulator import SimCamera

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
        mode: Literal["rpi", "webcam", "sim"] = "rpi",
        mav_comm=None,
    ) -> None:
        """
        Initialize and configure the Raspberry Pi Camera Module 2.

        Args:
            camera_index: Camera index (0 for first camera, 1 for second camera)
            exposure_time: Exposure time in microseconds
            analogue_gain: Analogue gain value
            auto_exposure: Enable/disable auto exposure
            mode: Camera mode (rpi, webcam, sim)
            mav_comm: MavlinkComm instance (required for sim mode)
        """
        self.camera_index = camera_index
        self.exposure_time = exposure_time
        self.analogue_gain = analogue_gain
        self.auto_exposure_enabled = auto_exposure
        self.mode = mode
        self._camera: BaseCameraDevice | None = None
        self._mav_comm = mav_comm
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
        elif self.mode == "webcam":
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
        else:
            # sim
            from warg_common.simulator.world import create_demo_world
            from warg_common.simulator.coordinates import (
                CoordinateTransform,
                GPSCoord,
                LocalCoord,
            )
            from warg_common.simulator.camera import (
                SimCamera,
                CameraConfig as SimCameraConfig,
            )

            if self._mav_comm is None:
                logging.error(
                    "MavlinkComm instance required for sim mode initialization"
                )
                return False

            # Wait for first GPS position from MAVLink
            logging.info("Waiting for initial GPS position from MAVLink...")
            max_attempts = 100
            for attempt in range(max_attempts):
                # Process MAVLink messages to get position
                self._mav_comm.process_data_stream()
                position = self._mav_comm.get_position()

                # Check if we got a valid position (not the default 0,0,0)
                if position.lat != 0 or position.lon != 0:
                    logging.info(f"Got initial GPS position: {position}")
                    break

                if attempt == max_attempts - 1:
                    logging.error("Failed to get initial GPS position from MAVLink")
                    return False

                time.sleep(0.1)

            # Initialize coordinate transform with first GPS position
            # Drone's starting position becomes local (0, 0, 0)
            transform = CoordinateTransform()
            gps_origin = GPSCoord(lat=position.lat, lon=position.lon, alt=position.alt)
            transform.set_origin(gps_origin, LocalCoord(0, 0, 0))
            logging.info(
                f"Initialized coordinate transform with origin: lat={position.lat}, lon={position.lon}, alt={position.alt}m"
            )

            pitch = 0 if self.camera_index else -90

            config = SimCameraConfig(
                fov_deg=90,
                pitch_deg=pitch,
                world=create_demo_world(),
                transform=transform,
            )
            status, obj = SimCamera.create(640, 480, config)
            self._camera = obj
            if status:
                logging.info(f"sim camera {self.camera_index} initialized successfully")
            else:
                logging.error(f"sim camera {self.camera_index} failed to initialize")
            return status

    def capture_frame(self) -> np.ndarray | None:
        """
        Capture a single frame from the camera.
        """
        if self._camera is not None:
            # Update simulation camera position from MAVLink
            if self.mode == "sim":
                from warg_common.simulator.coordinates import GPSCoord

                if self._mav_comm is None:
                    logging.error("MavlinkComm instance required for sim mode")
                    return None

                # Get current position and heading from MAVLink
                position = self._mav_comm.get_position()
                heading = self._mav_comm.get_heading()

                # Convert util.Coordinate to simulator.GPSCoord
                gps_coord = GPSCoord(
                    lat=position.lat, lon=position.lon, alt=position.alt
                )

                # Update simulation camera with current position
                self._camera.update_position(gps_coord, heading)

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
        Detect colored circular targets in the frame.
        Returns the color of the circular target closest to the frame center.
        """
        if frame is None or frame.size == 0:
            logging.warning("Invalid frame provided to colour_in_frame method")
            return None

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_height, frame_width = frame.shape[:2]
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2

        # Thresholds for detection
        MIN_AREA = 100  # Minimum contour area in pixels
        MIN_CIRCULARITY = 0.6  # Circularity threshold (1.0 = perfect circle)
        MIN_FILL_RATIO = 0.7  # Minimum ratio of colored pixels to contour area (1.0 = completely filled)

        # Track the closest circular target to frame center
        min_distance = float('inf')
        detected_colour = None

        for colour in [c.value for c in Colours]:
            mask = cv2.inRange(frame_hsv, colour.lower_hsv, colour.upper_hsv)

            # Find contours in the mask
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_AREA:
                    continue

                # Calculate circularity: 4*pi*area / perimeter^2
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue

                circularity = 4 * np.pi * area / (perimeter * perimeter)

                if circularity < MIN_CIRCULARITY:
                    continue

                # Check fill ratio: ensure the contour is solid, not hollow
                # Create a mask for just this contour
                contour_mask = np.zeros(frame_hsv.shape[:2], dtype=np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)

                # Count colored pixels within the contour
                colored_pixels_in_contour = cv2.countNonZero(cv2.bitwise_and(mask, contour_mask))
                fill_ratio = colored_pixels_in_contour / area if area > 0 else 0

                if fill_ratio < MIN_FILL_RATIO:
                    continue

                # Calculate center of this circular contour
                moments = cv2.moments(contour)
                if moments["m00"] != 0:
                    center_x = moments["m10"] / moments["m00"]
                    center_y = moments["m01"] / moments["m00"]

                    # Calculate distance from frame center
                    distance = np.sqrt(
                        (center_x - frame_center_x)**2 +
                        (center_y - frame_center_y)**2
                    )

                    if distance < min_distance:
                        min_distance = distance
                        detected_colour = colour

        if detected_colour is not None:
            logging.info(
                f"Detected {detected_colour.name} circular target "
                f"at distance {min_distance:.1f} pixels from center"
            )

        return detected_colour

    def center_of_target_in_frame(
        self,
        frame: np.ndarray,
        colour: Colour,
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
