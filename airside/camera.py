"""
Camera module for handling Picamera2 operations.

This module provides a Camera class that handles Raspberry Pi Camera Module 2
operations, including initialization and frame capture.
"""

from warg_common.camera import BaseCameraDevice, create_camera, CameraOption
from dataclasses import dataclass
import numpy as np
import cv2
import logging
import time
from typing import Literal, List, Tuple

from warg_common.simulator import SimCamera

from util import Colour, Colours


@dataclass
class TargetDetection:
    colour: Colour
    x: float
    y: float
    circularity: float
    coverage: float
    area: float
    contour: np.ndarray = None  # OpenCV contour for visualization


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
        mode: Literal["rpi", "webcam", "oakd", "sim", "dummy"] = "rpi",
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
        self._oakd_pipeline = None
        self._oakd_device = None
        self._oakd_queue = None
        self._mav_comm = mav_comm
        print("mode", self.mode)

        # Retry camera initialization
        if mode != "dummy":
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
        elif self.mode == "oakd":
            print("Attempting to initialize Oak-D camera")
            try:
                import depthai as dai
                
                # Create pipeline
                pipeline = dai.Pipeline()
                
                # Create color camera node
                cam = pipeline.createColorCamera()
                cam.setPreviewSize(500, 500)
                cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                cam.setInterleaved(False)
                cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                
                # Configure for IR beacon detection (similar to picam2 settings)
                cam.setManualExposure(self.exposure_time, self.analogue_gain * 1000)  # Convert to ISO
                cam.setAutoExposureEnable(False)
                
                # Create output node
                xout = pipeline.createXLinkOut()
                xout.setStreamName("preview")
                cam.preview.link(xout.input)
                
                # Store pipeline for frame capture
                self._oakd_pipeline = pipeline
                self._oakd_device = None  # Will be initialized in capture_frame
                
                logging.info(f"Oak-D camera {self.camera_index} initialized successfully")
                return True
                
            except Exception as e:
                logging.error(f"Oak-D camera {self.camera_index} failed to initialize: {e}")
                return False
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
        if self.mode == "oakd" and self._oakd_pipeline is not None:
            try:
                import depthai as dai
                
                # Initialize device if not already done
                if self._oakd_device is None:
                    self._oakd_device = dai.Device(self._oakd_pipeline)
                    self._oakd_queue = self._oakd_device.getOutputQueue(name="preview", maxSize=4, blocking=False)
                
                # Get frame from queue
                preview = self._oakd_queue.get()
                frame = preview.getCvFrame()
                
                return frame
                
            except Exception as e:
                logging.error(f"Failed to capture Oak-D frame: {e}")
                return None
        elif self._camera is not None:
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
        if self.mode == "oakd":
            return self._oakd_pipeline is not None
        return self._camera is not None

    def cleanup(self):
        """Clean up camera resources."""
        if self.mode == "oakd" and self._oakd_device is not None:
            try:
                self._oakd_device.close()
                self._oakd_device = None
                self._oakd_queue = None
                logging.info("Oak-D camera resources cleaned up")
            except Exception as e:
                logging.error(f"Error cleaning up Oak-D camera: {e}")

    def __enter__(self) -> "Camera":
        """Context manager entry - allows use with 'with' statement."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - clean up resources."""
        self.cleanup()

    def find_targets(self, frame: np.ndarray) -> List[TargetDetection]:
        """
        Detect colored circular targets in the frame.
        Returns the color of the circular target closest to the frame center.
        """
        if frame is None or frame.size == 0:
            logging.warning("Invalid frame provided to colour_in_frame method")
            return []

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        targets = []

        for colour in [c.value for c in Colours]:
            mask = cv2.inRange(frame_hsv, colour.lower_hsv, colour.upper_hsv)

            # Find contours in the mask
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                # Calculate circularity: 4*pi*area / perimeter^2
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue

                circularity = 4 * np.pi * area / (perimeter * perimeter)

                # Check fill ratio: ensure the contour is solid, not hollow
                # Create a mask for just this contour
                contour_mask = np.zeros(frame_hsv.shape[:2], dtype=np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)

                # Count colored pixels within the contour
                colored_pixels_in_contour = cv2.countNonZero(
                    cv2.bitwise_and(mask, contour_mask)
                )
                fill_ratio = colored_pixels_in_contour / area if area > 0 else 0

                # Calculate center of this circular contour
                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue

                center_x = moments["m10"] / moments["m00"]
                center_y = moments["m01"] / moments["m00"]
                targets.append(
                    TargetDetection(
                        colour,
                        center_x,
                        center_y,
                        circularity,
                        fill_ratio,
                        area,
                        contour,
                    )
                )

        return targets

    def get_best_target(self, targets: List[TargetDetection]) -> TargetDetection | None:
        best_target = None
        min_dist = float("inf")

        center_x = 250
        center_y = 250

        for target in targets:
            if not self.is_valid_target(target):
                continue
            dist = (center_x - target.x) ** 2 + (center_y - target.y) ** 2
            if dist < min_dist:
                min_dist = dist
                best_target = target

        return best_target

    def is_valid_target(self, target: TargetDetection) -> bool:
        # Thresholds for detection
        MIN_AREA = 300  # Minimum contour area in pixels
        MIN_CIRCULARITY = 0.6  # Circularity threshold (1.0 = perfect circle)
        MIN_FILL_RATIO = 0.7  # Minimum ratio of colored pixels to contour area (1.0 = completely filled)
        return not (
            target.area < MIN_AREA
            or target.circularity < MIN_CIRCULARITY
            or target.coverage < MIN_FILL_RATIO
        )
