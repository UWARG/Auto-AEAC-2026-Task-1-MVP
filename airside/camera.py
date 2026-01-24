"""
Camera module for handling OAK-D operations.

This module provides a Camera class that handles Luxonis OAK-D camera
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
import depthai as dai


@dataclass
class TargetDetection:
    colour: Colour
    x: float
    y: float
    circularity: float
    coverage: float
    area: float
    contour: np.ndarray = None  # OpenCV contour for visualization


class OakDCameraDevice:
    def __init__(self, camera_index: int, exposure_time: int, analogue_gain: float):
        logging.info(f"DepthAI version: {dai.__version__}")
        self.pipeline = dai.Pipeline()

        # Define source and output
        # Use factory methods for better compatibility
        cam_rgb = self.pipeline.createColorCamera()
        xout_video = self.pipeline.createXLinkOut()

        xout_video.setStreamName("video")

        # Properties
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # RGB
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setVideoSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Apply basic exposure/gain settings if provided
        # OAK-D sensitivity is ISO. Mapping gain to ISO (approx 100 * gain)
        # Exposure is in microseconds
        if exposure_time > 0 and analogue_gain > 0:
            iso = int(analogue_gain * 100)
            # Clamp ISO to valid range (usually 100-1600)
            iso = max(100, min(1600, iso))
            cam_rgb.initialControl.setManualExposure(exposure_time, iso)

        cam_rgb.video.link(xout_video.input)

        # Device selection
        devices = dai.Device.getAllAvailableDevices()
        if len(devices) > camera_index:
            device_info = devices[camera_index]
            self.device = dai.Device(self.pipeline, device_info)
            logging.info(f"Initialized OAK-D device {camera_index} (MxId: {device_info.getMxId()})")
        else:
            if len(devices) > 0:
                 logging.warning(f"Camera index {camera_index} out of range, using first available device")
                 self.device = dai.Device(self.pipeline)
            else:
                 # Attempt to open any device (will likely fail if none, or wait for one)
                 logging.info("No explicit devices found, attempting to open default OAK-D device")
                 self.device = dai.Device(self.pipeline)

        self.q_video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)

    def run(self) -> Tuple[bool, np.ndarray | None]:
        in_video = self.q_video.tryGet()
        if in_video is not None:
            return True, in_video.getCvFrame()
        return False, None

    def close(self):
        if self.device:
            self.device.close()


class Camera:
    """
    Handles Camera operations (OAK-D, Webcam, or Sim)
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
        mode: Literal["rpi", "webcam", "sim", "dummy"] = "rpi",
        mav_comm=None,
    ) -> None:
        """
        Initialize and configure the Camera.

        Args:
            camera_index: Camera index
            exposure_time: Exposure time in microseconds
            analogue_gain: Analogue gain value
            auto_exposure: Enable/disable auto exposure
            mode: Camera mode (rpi=OAK-D, webcam, sim)
            mav_comm: MavlinkComm instance (required for sim mode)
        """
        self.camera_index = camera_index
        self.exposure_time = exposure_time
        self.analogue_gain = analogue_gain
        self.auto_exposure_enabled = auto_exposure
        self.mode = mode
        self._camera = None
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
        # Initialize OAK-D (formerly rpi/picam2)
        if self.mode == "rpi":
            print(f"Attempting to initialize OAK-D camera (index {self.camera_index})")
            try:
                self._camera = OakDCameraDevice(
                    self.camera_index,
                    self.exposure_time,
                    self.analogue_gain
                )
                logging.info(f"OAK-D camera {self.camera_index} initialized successfully")
                return True
            except Exception as e:
                logging.error(f"OAK-D camera {self.camera_index} failed to initialize: {e}")
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
                # OAK-D might not always return a frame if queue is empty (non-blocking)
                # This is normal, so logging warning every time might be noisy.
                # However, for compat with other cams, let's keep it but maybe downgrade log level if needed.
                pass
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

    def find_targets(self, frame: np.ndarray) -> List[TargetDetection]:
        """
        Detect colored circular targets in the frame.
        Returns the color of the circular target closest to the frame center.
        """
        if frame is None or frame.size == 0:
            # logging.warning("Invalid frame provided to colour_in_frame method")
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
