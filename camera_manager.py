"""
Camera Manager Module
Handles dual camera setup and frame capture for downward and forward facing cameras
"""

import cv2
import logging
from typing import Optional, Tuple, Dict
from picamera2 import Picamera2
from config import (
    CAMERA_EXPOSURE_TIME,
    CAMERA_ANALOG_GAIN,
    CAMERA_RESOLUTION,
    DOWNWARD_CAMERA_ID,
    FORWARD_CAMERA_ID,
)

logger = logging.getLogger(__name__)


class Camera:
    """Individual camera instance"""

    def __init__(self, camera_id: int, name: str):
        self.camera_id = camera_id
        self.name = name
        self.picam2: Optional[Picamera2] = None
        self.is_initialized = False

    def initialize(self) -> bool:
        """Initialize the camera"""
        try:
            self.picam2 = Picamera2(camera_num=self.camera_id)

            # Configure camera
            config = self.picam2.create_preview_configuration(
                main={"size": CAMERA_RESOLUTION}
            )
            self.picam2.configure(config)

            # Set camera controls for consistent imaging
            self.picam2.set_controls(
                {
                    "AeEnable": False,  # Disable auto-exposure
                    "ExposureTime": CAMERA_EXPOSURE_TIME,
                    "AnalogueGain": CAMERA_ANALOG_GAIN,
                }
            )

            self.picam2.start()
            self.is_initialized = True

            logger.info(
                f"Camera {self.name} (ID: {self.camera_id}) initialized successfully"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to initialize camera {self.name}: {e}")
            self.is_initialized = False
            return False

    def capture_frame(self):
        """Capture a frame from the camera"""
        if not self.is_initialized or self.picam2 is None:
            logger.error(f"Camera {self.name} not initialized")
            return None

        try:
            frame = self.picam2.capture_array()
            return frame
        except Exception as e:
            logger.error(f"Failed to capture frame from camera {self.name}: {e}")
            return None

    def stop(self):
        """Stop the camera"""
        if self.picam2 is not None and self.is_initialized:
            try:
                self.picam2.stop()
                self.is_initialized = False
                logger.info(f"Camera {self.name} stopped successfully")
            except Exception as e:
                logger.error(f"Error stopping camera {self.name}: {e}")


class CameraManager:
    """
    Manages dual camera setup for drone target detection
    Handles both downward-facing and forward-facing cameras
    """

    def __init__(self):
        self.downward_camera = Camera(DOWNWARD_CAMERA_ID, "Downward")
        self.forward_camera = Camera(FORWARD_CAMERA_ID, "Forward")
        self.active_camera = None
        self.display_window_initialized = False

        # Camera calibration data (if available)
        self.camera_matrices = {}
        self.distortion_coefficients = {}

    def initialize_cameras(self) -> bool:
        """Initialize both cameras"""
        logger.info("Initializing camera system...")

        # Initialize downward camera
        downward_success = self.downward_camera.initialize()

        # Initialize forward camera
        forward_success = self.forward_camera.initialize()

        if downward_success and forward_success:
            logger.info("Both cameras initialized successfully")
            self.active_camera = self.downward_camera  # Default to downward
            self._setup_display_window()
            return True
        elif downward_success:
            logger.warning("Only downward camera initialized - forward camera failed")
            self.active_camera = self.downward_camera
            self._setup_display_window()
            return True
        elif forward_success:
            logger.warning("Only forward camera initialized - downward camera failed")
            self.active_camera = self.forward_camera
            self._setup_display_window()
            return True
        else:
            logger.error("Failed to initialize any cameras")
            return False

    def _setup_display_window(self):
        """Setup OpenCV display window"""
        try:
            cv2.namedWindow("WARG Drone Camera", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(
                "WARG Drone Camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )
            self.display_window_initialized = True
            logger.info("Display window initialized")
        except Exception as e:
            logger.error(f"Failed to setup display window: {e}")
            self.display_window_initialized = False

    def switch_to_downward_camera(self) -> bool:
        """Switch active camera to downward-facing"""
        if not self.downward_camera.is_initialized:
            logger.error("Downward camera not available")
            return False

        self.active_camera = self.downward_camera
        logger.info("Switched to downward camera")
        return True

    def switch_to_forward_camera(self) -> bool:
        """Switch active camera to forward-facing"""
        if not self.forward_camera.is_initialized:
            logger.error("Forward camera not available")
            return False

        self.active_camera = self.forward_camera
        logger.info("Switched to forward camera")
        return True

    def get_active_camera_name(self) -> str:
        """Get name of currently active camera"""
        return self.active_camera.name if self.active_camera else "None"

    def capture_frame(self):
        """Capture frame from active camera"""
        if self.active_camera is None:
            logger.error("No active camera available")
            return None

        return self.active_camera.capture_frame()

    def capture_both_frames(self) -> Tuple[Optional[any], Optional[any]]:
        """Capture frames from both cameras simultaneously"""
        downward_frame = None
        forward_frame = None

        if self.downward_camera.is_initialized:
            downward_frame = self.downward_camera.capture_frame()

        if self.forward_camera.is_initialized:
            forward_frame = self.forward_camera.capture_frame()

        return downward_frame, forward_frame

    def get_frame_from_camera(self, camera_name: str):
        """Get frame from specific camera by name"""
        if camera_name.lower() == "downward":
            return self.downward_camera.capture_frame()
        elif camera_name.lower() == "forward":
            return self.forward_camera.capture_frame()
        else:
            logger.error(f"Unknown camera name: {camera_name}")
            return None

    def display_frame(self, frame, overlay_text: str = ""):
        """Display frame with optional overlay text"""
        if not self.display_window_initialized or frame is None:
            return

        try:
            # Add camera name overlay
            camera_name = self.get_active_camera_name()
            cv2.putText(
                frame,
                f"Camera: {camera_name}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            # Add additional overlay text if provided
            if overlay_text:
                lines = overlay_text.split("\n")
                for i, line in enumerate(lines):
                    y_pos = 70 + (i * 30)
                    cv2.putText(
                        frame,
                        line,
                        (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

            cv2.imshow("WARG Drone Camera", frame)

        except Exception as e:
            logger.error(f"Error displaying frame: {e}")

    def add_crosshairs(self, frame):
        """Add crosshairs to frame for targeting"""
        if frame is None:
            return frame

        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        # Draw crosshairs
        cv2.line(
            frame,
            (center_x - 50, center_y),
            (center_x + 50, center_y),
            (255, 255, 255),
            2,
        )
        cv2.line(
            frame,
            (center_x, center_y - 50),
            (center_x, center_y + 50),
            (255, 255, 255),
            2,
        )
        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)

        return frame

    def get_camera_info(self) -> Dict:
        """Get information about camera system"""
        return {
            "downward_camera": {
                "initialized": self.downward_camera.is_initialized,
                "camera_id": self.downward_camera.camera_id,
            },
            "forward_camera": {
                "initialized": self.forward_camera.is_initialized,
                "camera_id": self.forward_camera.camera_id,
            },
            "active_camera": self.get_active_camera_name(),
            "display_window": self.display_window_initialized,
        }

    def cleanup(self):
        """Clean up camera resources"""
        logger.info("Cleaning up camera system...")

        self.downward_camera.stop()
        self.forward_camera.stop()

        if self.display_window_initialized:
            cv2.destroyAllWindows()

        logger.info("Camera cleanup complete")
