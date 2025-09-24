#!/usr/bin/env python3
"""
WARG Drone Competition 2026 - Airside Control System
Upgraded for colored target detection and building mapping
"""

import time
from typing import Optional
import pyautogui
import cv2
from pymavlink import mavutil
# Import our custom modules
from config import (
    LOCKED_RADIUS,
    LOG_LEVEL,
    LOG_FORMAT,
    MAVLINK_BAUD,
    MAVLINK_PORT,
    MAVLINK_SOURCE_COMPONENT,
    MAVLINK_SOURCE_SYSTEM,
    PX_TO_MS,
    RC_THRESHOLD,
    TEXT_COLOR,
)
from camera_manager import CameraManager
from target_detector import TargetDetector
from building_mapper import BuildingMapper
from utils import (
    GPS,
    MovingAverage,
    RateLimiter,
    pixel_to_velocity,
    euclidean_distance,
    setup_logging,
    create_status_text,
    calculate_fps,
    add_text_overlay,
    draw_crosshairs,
)

# Setup logging
logger = setup_logging(LOG_LEVEL, LOG_FORMAT)
logger.info("WARG Drone Competition 2025 - Airside System Starting")


class DroneControlSystem:
    """
    Main drone control system for autonomous target detection
    and building mapping
    """

    def __init__(self):
        # Initialize subsystems
        self.camera_manager = CameraManager()
        self.target_detector = TargetDetector()
        self.building_mapper = BuildingMapper()

        # MAVLink connection
        self.mav: Optional[mavutil.mavlink_connection] = None

        # State variables
        self.current_gps = GPS(0.0, 0.0, 0.0)
        self.pending_gps = GPS(0.0, 0.0, 0.0)
        self.rc_channels = {}

        # Control state
        self.target_detection_active = False
        self.corner_mapping_active = False
        self.coords_sent = False
        self.last_rc_state = {}

        # Filters and rate limiters
        self.velocity_smoother = MovingAverage(3)
        # Max 0.5 Hz coordinate sending
        self.coordinate_sender = RateLimiter(0.5)
        # Max 2 Hz status updates
        self.status_sender = RateLimiter(2.0)

        # Performance tracking
        self.frame_times = []
        self.target_lock_start_time = None

        # System state
        self.system_ready = False
        self.shutdown_requested = False

    def initialize(self) -> bool:
        """
        Initialize all drone systems
        """
        logger.info("Initializing drone systems...")

        # Disable PyAutoGUI failsafe
        pyautogui.FAILSAFE = False
        pyautogui.moveTo(0, pyautogui.size()[1])

        # Initialize camera system
        if not self.camera_manager.initialize_cameras():
            logger.error("Failed to initialize camera system")
            return False

        # Connect to MAVLink
        if not self._connect_mavlink():
            logger.error("Failed to connect to MAVLink")
            return False

        # Request data streams
        self._request_data_streams()

        self.system_ready = True
        logger.info("Drone systems initialized successfully")
        return True

    def _connect_mavlink(self) -> bool:
        """
        Connect to MAVLink autopilot
        """
        try:
            logger.info("Connecting to MAVLink...")
            self.mav = mavutil.mavlink_connection(
                MAVLINK_PORT,
                baud=MAVLINK_BAUD,
                source_component=MAVLINK_SOURCE_COMPONENT,
                source_system=MAVLINK_SOURCE_SYSTEM,
            )

            self.mav.wait_heartbeat()
            logger.info(
                f"Heartbeat received from system {self.mav.target_system}, "
                f"component {self.mav.target_component}"
            )
            return True

        except Exception as e:
            logger.error(f"MAVLink connection failed: {e}")
            return False

    def _request_data_streams(self):
        """
        Request necessary data streams from autopilot
        """
        if not self.mav:
            return

        # Request GPS data
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1,
            1,
        )

        # Request RC channel data
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            5,
            1,
        )

        logger.info("Requested GLOBAL_POSITION_INT and RC_CHANNELS streams")

    def run(self):
        """
        Main execution loop
        """
        logger.info("Starting main execution loop")

        try:
            while not self.shutdown_requested and self.system_ready:
                current_time = time.time()
                self.frame_times.append(current_time)

                # Process MAVLink messages
                self._process_mavlink_messages()

                # Capture frame from active camera
                frame = self.camera_manager.capture_frame()
                if frame is None:
                    logger.warning("Failed to capture frame")
                    continue

                # Process frame based on current mode
                self._process_frame(frame)

                # Check for exit condition
                key = cv2.waitKey(1) & 0xFF
                if key == 32:  # Space bar
                    logger.info("Exit requested by user")
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error in main loop: {e}")
        finally:
            self._cleanup()

    def _process_mavlink_messages(self):
        """
        Process incoming MAVLink messages
        """
        if not self.mav:
            return

        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == "RC_CHANNELS":
                self._handle_rc_channels(msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self._handle_gps_message(msg)

    def _handle_rc_channels(self, msg):
        """
        Handle RC channel messages for mode switching
        """
        # Target detection channel (channel 7)
        chan7_raw = getattr(msg, "chan7_raw", 0)
        current_detection_state = chan7_raw >= RC_THRESHOLD

        if current_detection_state != self.target_detection_active:
            self.target_detection_active = current_detection_state
            self.coords_sent = False

            if self.target_detection_active:
                logger.info("Target detection mode ACTIVATED")
            else:
                logger.info("Target detection mode DEACTIVATED")

        # Corner mapping channel (channel 8)
        chan8_raw = getattr(msg, "chan8_raw", 0)
        current_mapping_state = chan8_raw >= RC_THRESHOLD

        if current_mapping_state != self.corner_mapping_active:
            self.corner_mapping_active = current_mapping_state

            if self.corner_mapping_active:
                logger.info("Corner mapping mode ACTIVATED")
                self._record_building_corner()
            else:
                logger.info("Corner mapping mode DEACTIVATED")

        # Store RC values for display
        self.rc_channels = {"chan7": chan7_raw, "chan8": chan8_raw}

    def _handle_gps_message(self, msg):
        """
        Handle GPS position messages
        """
        self.pending_gps = GPS(
            lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt=msg.alt / 1000.0
        )

        # Update current GPS if we have a valid fix
        if (abs(self.pending_gps.lat) > 0.001
                and abs(self.pending_gps.lon) > 0.001):
            self.current_gps = self.pending_gps

    def _record_building_corner(self):
        """
        Record current GPS position as building corner
        """
        if not self.current_gps or (
            abs(self.current_gps.lat) < 0.001
            and abs(self.current_gps.lon) < 0.001
        ):
            logger.warning("Cannot record corner - no valid GPS fix")
            return

        success = self.building_mapper.add_corner(
            self.current_gps.lat, self.current_gps.lon
        )
        if success and self.mav:
            corner_count = len(self.building_mapper.corners)
            status_msg = (f"Corner {corner_count}/4 recorded: "
                          f"{self.current_gps.lat:.7f}, "
                          f"{self.current_gps.lon:.7f}")

            if self.status_sender.is_ready():
                self.mav.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO, status_msg.encode()
                )

    def _process_frame(self, frame):
        """
        Process captured frame based on current mode
        """
        # Determine which camera to use and what to detect
        if self.target_detection_active:
            self._process_target_detection(frame)
        else:
            # Just display frame with status information
            self._add_status_overlay(frame)
            self.camera_manager.display_frame(frame)

    def _process_target_detection(self, frame):
        """
        Process frame for target detection and tracking
        """
        # Detect targets in frame
        targets = self.target_detector.detect_targets(frame)

        if not targets:
            self._add_status_overlay(frame, "NO TARGETS DETECTED")
            self.camera_manager.display_frame(frame)
            return

        # Get best target
        best_target = self.target_detector.get_best_target(targets)

        # Draw all detected targets
        frame = self.target_detector.draw_targets(frame, targets)

        # Calculate error from center
        height, width = frame.shape[:2]
        # center_x, center_y = width // 2, height // 2

        offset_x, offset_y = self.target_detector.calculate_target_offset(
            best_target, width, height
        )

        error = euclidean_distance(0, 0, offset_x, offset_y)

        # Determine target status
        if error < LOCKED_RADIUS:
            status = "TARGET LOCKED"
            self._handle_target_locked(best_target)
        else:
            status = "TARGET ACQUIRED"
            self._send_velocity_command(offset_x, offset_y)

        # Add status information
        target_info = (f"{status} - {best_target.color.upper()} "
                       f"(Error: {error:.1f}px)")
        self._add_status_overlay(frame, target_info)

        # Display frame
        self.camera_manager.display_frame(frame)

    def _send_velocity_command(self, offset_x: int, offset_y: int):
        """
        Send velocity command to center target
        """
        if not self.mav:
            return

        # Convert pixel offsets to velocity
        velocity = pixel_to_velocity(offset_x, offset_y, PX_TO_MS)

        # Apply smoothing
        smooth_vel_x = self.velocity_smoother.update(velocity.x)
        smooth_vel_y = self.velocity_smoother.update(velocity.y)

        # Send velocity command
        self.mav.mav.set_position_target_local_ned_send(
            int(self.mav.time_since("SYSTEM_TIME") * 1e6),
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b110111100111,  # Use only vel_X, vel_Y
            0,
            0,
            0,  # Position (unused)
            smooth_vel_x,
            smooth_vel_y,
            0,  # Velocity
            0,
            0,
            0,  # Acceleration (unused)
            0,
            0,  # Yaw and yaw rate (unused)
        )

    def _handle_target_locked(self, target):
        """
        Handle target locked state - send coordinates
        """
        if self.coords_sent or not self.coordinate_sender.is_ready():
            return

        if not self.mav:
            return

        # Determine target location based on camera
        camera_name = self.camera_manager.get_active_camera_name().lower()

        if camera_name == "downward":
            # Ground target - use current GPS
            target_lat = self.current_gps.lat
            target_lon = self.current_gps.lon
            target_alt = self.current_gps.alt
            location_type = "Ground"
        else:
            # Wall target - estimate position (placeholder for now)
            target_lat = self.current_gps.lat
            target_lon = self.current_gps.lon
            target_alt = self.current_gps.alt
            location_type = "Wall"

        # Send coordinate message
        coord_msg = (f"Target-{target.color}: {target_lat:.7f}, "
                     f"{target_lon:.7f}, {target_alt:.1f}m ({location_type})")

        self.mav.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO, coord_msg.encode()
        )

        self.coords_sent = True
        logger.info(f"Sent target coordinates: {coord_msg}")

    def _add_status_overlay(self, frame, target_info: str = ""):
        """
        Add status overlay to frame
        """
        # Calculate FPS
        fps = calculate_fps(self.frame_times)

        # Create status text
        camera_name = self.camera_manager.get_active_camera_name()
        status_lines = create_status_text(
            gps=self.current_gps if self.current_gps.lat != 0 else None,
            target_info=target_info,
            camera_name=camera_name,
            fps=fps,
        )

        # Add building mapping status
        building_info = self.building_mapper.get_building_info()
        status_lines.append(f"Building corners: "
                            f"{building_info['corner_count']}/4")

        # Add mode status
        mode_status = []
        if self.target_detection_active:
            mode_status.append("DETECTION")
        if self.corner_mapping_active:
            mode_status.append("MAPPING")

        if mode_status:
            status_lines.append(f"Mode: {', '.join(mode_status)}")
        else:
            status_lines.append("Mode: STANDBY")

        # Draw status overlay
        for i, line in enumerate(status_lines):
            y_pos = 30 + (i * 25)
            frame = add_text_overlay(frame, line, (10, y_pos), TEXT_COLOR)

        # Add branding
        branding = "WARG Competition 2026 - Autonomous Target Detection"
        frame_height = frame.shape[0]
        frame = add_text_overlay(
            frame, branding, (10, frame_height - 20), TEXT_COLOR, 0.5, 1
        )

        # Add crosshairs
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        frame = draw_crosshairs(frame, center_x, center_y)

    def _cleanup(self):
        """
        Clean up system resources
        """
        logger.info("Cleaning up system resources...")

        if self.camera_manager:
            self.camera_manager.cleanup()

        if self.mav:
            self.mav.close()

        logger.info("Cleanup complete")


def main():
    """
    Main entry point
    """
    drone_system = DroneControlSystem()

    if not drone_system.initialize():
        logger.error("System initialization failed")
        return 1

    drone_system.run()
    return 0


if __name__ == "__main__":
    exit(main())
