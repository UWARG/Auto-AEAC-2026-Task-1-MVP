"""
Interactive flight demo with keyboard controls and synthetic camera view.

Controls:
    W/S - Move forward/backward
    A/D - Move left/right
    Q/E - Rotate left/right (yaw)
    R/F - Move up/down
    Space - Stop all movement
    ESC - Quit

Displays:
    - Synthetic camera view
    - Drone position (GPS and local)
    - Visible targets with ground truth
"""

import cv2
import numpy as np
import time
import logging
from pymavlink import mavutil

from synthetic_sim.coordinates import CoordinateTransform, LocalCoord, GPSCoord
from synthetic_sim.world import create_demo_world
from synthetic_sim.camera import SyntheticCamera, CameraConfig
from util import Vector3d


# Flight control parameters
VELOCITY_STEP = 2.0  # m/s
YAW_RATE_STEP = 30.0  # deg/s

# SITL connection
SITL_CONNECTION = "tcp:127.0.0.1:5762"


class FlightController:
    """Simple keyboard-controlled flight for testing synthetic camera."""

    def __init__(self, sitl_connection: str = SITL_CONNECTION):
        """Initialize flight controller and connect to SITL."""
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Connect to SITL
        self.logger.info(f"Connecting to SITL at {sitl_connection}...")
        self.mav = mavutil.mavlink_connection(sitl_connection)
        self.mav.wait_heartbeat()
        self.logger.info(f"Connected to system {self.mav.target_system}")

        # Request position updates
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,  # 10 Hz
            1
        )

        # Initialize world and camera
        self.world = create_demo_world()
        self.coord_transform = CoordinateTransform()
        self.camera_config = CameraConfig(
            fov_deg=90,
            pitch_deg=-90,  # Down-facing
            image_width=800,
            image_height=600
        )
        self.synthetic_camera = None  # Initialized after first GPS lock

        # Flight state
        self.current_velocity = Vector3d(0, 0, 0)
        self.current_yaw_rate = 0.0
        self.drone_gps = None
        self.drone_heading = 0.0

        self.logger.info("Flight controller initialized")
        self.logger.info("Waiting for GPS lock from SITL...")

    def update_position(self):
        """Read position from SITL."""
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            self.drone_gps = GPSCoord(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt=msg.alt / 1000.0
            )
            if msg.hdg != 65535:
                self.drone_heading = msg.hdg / 100.0

            # Initialize coordinate transform on first GPS lock
            if not self.coord_transform.is_initialized():
                self.logger.info(f"GPS lock acquired: {self.drone_gps.lat:.6f}, {self.drone_gps.lon:.6f}")
                self.logger.info("Aligning world origin to current SITL position...")

                # Lock local origin (0, 0, 0) to current SITL GPS
                self.coord_transform.set_origin(self.drone_gps, LocalCoord(0, 0, 0))

                # Initialize synthetic camera
                self.synthetic_camera = SyntheticCamera(
                    self.world,
                    self.coord_transform,
                    self.camera_config
                )

                self.logger.info("World aligned! Building is 20m North, 15m East of start position")

    def send_velocity_command(self):
        """Send current velocity command to SITL."""
        timestamp = int(self.mav.time_since("SYSTEM_TIME") * 1e6)
        type_mask = 0b110111000111  # Enable velocity
        frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED

        self.mav.mav.set_position_target_local_ned_send(
            timestamp,
            self.mav.target_system,
            self.mav.target_component,
            frame,
            type_mask,
            0, 0, 0,  # Position (not used)
            self.current_velocity.x,
            self.current_velocity.y,
            self.current_velocity.z,
            0, 0, 0,  # Acceleration (not used)
            0,  # Yaw (not used)
            self.current_yaw_rate / 57.3  # Yaw rate (rad/s)
        )

    def handle_keyboard(self, key: int):
        """Process keyboard input and update velocity."""
        if key == ord('w'):  # Forward
            self.current_velocity.x = VELOCITY_STEP
            self.logger.info("Moving forward")
        elif key == ord('s'):  # Backward
            self.current_velocity.x = -VELOCITY_STEP
            self.logger.info("Moving backward")
        elif key == ord('a'):  # Left
            self.current_velocity.y = -VELOCITY_STEP
            self.logger.info("Moving left")
        elif key == ord('d'):  # Right
            self.current_velocity.y = VELOCITY_STEP
            self.logger.info("Moving right")
        elif key == ord('r'):  # Up
            self.current_velocity.z = -VELOCITY_STEP
            self.logger.info("Moving up")
        elif key == ord('f'):  # Down
            self.current_velocity.z = VELOCITY_STEP
            self.logger.info("Moving down")
        elif key == ord('q'):  # Yaw left
            self.current_yaw_rate = -YAW_RATE_STEP
            self.logger.info("Rotating left")
        elif key == ord('e'):  # Yaw right
            self.current_yaw_rate = YAW_RATE_STEP
            self.logger.info("Rotating right")
        elif key == ord(' '):  # Stop
            self.current_velocity = Vector3d(0, 0, 0)
            self.current_yaw_rate = 0.0
            self.logger.info("STOP")
        elif key == 27:  # ESC
            return False

        return True

    def render_info_overlay(self, frame: np.ndarray, metadata: list):
        """Draw information overlay on frame."""
        if not self.coord_transform.is_initialized() or self.drone_gps is None:
            cv2.putText(frame, "Waiting for GPS lock...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return

        # Get local position
        local_pos = self.coord_transform.gps_to_local(self.drone_gps)

        # Draw position info
        info_lines = [
            f"GPS: {self.drone_gps.lat:.6f}, {self.drone_gps.lon:.6f}, {self.drone_gps.alt:.1f}m",
            f"Local: X={local_pos.x:.1f}m N, Y={local_pos.y:.1f}m E, Z={local_pos.z:.1f}m",
            f"Heading: {self.drone_heading:.1f} deg",
            f"Velocity: ({self.current_velocity.x:.1f}, {self.current_velocity.y:.1f}, {self.current_velocity.z:.1f}) m/s",
            f"Targets visible: {len(metadata)}"
        ]

        y_offset = 30
        for line in info_lines:
            cv2.putText(frame, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            # Black outline for readability
            cv2.putText(frame, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y_offset += 25

        # Draw target info
        if metadata:
            y_offset += 10
            cv2.putText(frame, "Visible Targets:", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            y_offset += 25

            for i, target in enumerate(metadata, 1):
                location = "GROUND" if target.is_on_ground else "WALL" if target.is_on_wall else "ROOF"
                target_info = f"  {i}. {target.color.name} @ {target.distance_meters:.1f}m ({location})"

                cv2.putText(frame, target_info, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                y_offset += 20

        # Draw controls help
        help_y = frame.shape[0] - 100
        controls = [
            "W/S: Forward/Back  |  A/D: Left/Right  |  R/F: Up/Down",
            "Q/E: Rotate  |  SPACE: Stop  |  ESC: Quit"
        ]
        for line in controls:
            cv2.putText(frame, line, (10, help_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            help_y += 20

    def run(self):
        """Main flight loop."""
        cv2.namedWindow("Synthetic Camera - Flight Demo", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Synthetic Camera - Flight Demo", 800, 600)

        self.logger.info("\n" + "="*60)
        self.logger.info("Flight Demo Started!")
        self.logger.info("="*60)
        self.logger.info("Make sure drone is armed and in GUIDED mode:")
        self.logger.info("  In MAVProxy: mode guided")
        self.logger.info("               arm throttle")
        self.logger.info("               takeoff 10")
        self.logger.info("="*60 + "\n")

        running = True
        last_command_time = time.time()

        while running:
            # Update position from SITL
            self.update_position()

            # Send velocity commands at 10Hz
            if time.time() - last_command_time > 0.1:
                self.send_velocity_command()
                last_command_time = time.time()

            # Render camera view
            if self.synthetic_camera and self.drone_gps:
                local_pos = self.coord_transform.gps_to_local(self.drone_gps)
                frame, metadata = self.synthetic_camera.render(local_pos, self.drone_heading)

                # Add info overlay
                self.render_info_overlay(frame, metadata)

                cv2.imshow("Synthetic Camera - Flight Demo", frame)
            else:
                # Show waiting screen
                waiting_frame = np.zeros((600, 800, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Waiting for GPS lock from SITL...",
                           (150, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow("Synthetic Camera - Flight Demo", waiting_frame)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # Key pressed
                if not self.handle_keyboard(key):
                    running = False

        # Cleanup
        self.logger.info("Stopping drone...")
        self.current_velocity = Vector3d(0, 0, 0)
        self.current_yaw_rate = 0.0
        self.send_velocity_command()

        cv2.destroyAllWindows()
        self.logger.info("Flight demo ended")


def main():
    """Entry point."""
    print("\n" + "="*70)
    print(" Synthetic Camera Flight Demo")
    print("="*70)
    print("\nPrerequisites:")
    print("1. SITL must be running:")
    print("   cd ~/ardupilot/ArduCopter")
    print("   sim_vehicle.py --console --map")
    print("\n2. In MAVProxy console, prepare the drone:")
    print("   mode guided")
    print("   arm throttle")
    print("   takeoff 10")
    print("\n3. Run this script:")
    print("   python3 -m synthetic_sim.flight_demo")
    print("\n" + "="*70 + "\n")

    input("Press Enter when ready...")

    controller = FlightController()
    controller.run()


if __name__ == "__main__":
    main()
