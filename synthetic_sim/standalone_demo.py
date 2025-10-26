"""
Standalone camera demo without SITL connection.

Simulates a drone flying with keyboard controls and shows synthetic camera view.
No MAVLink/SITL required - useful for testing camera projection and GPS logic.

Controls:
    W/S - Move forward/backward
    A/D - Move left/right
    Q/E - Rotate left/right (yaw)
    R/F - Move up/down
    Space - Stop all movement
    ESC - Quit
"""

import cv2
import numpy as np
import time
import logging

from synthetic_sim.coordinates import CoordinateTransform, LocalCoord, GPSCoord
from synthetic_sim.world import create_demo_world
from synthetic_sim.camera import SyntheticCamera, CameraConfig


# Flight control parameters
VELOCITY = 1.0  # meters per second
YAW_RATE = 20.0  # degrees per second


class StandaloneDrone:
    """Simulated drone with position-based controls."""

    def __init__(self, start_gps: GPSCoord, start_heading: float = 0.0):
        """
        Initialize drone at starting position.

        Args:
            start_gps: Initial GPS position
            start_heading: Initial heading in degrees (0=North, 90=East)
        """
        self.gps = start_gps
        self.heading = start_heading  # degrees

        # For GPS integration
        self.coord_transform = CoordinateTransform()
        self.coord_transform.set_origin(start_gps, LocalCoord(0, 0, 0))

        # Local position (meters from start)
        self.local_pos = LocalCoord(0, 0, 0)

        self.logger = logging.getLogger(__name__)

    def move(self, dx: float, dy: float, dz: float, dyaw: float):
        """
        Move drone by offset in world frame.

        Args:
            dx: Change in X (meters North)
            dy: Change in Y (meters East)
            dz: Change in Z (meters Up)
            dyaw: Change in heading (degrees)
        """
        # Update local position
        self.local_pos = LocalCoord(
            x=self.local_pos.x + dx,
            y=self.local_pos.y + dy,
            z=self.local_pos.z + dz
        )

        # Update heading
        self.heading = (self.heading + dyaw) % 360

        # Convert to GPS
        self.gps = self.coord_transform.local_to_gps(self.local_pos)

    def get_local_position(self) -> LocalCoord:
        """Get current position in local coordinates."""
        return self.local_pos


class StandaloneDemo:
    """Standalone demo without SITL."""

    def __init__(self):
        """Initialize demo."""
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Create world
        self.world = create_demo_world()

        # Initialize drone at a starting GPS position
        # Start 10 meters above ground to see things clearly
        start_gps = GPSCoord(lat=43.473146, lon=-80.540103, alt=10.0)
        self.drone = StandaloneDrone(start_gps, start_heading=0.0)

        # Set local position to 10m altitude (not on ground)
        self.drone.local_pos = LocalCoord(0, 0, 10)

        # Create camera config (adjustable pitch)
        self.camera_pitch = -45  # Start at 45 degrees down (not straight down)
        self.camera_config = CameraConfig(
            fov_deg=90,
            pitch_deg=self.camera_pitch,
            image_width=800,
            image_height=600
        )

        # Create synthetic camera
        self.synthetic_camera = SyntheticCamera(
            self.world,
            self.drone.coord_transform,
            self.camera_config
        )

        # Movement step size
        self.move_step = 0.5  # meters per key press
        self.rotate_step = 5.0  # degrees per key press

        self.logger.info("Standalone demo initialized")
        self.logger.info(f"Starting position: {start_gps.lat:.6f}, {start_gps.lon:.6f}")
        self.logger.info("World layout:")
        self.logger.info("  - Building at 20m North, 15m East")
        self.logger.info("  - Red target on north wall")
        self.logger.info("  - Blue target on roof")
        self.logger.info("  - Green target on ground (south of building)")
        self.logger.info("  - Yellow target on east wall")

        # Add debug reference markers
        self._add_debug_markers()

    def _add_debug_markers(self):
        """Add visible reference markers at key positions."""
        from synthetic_sim.world import CircleTarget, Color, LocalCoord

        # Add big white marker at origin (0, 0)
        self.world.targets.append(CircleTarget(
            position=LocalCoord(0, 0, 0.01),
            radius=1.0,  # 1 meter radius - very visible
            color=Color.WHITE,
            surface_normal=(0, 0, 1)
        ))

        # Add markers at 10m intervals along X axis (north)
        for x in [10, 20, 30]:
            self.world.targets.append(CircleTarget(
                position=LocalCoord(x, 0, 0.01),
                radius=0.5,
                color=Color.YELLOW,
                surface_normal=(0, 0, 1)
            ))

        # Add markers at 10m intervals along Y axis (east)
        for y in [10, 20, 30]:
            self.world.targets.append(CircleTarget(
                position=LocalCoord(0, y, 0.01),
                radius=0.5,
                color=Color.BLUE,
                surface_normal=(0, 0, 1)
            ))

    def handle_keyboard(self, key: int) -> bool:
        """
        Process keyboard input.

        Returns:
            False if should quit, True otherwise
        """
        if key == 27:  # ESC
            return False

        # Camera pitch controls (up/down arrows) - FIX INVERSION
        if key == 82:  # Up arrow - look up (increase pitch toward positive)
            self.camera_pitch = min(90, self.camera_pitch + 5)  # Increase pitch = look up
            self.camera_config.pitch_deg = self.camera_pitch
            self.logger.info(f"Camera pitch: {self.camera_pitch} deg (negative=down, 0=forward, positive=up)")
            return True
        elif key == 84:  # Down arrow - look down (decrease pitch toward negative)
            self.camera_pitch = max(-90, self.camera_pitch - 5)  # Decrease pitch = look down
            self.camera_config.pitch_deg = self.camera_pitch
            self.logger.info(f"Camera pitch: {self.camera_pitch} deg (negative=down, 0=forward, positive=up)")
            return True

        # Movement keys - body-relative control (relative to drone heading)
        heading_rad = np.radians(self.drone.heading)

        if key == ord('w'):
            # Move forward (in direction drone is facing)
            dx = self.move_step * np.cos(heading_rad)
            dy = self.move_step * np.sin(heading_rad)
            self.drone.move(dx, dy, 0, 0)
            self.logger.info(f"Moved forward to {self.drone.local_pos}")
        elif key == ord('s'):
            # Move backward
            dx = -self.move_step * np.cos(heading_rad)
            dy = -self.move_step * np.sin(heading_rad)
            self.drone.move(dx, dy, 0, 0)
            self.logger.info(f"Moved backward to {self.drone.local_pos}")
        elif key == ord('a'):
            # Move left (perpendicular to heading) - FIXED INVERSION
            dx = self.move_step * np.sin(heading_rad)
            dy = -self.move_step * np.cos(heading_rad)
            self.drone.move(dx, dy, 0, 0)
            self.logger.info(f"Moved left to {self.drone.local_pos}")
        elif key == ord('d'):
            # Move right (perpendicular to heading) - FIXED INVERSION
            dx = -self.move_step * np.sin(heading_rad)
            dy = self.move_step * np.cos(heading_rad)
            self.drone.move(dx, dy, 0, 0)
            self.logger.info(f"Moved right to {self.drone.local_pos}")
        elif key == ord('r'):
            self.drone.move(0, 0, self.move_step, 0)  # Move up - FIXED
            self.logger.info(f"Moved up to {self.drone.local_pos}")
        elif key == ord('f'):
            self.drone.move(0, 0, -self.move_step, 0)  # Move down - FIXED
            self.logger.info(f"Moved down to {self.drone.local_pos}")
        elif key == ord('q'):
            self.drone.move(0, 0, 0, -self.rotate_step)  # Rotate left
            self.logger.info(f"Rotated left to {self.drone.heading:.1f}°")
        elif key == ord('e'):
            self.drone.move(0, 0, 0, self.rotate_step)  # Rotate right
            self.logger.info(f"Rotated right to {self.drone.heading:.1f}°")

        return True

    def render_info_overlay(self, frame: np.ndarray, metadata: list, dt: float):
        """Draw information overlay on frame."""
        local_pos = self.drone.get_local_position()

        # Draw position info
        info_lines = [
            f"GPS: {self.drone.gps.lat:.6f}, {self.drone.gps.lon:.6f}, {self.drone.gps.alt:.1f}m",
            f"Local: X={local_pos.x:.1f}m N, Y={local_pos.y:.1f}m E, Z={local_pos.z:.1f}m",
            f"Heading: {self.drone.heading:.1f} deg | Camera Pitch: {self.camera_pitch:.0f} deg",
            f"FPS: {1.0/dt:.1f}" if dt > 0 else "FPS: --",
            f"Targets visible: {len(metadata)}",
            f"Building at: (20, 15) - Distance: {np.sqrt((local_pos.x-20)**2 + (local_pos.y-15)**2):.1f}m"
        ]

        y_offset = 30
        for line in info_lines:
            # Black outline for readability
            cv2.putText(frame, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            # White text
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
                target_gps = f"     GPS: ({target.gps_position.lat:.6f}, {target.gps_position.lon:.6f}, {target.gps_position.alt:.1f})"

                cv2.putText(frame, target_info, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
                y_offset += 18
                cv2.putText(frame, target_gps, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 200, 200), 1, cv2.LINE_AA)
                y_offset += 18

        # Draw controls help
        help_y = frame.shape[0] - 120
        controls = [
            "W/S: Forward/Back  |  A/D: Left/Right  |  R/F: Up/Down",
            "Q/E: Rotate  |  UP/DOWN Arrows: Camera Pitch",
            "SPACE: Stop  |  ESC: Quit"
        ]
        for line in controls:
            cv2.putText(frame, line, (10, help_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            help_y += 20

    def run(self):
        """Main demo loop."""
        cv2.namedWindow("Synthetic Camera - Standalone Demo", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Synthetic Camera - Standalone Demo", 800, 600)

        self.logger.info("\n" + "="*60)
        self.logger.info("Standalone Demo Started!")
        self.logger.info("Use keyboard to fly around and view targets")
        self.logger.info("="*60 + "\n")

        running = True
        last_time = time.time()

        while running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # Render camera view
            local_pos = self.drone.get_local_position()
            frame, metadata = self.synthetic_camera.render(local_pos, self.drone.heading)

            # Add info overlay
            self.render_info_overlay(frame, metadata, dt)

            cv2.imshow("Synthetic Camera - Standalone Demo", frame)

            # Handle keyboard (non-blocking)
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # Key pressed
                if not self.handle_keyboard(key):
                    running = False

        # Cleanup
        cv2.destroyAllWindows()
        self.logger.info("Demo ended")


def main():
    """Entry point."""
    print("\n" + "="*70)
    print(" Synthetic Camera - Standalone Demo (No SITL Required)")
    print("="*70)
    print("\nThis demo simulates a drone without connecting to SITL.")
    print("Perfect for testing camera projection and GPS calculations!")
    print("\nControls:")
    print("  W/S - Move forward/backward")
    print("  A/D - Move left/right")
    print("  Q/E - Rotate left/right")
    print("  R/F - Move up/down")
    print("  SPACE - Stop all movement")
    print("  ESC - Quit")
    print("\nWorld layout:")
    print("  - You start at (0, 0) - origin")
    print("  - Building is 20m North, 15m East")
    print("  - Fly north (W key) to approach building")
    print("  - Look for colored targets!")
    print("\n" + "="*70 + "\n")

    input("Press Enter to start...")

    demo = StandaloneDemo()
    demo.run()


if __name__ == "__main__":
    main()
