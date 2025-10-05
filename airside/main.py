"""
Airside main control loop for drone-based target detection and building mapping.

This module handles:
- Dual camera operation (down-facing and forward-facing)
- Building corner and height recording
- Target detection and auto-centering
- Target coordinate transmission to ground station
"""

import logging
import math

import cv2
import numpy as np
from airside.building import Building
from airside.camera import Camera
from airside.mavlink_comm import MavlinkComm
from airside.hud import HudState, overlay_hud
from util import Coordinate, Vector3d

# This proportional control gain determines how aggressively the drone moves
# to correct position errors. Smaller values = gentler, more stable movement
PX_TO_MS = 0.00002  # (m/s) per pixel

MODE_CHANGE_CHANNEL = 7
RESOURCE_RECORD_CHANNEL_A = 8
RESOURCE_RECORD_CHANNEL_B = 9

# Target locking threshold: maximum allowed pixel error for successful lock
ERROR_RADIUS_PX = 40  # pixels


def main() -> None:
    """Main control loop for airside drone operations."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting airside...")

    # Create HUD display windows
    cv2.namedWindow("Down Camera", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Forward Camera", cv2.WINDOW_NORMAL)

    mav_comm = MavlinkComm()

    # Initialize dual cameras
    # Camera 0: Down-facing (for building recording/mapping and roof targets)
    # Camera 1: Forward-facing (for target detection on walls)
    camera_down = Camera(camera_index=0)
    camera_forward = Camera(camera_index=1)

    # Initialize building geometry manager
    building = Building()

    is_building_record_mode = True
    recorded_resource = False

    # HUD state tracking
    hud_state_down = HudState()
    hud_state_forward = HudState()

    def building_record_functions():
        """
        Handle building record

        Channel A: Record building corner
        Channel B: Record building height
        """
        nonlocal recorded_resource
        channel_a = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_A).is_active
        channel_b = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_B).is_active

        if channel_a and channel_b:
            logging.error("Both building record channels are active")
            recorded_resource = False
            return

        # Record building corner (Channel A)
        if channel_a and not recorded_resource:
            position = mav_comm.get_position()
            building.record_corner(position)
            corner_idx = (building.corner_record_cursor - 1) % 4
            logging.info(f"Recorded building corner {corner_idx}: {position}")
            mav_comm.send_ack_to_ground(f"c_{building.corners[corner_idx]}")
            recorded_resource = True

        # Record building height (Channel B)
        elif channel_b and not recorded_resource:
            altitude = mav_comm.get_position().alt
            building.record_height(altitude)
            logging.info(f"Recorded building height: {building.height}m")
            mav_comm.send_ack_to_ground(f"h_{building.height}")
            recorded_resource = True

        # Reset flag when channels are released
        elif not (channel_a or channel_b) and recorded_resource:
            recorded_resource = False
            logging.debug("Resource recording flag reset")

    def target_detection_functions(frame_down: np.ndarray, frame_forward: np.ndarray):
        """
        Handle target detection and locking for both cameras.

        Channel A: Lock onto target in down-facing camera
        Channel B: Lock onto target in forward-facing camera
        """
        nonlocal recorded_resource
        channel_a = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_A).is_active
        channel_b = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_B).is_active

        if channel_a and channel_b:
            logging.error("Both target record channels are active")
            recorded_resource = False
            return

        # Reset recorded_resource when both channels are off
        if not (channel_a or channel_b) and recorded_resource:
            recorded_resource = False
            return

        # Process down-facing camera (Channel A)
        if channel_a and frame_down is not None:
            _process_target_locking(
                frame_down, camera_down, is_down_facing=True, hud_state=hud_state_down
            )

        # Process forward-facing camera (Channel B)
        elif channel_b and frame_forward is not None:
            _process_target_locking(
                frame_forward,
                camera_forward,
                is_down_facing=False,
                hud_state=hud_state_forward,
            )

    def _process_target_locking(
        frame: np.ndarray, camera: Camera, is_down_facing: bool, hud_state: dict
    ):
        """
        Process target detection and locking for a specific camera.
        """
        nonlocal recorded_resource
        camera_name = "DOWN" if is_down_facing else "FORWARD"

        if frame is None or frame.size == 0:
            logging.warning(f"{camera_name} camera frame is invalid")
            # Reset HUD state
            hud_state.reset()
            return

        # Calculate image center
        img_center_x = frame.shape[1] // 2  # Horizontal center (width / 2)
        img_center_y = frame.shape[0] // 2  # Vertical center (height / 2)

        target_colour = camera.colour_in_frame(frame)

        # Update HUD state with detected color
        hud_state.target_colour = target_colour

        # Early return if no target color detected
        if target_colour is None:
            logging.debug(f"{camera_name} camera - No target colour detected")
            # Clear target-related HUD state
            hud_state.reset()
            return

        target_center = camera.center_of_target_in_frame(frame, target_colour)

        if target_center:
            target_center_x, target_center_y = target_center

            offset_x = target_center_x - img_center_x
            offset_y = target_center_y - img_center_y

            error = math.hypot(offset_x, offset_y)

            # Update HUD state with target tracking data
            hud_state.update_target(
                center=target_center,
                colour=target_colour,
                offset_x=float(offset_x),
                offset_y=float(offset_y),
                error=float(error),
            )

            logging.info(
                f"{camera_name} camera - Target detected at ({target_center_x}, {target_center_y}), "
                f"offset: ({offset_x}, {offset_y}), error: {error:.2f} px"
            )

            if error <= ERROR_RADIUS_PX and not recorded_resource:
                drone_position = mav_comm.get_position()

                if not is_down_facing:
                    target_position = building.find_target_on_wall(
                        drone_position, mav_comm.get_heading()
                    )

                    # Validate that we found a valid wall intersection
                    if target_position.lat == 0 and target_position.lon == 0:
                        logging.warning(
                            f"{camera_name} camera - Failed to find target on wall, "
                            "building may not be fully mapped"
                        )
                        return
                else:
                    if building.in_bounds(drone_position):
                        target_position = Coordinate(
                            lat=drone_position.lat,
                            lon=drone_position.lon,
                            alt=building.height,
                        )
                    else:
                        target_position = Coordinate(
                            lat=drone_position.lat, lon=drone_position.lon, alt=0.0
                        )

                logging.info(
                    f"{camera_name} camera - Lock achieved! Error: {error:.2f} px <= {ERROR_RADIUS_PX} px. "
                    f"Sending target at {target_position} (colour: {target_colour.name}) to ground station"
                )

                velocity = Vector3d(0, 0, 0)
                mav_comm.set_body_velocity(velocity)
                mav_comm.send_target_to_ground(target_position, target_colour)
                recorded_resource = True

                # Update HUD state for lock
                hud_state.update_velocity(velocity)
                hud_state.set_locked(True)

                return

            if not recorded_resource:
                if is_down_facing:
                    velocity = Vector3d(
                        x=-offset_y * PX_TO_MS,
                        y=offset_x * PX_TO_MS,
                        z=0.0,
                    )
                else:
                    velocity = Vector3d(
                        x=0.0,
                        y=offset_x * PX_TO_MS,
                        z=offset_y * PX_TO_MS,
                    )

                logging.info(
                    f"{camera_name} camera - Sending velocity command: "
                    f"X={velocity.x:.6f}, Y={velocity.y:.6f}, Z={velocity.z:.6f} m/s"
                )
                mav_comm.set_body_velocity(velocity)

                # Update HUD state with velocity
                hud_state.update_velocity(velocity)
                hud_state.set_locked(False)
            else:
                logging.info(
                    f"{camera_name} camera - Target already sent to ground, stopping movement"
                )
                velocity = Vector3d(0, 0, 0)
                mav_comm.set_body_velocity(velocity)

                # Update HUD state
                hud_state.update_velocity(velocity)
                hud_state.set_locked(True)
        else:
            logging.warning(f"{camera_name} camera - No target detected in frame")
            # Clear target HUD state
            hud_state.reset_target()

    while True:
        frame_down = camera_down.capture_frame()
        frame_forward = camera_forward.capture_frame()

        while mav_comm.process_data_stream():
            pass

        # Check mode switch (Channel 7)
        mode_channel_active = mav_comm.get_rc_channel(MODE_CHANGE_CHANNEL).is_active

        # Send building info when transitioning from building mode to target mode
        if is_building_record_mode and not mode_channel_active:
            logging.info("Switching to target detection mode, sending building info")
            mav_comm.send_building_info_to_ground(building)

        # Update mode state
        is_building_record_mode = mode_channel_active

        # Execute mode-specific functions
        if is_building_record_mode:
            building_record_functions()
        else:
            target_detection_functions(frame_down, frame_forward)

        # Display HUD overlays
        if frame_down is not None and frame_down.size > 0:
            mode_str = "BUILDING_RECORD" if is_building_record_mode else "TARGET_DETECT"
            corner_count = (
                building.corner_record_cursor if is_building_record_mode else None
            )

            hud_frame_down = overlay_hud(
                frame=frame_down,
                camera_label="DOWN",
                mode=mode_str,
                hud_state=hud_state_down,
                corner_count=corner_count,
                error_threshold_px=ERROR_RADIUS_PX,
            )
            cv2.imshow("Down Camera", hud_frame_down)

        if frame_forward is not None and frame_forward.size > 0:
            mode_str = "BUILDING_RECORD" if is_building_record_mode else "TARGET_DETECT"
            corner_count = (
                building.corner_record_cursor if is_building_record_mode else None
            )

            hud_frame_forward = overlay_hud(
                frame=frame_forward,
                camera_label="FORWARD",
                mode=mode_str,
                hud_state=hud_state_forward,
                corner_count=corner_count,
                error_threshold_px=ERROR_RADIUS_PX,
            )
            cv2.imshow("Forward Camera", hud_frame_forward)

        # Process keyboard input (required for cv2.imshow to work)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            logging.info("'q' pressed, exiting...")
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received, exiting gracefully...")
    except Exception as e:
        logging.error(f"Unexpected error in main loop: {e}", exc_info=True)
        raise
    finally:
        # Clean up cv2 windows
        cv2.destroyAllWindows()
        logging.info("HUD windows closed")
