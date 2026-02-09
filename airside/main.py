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
import os
import pickle
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Literal

import cv2
import numpy as np
from .building import Building
from .camera import Camera
from .mavlink_comm import MavlinkComm
from .hud import HudState, overlay_hud
from util import Coordinate, Vector3d

# This proportional control gain determines how aggressively the drone moves
# to correct position errors. Smaller values = gentler, more stable movement
MODE_CHANGE_CHANNEL = 11
RESOURCE_RECORD_CHANNEL_A = 12
RESOURCE_RECORD_CHANNEL_B = 13
FRAME_CAPTURE_CHANNEL = 14
GUI_ENABLED = False

FOCAL_LENGTH_PX = 1000
in_loiter = False

# Target locking threshold: maximum allowed pixel error for successful lock
ERROR_RADIUS_PX = 40  # pixels


@dataclass
class CameraConfig:
    """Configuration for a single camera instance."""

    camera: Camera
    hud_state: HudState
    window_name: str
    label: str
    is_down_facing: bool
    channel: int  # RC channel for this camera (A or B)


def handle_building_record(
    mav_comm: MavlinkComm, building: Building, recorded_resource: bool
) -> bool:
    """
    Handle building record operations.

    Channel A: Record building corner
    Channel B: Record building height

    Returns:
        Updated recorded_resource flag
    """
    channel_a = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_A).is_active
    channel_b = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_B).is_active

    if channel_a and channel_b:
        logging.error("Both building record channels are active")
        return False

    # Record building corner (Channel A)
    if channel_a and not recorded_resource:
        position = mav_comm.get_position()
        building.record_corner(position)
        corner_idx = (building.corner_record_cursor - 1) % 4
        logging.info(f"Recorded building corner {corner_idx}: {position}")
        mav_comm.send_ack_to_ground(f"c_{building.corners[corner_idx]}")
        return True

    # Record building height (Channel B)
    elif channel_b and not recorded_resource:
        altitude = mav_comm.get_position().alt
        building.record_height(altitude)
        logging.info(f"Recorded building height: {building.height}m")
        mav_comm.send_ack_to_ground(f"h_{building.height}")
        return True

    # Reset flag when channels are released
    elif not (channel_a or channel_b) and recorded_resource:
        logging.debug("Resource recording flag reset")
        return False

    return recorded_resource


def process_target_locking(
    frame: np.ndarray,
    camera_config: CameraConfig,
    mav_comm: MavlinkComm,
    building: Building,
    recorded_resource: bool,
) -> bool:
    """
    Process target detection and locking for a specific camera.

    Returns:
        Updated recorded_resource flag
    """
    global in_loiter

    camera_name = camera_config.label
    hud_state = camera_config.hud_state

    if frame is None or frame.size == 0:
        logging.warning(f"{camera_name} camera frame is invalid")
        hud_state.reset()
        return recorded_resource

    # Calculate image center
    img_center_x = frame.shape[1] // 2  # Horizontal center (width / 2)
    img_center_y = frame.shape[0] // 2  # Vertical center (height / 2)

    target = camera_config.camera.get_best_target(
        camera_config.camera.find_targets(frame)
    )

    if target is None:
        logging.debug(f"{camera_name} camera - No target colour detected")
        hud_state.reset()
        return recorded_resource

    # Update HUD state with detected color
    hud_state.target_colour = target.colour

    target_center = (int(target.x), int(target.y))
    if target_center:
        target_center_x, target_center_y = target_center

        offset_x = target_center_x - img_center_x
        offset_y = target_center_y - img_center_y

        error = math.hypot(offset_x, offset_y)

        # Update HUD state with target tracking data
        hud_state.update_target(
            center=target_center,
            colour=target.colour,
            offset_x=float(offset_x),
            offset_y=float(offset_y),
            error=float(error),
        )

        logging.info(
            f"{camera_name} camera - Target detected at ({target_center_x}, {target_center_y}), "
            f"offset: ({offset_x}, {offset_y}), error: {error:.2f} px"
        )

        if not in_loiter:
            in_loiter = mav_comm.enter_loiter()

        if error <= ERROR_RADIUS_PX and not recorded_resource:
            drone_position = mav_comm.get_position()

            if not camera_config.is_down_facing:
                target_position = building.find_target_on_wall(
                    drone_position, mav_comm.get_heading()
                )

                # Validate that we found a valid wall intersection
                if target_position.lat == 0 and target_position.lon == 0:
                    logging.warning(
                        f"{camera_name} camera - Failed to find target on wall, "
                        "building may not be fully mapped"
                    )
                    return recorded_resource
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
                f"Sending target at {target_position} (colour: {target.colour.name}) to ground station"
            )

            mav_comm.set_landing_target()  # default angle is [0,0]
            mav_comm.send_target_to_ground(target_position, target.colour)
            recorded_resource = True

            # Update HUD state for lock
            hud_state.set_locked(True)

            return True

        if not recorded_resource:
            if camera_config.is_down_facing:
                angleOffset = [
                    math.atan2(offset_x, FOCAL_LENGTH_PX),
                    math.atan2(-offset_y, FOCAL_LENGTH_PX),
                ]
            else:
                """
                i believe this still needs to be changed to reflect the fact that this is a forward facing camera.
                even if we used matrix math to change the angles, since the precision loiter
                system expects angles relative to the down direction of the drone, i think it would just move
                forwards/backwards and left/right rather than up/down and left/right.

                setting PLND_ORIENT and PLND_YAW_ALIGN could fix this, but would require a reboot on change
                """
                angleOffset = [
                    math.atan2(offset_x, FOCAL_LENGTH_PX),
                    math.atan2(-offset_y, FOCAL_LENGTH_PX),
                ]

            logging.info(
                f"{camera_name} camera - Sending target angle offset: "
                f"X={angleOffset[0]:.6f}, Y={angleOffset[1]:.6f} radians"
            )
            mav_comm.set_landing_target(angleOffset)

            # Update HUD state with velocity
            hud_state.set_locked(False)
        else:
            logging.info(
                f"{camera_name} camera - Target already sent to ground, stopping movement"
            )
            angleOffset = [0, 0]
            mav_comm.set_landing_target(angleOffset)

            # Update HUD state
            hud_state.set_locked(True)
        # if target is not in the camera frame
    else:
        if in_loiter:
            # Reset loiter state
            in_loiter = (
                not mav_comm.enter_guided()
            )  # right now it enters guided mode, might need to change depending on what pilots say

        logging.warning(f"{camera_name} camera - No target detected in frame")
        # Clear target HUD state
        hud_state.reset_target()

    return recorded_resource


def handle_target_detection(
    camera_configs: dict[str, CameraConfig],
    frames: dict[str, np.ndarray],
    mav_comm: MavlinkComm,
    building: Building,
    recorded_resource: bool,
) -> bool:
    """
    Handle target detection and locking for all cameras.

    Returns:
        Updated recorded_resource flag
    """
    channel_a = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_A).is_active
    channel_b = mav_comm.get_rc_channel(RESOURCE_RECORD_CHANNEL_B).is_active

    if channel_a and channel_b:
        logging.error("Both target record channels are active")
        return False

    # Reset recorded_resource when both channels are off
    if not (channel_a or channel_b) and recorded_resource:
        return False

    # Find which camera config corresponds to the active channel
    for config in camera_configs.values():
        channel_active = mav_comm.get_rc_channel(config.channel).is_active
        if channel_active and frames.get(config.label) is not None:
            return process_target_locking(
                frames[config.label], config, mav_comm, building, recorded_resource
            )

    return recorded_resource


# store array of saved frames
saved_frames = []
previous_capture = False


def main() -> None:
    """Main control loop for airside drone operations."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting airside...")

    mav_comm = MavlinkComm()
    building = Building()

    # Initialize camera configurations
    # Camera 0: Down-facing (for building recording/mapping and roof targets)
    # Camera 1: Forward-facing (for target detection on walls)
    camera_configs = {
        "DOWN": CameraConfig(
            camera=Camera(camera_index=0),
            hud_state=HudState(),
            window_name="Down Camera",
            label="DOWN",
            is_down_facing=True,
            channel=RESOURCE_RECORD_CHANNEL_A,
        ),
        "FORWARD": CameraConfig(
            camera=Camera(camera_index=1),
            hud_state=HudState(),
            window_name="Forward Camera",
            label="FORWARD",
            is_down_facing=False,
            channel=RESOURCE_RECORD_CHANNEL_B,
        ),
    }

    # Create HUD display windows
    if GUI_ENABLED:
        for config in camera_configs.values():
            cv2.namedWindow(config.window_name, cv2.WINDOW_NORMAL)

    is_building_record_mode = True
    recorded_resource = False

    while True:
        # Capture frames from all cameras
        frames = {
            label: config.camera.capture_frame()
            for label, config in camera_configs.items()
        }

        # Process MAVLink data stream
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
            recorded_resource = handle_building_record(
                mav_comm, building, recorded_resource
            )
        else:
            recorded_resource = handle_target_detection(
                camera_configs, frames, mav_comm, building, recorded_resource
            )

        # Display HUD overlays for all cameras
        mode_str = "BUILDING_RECORD" if is_building_record_mode else "TARGET_DETECT"
        corner_count = (
            building.corner_record_cursor if is_building_record_mode else None
        )

        for label, config in camera_configs.items():
            frame = frames.get(label)
            if frame is not None and frame.size > 0:
                hud_frame = overlay_hud(
                    frame=frame,
                    camera_label=config.label,
                    mode=mode_str,
                    hud_state=config.hud_state,
                    corner_count=corner_count,
                    error_threshold_px=ERROR_RADIUS_PX,
                )
                if GUI_ENABLED:
                    cv2.imshow(config.window_name, hud_frame)

        # Process keyboard input (required for cv2.imshow to work)
        if GUI_ENABLED:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logging.info("'q' pressed, exiting...")
                break


def local_test() -> None:
    """Single-camera test for testing on local environments."""
    global previous_capture
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting airside...")

    mav_comm = MavlinkComm()
    logging.info("Mavlink communication established..")
    building = Building()

    logging.info("Starting airside...")
    # Initialize camera configurations
    # Camera 0: Down-facing (for building recording/mapping and roof targets)
    # Camera 1: Forward-facing (for target detection on walls)
    camera_configs = {
        "DOWN": CameraConfig(
            camera=Camera(camera_index=0, mode="sim", mav_comm=mav_comm),
            hud_state=HudState(),
            window_name="Down Camera",
            label="DOWN",
            is_down_facing=True,
            channel=RESOURCE_RECORD_CHANNEL_A,
        ),
        "FORWARD": CameraConfig(
            camera=Camera(camera_index=1, mode="oakd", mav_comm=mav_comm),
            hud_state=HudState(),
            window_name="Forward Camera",
            label="FORWARD",
            is_down_facing=False,
            channel=RESOURCE_RECORD_CHANNEL_B,
        ),
    }

    # Create HUD display windows
    for config in camera_configs.values():
        cv2.namedWindow(config.window_name, cv2.WINDOW_NORMAL)

    is_building_record_mode = True
    recorded_resource = False
    logging.info("Entering event loop..")

    while True:
        # Capture frames from all cameras
        frames = {
            label: config.camera.capture_frame()
            for label, config in camera_configs.items()
        }

        # Process MAVLink data stream
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
            recorded_resource = handle_building_record(
                mav_comm, building, recorded_resource
            )
        else:
            recorded_resource = handle_target_detection(
                camera_configs, frames, mav_comm, building, recorded_resource
            )

        # Display HUD overlays for all cameras
        mode_str = "BUILDING_RECORD" if is_building_record_mode else "TARGET_DETECT"
        corner_count = (
            building.corner_record_cursor if is_building_record_mode else None
        )

        frame_capture_signal = mav_comm.get_rc_channel(FRAME_CAPTURE_CHANNEL).is_active
        for label, config in camera_configs.items():
            frame = frames.get(label)
            if frame is not None and frame.size > 0:
                if frame_capture_signal and not previous_capture:
                    saved_frames.append(frame)

                hud_frame = overlay_hud(
                    frame=frame,
                    camera_label=config.label,
                    mode=mode_str,
                    hud_state=config.hud_state,
                    corner_count=corner_count,
                    error_threshold_px=ERROR_RADIUS_PX,
                )

                if GUI_ENABLED:
                    cv2.imshow(config.window_name, hud_frame)

        previous_capture = frame_capture_signal
        if len(saved_frames) >= 10:
            os.makedirs("frames", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"frames/{timestamp}.frames"
            with open(filename, "wb") as f:
                pickle.dump(saved_frames, f)
            logging.info(f"Saved {len(saved_frames)} frames to {filename}")
            saved_frames.clear()

        if GUI_ENABLED:
            # Process keyboard input (required for cv2.imshow to work)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logging.info("'q' pressed, exiting...")
                break


if __name__ == "__main__":
    try:
        local_test()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received, exiting gracefully...")
    except Exception as e:
        logging.error(f"Unexpected error in main loop: {e}", exc_info=True)
        raise
    finally:
        # Clean up cv2 windows
        if GUI_ENABLED:
            cv2.destroyAllWindows()
            logging.info("HUD windows closed")
