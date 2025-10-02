"""
MAVLink communication interface for drone control.

This module provides a MavlinkComm class that handles MAVLink communication with a drone,
including position tracking, RC channel monitoring, and data stream management.
"""

from pymavlink import mavutil
from util import UINT16_MAX, Coordinate, RCChannel, MavlinkMessageType, Vector3d
import logging
import time


class MavlinkComm:
    """Handles MAVLink communication and data processing for drone control."""

    def __init__(self) -> None:
        """Initialize drone connection and request data streams."""
        self.position: Coordinate | None = None
        # heading in degrees
        self.heading: float | None = None

        self.rc_channels: dict[int, RCChannel] = {
            i: RCChannel(channel=i, raw=0, is_active=False) for i in range(1, 10)
        }

        while not self.mavlink_connect():
            logging.info("Failed to connect to drone, retrying...")
            time.sleep(1)

        while not self.request_data_streams():
            logging.error("Failed to request data streams, retrying...")
            time.sleep(1)

    def mavlink_connect(self) -> bool:
        """Establish MAVLink connection to drone via serial port."""
        try:
            self.mav = mavutil.mavlink_connection(
                "/dev/ttyAMA0", baud=57600, source_component=191, source_system=1
            )
            self.mav.wait_heartbeat()
            logging.info(
                f"Heartbeat received from system {self.mav.target_system}, component {self.mav.target_component}"
            )
        except Exception as e:
            logging.error(f"Failed to connect to drone: {e}")
            return False

        logging.info("Connected to drone")
        return True

    def request_data_streams(self) -> bool:
        """Request position and RC channel data streams from drone."""
        try:
            # Request position data at 1 Hz
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,  # Target system ID (the drone)
                self.mav.target_component,  # Target component ID (autopilot)
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # Position data stream
                1,  # Rate: 1 Hz
                1,  # Start streaming (1=enable, 0=disable)
            )

            # Request RC channel data at 5 Hz
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,  # Target system ID (the drone)
                self.mav.target_component,  # Target component ID (autopilot)
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,  # RC channels data stream
                5,  # Rate: 5 Hz
                1,  # Start streaming (1=enable, 0=disable)
            )

            logging.info("Requested GLOBAL_POSITION_INT and RC_CHANNELS streams")
        except Exception as e:
            logging.error(f"Failed to request data streams: {e}")
            return False

        return True

    def process_data_stream(self) -> bool:
        """Process incoming MAVLink messages and update drone state."""
        msg = self.mav.recv_match(
            type=MavlinkMessageType.values(),
            blocking=False,
        )
        if msg is None:
            return False

        if msg.get_type() == MavlinkMessageType.GLOBAL_POSITION_INT.value:
            logging.info(f"Received GLOBAL_POSITION_INT: {msg}")
            # messages are sent as ints, so we need to convert them to floats
            self.position = Coordinate(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt=msg.alt / 1000.0,  # Convert from mm to m
            )

            # Extract heading from hdg field (in centidegrees, convert to degrees)
            if msg.hdg != UINT16_MAX:
                self.heading = msg.hdg / 100.0

            return True

        elif msg.get_type() == MavlinkMessageType.RC_CHANNELS.value:
            logging.info(f"Received RC_CHANNELS: {msg}")

            raw_channels: list[int] = [
                msg.chan1_raw,
                msg.chan2_raw,
                msg.chan3_raw,
                msg.chan4_raw,
                msg.chan5_raw,
                msg.chan6_raw,
                msg.chan7_raw,
                msg.chan8_raw,
                msg.chan9_raw,
            ]

            for i, raw in enumerate(raw_channels, start=1):
                raw = raw if raw is not None else 0
                self.rc_channels[i] = RCChannel(
                    channel=i, raw=raw, is_active=raw >= 1200
                )

            return True

        return False

    def get_position(self) -> Coordinate:
        """Get current drone position, returns (0, 0, 0) if unavailable."""
        if self.position is None:
            logging.warning("Position is not available")
            return Coordinate(lat=0, lon=0, alt=0)
        return self.position

    def get_rc_channel(self, channel: int) -> RCChannel:
        """Get RC channel data for specified channel number."""
        if channel not in self.rc_channels:
            logging.warning(f"Channel {channel} is not available")
            return RCChannel(channel=channel, raw=0, is_active=False)
        return self.rc_channels[channel]

    def get_heading(self) -> float:
        """Get current drone heading in degrees, returns 0 if unavailable."""
        if self.heading is None:
            logging.warning("Heading is not available")
            return 0.0
        return self.heading

    def set_body_velocity(self, velocity: Vector3d, attempt: int = 0) -> None:
        """Set drone velocity in body frame (x=forward, y=right, z=down)."""
        if attempt > 3:
            logging.error("Failed to set body velocity after 3 attempts")
            return

        try:
            # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned
            self.mav.mav.set_position_target_local_ned_send(
                int(
                    self.mav.time_since("SYSTEM_TIME") * 1e6
                ),  # Timestamp in microseconds
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b110111000111,  # Type mask: enable velocity X,Y (disable position, accel, etc.)
                0,
                0,
                0,  # Position (unused, masked out)
                velocity.x,
                velocity.y,
                velocity.z,
                0,
                0,
                0,  # Acceleration (unused, masked out)
                0,
                0,  # Yaw and yaw rate (unused, maintain current heading)
            )
        except Exception as e:
            logging.error(f"Failed to set body velocity: {e}")
            self.set_body_velocity(velocity, attempt + 1)

    def send_coordinate_to_ground(
        self, coordinate: Coordinate, attempt: int = 0
    ) -> None:
        """Send coordinate to ground station."""
        if attempt > 3:
            logging.error("Failed to send coordinate to ground after 3 attempts")
            return

        try:
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                f"coord_{coordinate.lat}_{coordinate.lon}_{coordinate.alt}".encode(),  # Convert string to bytes
            )
        except Exception as e:
            logging.error(f"Failed to send coordinate to ground: {e}")
            self.send_coordinate_to_ground(coordinate, attempt + 1)
