"""
MAVLink communication interface for drone control.

This module provides a MavlinkComm class that handles MAVLink communication with a drone,
including position tracking, RC channel monitoring, and data stream management.
"""

from pymavlink import mavutil
from util import (
    UINT16_MAX,
    AIRSIDE_COMPONENT_ID,
    Colour,
    Coordinate,
    RCChannel,
    MavlinkMessageType,
    Vector3d,
)
from airside.building import Building
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
            i: RCChannel(channel=i, raw=0, is_active=False) for i in range(1, 20)
        }

        while not self.__mavlink_connect():
            logging.info("Failed to connect to drone, retrying...")
            time.sleep(1)

        while not self.__request_data_streams():
            logging.error("Failed to request data streams, retrying...")
            time.sleep(1)

    def __mavlink_connect(self) -> bool:
        """Establish MAVLink connection to drone via serial port."""
        try:
            self.mav = mavutil.mavlink_connection(
                "tcp:192.168.196.67:5762",
                baud=57600,
                source_component=AIRSIDE_COMPONENT_ID,
                source_system=1,
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

    def __request_data_streams(self) -> bool:
        """Request position and RC channel data streams from drone."""
        try:
            # Request position data at 20 Hz for smooth camera simulation
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,  # Target system ID (the drone)
                self.mav.target_component,  # Target component ID (autopilot)
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # Position data stream
                20,  # Rate: 20 Hz
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

            logging.info(
                "Requested GLOBAL_POSITION_INT (20 Hz) and RC_CHANNELS (5 Hz) streams"
            )
        except Exception as e:
            logging.error(f"Failed to request data streams: {e}")
            return False

        return True

    def process_data_stream(self) -> bool:
        """Process incoming MAVLink messages and update drone state."""
        msg = self.mav.recv_match(
            type=[m.value for m in MavlinkMessageType],
            blocking=False,
        )
        if msg is None:
            return False

        if msg.get_type() == MavlinkMessageType.GLOBAL_POSITION_INT.value:
            # logging.info(f"Received GLOBAL_POSITION_INT: {msg}")
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
            # logging.info(f"Received RC_CHANNELS: {msg}")

            # Update RC channels by reading 'chanX_raw' attributes from message
            for rc_channel_num in self.rc_channels.keys():
                attr_name = f"chan{rc_channel_num}_raw"
                if not hasattr(msg, attr_name):
                    continue
                raw = getattr(msg, attr_name)
                raw = raw if raw is not None else 0
                self.rc_channels[rc_channel_num] = RCChannel(
                    channel=rc_channel_num, raw=raw, is_active=raw >= 1200
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
    def set_landing_target(self, angleOffset: list[float] = [0, 0], attempt: int = 0) -> None:
            """Set landing_target in body frame with (radian) angle offset."""
            if attempt > 3:
                logging.error("Failed to send landing target after 3 attempts")
                return

            try:
                # https://ardupilot.org/dev/docs/mavlink-precision-landing.html
                timestamp = int(
                    self.mav.time_since("SYSTEM_TIME") * 1e6
                )  # Timestamp in microseconds

                frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED # same as MAV_FRAME_BODY_FRD
                self.mav.mav.landing_target_send(
                    timestamp,
                    0,  # target_num (unused)
                    frame,
                    angleOffset[0], # X angle offset in radians
                    angleOffset[1], # Y angle offset in radians
                    0,  # Distance (unknown)
                    0,  # Size_x (unused)
                    0,  # Size_y (unused)
                    0,  # x position (unknown)
                    0,  # y position (unknown)
                    0,  # z position (unknown)
                    0,  # quaternion (unused)
                    0,  # type (unused)
                    0,  # position_valid = 0 to use angles rather than xyz
                        # https://ardupilot.org/dev/docs/mavlink-precision-landing.html
                )

            except Exception as e:
                logging.error(f"Failed to set landing_target: {e}")
                self.set_landing_target(angleOffset, attempt + 1)


    def enter_loiter(self, attempt: int = 0) -> bool:
        """Enter loiter mode."""
        if attempt > 3:
            logging.error("Failed to enter loiter mode after 3 attempts")
            return False
    
        try:
            # send command to change to loiter mode
            self.mav.mav.command_long_send(
                self.mav.target_system,     # Target system ID
                self.mav.target_component,  # Target component ID
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # ID of command to send
                0,                          # Confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   # param1: set custom mode
                5,                          # param2: loiter mode (https://ardupilot.org/copter/docs/parameters.html#fltmode1)
                0,
                0,
                0,
                0,
                0
            )
            logging.info("Entered loiter mode")

        except Exception as e:
            logging.error(f"Failed to enter loiter mode: {e}")
            self.enter_loiter(attempt + 1)

        return True

    def enter_guided(self, attempt: int = 0) -> bool:
        """Exit guided mode."""
        if attempt > 3:
            logging.error("Failed to enter guided mode after 3 attempts")
            return False
        
        try:
            # send command to change to guided mode
            self.mav.mav.command_long_send(
                self.mav.target_system,     # Target system ID
                self.mav.target_component,  # Target component ID
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # ID of command to send
                0,                          # Confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   # param1: set custom mode
                4,                          # param2: guided mode (https://ardupilot.org/copter/docs/parameters.html#fltmode1)
                0,
                0,
                0,
                0,
                0
            )

            logging.info("Entered guided mode")
            
        except Exception as e:
            logging.error(f"Failed to enter guided mode: {e}")
            self.enter_guided(attempt + 1)

        return True

    def send_target_to_ground(
        self, coordinate: Coordinate, colour: Colour, attempt: int = 0
    ) -> None:
        """Send target to ground station."""
        if attempt > 3:
            logging.error("Failed to send target to ground after 3 attempts")
            return
        logging.info(
            f"sending target with colour {colour.name} at {coordinate.lat}, {coordinate.lon}, {coordinate.alt}"
        )

        try:
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                f"t_{coordinate.lat}_{coordinate.lon}_{coordinate.alt}_{colour.name}".encode(),  # Convert string to bytes
            )
        except Exception as e:
            logging.error(f"Failed to send target to ground: {e}")
            self.send_target_to_ground(coordinate, colour, attempt + 1)

    def send_ack_to_ground(self, msg: str, attempt: int = 0) -> None:
        """Send acknowledgement to ground station."""
        if attempt > 3:
            logging.error("Failed to send acknowledgement to ground after 3 attempts")
            return

        try:
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                f"a_{msg}".encode(),
            )
        except Exception as e:
            logging.error(f"Failed to send acknowledgement to ground: {e}")
            self.send_ack_to_ground(msg, attempt + 1)

    def send_building_info_to_ground(
        self, building: Building, attempt: int = 0
    ) -> None:
        """Send building info to ground station."""
        if attempt > 3:
            logging.error("Failed to send building info to ground after 3 attempts")
            return

        try:
            for corner in building.corners:
                print("Corner: ", corner)
                if corner is None:
                    continue
                self.mav.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                    f"b_{corner.lat}_{corner.lon}_{corner.alt}".encode(),
                )
        except Exception as e:
            logging.error(f"Failed to send building info to ground: {e}")
            self.send_building_info_to_ground(building, attempt + 1)
