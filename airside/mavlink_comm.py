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
            i: RCChannel(channel=i, raw=0, is_active=False) for i in range(7, 11)
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
                "/dev/ttyAMA0",
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
            type=[m.value for m in MavlinkMessageType],
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

    def set_body_velocity(self, velocity: Vector3d, attempt: int = 0) -> None:
        """Set drone velocity in body frame (x=forward, y=right, z=down)."""
        if attempt > 3:
            logging.error("Failed to set body velocity after 3 attempts")
            return

        try:
            # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned
            timestamp = int(
                self.mav.time_since("SYSTEM_TIME") * 1e6
            )  # Timestamp in microseconds
            type_mask = 0b110111000111  # Type mask: enable velocity X,Y (disable position, accel, etc.)
            frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
            self.mav.mav.set_position_target_local_ned_send(
                timestamp,
                self.mav.target_system,
                self.mav.target_component,
                frame,
                type_mask,
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

    def send_target_to_ground(
        self, coordinate: Coordinate, colour: Colour, attempt: int = 0
    ) -> None:
        """Send target to ground station."""
        if attempt > 3:
            logging.error("Failed to send target to ground after 3 attempts")
            return

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
                if corner is None:
                    continue
                self.mav.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                    f"b_{corner.lat}_{corner.lon}_{corner.alt}".encode(),
                )
        except Exception as e:
            logging.error(f"Failed to send building info to ground: {e}")
            self.send_building_info_to_ground(building, attempt + 1)

    def set_waypoint(self, coordinate: Coordinate, attempt: int = 0) -> bool:
        """
        Set a waypoint for the drone to navigate to.

        Args:
            coordinate: Target waypoint location (lat, lon, alt)
            attempt: Retry attempt number (internal use)

        Returns:
            True if waypoint command sent successfully, False otherwise
        """
        if attempt > 3:
            logging.error("Failed to set waypoint after 3 attempts")
            return False

        try:
            # Convert coordinates to MAVLink format (degrees * 1e7 for lat/lon, mm for alt)
            lat_int = int(coordinate.lat * 1e7)
            lon_int = int(coordinate.lon * 1e7)
            alt_int = int(coordinate.alt * 1000)  # Convert meters to mm

            # Use SET_POSITION_TARGET_GLOBAL_INT for waypoint navigation
            # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
            timestamp = int(self.mav.time_since("SYSTEM_TIME") * 1e6)
            type_mask = 0b110111111000  # Type mask: enable position (disable velocity, accel, etc.)
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

            self.mav.mav.set_position_target_global_int_send(
                timestamp,
                self.mav.target_system,
                self.mav.target_component,
                frame,
                type_mask,
                lat_int,
                lon_int,
                alt_int,
                0,  # Velocity X (unused)
                0,  # Velocity Y (unused)
                0,  # Velocity Z (unused)
                0,  # Acceleration X (unused)
                0,  # Acceleration Y (unused)
                0,  # Acceleration Z (unused)
                0,  # Yaw (unused, maintain current)
                0,  # Yaw rate (unused)
            )

            logging.info(f"Waypoint set to {coordinate}")
            return True

        except Exception as e:
            logging.error(f"Failed to set waypoint: {e}")
            self.set_waypoint(coordinate, attempt + 1)
            return False

    def activate_extinguish(self, duration: float = 2.0, attempt: int = 0) -> bool:
        """
        Activate extinguishing mechanism via servo control.

        This assumes a servo is connected to control the extinguishing mechanism.
        The servo channel and PWM values should be configured based on hardware setup.

        Args:
            duration: How long to keep extinguishing mechanism active (seconds)
            attempt: Retry attempt number (internal use)

        Returns:
            True if servo command sent successfully, False otherwise
        """
        if attempt > 3:
            logging.error("Failed to activate extinguishing mechanism after 3 attempts")
            return False

        try:
            # Servo channel for extinguishing mechanism (adjust based on hardware)
            # Channel 10 is commonly used for auxiliary functions
            EXTINGUISH_SERVO_CHANNEL = 10

            # PWM values (microseconds)
            # These values depend on your servo setup - adjust as needed
            SERVO_OFF_PWM = 1500  # Neutral/off position
            SERVO_ON_PWM = 2000  # Active/extinguishing position

            # Set servo to active position
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # Confirmation
                EXTINGUISH_SERVO_CHANNEL,  # Servo channel
                SERVO_ON_PWM,  # PWM value (microseconds)
                0,  # Unused
                0,  # Unused
                0,  # Unused
                0,  # Unused
                0,  # Unused
            )

            logging.info(
                f"Extinguishing mechanism activated on channel {EXTINGUISH_SERVO_CHANNEL} "
                f"for {duration}s"
            )

            # Note: The servo will remain active until we send the OFF command
            # The duration is handled by the calling code

            return True

        except Exception as e:
            logging.error(f"Failed to activate extinguishing mechanism: {e}")
            self.activate_extinguish(duration, attempt + 1)
            return False

    def deactivate_extinguish(self, attempt: int = 0) -> bool:
        """
        Deactivate extinguishing mechanism by returning servo to neutral position.

        Args:
            attempt: Retry attempt number (internal use)

        Returns:
            True if servo command sent successfully, False otherwise
        """
        if attempt > 3:
            logging.error("Failed to deactivate extinguishing mechanism after 3 attempts")
            return False

        try:
            EXTINGUISH_SERVO_CHANNEL = 10
            SERVO_OFF_PWM = 1500  # Neutral/off position

            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # Confirmation
                EXTINGUISH_SERVO_CHANNEL,  # Servo channel
                SERVO_OFF_PWM,  # PWM value (microseconds)
                0,  # Unused
                0,  # Unused
                0,  # Unused
                0,  # Unused
                0,  # Unused
            )

            logging.info(f"Extinguishing mechanism deactivated on channel {EXTINGUISH_SERVO_CHANNEL}")
            return True

        except Exception as e:
            logging.error(f"Failed to deactivate extinguishing mechanism: {e}")
            self.deactivate_extinguish(attempt + 1)
            return False

    def send_extinguish_status_to_ground(
        self, target, success: bool, attempt: int = 0
    ) -> None:
        """
        Send extinguishing status to ground station.

        Args:
            target: Target object that was extinguished (or failed)
            success: True if extinguished successfully, False if failed
            attempt: Retry attempt number (internal use)
        """
        if attempt > 3:
            logging.error("Failed to send extinguish status to ground after 3 attempts")
            return

        try:
            status_str = "SUCCESS" if success else "FAILED"
            msg = (
                f"e_{target.target_id}_{target.coordinate.lat}_{target.coordinate.lon}_"
                f"{target.coordinate.alt}_{target.colour.name}_{status_str}"
            )

            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                msg.encode(),
            )

            logging.info(f"Sent extinguish status to ground: {status_str} for target {target.target_id}")
        except Exception as e:
            logging.error(f"Failed to send extinguish status to ground: {e}")
            self.send_extinguish_status_to_ground(target, success, attempt + 1)
