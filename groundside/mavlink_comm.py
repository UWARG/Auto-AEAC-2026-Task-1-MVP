"""
MAVLink communication receiver for groundside station.

This module receives STATUSTEXT messages from the drone containing:
- Building corner data (b_lat_lon_alt)
- Target locations (t_lat_lon_alt_colour)
- Acknowledgements (a_...)
- Pickled objects (p_{idx}_{total}_{base64_chunk})
"""

import base64
import binascii
import logging
import pickle
import time
from pymavlink import mavutil
from util import (
    Coordinate,
    AIRSIDE_COMPONENT_ID,
    MAVLINK_TCP_HOST,
    MAVLINK_TCP_PORT,
    MAVLINK_RECEIVE_TIMEOUT_SEC,
    PICKLE_PREFIX,
)


class MavlinkReceiver:
    """Handles MAVLink message reception and parsing for ground station."""

    def __init__(self):
        """Initialize MAVLink connection."""
        self._pickle_chunk_buffer: dict[int, str] = {}
        while not self.__mavlink_connect():
            logging.info("Failed to connect to drone, retrying...")
            time.sleep(1)

    def __mavlink_connect(self) -> bool:
        """Establish MAVLink connection to receive messages from drone."""
        try:
            self.mav = mavutil.mavlink_connection(
                f'tcp:{MAVLINK_TCP_HOST}:{MAVLINK_TCP_PORT}'
            )
            self.mav.wait_heartbeat()
            logging.info(
                f"Heartbeat received from system {self.mav.target_system}, "
                f"component {self.mav.target_component}"
            )
        except Exception as e:
            logging.error(f"Failed to connect: {e}")
            return False

        logging.info("Ground station connected to drone")
        return True

    def process_messages(self) -> dict | None:
        """
        Process incoming MAVLink STATUSTEXT messages.

        Returns dict with 'type' and 'data' keys, or None if no message.
        Types: 'building_corner', 'target', 'ack', 'pickled'
        """
        msg = self.mav.recv_match(
            type='STATUSTEXT', blocking=True, timeout=MAVLINK_RECEIVE_TIMEOUT_SEC
        )

        if msg is None:
            return None

        # Filter messages from airside component
        if msg.get_srcComponent() != AIRSIDE_COMPONENT_ID:
            logging.debug(f"Ignoring message from component {msg.get_srcComponent()}")
            return None

        # Decode message text
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode('utf-8', errors='ignore').strip('\x00')

        logging.debug(f"Received STATUSTEXT: {text}")

        # Parse message based on prefix
        if text.startswith(PICKLE_PREFIX):
            return self._handle_pickled_message(text)
        elif text.startswith('b_'):
            return self._parse_building_corner(text)
        elif text.startswith('t_'):
            return self._parse_target(text)
        elif text.startswith('a_'):
            return self._parse_acknowledgement(text)
        else:
            logging.debug(f"Unknown message format: {text}")
            return None

    def _handle_pickled_message(self, text: str) -> dict | None:
        """
        Reassemble pickled chunks and unpickle.
        Returns {type: 'pickled', data: obj} when complete, else None.
        """
        parsed = self._parse_pickled_chunk(text)
        if parsed is None:
            return None

        chunk_idx, total, chunk_data = parsed

        # When chunk 0 arrives, start new buffer (overwrite any incomplete message)
        if chunk_idx == 0:
            self._pickle_chunk_buffer = {}

        self._pickle_chunk_buffer[chunk_idx] = chunk_data

        if len(self._pickle_chunk_buffer) != total:
            return None  # Not all chunks received yet

        # Reassemble and unpickle
        base64_str = "".join(
            self._pickle_chunk_buffer[i] for i in range(total)
        )
        self._pickle_chunk_buffer = {}  # Clear buffer

        try:
            pickled_bytes = base64.b64decode(base64_str)
            obj = pickle.loads(pickled_bytes)
            logging.info(f"Unpickled object: {type(obj).__name__}")
            return {"type": "pickled", "data": obj}
        except (binascii.Error, ValueError, pickle.UnpicklingError) as e:
            logging.error(f"Failed to unpickle: {e}")
            return None

    def _parse_pickled_chunk(self, text: str) -> tuple[int, int, str] | None:
        """Parse p_{idx}_{total}_{base64_data} format. Returns (idx, total, data) or None."""
        if not text.startswith(PICKLE_PREFIX):
            return None
        try:
            rest = text[len(PICKLE_PREFIX) :]
            parts = rest.split("_", 2)  # Split into max 3 parts
            if len(parts) != 3:
                return None
            chunk_idx = int(parts[0])
            total = int(parts[1])
            chunk_data = parts[2]
            return (chunk_idx, total, chunk_data)
        except (ValueError, IndexError) as e:
            logging.warning(f"Invalid pickled chunk format '{text}': {e}")
            return None

    def _parse_building_corner(self, text: str) -> dict | None:
        """Parse building corner from message format: b_lat_lon_alt"""
        try:
            # Remove 'b_' prefix and split
            parts = text[2:].split('_')
            if len(parts) != 3:
                logging.warning(f"Invalid building corner format: {text}")
                return None

            lat = float(parts[0])
            lon = float(parts[1])
            alt = float(parts[2])

            corner = Coordinate(lat=lat, lon=lon, alt=alt)
            logging.info(f"Parsed building corner: {corner}")

            return {
                'type': 'building_corner',
                'data': corner
            }

        except (ValueError, IndexError) as e:
            logging.error(f"Failed to parse building corner '{text}': {e}")
            return None

    def _parse_target(self, text: str) -> dict | None:
        """Parse target from format: t_lat_lon_alt_colour"""
        try:
            # Remove 't_' prefix and split
            parts = text[2:].split('_')
            if len(parts) != 4:
                logging.warning(f"Invalid target format: {text}")
                return None

            lat = float(parts[0])
            lon = float(parts[1])
            alt = float(parts[2])
            colour = parts[3]

            target = Coordinate(lat=lat, lon=lon, alt=alt)
            logging.info(f"Parsed target: {target}, colour: {colour}")

            return {
                'type': 'target',
                'data': {
                    'coordinate': target,
                    'colour': colour
                }
            }

        except (ValueError, IndexError) as e:
            logging.error(f"Failed to parse target '{text}': {e}")
            return None

    def _parse_acknowledgement(self, text: str) -> dict:
        """Parse acknowledgement messages from drone."""
        # Remove 'a_' prefix
        ack_msg = text[2:]
        logging.info(f"Acknowledgement from drone: {ack_msg}")

        return {
            'type': 'ack',
            'data': ack_msg
        }
