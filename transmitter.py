import logging
import math
import signal
import socket
import struct
import sys
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
from pymavlink import mavutil

# =========================
# Configuration
# =========================

# Network settings for the Transmission Server (this script)
HOST = "0.0.0.0" 
PORT = 5000

# Camera settings
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
JPEG_QUALITY = 90

# Flight Controller UDP Settings
# format: "udpout:IP_ADDRESS:PORT"
FC_ADDR = "udpout:192.168.144.14:14550"

# How long to wait for MAVLink messages before giving up (seconds)
MAVLINK_TIMEOUT = 1.0

# Pitch tolerance for 'level' in radians (~0.5 degrees)
PITCH_LEVEL_TOLERANCE_RAD = math.radians(0.5)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

class RangefinderState:
    """Thread-safe storage for latest rangefinder readings and attitude (pitch/roll in rad)."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._range1: Optional[float] = None
        self._range2: Optional[float] = None
        self._pitch: Optional[float] = None  # radians, from ATTITUDE
        self._roll: Optional[float] = None   # radians, from ATTITUDE

    def update(
        self,
        r1: Optional[float] = None,
        r2: Optional[float] = None,
        pitch: Optional[float] = None,
        roll: Optional[float] = None,
    ) -> None:
        with self._lock:
            if r1 is not None:
                self._range1 = r1
            if r2 is not None:
                self._range2 = r2
            if pitch is not None:
                self._pitch = pitch
            if roll is not None:
                self._roll = roll

    def get(self) -> Tuple[float, float, float, float]:
        with self._lock:
            r1 = self._range1 if self._range1 is not None else float("nan")
            r2 = self._range2 if self._range2 is not None else float("nan")
            p = self._pitch if self._pitch is not None else float("nan")
            r = self._roll if self._roll is not None else float("nan")
        return r1, r2, p, r

class MavlinkReader(threading.Thread):
    """Background thread to read MAVLink via UDP."""
    def __init__(self, connection_str: str, state: RangefinderState) -> None:
        super().__init__(daemon=True)
        self.connection_str = connection_str
        self.state = state
        self._stop_event = threading.Event()
        self._mav: Optional[mavutil.mavfile] = None

    def stop(self) -> None:
        self._stop_event.set()

    def _request_rangefinder_streams(self) -> None:
        if self._mav is None:
            return
        mav = self._mav.mav
        sys_id = self._mav.target_system
        comp_id = self._mav.target_component

        # Request rangefinders at 10 Hz and attitude at 50 Hz for smoother pitch updates.
        requests = [
            ("RANGEFINDER", mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 100_000),   # 10 Hz
            ("DISTANCE_SENSOR", mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 100_000),  # 10 Hz
            ("ATTITUDE", mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20_000),         # 50 Hz
        ]

        for name, msg_id, interval_us in requests:
            mav.command_long_send(
                sys_id,
                comp_id,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                float(interval_us),
                0,
                0,
                0,
                0,
                0,
            )
            logging.info(f"Requested {name} via UDP at {1_000_000/interval_us:.1f} Hz")

    def run(self) -> None:
        try:
            logging.info(f"Connecting to FC MAVLink via UDP: {self.connection_str}")
            # UDP doesn't need a baud rate
            self._mav = mavutil.mavlink_connection(self.connection_str, dialect="ardupilotmega")
        except Exception as exc:
            logging.error(f"UDP Connection failed: {exc}")
            return

        logging.info("Waiting for UDP Heartbeat...")
        try:
            self._mav.wait_heartbeat(timeout=10)
            logging.info(f"Heartbeat received from {self._mav.target_system}")
        except:
            logging.warning("No heartbeat. Ensure FC is powered and IP is correct.")

        try:
            self._request_rangefinder_streams()
        except Exception as exc:
            logging.error(f"Stream request failed: {exc}")

        while not self._stop_event.is_set():
            try:
                msg = self._mav.recv_match(
                    type=["DISTANCE_SENSOR", "RANGEFINDER", "ATTITUDE"],
                    blocking=True,
                    timeout=MAVLINK_TIMEOUT,
                )
                if msg is None:
                    continue

                if msg.get_type() == "ATTITUDE":
                    self.state.update(pitch=float(msg.pitch), roll=float(msg.roll))

                elif msg.get_type() == "DISTANCE_SENSOR":
                    dist = float(msg.current_distance) / 100.0
                    sensor_id = getattr(msg, "id", 0)
                    if sensor_id == 1: self.state.update(r2=dist)
                    else: self.state.update(r1=dist)
                
                elif msg.get_type() == "RANGEFINDER":
                    dist = float(msg.distance)
                    self.state.update(r1=dist, r2=dist)

            except Exception as exc:
                continue

class Camera:
    """OpenCV Background Capture."""
    def __init__(self, index: int, width: int, height: int) -> None:
        self.index, self.width, self.height = index, width, height
        self._cap = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        self._cap = cv2.VideoCapture(self.index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        while not self._stop_event.is_set():
            ret, frame = self._cap.read()
            if ret:
                with self._lock: self._latest_frame = frame
            time.sleep(0.01)

    def capture_jpeg(self) -> bytes:
        with self._lock:
            if self._latest_frame is None: return b""
            frame = cv2.flip(self._latest_frame, -1)
        _, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        return buf.tobytes()

    def release(self):
        self._stop_event.set()
        if self._cap: self._cap.release()

def handle_client(conn, addr, camera, rf_state):
    try:
        if conn.recv(1) == b"C":
            # Wait until pitch is within tolerance of level before capturing.
            while True:
                r1, r2, pitch, roll = rf_state.get()
                if not math.isnan(pitch) and abs(pitch) <= PITCH_LEVEL_TOLERANCE_RAD:
                    break
                # Update at ~50 Hz while waiting for level
                time.sleep(0.02)

            jpeg_bytes = camera.capture_jpeg()
            # Header: range1, range2, pitch (rad), roll (rad), image length
            header = struct.pack("!ffffQ", float(r1), float(r2), float(pitch), float(roll), len(jpeg_bytes))
            conn.sendall(header + jpeg_bytes)
    finally:
        conn.close()

def run_server():
    rf_state = RangefinderState()
    mav_thread = MavlinkReader(FC_ADDR, rf_state)
    mav_thread.start()
    camera = Camera(CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(5)
    logging.info(f"Server alive on port {PORT}")

    try:
        while True:
            conn, addr = server_sock.accept()
            handle_client(conn, addr, camera, rf_state)
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        mav_thread.stop()
        server_sock.close()

if __name__ == "__main__":
    run_server()