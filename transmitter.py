import logging
import math
import socket
import struct
import threading
import time
from typing import Any, Optional, Tuple, cast

import cv2
import depthai as dai
import numpy as np
from pymavlink import mavutil

# =========================
# Configuration
# =========================

# Network settings for the Transmission Server (this script)
HOST = "0.0.0.0"
PORT = 5000

# OAK-D Pro output settings
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
JPEG_QUALITY = 90
DEPTH_PNG_COMPRESSION = 3

# Flight Controller Connection Settings
# For serial (e.g. Raspberry Pi GPIO): "/dev/ttyAMA0" or "/dev/serial0"
# For UDP (e.g. SITL or network): "udpout:IP_ADDRESS:PORT"
FC_ADDR = "/dev/ttyAMA0"
FC_BAUD = 57600

# How long to wait for MAVLink messages before giving up (seconds)
MAVLINK_TIMEOUT = 1.0

# Pitch tolerance for 'level' in radians (~0.5 degrees)
PITCH_LEVEL_TOLERANCE_RAD = math.radians(0.5)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


class TelemetryState:
    """Thread-safe storage for downward range and attitude (pitch/roll in rad)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._downward_range: Optional[float] = None
        self._pitch: Optional[float] = None  # radians, from ATTITUDE
        self._roll: Optional[float] = None  # radians, from ATTITUDE

    def update(
        self,
        downward_range: Optional[float] = None,
        pitch: Optional[float] = None,
        roll: Optional[float] = None,
    ) -> None:
        with self._lock:
            if downward_range is not None:
                self._downward_range = downward_range
            if pitch is not None:
                self._pitch = pitch
            if roll is not None:
                self._roll = roll

    def get(self) -> Tuple[float, float, float]:
        with self._lock:
            downward_range = (
                self._downward_range
                if self._downward_range is not None
                else float("nan")
            )
            p = self._pitch if self._pitch is not None else float("nan")
            r = self._roll if self._roll is not None else float("nan")
        return downward_range, p, r


class MavlinkReader(threading.Thread):
    """Background thread to read MAVLink via Serial or UDP."""

    def __init__(self, connection_str: str, state: TelemetryState) -> None:
        super().__init__(daemon=True)
        self.connection_str = connection_str
        self.state = state
        self._stop_event = threading.Event()
        self._mav: Optional[Any] = None

    def stop(self) -> None:
        self._stop_event.set()

    def _request_telemetry_streams(self) -> None:
        if self._mav is None:
            return
        mav = self._mav.mav
        sys_id = self._mav.target_system
        comp_id = self._mav.target_component

        # Try modern SET_MESSAGE_INTERVAL method
        requests = [
            ("RANGEFINDER", mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 100_000),
            (
                "DISTANCE_SENSOR",
                mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
                100_000,
            ),
            ("ATTITUDE", mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20_000),  # 50 Hz
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

        # Fallback: legacy request stream for older ArduPilot versions
        # MAV_DATA_STREAM_EXTRA1 includes ATTITUDE
        mav.request_data_stream_send(
            sys_id,
            comp_id,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            5,  # 5 Hz fallback
            1,  # Start
        )
        logging.info("Requested telemetry streams (Modern & Legacy)")

    def run(self) -> None:
        try:
            logging.info(f"Connecting to FC MAVLink: {self.connection_str}")
            # Use FC_BAUD if it's a serial connection
            self._mav = mavutil.mavlink_connection(
                self.connection_str, baud=FC_BAUD, dialect="ardupilotmega"
            )
        except Exception as exc:
            logging.error(f"MAVLink Connection failed: {exc}")
            return

        mav = self._mav
        if mav is None:
            logging.error("MAVLink Connection failed: no handle")
            return

        logging.info("Waiting for Heartbeat...")
        try:
            mav.wait_heartbeat(timeout=10)
            logging.info(f"Heartbeat received from system {mav.target_system}")
        except:
            logging.warning(
                "No heartbeat. Ensure FC is powered and baud rate/port is correct."
            )

        try:
            self._request_telemetry_streams()
        except Exception as exc:
            logging.error(f"Stream request failed: {exc}")

        while not self._stop_event.is_set():
            try:
                msg = mav.recv_match(
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
                    if sensor_id != 1:
                        self.state.update(downward_range=dist)

                elif msg.get_type() == "RANGEFINDER":
                    dist = float(msg.distance)
                    self.state.update(downward_range=dist)

            except Exception as exc:
                continue


class OakCamera:
    """Background capture for aligned RGB and depth frames from an OAK-D Pro."""

    def __init__(self, width: int, height: int) -> None:
        self.width, self.height = width, height
        self._pipeline: Optional[dai.Pipeline] = None
        self._rgb_queue = None
        self._depth_queue = None
        self._lock = threading.Lock()
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        rgb = pipeline.create(dai.node.ColorCamera)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setPreviewSize(self.width, self.height)
        rgb.setInterleaved(False)
        rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        rgb.setFps(20)

        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        left.setFps(20)
        right.setFps(20)

        rgb.initialControl.setManualExposure(500, 50)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(self.width, self.height)
        stereo.setSubpixel(True)
        stereo.setLeftRightCheck(True)

        left.out.link(stereo.left)
        right.out.link(stereo.right)
        return pipeline, rgb.preview, stereo.depth

    def _capture_loop(self):
        try:
            pipeline, rgb_output, depth_output = self._build_pipeline()
            self._rgb_queue = rgb_output.createOutputQueue(maxSize=2, blocking=False)
            self._depth_queue = depth_output.createOutputQueue(
                maxSize=2, blocking=False
            )
            pipeline.start()
            self._pipeline = pipeline
        except Exception as exc:
            logging.exception("Failed to initialize OAK-D Pro pipeline")
            return

        while not self._stop_event.is_set():
            rgb_frame = (
                self._rgb_queue.tryGet() if self._rgb_queue is not None else None
            )
            depth_frame = (
                self._depth_queue.tryGet() if self._depth_queue is not None else None
            )

            if rgb_frame is not None:
                with self._lock:
                    rgb_img = cast(Any, rgb_frame).getCvFrame()
                    self._latest_rgb = rgb_img
            if depth_frame is not None:
                with self._lock:
                    depth_img = cast(Any, depth_frame).getFrame().copy()
                    self._latest_depth = depth_img
            if rgb_frame is None and depth_frame is None:
                time.sleep(0.01)

    def capture_payloads(self) -> Tuple[bytes, bytes, float]:
        with self._lock:
            if self._latest_rgb is None or self._latest_depth is None:
                return b"", b"", float("nan")
            rgb_frame = self._latest_rgb
            depth_frame = self._latest_depth

        ok_jpeg, jpeg_buf = cv2.imencode(
            ".jpg",
            rgb_frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )
        ok_png, depth_buf = cv2.imencode(
            ".png",
            depth_frame,
            [int(cv2.IMWRITE_PNG_COMPRESSION), DEPTH_PNG_COMPRESSION],
        )
        if not ok_jpeg or not ok_png:
            return b"", b"", float("nan")

        center_y = depth_frame.shape[0] // 2
        center_x = depth_frame.shape[1] // 2
        center_patch = depth_frame[
            max(0, center_y - 4) : min(depth_frame.shape[0], center_y + 5),
            max(0, center_x - 4) : min(depth_frame.shape[1], center_x + 5),
        ]
        valid = center_patch[center_patch > 0]
        center_depth_m = (
            float(np.median(valid) / 1000.0) if valid.size else float("nan")
        )
        return jpeg_buf.tobytes(), depth_buf.tobytes(), center_depth_m

    def release(self):
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._pipeline is not None:
            self._pipeline.stop()


def handle_client(conn, addr, camera: OakCamera, telemetry_state: TelemetryState):
    try:
        if conn.recv(1) == b"C":
            # Wait until pitch is within tolerance of level before capturing.
            while True:
                downward_range, pitch, roll = telemetry_state.get()
                if not math.isnan(pitch) and abs(pitch) <= PITCH_LEVEL_TOLERANCE_RAD:
                    break
                # Update at ~50 Hz while waiting for level
                time.sleep(0.02)

            jpeg_bytes, depth_bytes, center_depth_m = camera.capture_payloads()
            header = struct.pack(
                "!ffffQQ",
                float(downward_range),
                float(center_depth_m),
                float(pitch),
                float(roll),
                len(jpeg_bytes),
                len(depth_bytes),
            )
            conn.sendall(header + jpeg_bytes + depth_bytes)
    finally:
        conn.close()


def run_server():
    telemetry_state = TelemetryState()
    mav_thread = MavlinkReader(FC_ADDR, telemetry_state)
    mav_thread.start()
    camera = OakCamera(FRAME_WIDTH, FRAME_HEIGHT)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(5)
    logging.info(f"Server alive on port {PORT}")

    try:
        while True:
            conn, addr = server_sock.accept()
            handle_client(conn, addr, camera, telemetry_state)
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        mav_thread.stop()
        server_sock.close()


if __name__ == "__main__":
    run_server()
