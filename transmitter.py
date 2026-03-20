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

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

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
OAK_FRAME_WAIT_TIMEOUT_S = 2.0

#ArduCam
ARDU_WIDTH=640
ARDU_HEIGHT=360
ARDU_CAMERA_INDEX = 0
ARDU_FRAME_WAIT_TIMEOUT_S = 2.0

# Flight Controller Connection Settings
# For serial (e.g. Raspberry Pi GPIO): "/dev/ttyAMA0" or "/dev/serial0"
# For UDP (e.g. SITL or network): "udpout:IP_ADDRESS:PORT"
FC_ADDR = "/dev/ttyAMA0"
FC_BAUD = 57600

# How long to wait for MAVLink messages before giving up (seconds)
MAVLINK_TIMEOUT = 1.0

# Pitch tolerance for 'level' in radians (~0.5 degrees)
PITCH_LEVEL_TOLERANCE_RAD = math.radians(0.5)
ROLL_LEVEL_TOLERANCE_RAD=math.radians(0.5)

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
        self._frame_ready = threading.Event()
        self._logged_first_rgb = False
        self._logged_first_depth = False
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

        rgb.initialControl.setAutoExposureEnable()

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
                if not self._logged_first_rgb:
                    logging.info(
                        "Received first OAK-D RGB frame (%sx%s)",
                        rgb_img.shape[1],
                        rgb_img.shape[0],
                    )
                    self._logged_first_rgb = True
            if depth_frame is not None:
                with self._lock:
                    depth_img = cast(Any, depth_frame).getFrame().copy()
                    self._latest_depth = depth_img
                if not self._logged_first_depth:
                    logging.info(
                        "Received first OAK-D depth frame (%sx%s)",
                        depth_img.shape[1],
                        depth_img.shape[0],
                    )
                    self._logged_first_depth = True
            if self._latest_rgb is not None and self._latest_depth is not None:
                self._frame_ready.set()
            if rgb_frame is None and depth_frame is None:
                time.sleep(0.01)

    def _encode_payloads(
        self, rgb_frame: np.ndarray, depth_frame: np.ndarray
    ) -> Tuple[bytes, bytes]:
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
            return b"", b""
        return jpeg_buf.tobytes(), depth_buf.tobytes()

    def _build_fallback_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        logging.warning(
            "Using blank fallback frames for OAK-D because no valid frame pair was available"
        )
        rgb_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth_frame = np.zeros((self.height, self.width), dtype=np.uint16)
        return rgb_frame, depth_frame

    def capture_payloads(self) -> Tuple[bytes, bytes, float]:
        if not self._frame_ready.is_set():
            logging.info(
                "Waiting up to %.1f s for first OAK-D RGB/depth frame pair before capture",
                OAK_FRAME_WAIT_TIMEOUT_S,
            )
            self._frame_ready.wait(timeout=OAK_FRAME_WAIT_TIMEOUT_S)

        with self._lock:
            rgb_frame = None if self._latest_rgb is None else self._latest_rgb.copy()
            depth_frame = (
                None if self._latest_depth is None else self._latest_depth.copy()
            )

        if rgb_frame is None or depth_frame is None:
            rgb_frame, depth_frame = self._build_fallback_frames()

        jpeg_bytes, depth_bytes = self._encode_payloads(rgb_frame, depth_frame)
        if not jpeg_bytes or not depth_bytes:
            logging.error("Failed to encode OAK-D payloads; retrying with fallback frames")
            jpeg_bytes, depth_bytes = self._encode_payloads(*self._build_fallback_frames())
        if not jpeg_bytes or not depth_bytes:
            logging.error("OAK-D fallback encoding failed; returning empty payloads")
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
        return jpeg_bytes, depth_bytes, center_depth_m

    def release(self):
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._pipeline is not None:
            self._pipeline.stop()


class Arducam:
    def __init__(self, height: int, width: int, camera_index: int = ARDU_CAMERA_INDEX):
        self.height = height
        self.width = width
        self.camera_index = camera_index
        self.lock = threading.Lock()
        self.last_frame: Optional[np.ndarray] = None
        self._stop_event = threading.Event()
        self._picam: Optional[Any] = None
        self._started = False
        self._logged_first_frame = False
        self._frame_ready = threading.Event()

        if Picamera2 is None:
            logging.warning(
                "Picamera2 is not installed; CSI camera capture disabled."
            )
            return

        try:
            logging.info("Initializing CSI camera %s", self.camera_index)
            self._picam = Picamera2(camera_num=self.camera_index)
            config = self._picam.create_preview_configuration(
                main={"size": (width, height), "format": "RGB888"}
            )
            logging.info(
                "Configuring CSI camera %s preview stream to %sx%s",
                self.camera_index,
                self.width,
                self.height,
            )
            self._picam.configure(config)
            self._picam.start()
            self._started = True
            logging.info(
                "Started CSI camera %s at %sx%s",
                self.camera_index,
                self.width,
                self.height,
            )
        except Exception:
            logging.exception(
                "Failed to initialize CSI camera %s; secondary capture disabled",
                self.camera_index,
            )
            if self._picam is not None:
                try:
                    self._picam.close()
                except Exception:
                    pass
                self._picam = None
            return

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while not self._stop_event.is_set():
            if self._picam is None:
                time.sleep(0.1)
                continue

            try:
                frame = self._picam.capture_array("main")
            except Exception:
                logging.exception("CSI camera frame capture failed")
                time.sleep(0.01)
                continue

            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.last_frame = frame_bgr
            if not self._logged_first_frame:
                logging.info(
                    "Received first CSI frame from camera %s (%sx%s)",
                    self.camera_index,
                    frame_bgr.shape[1],
                    frame_bgr.shape[0],
                )
                self._logged_first_frame = True
            self._frame_ready.set()
    
    def release(self):
        self._stop_event.set()
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self._picam is not None and self._started:
            try:
                self._picam.stop()
            except Exception:
                pass
        if self._picam is not None:
            try:
                self._picam.close()
            except Exception:
                pass
        logging.info("Released CSI camera %s", self.camera_index)

    def _encode_jpeg(self, frame: np.ndarray) -> bytes:
        result, jpeg = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if not result:
            logging.error("Failed to JPEG-encode CSI frame")
            return b""
        return jpeg.tobytes()

    def _build_fallback_frame(self) -> np.ndarray:
        logging.warning(
            "Using blank fallback image for CSI camera %s because no valid frame was available",
            self.camera_index,
        )
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def capture_payloads(self):
        if not self._frame_ready.is_set():
            logging.info(
                "Waiting up to %.1f s for first CSI frame before capture",
                ARDU_FRAME_WAIT_TIMEOUT_S,
            )
            self._frame_ready.wait(timeout=ARDU_FRAME_WAIT_TIMEOUT_S)

        with self.lock:
            frame = None if self.last_frame is None else self.last_frame.copy()

        if frame is None:
            frame = self._build_fallback_frame()

        jpeg_bytes = self._encode_jpeg(frame)
        if not jpeg_bytes:
            jpeg_bytes = self._encode_jpeg(self._build_fallback_frame())
        if not jpeg_bytes:
            logging.error("CSI fallback JPEG encoding failed; returning empty payload")
            return b""
        logging.info("Prepared CSI JPEG payload (%s bytes)", len(jpeg_bytes))
        return jpeg_bytes

def handle_client(conn, addr, camera: OakCamera,camera2:Arducam, telemetry_state: TelemetryState):
    try:
        logging.info("Client connected from %s", addr)
        try:
            request_code = conn.recv(1)
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            logging.warning("Client %s disconnected before sending request: %s", addr, exc)
            return
        logging.info("Received client request code %r", request_code)
        if request_code == b"C":
            logging.info("Capture request acknowledged; waiting for vehicle to level")
            # Wait until pitch is within tolerance of level before capturing.
            wait_logged = False
            while True:
                downward_range, pitch, roll = telemetry_state.get()
                if not math.isnan(pitch) and abs(pitch) <= PITCH_LEVEL_TOLERANCE_RAD:
                    #Wait for roll to be within tolerance of level as well
                    if not math.isnan(roll) and abs(roll)<= ROLL_LEVEL_TOLERANCE_RAD:
                        logging.info(
                            "Vehicle level condition met (pitch=%.4f rad, roll=%.4f rad)",
                            pitch,
                            roll,
                        )
                        break
                if not wait_logged:
                    logging.info(
                        "Waiting for level attitude; current pitch=%s roll=%s range=%s",
                        pitch,
                        roll,
                        downward_range,
                    )
                    wait_logged = True
                # Update at ~50 Hz while waiting for level
                time.sleep(0.02)
            logging.info("Capturing CSI JPEG payload")
            jpeg_bytes_ardu = camera2.capture_payloads()
            logging.info("Capturing OAK-D RGB and depth payloads")
            jpeg_bytes, depth_bytes, center_depth_m = camera.capture_payloads()
            logging.info(
                "Prepared payloads: oak_rgb=%s bytes, oak_depth=%s bytes, csi=%s bytes, center_depth=%.3f m",
                len(jpeg_bytes),
                len(depth_bytes),
                len(jpeg_bytes_ardu),
                center_depth_m,
            )
            header = struct.pack(
                "!ffffQQQ",
                float(downward_range),
                float(center_depth_m),
                float(pitch),
                float(roll),
                len(jpeg_bytes),
                len(depth_bytes),
                len(jpeg_bytes_ardu)
            )
            try:
                conn.sendall(header + jpeg_bytes + depth_bytes + jpeg_bytes_ardu)
            except (ConnectionResetError, BrokenPipeError, OSError) as exc:
                logging.warning(
                    "Client %s disconnected while sending capture payload: %s",
                    addr,
                    exc,
                )
                return
            logging.info("Capture payload sent to %s", addr)
        else:
            logging.warning("Ignoring unknown client request code %r from %s", request_code, addr)
    except Exception:
        logging.exception("Unhandled error while serving client %s", addr)
    finally:
        try:
            conn.close()
        except OSError:
            pass
        logging.info("Closed client connection from %s", addr)

        


def run_server():
    telemetry_state = TelemetryState()
    mav_thread = MavlinkReader(FC_ADDR, telemetry_state)
    mav_thread.start()
    logging.info("MAVLink reader thread started")
    camera = OakCamera(FRAME_WIDTH, FRAME_HEIGHT)
    logging.info("OAK-D capture thread started")
    camera_down = Arducam(ARDU_HEIGHT, ARDU_WIDTH)
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(5)
    logging.info(f"Server alive on port {PORT}")

    try:
        while True:
            conn, addr = server_sock.accept()
            handle_client(conn, addr, camera,camera_down, telemetry_state)
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        camera_down.release()
        mav_thread.stop()
        server_sock.close()
        logging.info("Server shutdown complete")


if __name__ == "__main__":
    run_server()
