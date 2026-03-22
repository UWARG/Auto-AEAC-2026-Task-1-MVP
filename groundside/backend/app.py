import io
import json
from datetime import datetime, timezone
import math
from pathlib import Path
import socket
import struct
from typing import Tuple
from PIL import Image, ImageDraw, ImageFont, ImageTk
import cv2
from flask import Flask, jsonify, request
from flask_cors import CORS
import numpy as np
import time
from pathlib import Path
import base64
import threading


TRANSMITTER_HOST = "192.168.232.15"
TRANSMITTER_PORT = 5000
SOCKET_TIMEOUT = 10.0  # seconds


app = Flask(__name__)
CORS(app)

DB_PATH = Path(__file__).with_name("data.json")
capture_in_progress=threading.Lock()


def recv_exact(sock: socket.socket, num_bytes: int) -> bytes:
    """
    Receive exactly num_bytes from the socket, unless the connection closes or times out.
    """
    data = bytearray()
    while len(data) < num_bytes:
        chunk = sock.recv(num_bytes - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)
def request_image() -> Tuple[float, float, float, float, Image.Image, Image.Image,np.ndarray]:
    """
    Connects to the transmitter, sends a capture command, and receives:
    - downward range (float32), center depth (float32), pitch (float32 rad), roll (float32 rad)
    - RGB JPEG length (uint64), depth PNG length (uint64)
    - JPEG image bytes, 16-bit PNG depth bytes
    Returns (downward_range_m, center_depth_m, pitch_rad, roll_rad, PIL.Image, depth_map_mm).
    """
    with socket.create_connection(
        (TRANSMITTER_HOST, TRANSMITTER_PORT), timeout=SOCKET_TIMEOUT
    ) as sock:
        sock.settimeout(SOCKET_TIMEOUT)

        # Send 1-byte capture command
        sock.sendall(b"C")

        header = recv_exact(sock, 40)
        if len(header) != 40:
            raise RuntimeError(
                f"Incomplete header received (expected 40 bytes, got {len(header)})"
            )

        downwards_range, center_depth, pitch, roll, image_length, depth_length, ardu_image_length = (
            struct.unpack("!ffffQQQ", header)
        )

        if image_length == 0:
            raise RuntimeError("Transmitter reported zero-length image from oak-d")
        if depth_length == 0:
            raise RuntimeError("Transmitter reported zero-length depth map")
        if ardu_image_length==0:
            raise RuntimeError("Transmitter reported zero-length image from arducam")

        jpeg_bytes = recv_exact(sock, image_length)
        if len(jpeg_bytes) != image_length:
            raise RuntimeError(
                f"Incomplete oakd image received (expected {image_length} bytes, got {len(jpeg_bytes)})"
            )
        depth_bytes = recv_exact(sock, depth_length)
        if len(depth_bytes) != depth_length:
            raise RuntimeError(
                f"Incomplete depth map received (expected {depth_length} bytes, got {len(depth_bytes)})"
            )
        ardu_image_bytes=recv_exact(sock,ardu_image_length)
        if len(ardu_image_bytes)!=ardu_image_length:
            raise RuntimeError(
                f"Incomplete arducam image received (expected {ardu_image_length} bytes, got {len(ardu_image_bytes)})"
            )

    try:
        image = Image.open(io.BytesIO(jpeg_bytes))
        image.load()
        image2=Image.open(io.BytesIO(ardu_image_bytes))
        image2.load()
    except Exception as exc:
        raise RuntimeError(f"Failed to decode JPEG image: {exc}") from exc

    depth_buffer = np.frombuffer(depth_bytes, dtype=np.uint8)
    depth_map = cv2.imdecode(depth_buffer, cv2.IMREAD_UNCHANGED)
    if depth_map is None:
        raise RuntimeError("Failed to decode depth PNG")
    if depth_map.ndim != 2:
        raise RuntimeError(f"Depth map has unexpected shape: {depth_map.shape}")

    return downwards_range, center_depth, pitch, roll, image,image2, depth_map

def rotate_depth_map(depth_map: np.ndarray, angle_deg: float) -> np.ndarray:
    height, width = depth_map.shape[:2]
    center = (width / 2.0, height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos = abs(matrix[0, 0])
    sin = abs(matrix[0, 1])

    new_width = int((height * sin) + (width * cos))
    new_height = int((height * cos) + (width * sin))
    matrix[0, 2] += (new_width / 2) - center[0]
    matrix[1, 2] += (new_height / 2) - center[1]

    return cv2.warpAffine(
        depth_map,
        matrix,
        (new_width, new_height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )

def handle_capture_success(
    downwards_range: float,
    center_depth: float,
    pitch: float,
    roll: float,
    image: Image.Image,
    image2: Image.Image,
    depth_map: np.ndarray,
) -> Tuple[Image.Image,Image.Image]:
    db=load_db()
    latest={
        "downwards_range":downwards_range,
        "pitch":pitch if pitch == pitch else None,
        "roll":roll if roll == roll else None
    }


    # Rotate image to remove roll before any geometric calculations.
    display_image = image.copy()
    display_image2=image2.copy()
    rotated_depth_map = depth_map.copy()
    if latest["roll"] is not None:
        # Negative roll de-rotates the image so the horizon appears level.
        roll_deg = math.degrees(roll)
        display_image = display_image.rotate(
            -roll_deg, expand=True, resample=Image.BICUBIC
        )
        """ Do we need rotation matrix adjustment for YAW?
        display_image2=display_image2.rotate(
            -yaw_deg, expand=True, resample=Image.BICUBIC
        )
        """
        rotated_depth_map = rotate_depth_map(rotated_depth_map, -roll_deg)

    # Draw red dot in center of (possibly rotated) image
    draw = ImageDraw.Draw(display_image)
    draw2=ImageDraw.Draw(display_image2)
    cx, cy = display_image.width // 2, display_image.height // 2
    cx2,cy2=display_image2.width//2,display_image2.height//2
    r = 2  # radius in pixels
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill="red", outline="red")
    draw2.ellipse([cx2-r,cy2-r,cx2+r,cy2+r], fill="red", outline="red")


    timestamp=time.strftime("%Y%m%d_%H%M%S")
    cur_path=Path.joinpath(Path(__file__).parent,"images")
    cur_path.mkdir(exist_ok=True)
    depth_map_name=Path.joinpath(cur_path,f"depth_map_{timestamp}")
    arducam_name=Path.joinpath(cur_path,f"arducam_image_{timestamp}.jpg")
    oakd_name=Path.joinpath(cur_path,f"oakd_image_{timestamp}.jpg")

    display_image.save(oakd_name,format="JPEG")
    display_image2.save(arducam_name,format="JPEG")
    np.save(depth_map_name,rotated_depth_map)

    latest["ardufile_name"]=str(arducam_name)
    latest["oakd_name"]=str(oakd_name)
    latest["time"]=timestamp
    latest["depth_map_name"]=str(depth_map_name)
    db["captures"].append(latest)
    save_db(db)
    return display_image,display_image2



def load_db():
    with open(DB_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def save_db(data):
    with open(DB_PATH, "w", encoding="utf-8") as f:
        return json.dump(data, f, indent=3)


@app.route("/api/ackme")
def ack():
    return jsonify({"message": "test"})


@app.route("/api/captures", methods=["GET"])
def list_captures():
    db = load_db()
    return jsonify(db.get("captures", []))


@app.route("/api/captures", methods=["POST"])
def create_capture():
    payload = request.get_json(silent=True) or {}
    filename = str(payload.get("filename", "")).strip()
    desc = str(payload.get("desc", "")).strip()
    time_value = str(payload.get("time", "")).strip()

    if not filename or not desc:
        return jsonify({"message": "filename and desc are required"}), 400

    if not time_value:
        time_value = datetime.now(timezone.utc).replace(microsecond=0).isoformat()

    capture = {
        "time": time_value,
        "filename": filename,
        "desc": desc,
    }

    db = load_db()
    db.setdefault("captures", []).append(capture)
    save_db(db)

    return jsonify(capture), 201


@app.route("/api/db_reset", methods=["POST"])
def reset_db():
    db = load_db()
    db["captures"].clear()
    save_db(db)
    return jsonify({"message": "Database reset successfully"}), 200

@app.route("/api/capture_image", methods=["POST"])
def capture_image():
    if not capture_in_progress.acquire(blocking=False):
        return jsonify({"message":"image being captured"}),400
    try:
        downwards_range, center_depth, pitch, roll, image,image2, depth_map = (
        request_image()
    )
    except Exception as exc:
        return jsonify({"message":"failed to read image"}),500
    else:
        oakd,arducam=handle_capture_success(
        downwards_range,
        center_depth,
        pitch,
        roll,
        image,
        image2,
        depth_map,
        )   
        buffer=io.BytesIO()
        oakd.save(buffer,format="JPEG")
        oakd_image=buffer.getvalue()
        buffer=io.BytesIO()
        arducam.save(buffer,format="JPEG")
        arducam_image=buffer.getvalue()
        arducam_string=base64.b64encode(arducam_image).decode()
        oakd_string=base64.b64encode(oakd_image).decode()

        return jsonify({
            "roll":roll,
            "pitch":pitch,
            "downward_range":downwards_range,
            "center_depth":center_depth,
            "arducam_image":arducam_string,
            "oakd_image":oakd_string
        })
    finally:
        capture_in_progress.release()


if __name__ == "__main__":
    app.run(debug=True)
