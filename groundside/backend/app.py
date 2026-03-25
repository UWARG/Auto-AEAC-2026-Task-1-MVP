import io
import json
from datetime import datetime, timezone
import math
from pathlib import Path
import socket
import struct
from typing import Optional, Tuple
from PIL import Image, ImageDraw, ImageFont, ImageTk
import cv2
from flask import Flask, jsonify, request
from flask_cors import CORS
import numpy as np
import time
from pathlib import Path
import base64
import threading


# Click mode for "Generate output" flow
SELECT_TARGET = "target"
SELECT_REFERENCE = "reference"

# Crosshair colors (match prompt text color)
TARGET_CROSSHAIR_COLOUR = "lime"  # green, for "Select target center"
REFERENCE_CROSSHAIR_COLOUR = "deepskyblue"  # blue, for "Select reference point"
CROSSHAIR_SIZE = 8  # half-length of each crosshair arm in pixels


# =========================
# Configuration
# =========================

# IP and port of the transmitter (Raspberry Pi on the drone)
TRANSMITTER_HOST = "192.168.232.15"
TRANSMITTER_PORT = 5000

SOCKET_TIMEOUT = 10.0  # seconds

# Camera FOV (radians) used to project clicked pixels into metric offsets
CAMERA_HFOV_RAD = math.radians(80)
CAMERA_VFOV_RAD = math.radians(55)

ARDU_CAMERA_VFOV_RAD=math.radians(55)
ARDU_CAMERA_HFOV_RAD=math.radians(80)


# Fisheye correction (only near the edges). Correction = 1 + this * (angle / edge_angle)^2.
# 0.0 = no correction. Use e.g. 0.1–0.3 if the lens compresses angles toward the edges.
FISHEYE_EDGE_FACTOR = 0.0



app = Flask(__name__)
CORS(app)

DB_PATH = Path(__file__).with_name("data.json")
capture_in_progress=threading.Lock()
db_lock=threading.Lock()

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
        "downward_range":downwards_range,
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
    depth_map_name=Path.joinpath(cur_path,f"depth_map_{timestamp}.npy")
    arducam_name=Path.joinpath(cur_path,f"arducam_image_{timestamp}.jpg")
    oakd_name=Path.joinpath(cur_path,f"oakd_image_{timestamp}.jpg")

    display_image.save(oakd_name,format="JPEG")
    display_image2.save(arducam_name,format="JPEG")
    np.save(depth_map_name,rotated_depth_map)

    #get image in bytes

    latest["ardufile_name"]=str(arducam_name)
    latest["oakd_name"]=str(oakd_name)
    latest["time"]=timestamp
    latest["depth_map_name"]=str(depth_map_name)
    db["captures"].append(latest)
    save_db(db)
    return oakd_name,arducam_name


def sample_depth_m(x: int, y: int,depth_map:np.ndarray, radius: int = 4) -> Optional[float]:
    if depth_map is None:
        return None
    height, width = depth_map.shape[:2]
    if x < 0 or y < 0 or x >= width or y >= height:
        return None
    x0 = max(0, x - radius)
    x1 = min(width, x + radius + 1)
    y0 = max(0, y - radius)
    y1 = min(height, y + radius + 1)
    patch = depth_map[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid) / 1000.0)

def compute_delta_up_sideways_m(
    x1:int,
    y1:int,
    x2:int,
    y2:int,
    oakd_image:Image.Image,
    depth_map:np.ndarray
) -> Tuple[Optional[float], Optional[float]]:
    """
    Distance up (vertical) and sideways (horizontal) in metres between the two
    selected points (point 1 = target_xy, point 2 = ref_xy). Uses FOV and
    sampled OAK-D depth at each point.
    Returns (delta_up_m, delta_sideways_m); either can be None if unavailable.
    """
    if (
        oakd_image is None
        or x1 is None
        or x2 is None
        or y1 is None
        or y2 is None
    ):
        return None, None
    W, H = oakd_image.width, oakd_image.height
    depth1 = sample_depth_m(x1, y1,depth_map=depth_map)
    depth2 = sample_depth_m(x2, y2,depth_map=depth_map)
    if depth1 is None or depth2 is None:
        return None, None
    half_vfov = CAMERA_VFOV_RAD / 2
    half_hfov = CAMERA_HFOV_RAD / 2
    # Vertical angles (rad)
    a1 = (y1 - H / 2) / (H / 2) * half_vfov
    a2 = (y2 - H / 2) / (H / 2) * half_vfov
    a1 = a1 * (1.0 + FISHEYE_EDGE_FACTOR * (a1 / half_vfov) ** 2)
    a2 = a2 * (1.0 + FISHEYE_EDGE_FACTOR * (a2 / half_vfov) ** 2)
    delta_up = (depth2 * math.tan(a2)) - (depth1 * math.tan(a1))
    # Horizontal angles (rad)
    b1 = (x1 - W / 2) / (W / 2) * half_hfov
    b2 = (x2 - W / 2) / (W / 2) * half_hfov
    b1 = b1 * (1.0 + FISHEYE_EDGE_FACTOR * (b1 / half_hfov) ** 2)
    b2 = b2 * (1.0 + FISHEYE_EDGE_FACTOR * (b2 / half_hfov) ** 2)
    delta_sideways = (depth2 * math.tan(b2)) - (depth1 * math.tan(b1))
    return delta_up, delta_sideways

def draw_manual_measurements(x_target:float,
                             y_target:float,
                             x_ref:float,
                             y_ref:float,
                             oakd_image:Image.Image,
                             depth_map:np.ndarray) -> Image.Image:
    """Draw both points and the distance up/sideways between them on the image."""
    if oakd_image is None:
        return
    delta_up, delta_sideways = compute_delta_up_sideways_m(x_target,y_target,x_ref,y_ref,oakd_image,depth_map)
    draw = ImageDraw.Draw(oakd_image)
    s = CROSSHAIR_SIZE
    if x_target is not None and y_target is not None:
        tx=x_target
        ty=y_target
        draw.line((tx - s, ty, tx + s, ty), fill=TARGET_CROSSHAIR_COLOUR, width=2)
        draw.line((tx, ty - s, tx, ty + s), fill=TARGET_CROSSHAIR_COLOUR, width=2)
    if x_ref is not None and y_ref is not None:
        rx=x_ref
        ry=y_ref
        draw.line(
            (rx - s, ry, rx + s, ry), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
        )
        draw.line(
            (rx, ry - s, rx, ry + s), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
        )
    up_str = f"{delta_up:.2f} m" if delta_up is not None else "N/A"
    side_str = f"{delta_sideways:.2f} m" if delta_sideways is not None else "N/A"
    try:
        font = ImageFont.truetype("arial.ttf", 16)
    except (OSError, ImportError):
        font = ImageFont.load_default()
    draw.text((10, 10), f"Up: {up_str}", fill="white", font=font)
    draw.text((10, 28), f"Sideways: {side_str}", fill="white", font=font)
    return oakd_image


def compute_corrected_up_m(x:int,y:int,downward:float,img:Image.Image,depth_map:np.ndarray) -> Optional[float]:
    """
    Corrected 'up' distance (m): downward rangefinder minus the vertical offset of the
    target from the drone, using the target's sampled OAK-D depth and the vertical FOV.
    Returns None if readings or image are missing/invalid.
    """
    if img is None or x is None or y is None:
        return None
    d_down = downward
    tx, ty = x,y
    target_depth = sample_depth_m(tx, ty,depth_map)
    if d_down is None or target_depth is None:
        return None
    if d_down != d_down or target_depth != target_depth:  # NaN
        return None
    H = img.height
    # Vertical angle from image center (rad): positive when target below center
    half_vfov = CAMERA_VFOV_RAD / 2
    angle_v = (ty - H / 2) / (H / 2) * half_vfov
    angle_v = angle_v * (1.0 + FISHEYE_EDGE_FACTOR * (angle_v / half_vfov) ** 2)
    delta_h = target_depth * math.tan(angle_v)
    return d_down - delta_h

def compute_lateral_offset_m(img:Image.Image,x_tar:int,y_tar:int,x_ref:int,y_ref:int,depth_map:np.ndarray) -> Optional[float]:
    """
    Horizontal distance (m) of the target from the reference, using each point's
    sampled OAK-D depth and the horizontal FOV. Positive = target to the right
    of reference in image.
    Returns None if cannot compute.
    """
    if (
        img is None
        or x_tar is None or y_tar is None
        or x_ref is None or y_ref is None
    ):
        return None
    W = img.width
    tx, ty = x_tar,y_tar
    rx, ry = x_ref,y_ref
    target_depth = sample_depth_m(tx, ty,depth_map)
    ref_depth = sample_depth_m(rx, ry,depth_map)
    if target_depth is None or ref_depth is None:
        return None
    half_hfov = CAMERA_HFOV_RAD / 2
    angle_tx = (tx - W / 2) / (W / 2) * half_hfov
    angle_rx = (rx - W / 2) / (W / 2) * half_hfov
    angle_tx = angle_tx * (1.0 + FISHEYE_EDGE_FACTOR * (angle_tx / half_hfov) ** 2)
    angle_rx = angle_rx * (1.0 + FISHEYE_EDGE_FACTOR * (angle_rx / half_hfov) ** 2)
    target_x_m = target_depth * math.tan(angle_tx)
    ref_x_m = ref_depth * math.tan(angle_rx)
    return target_x_m - ref_x_m


def compute_cross_camera_offset(x_tar:int,y_tar:int,x_ref:int,y_ref:int,downward:float,target_on_ground:bool,ardu_image:Image.Image,oakd_image:Image.Image,depth_map:np.ndarray) -> tuple[float,float,float]|None:
    if not target_on_ground:
        return None
    tx, ty = x_tar,y_tar
    rx, ry = x_ref,y_ref
    ref_depth=sample_depth_m(rx,ry,depth_map)
    downward_range=downward
    if ref_depth is None or downward_range is None:
        return None
    half_hfov=CAMERA_HFOV_RAD/2
    half_vfov=CAMERA_VFOV_RAD/2
    W=oakd_image.width
    H=oakd_image.height
    a_horizontal=((rx-W/2)/(W/2))*half_hfov
    a_vertical=((ry-H/2)/(H/2))*half_vfov
    ref_x=ref_depth
    ref_y=ref_depth*math.tan(a_horizontal)
    ref_z=ref_depth*math.tan(a_vertical)
    #for arducam ground target
    W=ardu_image.width
    H=ardu_image.height
    half_hfov=ARDU_CAMERA_HFOV_RAD/2
    half_vfov=ARDU_CAMERA_VFOV_RAD/2
    a_horizontal=((tx-W/2)/(W/2))*half_hfov
    a_vertical=((H/2-ty)/(H/2))*half_vfov #flip h/2 and ty signs for oakd to be facing in positive x dir
    target_x=downward_range*math.tan(a_vertical) 
    target_y=downward_range*math.tan(a_horizontal)
    target_z=downward_range
    #positive coordinates:
    #oak-d image right
    #down
    #arducam up
    return (target_x-ref_x, #should always be negative
            target_y-ref_y,
            target_z-ref_z)
def write_output(colour: str, ref_desc: str,x_tar:int,y_tar:int,x_ref:int,y_ref:int,downward:float,ardu_image:Image.Image,oakd_image:Image.Image,depth_map:np.ndarray,target_on_ground:bool) -> str:
    """Build the output sentence and write it to the output text box."""
    if not target_on_ground:           
        corrected_up = compute_corrected_up_m(x_tar,y_tar,downward,oakd_image,depth_map)
        if corrected_up is not None:
            downwards_str = f"{corrected_up:.2f} m"
        else:
            downwards_str = "N/A"
        direction = "left" if x_tar < x_ref else "right"
        lateral = compute_lateral_offset_m(oakd_image,x_tar,y_tar,x_ref,y_ref,depth_map)
        if lateral is not None:
            lateral_str = f"{abs(lateral):.2f} m to the {direction}"
        else:
            lateral_str = f"to the {direction}"
        output = (
            f"The target is {colour}, and is located {downwards_str} off the ground and {lateral_str} "
            f"of the {ref_desc}."
        )
    else:
        result=compute_cross_camera_offset(x_tar,y_tar,x_ref,y_ref,downward,target_on_ground,ardu_image,oakd_image,depth_map)
        if result is None:
            output=("error generating description")
        else:
            x,y,z=result
            output=(
                f"The target is {colour}, and is located {x:.2f} meters forward, {z:.2f} meters down"
                f" and {y:.2f} meters right from the reference"
            ) 
    return output



def redraw_image_with_crosshairs(ardu_image:Image.Image,oakd_image:Image.Image,x_tar:int,y_tar:int,x_ref:int,y_ref:int,target_on_ground:bool) -> Optional[Tuple[Image.Image,Image.Image]]:
    """Redraw the image from base, adding target (green) and reference (blue) crosshairs."""
    if ardu_image is None or oakd_image is None:
        return None
    img = oakd_image.copy()
    img2=ardu_image.copy()
    draw = ImageDraw.Draw(img)
    draw2=ImageDraw.Draw(img2)
    s = CROSSHAIR_SIZE
    if x_tar is not None and y_tar is not None and not target_on_ground:
        tx, ty = x_tar,y_tar
        draw.line((tx - s, ty, tx + s, ty), fill=TARGET_CROSSHAIR_COLOUR, width=2)
        draw.line((tx, ty - s, tx, ty + s), fill=TARGET_CROSSHAIR_COLOUR, width=2)
    elif x_tar is not None and y_tar is not None and target_on_ground:
        tx,ty= x_tar,y_tar
        draw2.line((tx - s, ty, tx + s, ty), fill=TARGET_CROSSHAIR_COLOUR, width=2)
        draw2.line((tx, ty - s, tx, ty + s), fill=TARGET_CROSSHAIR_COLOUR, width=2)
    if x_ref is not None and y_ref is not None:
        rx, ry = x_ref,y_ref
        draw.line(
            (rx - s, ry, rx + s, ry), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
        )
        draw.line(
            (rx, ry - s, rx, ry + s), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
        )
    return (img,img2)

def load_db():
    success=db_lock.acquire(blocking=False)
    if success:
        with open(DB_PATH, "r", encoding="utf-8") as f:
            file=json.load(f)
        db_lock.release()
        return file
    print("Can't load db while save in process")


def save_db(data):
    success=db_lock.acquire()
    if success:
        with open(DB_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=3)
            db_lock.release()
            return
    print("can't save db while loading in process")


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
        oakd_name,arducam_name=handle_capture_success(
        downwards_range,
        center_depth,
        pitch,
        roll,
        image,
        image2,
        depth_map,
        )   
        with open(oakd_name,"rb") as f:
            oakd_bytes=f.read()
        with open(arducam_name,"rb") as f:
            arducam_bytes=f.read()
        arducam_string=base64.b64encode(arducam_bytes).decode()
        oakd_string=base64.b64encode(oakd_bytes).decode()

        return jsonify({
            "roll":roll,
            "pitch":pitch,
            "downward_range":downwards_range,
            "center_depth":center_depth,
            "arducam_image":arducam_string,
            "oakd_image":oakd_string
        }),200
    finally:
        capture_in_progress.release()
@app.route("/api/generate_output",methods=["POST"])
def generate_output():
    payload = request.get_json(silent=True) or {}
    x_ref=str(payload.get("reference_x","")).strip()
    y_ref=str(payload.get("reference_y","")).strip()
    x_tar=str(payload.get("target_x","")).strip()
    y_tar=str(payload.get("target_y","")).strip()
    mode=str(payload.get("mode","")).strip()
    color=str(payload.get("color","")).strip()
    ref_desc=str(payload.get("ref_description","")).strip()
    target_on_ground=str(payload.get("target_on_ground","")).strip()
    if not x_ref or not y_ref or not x_tar or not y_tar or not mode or not color or not ref_desc or not target_on_ground:
        return jsonify({"message":"invalid payload field"}),400
    try:
        x_ref=int(x_ref)
        y_ref=int(y_ref)
        x_tar=int(x_tar)
        y_tar=int(y_tar)
    except ValueError as e:
        return jsonify({"message":f"{e}"}),400
    if target_on_ground.casefold()=="true":
        target_on_ground=True
    elif target_on_ground.casefold()=="false":
        target_on_ground=False
    else:
        return jsonify({"message":"target_on_ground field invalid"}),400
    db=load_db()
    if not db["captures"]:
        return jsonify({"message":"Capture image first"}),400
    latest=db["captures"][-1]
    if not latest["ardufile_name"] or not latest["oakd_name"] or not latest["time"] or not latest["depth_map_name"]:
        return jsonify({"message":"Invalid Image data"}),400
    ardufile_image=Image.open(latest["ardufile_name"])
    oakd_image=Image.open(latest["oakd_name"])
    depth_map=np.load(latest["depth_map_name"])
    if not latest["downward_range"] or not latest["roll"] or not latest["pitch"]:
        return jsonify({"message":"Invalid Image data"}),400
    downward=latest["downward_range"]

    #need 0 < x_ref,y_ref,x_tar,y_tar < image.height/width and downward_range > 0 validation? 

    if mode == "full_manual":
        new_oakd_image=draw_manual_measurements(x_tar,y_tar,x_ref,y_ref,oakd_image,depth_map)
        buffer=io.BytesIO()
        new_oakd_image.save(buffer,format="JPEG")
        oak_d=base64.b64encode(buffer.getvalue()).decode()
        with open(latest["ardufile_name"],"rb") as f:
            arducam_img=f.read()
        return jsonify({
            "oakd_image":oak_d,
            "ardu_image":base64.b64encode(arducam_img).decode(),
            "output":None
        }),200
    elif mode=="aided":
        if not color or not ref_desc:
            return jsonify({"message":"Missing color and or reference description"}),400
        output=write_output(color,ref_desc,x_tar,y_tar,x_ref,y_ref,downward,ardufile_image,oakd_image,depth_map,target_on_ground)
        try:
            new_oakd_image,new_ardu_img=redraw_image_with_crosshairs(ardufile_image,oakd_image,x_tar,y_tar,x_ref,y_ref,target_on_ground)
        except Exception as e:
            return jsonify({"message":f"{e}"})
        buffer=io.BytesIO()
        new_oakd_image.save(buffer,format="JPEG")
        oak_d=base64.b64encode(buffer.getvalue()).decode()
        buffer=io.BytesIO()
        new_ardu_img.save(buffer,format="JPEG")
        ardu=base64.b64encode(buffer.getvalue()).decode()
        return jsonify({"output":output,
                        "oak_d_image":oak_d,
                        "ardu_image":ardu
                        }),200
    return jsonify({"message":"invalid mode"}),400
if __name__ == "__main__":
    app.run(debug=True)
