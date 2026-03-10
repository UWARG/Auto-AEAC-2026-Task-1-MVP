# Auto AEAC 2026 Task 1 MVP

This repository contains a two-part Python MVP for AEAC 2026 Task 1:

- `transmitter.py` runs on the drone-side computer, reads MAVLink rangefinder and attitude data, captures a camera frame, and serves the result over TCP.
- `receiver.py` runs on the operator laptop, requests an image, displays telemetry, and helps generate measurement output from clicked points in the image.

## Repository layout

- `transmitter.py`: TCP image server, camera capture, MAVLink reader
- `receiver.py`: Tkinter desktop client for capture, measurement, and output generation
- `requirements.txt`: Python dependencies for both scripts

## Requirements

- Python 3.10 or newer
- A working camera accessible to OpenCV on the transmitter machine
- MAVLink telemetry available over UDP from the flight controller
- Tk support for Python on the receiver machine

Install Python packages with:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Dependencies

The project uses these third-party Python packages:

- `numpy`
- `opencv-python`
- `Pillow`
- `pymavlink`

`tkinter` is also required by `receiver.py`, but it normally ships with the system Python rather than being installed from `requirements.txt`.

## How the system works

1. The receiver opens a TCP connection to the transmitter and sends a 1-byte capture command: `b"C"`.
2. The transmitter waits until the latest MAVLink pitch is close to level.
3. The transmitter captures a JPEG frame from the camera.
4. The transmitter sends a binary header containing:
   - downward rangefinder distance
   - forward rangefinder distance
   - pitch in radians
   - roll in radians
   - JPEG byte length
5. The receiver downloads the JPEG, displays it, and allows the operator to click points for aided or manual measurement workflows.

## Configuration

Both scripts are currently configured with constants near the top of each file.

### `transmitter.py`

Important settings:

- `HOST` / `PORT`: TCP server bind address and port
- `CAMERA_INDEX`: OpenCV camera index
- `FRAME_WIDTH` / `FRAME_HEIGHT`: capture resolution
- `JPEG_QUALITY`: transmitted JPEG quality
- `FC_ADDR`: MAVLink UDP endpoint, for example `udpout:192.168.144.14:14550`
- `PITCH_LEVEL_TOLERANCE_RAD`: pitch tolerance before an image is captured

### `receiver.py`

Important settings:

- `TRANSMITTER_HOST` / `TRANSMITTER_PORT`: address of the transmitter machine
- `SOCKET_TIMEOUT`: network timeout in seconds
- `CAMERA_HFOV_RAD` / `CAMERA_VFOV_RAD`: camera field-of-view values used for geometric calculations
- `FISHEYE_EDGE_FACTOR`: optional edge correction factor for lens distortion

Adjust these constants before running on real hardware.

## Running the transmitter

Run this on the drone-side computer:

```bash
python transmitter.py
```

Behavior:

- starts a TCP server on port `5000`
- connects to the configured MAVLink UDP endpoint
- requests `RANGEFINDER`, `DISTANCE_SENSOR`, and `ATTITUDE` streams
- waits for capture requests from the receiver

## Running the receiver

Run this on the operator computer:

```bash
python receiver.py
```

Behavior:

- opens a Tkinter GUI
- requests a frame from the transmitter when `Capture image` is pressed
- displays downward and forward ranges plus pitch and roll
- supports:
  - `Aided` mode for generating an output sentence after selecting a target and reference point
  - `Full manual` mode for measuring vertical and sideways offsets between two selected points

## Notes and assumptions

- The transmitter currently handles one TCP client at a time.
- The camera image is flipped before transmission.
- If pitch data is not available, the transmitter will wait indefinitely for a level state before capturing.
- `receiver.py` assumes the first transmitted range is downward distance and the second is forward distance.
- The receiver de-rotates the image using the reported roll before performing measurements.

## Typical workflow

1. Start `transmitter.py` on the drone-side device.
2. Confirm camera access and MAVLink telemetry are working.
3. Update `TRANSMITTER_HOST` in `receiver.py` to point to the transmitter machine.
4. Start `receiver.py` on the operator machine.
5. Press `Capture image`.
6. Use the GUI to select points and read or generate the required output.
