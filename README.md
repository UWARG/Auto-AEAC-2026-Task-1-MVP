# Auto Task 1 MVP system

Main Task 1 MVP

## Setup

### Drone (Airside)
```bash
cd airside
python3.11 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Ground Station (Groundside)
```bash
cd groundside
python3.11 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Running

### On Drone
```bash
cd airside
source venv/bin/activate
python main.py
```
TODO: setup autostart on drone

### On Ground Station
```bash
cd groundside
source venv/bin/activate
python main.py
```

## Controls

### RC Channels
- **Channel 7**: Mode switch
  - ON = Building record mode
  - OFF = Target detection mode
- **Channel 8**: Resource record A
  - Building mode: Record building corner
  - Target mode: Lock target in down-facing camera
- **Channel 9**: Resource record B
  - Building mode: Record building height
  - Target mode: Lock target in forward-facing camera

### Keyboard
- **'q'**: Quit application (airside only)

## Camera Setup
- **Camera 0**: Down-facing camera (for building mapping and roof targets)
- **Camera 1**: Forward-facing camera (for wall targets)

## Important Parameters

### Tuning Constants (airside/main.py)
- `PX_TO_MS = 0.00002`: Proportional control gain (m/s per pixel). Controls how aggressively drone corrects position errors. Smaller = gentler movement.
- `ERROR_RADIUS_PX = 40`: Target locking threshold in pixels. Maximum allowed error to successfully lock onto target.

### Channel Assignments (airside/main.py)
- `MODE_CHANGE_CHANNEL = 7`
- `RESOURCE_RECORD_CHANNEL_A = 8`
- `RESOURCE_RECORD_CHANNEL_B = 9`

## Mission plan

- **Map the building**
  - manually fly over each of the 4 corners of the building
  - above a corner, flip switch X to record it
    - Recording will reset beyond 4 flips
  - manually fly level to the roof the building
  - flip switch Y to record building height
- **Detect Targets**
  - manually fly around the building until target is visible on Camera
  - if detect target on roof of building or ground
    - flip switch A to start auto-centring
    - flip switch A againt to stop auto-centring and record
  - if detect target on walls of the building
    - flip switch B to start auto-centring
    - flip switch B againt to stop auto-centring and record
