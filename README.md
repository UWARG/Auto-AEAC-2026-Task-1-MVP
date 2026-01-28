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
- **Channel 10**: Extinguishing mode (NEW)
  - ON = Activate extinguishing mode to process target queue
  - OFF = Exit extinguishing mode, return to normal operation

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
- `EXTINGUISH_MODE_CHANNEL = 10` (NEW)

### Extinguishing Parameters (airside/extinguish.py)
- `POSITION_TOLERANCE_M = 1.0`: Horizontal distance tolerance for target position (meters)
- `ALTITUDE_TOLERANCE_M = 0.5`: Altitude tolerance for extinguishing (meters)
- `EXTINGUISH_DURATION_SEC = 2.0`: Duration to keep extinguishing mechanism active (seconds)
- `MAX_EXTINGUISH_ATTEMPTS = 3`: Maximum attempts per target before marking as failed

### Servo Configuration (airside/mavlink_comm.py)
- `EXTINGUISH_SERVO_CHANNEL = 10`: MAVLink servo channel for extinguishing mechanism
- `SERVO_OFF_PWM = 1500`: Neutral/off position PWM value (microseconds)
- `SERVO_ON_PWM = 2000`: Active/extinguishing position PWM value (microseconds)

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
  - **Note**: Detected targets are automatically added to the extinguishing queue
- **Extinguish Targets** (NEW)
  - Ensure drone is in GUIDED mode for waypoint navigation
  - Activate Channel 10 to enter extinguishing mode
  - Drone will automatically:
    1. Navigate to first target in queue
    2. Wait until within position tolerance
    3. Activate extinguishing mechanism for configured duration
    4. Verify extinguishing success
    5. Move to next target in queue
  - Deactivate Channel 10 to exit extinguishing mode
  - Extinguishing status is sent to ground station for each target

## Extinguishing System

### Overview
The extinguishing system automatically processes detected targets by navigating to each target position and activating an extinguishing mechanism (servo-controlled). Targets are queued automatically when detected during target detection mode.

### Workflow
1. **Target Detection**: When a target is detected and locked, it is automatically added to the extinguishing queue (if `AUTO_QUEUE_TARGETS = True`)
2. **Extinguishing Mode**: Activate Channel 10 to enter extinguishing mode
3. **Navigation**: Drone navigates to target position using MAVLink waypoint commands
4. **Position Check**: System verifies drone is within tolerance (1.0m horizontal, 0.5m altitude)
5. **Extinguishing**: Servo on Channel 10 is activated for configured duration (default 2.0s)
6. **Verification**: System verifies extinguishing was successful
7. **Next Target**: Process repeats for next target in queue
8. **Status Reporting**: Success/failure status sent to ground station for each target

### Requirements
- **Hardware**: Servo connected to MAVLink servo output (Channel 10)
- **Flight Mode**: Drone must be in GUIDED mode for waypoint navigation
- **GPS**: GPS lock required for accurate positioning
- **RC Setup**: Channel 10 must be mapped on transmitter

### Configuration
- **Auto-Queue**: Set `AUTO_QUEUE_TARGETS = True` in `airside/main.py` to automatically queue detected targets
- **Tolerances**: Adjust `POSITION_TOLERANCE_M` and `ALTITUDE_TOLERANCE_M` in `airside/extinguish.py` for positioning accuracy
- **Servo Settings**: Modify `SERVO_ON_PWM` and `SERVO_OFF_PWM` in `airside/mavlink_comm.py` based on your servo configuration
- **Duration**: Change `EXTINGUISH_DURATION_SEC` to adjust how long the mechanism stays active

### Troubleshooting
- **Drone not navigating**: Ensure GPS lock and GUIDED mode are active
- **Servo not activating**: Check servo channel assignment and PWM values match your hardware
- **Targets not queuing**: Verify `AUTO_QUEUE_TARGETS` is set to `True`
- **Position tolerance too strict**: Increase `POSITION_TOLERANCE_M` if drone struggles to reach targets

## See Also
- `CHANGES.md` - Detailed changelog of all modifications
- `GROUND_TESTING.md` - Ground testing procedures for detection system