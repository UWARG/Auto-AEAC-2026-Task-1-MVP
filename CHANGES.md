# Changelog

This document tracks all significant changes made to the Auto-AEAC-2026-Task-1-MVP codebase.

## [Unreleased] - Extinguish Script Implementation

### Added
- **Extinguish Module** (`airside/extinguish.py`)
  - `Target` class: Represents a target with coordinate, color, and extinguishing status
  - `ExtinguishController` class: Manages target queue, navigation, and extinguishing operations
  - Automatic target queueing when targets are detected
  - Navigation to target positions with position tolerance checking
  - Extinguishing mechanism control via servo
  - Extinguishing verification and status reporting
  - Support for multiple extinguishing attempts per target
  - Statistics tracking (extinguished count, failed count, queue length)

- **MAVLink Extinguishing Methods** (`airside/mavlink_comm.py`)
  - `set_waypoint()`: Navigate drone to a specific coordinate using MAVLink waypoint commands
  - `activate_extinguish()`: Activate extinguishing mechanism via servo control (Channel 10)
  - `deactivate_extinguish()`: Deactivate extinguishing mechanism, return servo to neutral
  - `send_extinguish_status_to_ground()`: Send extinguishing success/failure status to ground station
  - Extended RC channel tracking to include Channel 10 for extinguishing mode activation

- **Extinguishing Mode Integration** (`airside/main.py`)
  - New extinguishing mode activated via RC Channel 10
  - Automatic target queueing when targets are detected and locked
  - Extinguishing mode processing loop that:
    - Navigates to queued targets
    - Activates extinguishing mechanism when at target position
    - Verifies extinguishing success
    - Processes next target in queue
  - HUD display updates to show "EXTINGUISH" mode
  - Configuration flag `AUTO_QUEUE_TARGETS` to control automatic queueing

### Configuration
- **New Constants** (`airside/main.py`)
  - `EXTINGUISH_MODE_CHANNEL = 10`: RC channel for activating extinguishing mode
  - `AUTO_QUEUE_TARGETS = True`: Automatically add detected targets to extinguishing queue

- **Extinguishing Parameters** (`airside/extinguish.py`)
  - `POSITION_TOLERANCE_M = 1.0`: Horizontal distance tolerance for target position (meters)
  - `ALTITUDE_TOLERANCE_M = 0.5`: Altitude tolerance for extinguishing (meters)
  - `EXTINGUISH_DURATION_SEC = 2.0`: Duration to keep extinguishing mechanism active (seconds)
  - `MAX_EXTINGUISH_ATTEMPTS = 3`: Maximum attempts per target before marking as failed
  - `VERIFICATION_DELAY_SEC = 1.0`: Wait time after extinguishing before verification

- **Servo Configuration** (`airside/mavlink_comm.py`)
  - `EXTINGUISH_SERVO_CHANNEL = 10`: MAVLink servo channel for extinguishing mechanism
  - `SERVO_OFF_PWM = 1500`: Neutral/off position PWM value (microseconds)
  - `SERVO_ON_PWM = 2000`: Active/extinguishing position PWM value (microseconds)

### Changed
- **Target Detection Flow** (`airside/main.py`)
  - When a target is detected and locked, it is automatically added to the extinguishing queue (if `AUTO_QUEUE_TARGETS` is enabled)
  - Target coordinates and color are stored for later extinguishing

- **Mode System** (`airside/main.py`)
  - Extended from 2 modes (Building Record, Target Detection) to 3 modes (Building Record, Target Detection, Extinguishing)
  - Extinguishing mode takes priority when Channel 10 is active
  - Mode transitions are logged for debugging

### Note
- **AIRSIDE IMPLEMENTATION COMPLETE** ✅
- **GROUNDSIDE IMPLEMENTATION PENDING** ❌
- Extinguish status messages are sent to ground station but not currently parsed by groundside
- Groundside implementation is planned for Task 2 MVP branch
- See `TASK2_GROUNDSIDE_EXTINGUISH_IMPLEMENTATION.md` for complete implementation guide
- See `IMPLEMENTATION_STATUS.md` for detailed status of all changes

### Message Protocol
- **New MAVLink STATUSTEXT Format** (`airside/mavlink_comm.py`)
  - Extinguish status messages: `e_{target_id}_{lat}_{lon}_{alt}_{colour}_{status}`
  - Status can be "SUCCESS" or "FAILED"
  - Example: `e_0_43.123_-80.456_10.5_RED_SUCCESS`

### Documentation
- Added comprehensive docstrings to all new classes and methods
- Inline comments explaining extinguishing workflow and parameters
- Usage examples in method docstrings

### Testing Notes
- Extinguishing mode requires:
  - RC Channel 10 configured and mapped on transmitter
  - Servo connected to MAVLink servo output (Channel 10)
  - Drone in GUIDED mode for waypoint navigation
  - GPS lock for accurate positioning
- Ground testing limitations:
  - Waypoint navigation may not work without GPS lock
  - Servo commands will be sent but may not execute without hardware
  - Position tolerance checking uses simplified distance calculation

### Future Improvements
- Camera-based extinguishing verification (check if target color disappears)
- More sophisticated navigation (accounting for wind, obstacles)
- Extinguishing mechanism feedback (sensors to verify activation)
- Retry logic with position adjustment if extinguishing fails
- Priority queue system for targets (e.g., extinguish closest first)
- Integration with precision loiter for more accurate positioning

---

## Previous Versions

### Initial Release
- Building mapping functionality
- Target detection with dual cameras
- Auto-centering control
- MAVLink communication
- HUD overlay system
- Ground station target description generation
