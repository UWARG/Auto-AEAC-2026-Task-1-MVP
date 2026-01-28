# Extinguishing System - Quick Reference

## Overview
The extinguishing system allows the drone to automatically navigate to detected targets and activate an extinguishing mechanism to extinguish them.

## Quick Start

### 1. Hardware Setup
- Connect servo to MAVLink servo output (Channel 10)
- Configure servo PWM values in `airside/mavlink_comm.py`:
  - `SERVO_OFF_PWM = 1500` (neutral position)
  - `SERVO_ON_PWM = 2000` (active position)
  - Adjust these values based on your servo specifications

### 2. RC Configuration
- Map Channel 10 on your transmitter to a switch/knob
- Channel 10 ON = Enter extinguishing mode
- Channel 10 OFF = Exit extinguishing mode

### 3. Flight Mode
- Ensure drone is in **GUIDED mode** for waypoint navigation
- GPS lock is required for accurate positioning

### 4. Operation Flow
1. **Detect Targets**: Use normal target detection mode (Channel 7 OFF, Channels 8/9 for detection)
2. **Queue Targets**: Detected targets are automatically added to extinguishing queue
3. **Activate Extinguishing**: Toggle Channel 10 ON to enter extinguishing mode
4. **Automatic Processing**: Drone will:
   - Navigate to each target
   - Wait until within tolerance (1.0m horizontal, 0.5m altitude)
   - Activate extinguishing mechanism for 2 seconds
   - Verify success
   - Move to next target
5. **Monitor Status**: Ground station displays extinguishing status for each target

## Configuration

### Key Parameters

**Position Tolerances** (`airside/extinguish.py`):
```python
POSITION_TOLERANCE_M = 1.0      # Horizontal tolerance (meters)
ALTITUDE_TOLERANCE_M = 0.5      # Altitude tolerance (meters)
```

**Extinguishing Duration** (`airside/extinguish.py`):
```python
EXTINGUISH_DURATION_SEC = 2.0   # How long mechanism stays active
MAX_EXTINGUISH_ATTEMPTS = 3     # Max retries per target
```

**Auto-Queue Setting** (`airside/main.py`):
```python
AUTO_QUEUE_TARGETS = True        # Auto-add detected targets to queue
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Drone not navigating | Check GPS lock and GUIDED mode |
| Servo not activating | Verify channel assignment and PWM values |
| Targets not queuing | Check `AUTO_QUEUE_TARGETS = True` |
| Position tolerance too strict | Increase `POSITION_TOLERANCE_M` |
| Extinguishing fails | Check servo connection and PWM values |

## Message Format

**Extinguish Status** (sent to ground station):
```
e_{target_id}_{lat}_{lon}_{alt}_{colour}_{status}
```

Example:
```
e_0_43.123_-80.456_10.5_RED_SUCCESS
```

## Files Modified

- `airside/extinguish.py` - New extinguishing module
- `airside/mavlink_comm.py` - Added navigation and servo control methods
- `airside/main.py` - Integrated extinguishing mode
- `groundside/mavlink_comm.py` - Added extinguish status parsing
- `groundside/main.py` - Added extinguish status display

## See Also
- `README.md` - Full system documentation
- `CHANGES.md` - Detailed changelog
