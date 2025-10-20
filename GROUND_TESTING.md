# GROUND TEST PROCEDURES - DETECTION SYSTEM ONLY

## TEST SETUP REQUIREMENTS

**Hardware:**

- Pixhawk autopilot powered on and connected to the Raspberry Pi via the configured MAVLink serial link
- Raspberry Pi with dual camera modules attached to the CSI ports
- RC transmitter paired to the Pixhawk with mode and detection channels mapped as in `airside.main`
- Ground station PC running Mission Planner, TCP connection to `MAVLINK_TCP_HOST:MAVLINK_TCP_PORT`

**Software:**

- Raspberry Pi: `python -m airside.main`
- Ground station: `python -m groundside.main`

**Test targets:**

- Paper squares or panels matching each classification defined in `util.Colours`

---

## STAGE 1: SYSTEM INITIALIZATION

### Actions

1. Power the Pixhawk and wait for the boot status indicator to stabilize
2. SSH into the Raspberry Pi
3. Run: `python -m airside.main`
4. On ground station, run: `python -m groundside.main`

### Expected Logs (Airside)

```
<timestamp> - INFO - Starting airside...
<timestamp> - INFO - Heartbeat received from system <id>, component <id>
<timestamp> - INFO - Connected to drone
<timestamp> - INFO - Requested GLOBAL_POSITION_INT and RC_CHANNELS streams
```

### Expected Behavior

- Two OpenCV windows open: "Down Camera" and "Forward Camera"
- Both windows show live camera feeds
- Top-left HUD shows the mapping mode label, the corner counter at its initial state, and the tracking status indicator

### Failure Modes

- **"Failed to connect to drone"**: Check the configured MAVLink serial connection exists and uses the expected baud rate
- **"Failed to request data streams"**: Pixhawk not responding, check heartbeat
- **Black camera windows**: Camera hardware not detected, check `libcamera-hello` works

---

## STAGE 2: MAVLINK DATA STREAM VALIDATION

### Actions

1. Ensure RC transmitter is on, all channels centered
2. Move the mode switch channel through its range
3. Move the down-facing detection channel through its range
4. Move the forward-facing detection channel through its range

### Expected Logs (Airside)

```
<timestamp> - INFO - Received GLOBAL_POSITION_INT: <MAVLink_message>
<timestamp> - INFO - Received RC_CHANNELS: <MAVLink_message>
<timestamp> - INFO - Received RC_CHANNELS: <MAVLink_message>
```

**Position and RC message rates should match the requested stream frequencies.**

### Expected Behavior

- If GPS locked outdoors: Position shows real lat/lon/alt
- If GPS not locked (indoors): `WARNING - Position is not available` repeats when recording attempted

### Validation

Check Mission Planner MAVLink Inspector shows:

- `GLOBAL_POSITION_INT` rate matches the configured position stream frequency
- `RC_CHANNELS` rate matches the configured RC stream frequency
- Mode, detection, and auxiliary RC channels show activity across their travel range when sticks are moved

---

## STAGE 3: BUILDING MAPPING MODE

### Actions

1. Set the mode switch channel (`MODE_CHANGE_CHANNEL`) to its active position — mapping mode engages
2. HUD should show the mapping mode label
3. Toggle the corner-record channel (`RESOURCE_RECORD_CHANNEL_A`) active then inactive to capture a corner
4. Repeat the corner-record toggle until all corners are captured
5. Toggle the height-record channel (`RESOURCE_RECORD_CHANNEL_B`) active then inactive to capture height
6. Return the mode switch channel to its inactive position to exit building mode

### Expected Logs (Airside)

```
<timestamp> - INFO - Recorded building corner <index>: (<coordinate>)
<timestamp> - INFO - Recorded building corner <index>: (<coordinate>)
<timestamp> - INFO - Recorded building corner <index>: (<coordinate>)
<timestamp> - INFO - Recorded building corner <index>: (<coordinate>)
<timestamp> - INFO - Recorded building height: <altitude>
<timestamp> - INFO - Switching to target detection mode, sending building info
```

**Note:** Without GPS lock indoors, latitude and longitude remain at their default or invalid values, though altitude may still drift because of barometer readings. Position data is meaningless for ground testing but the detection pipeline still functions.

### Expected Logs (Groundside)

```
<timestamp> - INFO - Received building corner: (<coordinate>)
<timestamp> - INFO - Received building corner: (<coordinate>)
<timestamp> - INFO - Received building corner: (<coordinate>)
<timestamp> - INFO - Received building corner: (<coordinate>)
```

### Expected Behavior

- HUD corner counter advances after each successful record action until the set is complete
- After the mode switch channel moves to the inactive state, the HUD mode label flips to the detection state
- Status label stays on the tracking indicator during building operations and only changes to the lock indicator once a target has been transmitted

### Failure Modes

- **"Both building record channels are active"**: the corner and height channels are high simultaneously, fix RC mixing
- **Corners not incrementing**: the corner-record channel never reaches its active threshold, check transmitter calibration

---

## STAGE 4: TARGET DETECTION - DOWN CAMERA (ROOF/GROUND)

### Actions

1. Set the mode switch channel (`MODE_CHANGE_CHANNEL`) to the inactive position so detection mode is active
2. Place a target that matches one of the configured classifications in the down-facing camera view
3. Center the target in frame using the HUD crosshairs
4. Enable the down-facing detection channel (`RESOURCE_RECORD_CHANNEL_A`)
5. Adjust target position until the HUD reports a lock
6. Disable the down-facing detection channel to conclude the run

### Expected Logs (Airside)

```
<timestamp> - INFO - DOWN camera - Target detected at (<pixel_location>), offset: (<offset_x>, <offset_y>), error: <error> px
<timestamp> - INFO - DOWN camera - Sending velocity command: X=<value>, Y=<value>, Z=<value> m/s
<timestamp> - INFO - DOWN camera - Lock achieved! Error: <error> px <= <threshold> px. Sending target at (<coordinate>) (<classification>) to ground station
```

### Expected Logs (Groundside)

```
<timestamp> - INFO - Received target: (<coordinate>), classification: <classification>

================================================================================
TARGET DETECTED:
Target is on the ground, <distance> from the reference face and <distance> from the adjacent wall. Classification reported as <classification>.
================================================================================
```

### Expected HUD Behavior

- **Tracking phase** (error above the lock threshold):
  - Crosshairs display the tracking styling
  - Lock-radius circle appears around the crosshairs in the highlight styling
  - Target indicator box surrounds the detected contour
  - Offset lines and distance label render using the indicator styling
  - Bottom panel shows velocity commands (X, Y non-zero)
  - Status: tracking indicator

- **Lock phase** (error within the lock threshold):
  - Crosshairs switch to the locked styling
  - Error value adopts the lock styling
  - Velocity commands zero out
  - Status: lock indicator

### Validation Checklist

- [ ] HUD displays a target classification label for the detected contour
- [ ] Target centroid tracked in real-time (indicator box follows target)
- [ ] Offset X/Y update continuously when target moves
- [ ] Error decreases as target approaches center
- [ ] Lock achieved when error falls within the configured threshold (lock styling applied)
- [ ] Ground station receives target message
- [ ] Ground station prints human-readable description

### Failure Modes

- **"No target colour detected"**: Lighting too dim, adjust HSV ranges in `util.py` or increase brightness
- **Target box jitters**: Multiple contours detected, use a solid, uniform target without patterns
- **Lock never achieved**: Camera resolution too high, error threshold too small, or target too small

---

## STAGE 5: TARGET DETECTION - FORWARD CAMERA (WALL)

### Actions

1. Ensure the mode switch channel is inactive and the down-facing detection channel is inactive
2. Place a classified target in front of the forward-facing camera
3. Center the target in frame
4. Enable the forward-facing detection channel (`RESOURCE_RECORD_CHANNEL_B`)
5. Adjust the setup until the HUD reports a lock
6. Disable the forward-facing detection channel

### Expected Logs (Airside)

```
<timestamp> - INFO - FORWARD camera - Target detected at (<pixel_location>), offset: (<offset_x>, <offset_y>), error: <error> px
<timestamp> - INFO - FORWARD camera - Sending velocity command: X=<value>, Y=<value>, Z=<value> m/s
<timestamp> - INFO - FORWARD camera - Lock achieved! Error: <error> px <= <threshold> px. Sending target at (<coordinate>) (<classification>) to ground station
```

**Note:** Wall intersection calculation will fail if building corners still map to the origin because the building has zero area. Log will show:

```
<timestamp> - WARNING - FORWARD camera - Failed to find target on wall, building may not be fully mapped
```

This is expected for ground testing with fake position data.

### Expected Logs (Groundside)

```
<timestamp> - INFO - Received target: (<coordinate>)

================================================================================
TARGET DETECTED:
Target is on the referenced face, <distance> above ground and <distance> from the adjacent wall. Classification reported as <classification>.
================================================================================
```

### Expected HUD Behavior

- Same tracking/lock behavior as down camera
- Velocity commands follow the pixel-to-velocity gain defined by `PX_TO_MS`, driving lateral and vertical corrections while forward motion remains neutral

### Validation Checklist

- [ ] Target classification reported and tracked from the forward camera path
- [ ] Lateral and vertical velocity components non-zero during tracking
- [ ] Lock achieved once error drops within the configured threshold
- [ ] Ground station receives wall target message

---

## STAGE 6: MULTI-CLASSIFICATION DETECTION TEST

### Actions

Test each configured classification sequentially with the down camera channel active. Use a solid target that matches each entry in `util.Colours`.

### Expected Logs (Airside)

```
<timestamp> - INFO - DOWN camera - Target detected at (<pixel_location>)
<timestamp> - INFO - DOWN camera - Target detected at (<pixel_location>)
<timestamp> - INFO - DOWN camera - Target detected at (<pixel_location>)
<timestamp> - INFO - DOWN camera - Target detected at (<pixel_location>)
```

### Validation Checklist

- [ ] Each configured classification appears in the HUD label
- [ ] Lock achievable for every classification entry
- [ ] Ground station receives matching classification identifiers

### Failure Modes

- **Incorrect classification**: HSV ranges overlap, tune thresholds in `util.py`
- **No classification detected**: Target outside HSV bounds, check with `cv2.cvtColor()` test script
- **Unexpected priority result when multiple targets visible**: detection short-circuits on the first entry in `util.Colours` that has any pixels, so ties respect enumeration order

---

## STAGE 7: VELOCITY COMMAND TRANSMISSION

**THIS STAGE VERIFIES VELOCITY COMMANDS ARE SENT EVEN THOUGH THE DRONE IGNORES THEM OUTSIDE GUIDED MODE**

### Actions

1. In Mission Planner, open MAVLink Inspector
2. Filter for `SET_POSITION_TARGET_LOCAL_NED` messages
3. During target tracking while error remains above the lock threshold, observe the message stream

### Expected Behavior

- `SET_POSITION_TARGET_LOCAL_NED` messages appear in MAVLink Inspector
- `vx`, `vy`, `vz` fields show non-zero values during tracking
- Type mask matches the bitmask defined in `MavlinkComm.set_body_velocity`
- Frame matches the value used in `MavlinkComm.set_body_velocity`

### Expected Values During Tracking

**Down camera tracking example:**

- Velocity commands reflect the proportional gain `PX_TO_MS` against the pixel offsets (lateral and sideways corrections only)

**Forward camera tracking example:**

- Velocity commands use the same gain, applying horizontal and vertical corrections while forward velocity remains neutral

### Expected Behavior During Lock

- Velocity commands drop to zero in all axes once locked

### Validation

- Mission Planner should show the drone is NOT moving (position static)
- Velocity commands are present on the link, but the Pixhawk ignores them in STABILIZE/ALT_HOLD

---

## STAGE 8: ERROR INJECTION TESTS

### Test A: No Target Visible

**Actions:**

1. Enable the down-facing detection channel
2. Remove all classified targets from view

**Expected Logs:**

```
<timestamp> - DEBUG - DOWN camera - No target colour detected
```

**Expected HUD:**

- No target box overlay
- No offset lines
- Status indicator stays in the tracking state
- Classification label indicates absence of detection
- Velocity readout shows zeros

---

### Test B: Ambiguous Target (Multiple Classifications)

**Actions:**

1. Place two different classified targets simultaneously in the down-facing camera view

**Expected Behavior:**

- System flags the first entry in `util.Colours` that has any pixels (respecting enumeration priority)
- HUD shows a single classification label and indicator box for that entry only

---

### Test C: Both Channels Active Simultaneously

**Actions:**

1. Enable both detection channels simultaneously

**Expected Logs:**

```
<timestamp> - ERROR - Both target record channels are active
```

**Expected Behavior:**

- Velocity command is forced to zero immediately
- HUD status remains in the tracking state

---

### Test D: Channel Toggle Stress

**Actions:**

1. Rapidly toggle the down-facing detection channel between active and inactive states

**Expected Behavior:**

- `recorded_resource` flag prevents duplicate target sends
- Only a single target message sent per lock event
- Log shows: `"Target already sent to ground, stopping movement"`

---

## STAGE 9: GROUNDSIDE TIMEOUT TEST

### Actions

1. Stop the airside program with an interrupt signal
2. Groundside should still be running

**Expected Logs (Groundside):**

```
(No new messages, recv_match() blocks for a short timeout and loops indefinitely)
```

**Expected Behavior:**

- Groundside program does NOT crash
- Waits silently for new STATUSTEXT messages
- Timeout handled gracefully

---

## STAGE 10: HUD VISUALIZATION VERIFICATION

### Checklist

Verify all HUD elements render correctly:

**Top-left status panel:**

- [ ] Camera label for each feed
- [ ] Mode display shows mapping vs detection styling
- [ ] Corner count (building mode only)
- [ ] Lock status indicator toggles between tracking and lock states

**Bottom-left metrics panel:**

- [ ] Offset X/Y in pixels
- [ ] Error magnitude
- [ ] Velocity X/Y/Z components
- [ ] Detected classification label

**Overlay graphics:**

- [ ] Crosshairs reflect tracking vs lock styling
- [ ] Error radius circle appears around the crosshairs when unlocked
- [ ] Target bounding box renders around the detected contour
- [ ] Offset lines extend from center to target
- [ ] Distance label draws along the offset line using the indicator styling

**Press 'q' key:**

- [ ] Both HUD windows close
- [ ] Airside program exits cleanly
- [ ] Log shows: `"'q' pressed, exiting..."`

---

## CRITICAL LIMITATIONS FOR GROUND TESTING

- **Position data remains zeroed** - GPS won't lock indoors. Detection works but coordinates are meaningless.
- **Wall target calculation WILL FAIL** - Building corners stay collapsed at the origin, creating degenerate geometry. Expect `"Failed to find target on wall"` warnings.
- **Velocity commands still transmit** - The system always pushes `SET_POSITION_TARGET_LOCAL_NED` commands. In manual-stabilized modes the Pixhawk ignores them, but they are real outputs, not mocks.
- **Building descriptions remain trivial** - Ground station reports zero offsets because the building has no measurable footprint.

These limitations are acceptable. The detection pipeline (classification detection → centroid calculation → velocity control → target reporting) is fully testable.

---

## SUCCESS CRITERIA

Detection system passes ground testing if:

- [ ] Both cameras initialize and show live feeds
- [ ] MAVLink position/RC data streams at expected rates
- [ ] All configured classifications detected and tracked
- [ ] Target lock achieved within the configured error threshold
- [ ] Velocity commands generated during tracking
- [ ] Target coordinates transmitted to ground station
- [ ] Ground station receives and parses messages
- [ ] HUD renders all overlays correctly
- [ ] System handles error conditions without crashes

The actual coordinate values and velocity execution are irrelevant for this test phase.
