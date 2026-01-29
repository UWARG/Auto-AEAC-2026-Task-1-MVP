# Task 2 MVP - Groundside Extinguisher System Implementation Guide

## Overview

This document provides complete instructions for implementing the groundside extinguisher status tracking and display system. The airside system sends extinguish status messages via MAVLink STATUSTEXT, and groundside needs to parse, display, and track these messages.

## Background

The airside system has been implemented with extinguishing functionality that:
- Automatically queues detected targets for extinguishing
- Navigates to target positions
- Activates extinguishing mechanism (servo-controlled)
- Sends status updates to groundside via MAVLink

Groundside needs to receive and display these status messages to provide real-time feedback and mission statistics.

## Message Format

### Extinguish Status Messages

Airside sends extinguish status messages with the prefix `e_` in the following format:

```
e_{target_id}_{lat}_{lon}_{alt}_{colour}_{status}
```

**Format Details:**
- `target_id`: Integer ID of the target (0, 1, 2, ...)
- `lat`: Target latitude (float)
- `lon`: Target longitude (float)
- `alt`: Target altitude in meters (float)
- `colour`: Target color name (RED, GREEN, BLUE, WHITE)
- `status`: Either "SUCCESS" or "FAILED"

**Examples:**
```
e_0_43.123_-80.456_10.5_RED_SUCCESS
e_1_43.125_-80.458_0.0_GREEN_FAILED
e_2_43.130_-80.460_5.2_BLUE_SUCCESS
```

### Message Transmission

- Messages are sent via MAVLink STATUSTEXT
- Component ID filtering: Only messages from `AIRSIDE_COMPONENT_ID = 191` should be processed
- Messages are sent after each extinguishing attempt (success or failure)

## Implementation Requirements

### 1. Message Parsing (`groundside/mavlink_comm.py`)

#### Changes Needed:

**A. Update `process_messages()` method:**

Add extinguish status parsing to the message routing logic:

```python
# Parse message based on prefix
if text.startswith('b_'):
    return self._parse_building_corner(text)
elif text.startswith('t_'):
    return self._parse_target(text)
elif text.startswith('a_'):
    return self._parse_acknowledgement(text)
elif text.startswith('e_'):  # NEW: Add this line
    return self._parse_extinguish_status(text)  # NEW: Add this line
else:
    logging.debug(f"Unknown message format: {text}")
    return None
```

**B. Update `process_messages()` docstring:**

```python
"""
Process incoming MAVLink STATUSTEXT messages.

Returns dict with 'type' and 'data' keys, or None if no message.
Types: 'building_corner', 'target', 'ack', 'extinguish_status'
"""
```

**C. Add new parsing method `_parse_extinguish_status()`:**

Add this method to the `MavlinkReceiver` class:

```python
def _parse_extinguish_status(self, text: str) -> dict | None:
    """Parse extinguish status from format: e_target_id_lat_lon_alt_colour_status"""
    try:
        # Remove 'e_' prefix and split
        parts = text[2:].split('_')
        if len(parts) != 6:
            logging.warning(f"Invalid extinguish status format: {text}")
            return None

        target_id = int(parts[0])
        lat = float(parts[1])
        lon = float(parts[2])
        alt = float(parts[3])
        colour = parts[4]
        status = parts[5]  # "SUCCESS" or "FAILED"

        coordinate = Coordinate(lat=lat, lon=lon, alt=alt)
        logging.info(
            f"Parsed extinguish status: target_id={target_id}, {coordinate}, "
            f"colour={colour}, status={status}"
        )

        return {
            'type': 'extinguish_status',
            'data': {
                'target_id': target_id,
                'coordinate': coordinate,
                'colour': colour,
                'status': status
            }
        }

    except (ValueError, IndexError) as e:
        logging.error(f"Failed to parse extinguish status '{text}': {e}")
        return None
```

### 2. Status Display and Tracking (`groundside/main.py`)

#### Changes Needed:

**A. Update module docstring:**

```python
"""
Groundside main control loop for receiving and displaying drone target data.

This module listens for MAVLink STATUSTEXT messages containing:
- Building geometry data
- Target locations with color information
- Extinguishing status updates
- Generates human-readable descriptions of target positions
"""
```

**B. Add statistics tracking:**

Add this after initializing the receiver:

```python
# Extinguishing statistics tracking
extinguish_stats = {
    'total_attempted': 0,
    'successful': 0,
    'failed': 0,
    'targets': []  # Store extinguished targets for reporting
}
```

**C. Add message handler:**

Add this in the message processing loop (after the `'ack'` handler):

```python
elif msg_type == 'extinguish_status':
    target_id = data['target_id']
    coordinate = data['coordinate']
    colour = data['colour']
    status = data['status']

    # Update statistics
    extinguish_stats['total_attempted'] += 1
    if status == "SUCCESS":
        extinguish_stats['successful'] += 1
    else:
        extinguish_stats['failed'] += 1

    # Store target info
    extinguish_stats['targets'].append({
        'id': target_id,
        'coordinate': coordinate,
        'colour': colour,
        'status': status
    })

    # Generate target description for context
    description = building.generate_target_description(coordinate, colour)

    # Display extinguish status
    status_emoji = "✅" if status == "SUCCESS" else "❌"
    status_color = "SUCCESS" if status == "SUCCESS" else "FAILED"
    
    print(f"\n{'='*80}")
    print(f"EXTINGUISH STATUS {status_emoji}: {status_color}")
    print(f"Target ID: {target_id}")
    print(f"Location: {coordinate}")
    print(f"Colour: {colour}")
    print(f"Description: {description}")
    print(f"\nStatistics: {extinguish_stats['successful']}/{extinguish_stats['total_attempted']} successful")
    print(f"{'='*80}\n")

    logging.info(
        f"Extinguish status - Target {target_id}: {status} at {coordinate} ({colour})"
    )
```

**D. Add mission summary on exit:**

Update the KeyboardInterrupt handler:

```python
except KeyboardInterrupt:
    # Print final statistics on exit
    print(f"\n{'='*80}")
    print("EXTINGUISHING MISSION SUMMARY")
    print(f"{'='*80}")
    print(f"Total Targets Attempted: {extinguish_stats['total_attempted']}")
    print(f"Successful: {extinguish_stats['successful']}")
    print(f"Failed: {extinguish_stats['failed']}")
    if extinguish_stats['total_attempted'] > 0:
        success_rate = (extinguish_stats['successful'] / extinguish_stats['total_attempted']) * 100
        print(f"Success Rate: {success_rate:.1f}%")
    print(f"{'='*80}\n")
    
    logging.info("Shutting down groundside station...")
```

## Expected Output Format

### Individual Extinguish Status Display

When an extinguish status message is received, the output should look like:

```
================================================================================
EXTINGUISH STATUS ✅: SUCCESS
Target ID: 0
Location: (43.123, -80.456, 10.5)
Colour: RED
Description: Target is on the north face of the building, 10.5m above ground and 1.6m from the western wall. The colour is RED

Statistics: 1/1 successful
================================================================================
```

Or for failed attempts:

```
================================================================================
EXTINGUISH STATUS ❌: FAILED
Target ID: 1
Location: (43.125, -80.458, 0.0)
Colour: GREEN
Description: Target is on the ground, 5.2m away from the west face of the building and 0.2m from the western wall when facing it from the outside. The colour is GREEN

Statistics: 1/2 successful
================================================================================
```

### Mission Summary (on exit)

```
================================================================================
EXTINGUISHING MISSION SUMMARY
================================================================================
Total Targets Attempted: 5
Successful: 4
Failed: 1
Success Rate: 80.0%
================================================================================
```

## Testing Considerations

### Unit Testing

Test the parsing function with various inputs:

1. **Valid success message:**
   - Input: `"e_0_43.123_-80.456_10.5_RED_SUCCESS"`
   - Expected: Dictionary with correct target_id, coordinate, colour, and status

2. **Valid failed message:**
   - Input: `"e_1_43.125_-80.458_0.0_GREEN_FAILED"`
   - Expected: Dictionary with status="FAILED"

3. **Invalid format (too few parts):**
   - Input: `"e_0_43.123_-80.456"`
   - Expected: None, warning logged

4. **Invalid format (too many parts):**
   - Input: `"e_0_43.123_-80.456_10.5_RED_SUCCESS_EXTRA"`
   - Expected: None, warning logged

5. **Invalid numeric values:**
   - Input: `"e_abc_43.123_-80.456_10.5_RED_SUCCESS"`
   - Expected: None, error logged

### Integration Testing

1. **Message Flow:**
   - Verify messages from airside are received and parsed correctly
   - Verify statistics are updated correctly
   - Verify display output matches expected format

2. **Statistics Tracking:**
   - Test with multiple success messages
   - Test with multiple failed messages
   - Test with mixed success/failure
   - Verify final summary calculation is correct

3. **Edge Cases:**
   - Empty statistics (no messages received)
   - All successes
   - All failures
   - Rapid message sequence

## Dependencies

No new dependencies are required. The implementation uses existing imports:
- `logging` (standard library)
- `Coordinate` from `util`
- Existing `Building` class for target descriptions

## Error Handling

### Parsing Errors

- Invalid format: Log warning and return None (message is ignored)
- Invalid numeric values: Log error and return None
- Missing fields: Log error and return None

### Display Errors

- Missing building data: Target description will show "Building not fully mapped"
- Invalid coordinate: Coordinate will display as-is (may be (0,0,0) if parsing failed)

## Integration Points

### With Existing Code

- Uses existing `Building.generate_target_description()` for context
- Follows same message parsing pattern as building corners and targets
- Uses same logging format and level as other message handlers
- Integrates seamlessly with existing message loop

### With Airside System

- Receives messages sent by `MavlinkComm.send_extinguish_status_to_ground()`
- Message format matches exactly what airside sends
- No changes needed to airside code

## Code Quality Requirements

1. **Type Hints:** Use proper type hints (`dict | None`, etc.)
2. **Docstrings:** All new methods should have docstrings
3. **Error Handling:** Proper try/except blocks with logging
4. **Logging:** Use appropriate log levels (INFO for normal operations, WARNING/ERROR for issues)
5. **Code Style:** Follow existing code style and formatting

## File Locations

- **Parser:** `groundside/mavlink_comm.py`
- **Handler:** `groundside/main.py`
- **Utilities:** `util.py` (Coordinate class)

## Implementation Checklist

- [ ] Add `e_` prefix check in `process_messages()`
- [ ] Implement `_parse_extinguish_status()` method
- [ ] Update `process_messages()` docstring
- [ ] Add statistics tracking dictionary
- [ ] Add `extinguish_status` message handler
- [ ] Add mission summary on exit
- [ ] Update module docstring
- [ ] Test parsing with valid messages
- [ ] Test parsing with invalid messages
- [ ] Test statistics tracking
- [ ] Test display output format
- [ ] Verify integration with existing code

## Notes

- The airside system automatically queues targets when detected, so groundside will receive extinguish status messages for all targets that were attempted
- Statistics are tracked in memory and reset when the program restarts
- The mission summary is only printed on program exit (KeyboardInterrupt)
- Target descriptions use the same format as regular target detection messages

## Future Enhancements (Optional)

- Save statistics to file for persistence
- Add timestamp tracking for each extinguish attempt
- Generate detailed mission report file
- Add filtering/sorting of extinguished targets
- Add real-time statistics dashboard
- Export statistics to CSV/JSON format
