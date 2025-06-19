# Test Plan: test_get_temperature.py

## Overview
This test verifies the "Get temperature" command functionality by measuring temperature changes during high-power motor operation. The test validates temperature measurement accuracy, thermal protection behavior, and motor step-skipping detection due to overheating.

## Key Features
- **No Bootloader Support**: Temperature command is not available in bootloader mode
- **Multi-Motor Support**: Tests all detected motors simultaneously
- **Thermal Protection Detection**: Monitors for motor driver thermal shutdown
- **Step Skipping Detection**: Uses position deviation limits to detect motor failures
- **Robust Device Detection**: Multiple detection attempts to handle communication collisions
- **Timeout Handling**: Graceful handling of motor communication failures

## Command Line Arguments
- `-p, --port`: Serial port device name (required unless `-P` is used)
- `-P, --PORT`: Show available ports and prompt for selection
- `--initial-sleep`: Initial cooling time in seconds (default: 30)
- `--motor-run-time`: High-power motor run time in seconds (default: 120)
- `--min-temp`: Minimum acceptable temperature in °C (default: 10)
- `--max-temp`: Maximum acceptable temperature in °C (default: 80)
- `--temp-increase`: Minimum expected temperature increase in °C (default: 5)
- `--max-current`: Maximum motor current setting (default: 390)
- `--velocity`: Motor velocity in rotations/second (default: 1.0)
- `--position-deviation`: Max allowable position deviation (default: 100000)
- `--queue-threshold`: Queue size threshold for refilling (default: 5)
- `--repeat`: Number of times to repeat the test (default: 1)
- `--devices-per-batch`: Maximum number of devices to test concurrently in a single batch (default: test all devices)
- `--verbose`: Enable verbose output

## Test Flow

### 1. Initialization Phase
```
1. Parse command line arguments
2. Open serial port with timeout handling
3. Reset all devices (broadcast reset)
4. Detect devices with multiple attempts (handle collisions)
5. Create motor objects for each detected device
```

### 2. Configuration Phase
```
1. Reset each motor individually
2. Enable MOSFETs on all motors
3. Set maximum motor current (390)
4. Set position deviation limits (100000)
5. Configure motors for continuous rotation
```

### 3. Baseline Temperature Phase
```
1. Initial sleep with countdown display (every minute)
2. Take baseline temperature readings from all motors
3. Validate temperatures are within acceptable range
4. Store baseline values for comparison
```

### 4. High-Power Operation Phase
```
1. Start continuous motor rotation at specified velocity
2. Monitor loop:
   - Check queue levels on all motors
   - Refill queues when below threshold
   - Monitor for fatal errors and timeouts
   - Handle communication failures gracefully
   - Continue for specified motor run time
```

### 5. Final Temperature Phase
```
1. Take final temperature readings from all motors
2. Stop motor rotation and disable MOSFETs
3. Calculate temperature increases
4. Validate results against criteria
```

### 6. Results Analysis
```
1. Compare initial vs final temperatures
2. Check for minimum temperature increase
3. Verify temperatures stayed within limits
4. Report thermal protection events
5. Report communication timeouts/failures
```

## Key Implementation Details

### Device Detection Strategy
```python
# Multiple detection attempts to handle collisions
for attempt in range(3):
    devices = motor_broadcast.detect_devices()
    if devices:
        break
    time.sleep(1.0)  # Wait before retry
```

### Batched Testing

The test now supports batched execution to prevent excessive current draw when testing many motors simultaneously:

```python
# Test all devices in batches of 2
python3 test_get_temperature.py -p /dev/ttyUSB0 --devices-per-batch 2

# Test with default behavior (all devices at once)
python3 test_get_temperature.py -p /dev/ttyUSB0
```

**Batching Benefits:**
- Prevents power supply overload when testing many motors
- Maintains thermal safety through controlled batch sizes
- Allows testing of larger motor arrays safely
- Configurable batch sizes for different hardware setups

**Batch Output Format:**
```
========== BATCH 1 of 3 (2 devices) ==========
[Test execution for batch 1]

========== BATCH 2 of 3 (2 devices) ==========
[Test execution for batch 2]

========== BATCH 3 of 3 (1 device) ==========
[Test execution for batch 3]

========== CONSOLIDATED TEMPERATURE TEST SUMMARY ==========
[Combined results from all batches]
```

### Timeout and Error Handling
```python
# Handle command timeouts (likely fatal error state)
try:
    result = motor.some_command()
except TimeoutError:
    # Try to get status up to 3 times
    for retry in range(3):
        try:
            status = motor.get_status()
            # Record fatal error state
            break
        except:
            continue
    # Mark motor as failed, exclude from further testing
```

### Queue Management
```python
# Continuous queue monitoring and refilling
while motor_running:
    queue_size = motor.get_n_queued_items()
    if queue_size < queue_threshold:
        # Queue more velocity commands
        motor.move_with_velocity(velocity, duration)
```

### Temperature Validation
```python
# Validate temperature readings
def validate_temperature(temp, motor_id):
    if temp < min_temp or temp > max_temp:
        raise ValueError(f"Motor {motor_id}: Temperature {temp}°C out of range")
    return True
```

## Motor Configuration Parameters

### Current Settings
- **Maximum Motor Current**: 390 (maximum safe value)

### Motion Settings
- **Velocity**: 1.0 rotations/second (moderate continuous speed)
- **Position Deviation Limit**: 100000 counts (small fraction of rotation)
- **Queue Threshold**: 5 items (maintain smooth motion)

### Safety Limits
- **Temperature Range**: 10°C to 80°C (safe operating range)
- **Minimum Temperature Increase**: 5°C (detectable heating)
- **Communication Timeout**: 1.5 seconds (detect fatal errors)

## Expected Test Results

### Successful Test Output
```
========== TEMPERATURE TEST SUMMARY ==========
Motor 1 (Alias: 42, Unique ID: 0x123456789ABCDEF0):
  Initial Temperature: 23°C
  Final Temperature: 31°C
  Temperature Increase: 8°C
  Thermal Protection Events: 0
  Position Deviation Errors: 0
  Communication Timeouts: 0
  Status: PASSED

Motor 2 (Alias: 43, Unique ID: 0x123456789ABCDEF1):
  Initial Temperature: 24°C
  Final Temperature: 35°C
  Temperature Increase: 11°C
  Thermal Protection Events: 1
  Position Deviation Errors: 0
  Communication Timeouts: 0
  Status: PASSED (thermal protection detected)

Overall Test Result: PASSED
All 2 motors completed temperature test successfully.
```

### Failed Test Scenarios
1. **Temperature Out of Range**: Initial or final temperature outside limits
2. **Insufficient Heating**: Temperature increase below minimum threshold
3. **Communication Failure**: Multiple timeout errors preventing test completion
4. **Position Deviation**: Motor step skipping due to thermal protection
5. **Fatal Error State**: Motor enters unrecoverable error condition

## Test Validation Criteria

### Pass Conditions
- All temperature readings within specified range (10°C - 80°C)
- Temperature increase meets minimum threshold (≥5°C)
- No unhandled communication failures
- Thermal protection events properly detected and handled
- Position deviation errors properly detected (if any)

### Fail Conditions
- Temperature readings outside safe operating range
- Insufficient temperature increase (motor not heating properly)
- Unrecoverable communication failures
- Unexpected fatal errors not related to thermal protection

## Integration with Test Framework

### File Structure
- **Test File**: `test_get_temperature.py`
- **Documentation**: `test_get_temperature.md`
- **Test Summary**: Update `TEST_SUMMARY.md` with new test details

### Test Runner Integration
- Compatible with `run_all_tests.py`
- Prints clear PASS/FAIL results
- Exits with code 0 on success, 1 on failure
- Supports `--repeat` for stress testing

### Best Practices Compliance
- Uses `servomotor.M3` class for motor control
- Implements proper argument parsing with `argparse`
- Uses `try...finally` blocks for cleanup
- Follows established test patterns from existing tests
- Provides verbose output options for debugging

## Hardware Requirements
- One or more M3 servomotors connected via RS485
- Sufficient power supply for high-current operation
- Adequate ventilation for thermal testing
- Serial port connection (USB-to-RS485 adapter)

## Safety Considerations
- Temperature monitoring prevents overheating
- Position deviation limits prevent mechanical damage
- Automatic motor shutdown on fatal errors
- Maximum current limited to safe operating value (390)
- Test duration limits prevent extended overheating