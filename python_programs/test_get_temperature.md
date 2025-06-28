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
   - Display estimated time remaining in current batch
   - Check queue levels on all motors
   - Get current temperature readings from all motors
   - Display queue size and temperature for each motor
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
# Continuous queue monitoring and refilling with temperature display and countdown
start_time = time.time()
while motor_running:
    # Calculate estimated time remaining
    elapsed_time = time.time() - start_time
    estimated_time_left = max(0, motor_run_time - elapsed_time)
    
    # Display countdown timer
    print(f"======== Approximate time left in this batch: {estimated_time_left:.0f}s ========")
    
    queue_size = motor.get_n_queued_items()
    
    # Get temperature reading
    try:
        temp = motor.get_temperature()
        temperature = temp[0] if isinstance(temp, (list, tuple)) else temp
        temp_str = f", Temperature = {temperature}°C"
    except Exception as temp_e:
        temp_str = f", Temperature = ERROR ({temp_e})"
    
    # Display queue size and temperature
    print(f"Motor {motor_id}: Queue size = {queue_size}{temp_str}")
    
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
--- Phase 3: High-Power Motor Operation ---
Running motors at velocity 1.0 rot/s for 120 seconds
Motor 132DB7981EED77A0: Queuing long move for 120 seconds
Motor 132DB7981EED77A0: Queuing velocity=0 to prevent queue empty error
Motor 132DB7981EED77A0: Started high-power operation
Motor 66C23BFB39E4373F: Queuing long move for 120 seconds
Motor 66C23BFB39E4373F: Queuing velocity=0 to prevent queue empty error
Motor 66C23BFB39E4373F: Started high-power operation

Waiting for all motor queues to become empty...
======== Approximate time left in this batch: 120s ========
Motor 132DB7981EED77A0: Queue size = 2, Temperature = 23.5°C
Motor 66C23BFB39E4373F: Queue size = 2, Temperature = 24.1°C
======== Approximate time left in this batch: 115s ========
Motor 132DB7981EED77A0: Queue size = 2, Temperature = 28.3°C
Motor 66C23BFB39E4373F: Queue size = 2, Temperature = 29.7°C
======== Approximate time left in this batch: 110s ========
Motor 132DB7981EED77A0: Queue size = 2, Temperature = 32.8°C
Motor 66C23BFB39E4373F: Queue size = 2, Temperature = 34.2°C
...

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

Motor 3 (Alias: 44, Unique ID: 0x123456789ABCDEF2):
  Initial Temperature: 42°C
  Final Temperature: Not available
  Last Known Temperature: 79°C
  Last Valid Reading: 85.3s after motor start
  Temperature Increase: 37°C (based on last known temp)
  Thermal Protection Events: 0
  Position Deviation Errors: 0
  Communication Timeouts: 18
  Error Types: timeout, final_temp_timeout
  Status: PASSED (completed despite communication timeouts)

Overall Test Result: PASSED
All 3 motors completed temperature test successfully.
```

### Failed Test Scenarios
1. **Temperature Out of Range**: Initial or final temperature outside limits
2. **Insufficient Heating**: Temperature increase below minimum threshold
3. **No Temperature Data**: Unable to obtain any temperature readings
4. **Position Deviation**: Motor step skipping due to thermal protection
5. **Fatal Error State**: Motor enters unrecoverable error condition

## Test Validation Criteria

### Pass Conditions
- Temperature readings within specified range (10°C - 80°C) when available
- Temperature increase meets minimum threshold (≥5°C)
- Communication timeouts are acceptable (motors may timeout during thermal protection)
- Thermal protection events properly detected and handled
- Position deviation errors properly detected (if any)

### Fail Conditions
- Temperature readings outside safe operating range
- Insufficient temperature increase (motor not heating properly)
- No initial temperature reading available
- No temperature data available during entire test

### Enhanced Error Handling
- **Last Known Temperature**: When final temperature reading fails, displays the last valid temperature reading obtained during the test
- **Timing Information**: Shows when the last valid temperature reading was taken relative to motor operation start time
- **Error Type Tracking**: Categorizes different types of errors (timeouts, communication errors, thermal protection)
- **Timeout Tolerance**: Communication timeouts are expected during thermal protection events and do not cause test failure
- **Graceful Degradation**: Test continues and reports best available data even when some readings fail

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