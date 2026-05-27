# Servomotor Test Summary

## Test Files

| Filename                                                                       | Description                                                                                                                               | Module Used     | Obsolete | Prints PASS/FAIL |
| :----------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------- | :-------------- | :------- | :--------------- |
| `test_firmware_upgrade.py` | Per-command test for 'Firmware upgrade' (cmd 23): upgrade to latest (assert Get firmware version matches), downgrade to a kept older firmware (assert match), upgrade back to latest (assert match). Uses ONLY the new firmware/bootloader protocol. Heavy (three full reflashes). Ordered FIRST so every later test runs against the latest firmware. | `servomotor`    | No       | Yes              |
| `test_start_calibration.py` | Per-command test for 'Start calibration' (cmd 6): pre-call state clean; calibrating bit (3) sets after start_calibration and clears within timeout with no fatal error; post-call MOSFETs off and motor not busy; end-to-end usability via go_to_closed_loop + a small closed-loop move on target. Heavy (~30-60 s). Ordered SECOND so every closed-loop / position-dependent test runs against a freshly calibrated motor. | `servomotor`    | No       | Yes              |
| `test_all_leds_on.py`                                                          | Resets devices and turns on all LEDs using a test mode command. Uses older `communication` module.                                        | `communication` | Yes      | No               |
| `test_capture_hall_sensor_data.py` | Per-command test for 'Capture hall sensor data' (cmd 7): the work-in-progress streaming path is intentionally NOT issued; five invalid-parameter cases each raise FatalError 49 (ERROR_CAPTURE_BAD_PARAMETERS) and recover cleanly via system_reset. | `servomotor`    | No       | Yes              |
| `test_closed_loop_motion.py`                                                   | Functional test: closed-loop entry, move accuracy across a sequence of trapezoid moves, and position-holding stability at rest.           | `servomotor`    | No       | Yes              |
| `test_communication_while_high_speed.py`                                       | Stress tests communication (ping) while the motor executes a high-speed accelerate-coast-decelerate profile. Uses `servomotor` library.     | `servomotor`    | No       | Yes              |
| `test_continuous_graphing.py`                                                  | Simple Matplotlib animation example, unrelated to motor control.                                                                          | `N/A`           | Yes      | No               |
| `test_control_hall_sensor_statistics.py` | Per-command test for 'Control hall sensor statistics' (cmd 32): off-by-default after reset; control=1 starts gathering and grows measurementCount; control=0 freezes it; no-op values are accepted without state change; control=1 again resets and resumes. | `servomotor`    | No       | Yes              |
| `test_correct_and_incorrect_addressing.py`                                     | Functional test of alias / unique-ID / broadcast addressing: correct ones respond, wrong alias, wrong unique ID and 255 time out.         | `servomotor`    | No       | Yes              |
| `test_crc32_control.py` | Per-command test for 'CRC32 control' (cmd 46): verifies CRC enforcement when enabled (a no-CRC command is rejected), then disabling and re-enabling CRC both work. Driven via execute_command with explicit CRC flags. | `servomotor`    | No       | Yes              |
| `test_detect_devices.py`                                                       | Per-command test for 'Detect devices' (cmd 20): runs detection rounds and passes if at least one device is found on the bus.              | `servomotor`    | No       | Yes              |
| `test_disable_mosfets.py` | Per-command test for 'Disable MOSFETs' (cmd 0): disabling clears status bit 1, is idempotent, and a reset leaves the MOSFETs disabled. | `servomotor`    | No       | Yes              |
| `test_emergency_stop.py` | Per-command test for 'Emergency stop' (cmd 12): clears the move queue and disables the MOSFETs with no fatal error, and the device stays responsive afterwards. | `servomotor`    | No       | Yes              |
| `test_enable_disable.py`                                                       | Repeatedly enables and disables motor MOSFETs for many iterations. Reliability test. Uses older `communication` module.                   | `communication` | Yes      | No               |
| `test_enable_disable_reliability.py`                                           | Functional reliability test for Enable/Disable MOSFETs: rapid enable/disable cycling plus reset cycling. Uses `servomotor` library.       | `servomotor`    | No       | Yes              |
| `test_enable_mosfets.py` | Per-command test for 'Enable MOSFETs' (cmd 1): enabling sets status bit 1, is idempotent, and leaves no fatal error. | `servomotor`    | No       | Yes              |
| `test_fast_short_move_with_velocity.py`                                        | Tests queuing and executing many short, fast moves using velocity control. Uses `servomotor` library.                                     | `servomotor`    | No       | Yes              |
| `test_get_communication_statistics.py` | Per-command test for 'Get communication statistics' (cmd 47): six-element response shape and non-negative ints; reset (resetCounter=1) zeros all counters; well-formed traffic does not bump any counter on a quiet bus. | `servomotor`    | No       | Yes              |
| `test_get_comprehensive_position.py`                                           | Continuously polls and prints the comprehensive position (motor, hall, external encoder) for a specified motor. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_get_current_time.py` | Per-command test for 'Get current time' (cmd 9): non-negative monotonic clock that tracks wall-clock time in raw microseconds (time_unit="timesteps" passthrough), with cross-unit (us vs s) consistency. | `servomotor`    | No       | Yes              |
| `test_get_debug_values.py` | Per-command test for 'Get debug values' (cmd 45): 30-field response with each value inside its declared int range; quiescent post-reset values (current_velocity=0, measured_velocity within noise band, n_time_steps=0); max_acceleration/max_velocity stable across reads; current_velocity becomes non-zero during a closed-loop move and returns to 0 afterwards. | `servomotor`    | No       | Yes              |
| `test_get_firmware_version.py`                                                 | Tests the "Get firmware version" command in both main firmware and bootloader modes. Validates version format, consistency, and mode detection. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_get_hall_sensor_position.py` | Per-command test for 'Get hall sensor position' (cmd 15): reads ~0 at rest, advances under an open-loop move, tracks the desired position within tolerance, and is consistent across position units. | `servomotor`    | No       | Yes              |
| `test_get_hall_sensor_statistics.py` | Per-command test for 'Get hall sensor statistics' (cmd 33): boot-zero state pre-enable; after control=1 the 10 fields decode as 3 max + 3 min + 3 sum + measurementCount with min<=max, sum/n inside [min,max], and the count grows; across consecutive reads min is non-increasing, max is non-decreasing, sum is non-decreasing; control=1 again resets the accumulators. | `servomotor`    | No       | Yes              |
| `test_get_max_pid_error.py` | Per-command test for 'Get max PID error' (cmd 39): sentinel [INT32_MAX, INT32_MIN] before closed loop, real min<=max range after a closed-loop move, and read-then-reset semantics (second read returns a different window). | `servomotor`    | No       | Yes              |
| `test_get_n_queued_items.py` | Per-command test for 'Get n queued items' (cmd 11): queue is 0 at rest, rises when moves are queued, never exceeds 32, and drains back to 0. | `servomotor`    | No       | Yes              |
| `test_get_position.py` | Per-command test for 'Get position' (cmd 34): reads desired position consistently across units and confirms it tracks a commanded move. | `servomotor`    | No       | Yes              |
| `test_get_product_description.py` | Per-command test for 'Get product description' (cmd 24): returns a non-empty printable-ASCII string after stripping the trailing NUL, identical across reads. | `servomotor`    | No       | Yes              |
| `test_get_product_info.py`                                                     | Per-command test for 'Get product info' (cmd 22): detects the device and validates the product-info fields and consistency.               | `servomotor`    | No       | Yes              |
| `test_get_product_specs.py` | Per-command test for 'Get product specs' (cmd 18): updateFrequency in a plausible range and countsPerRotation matches the host library's shaft_rotations factor (cross-check against unit_conversions_*.json). | `servomotor`    | No       | Yes              |
| `test_get_status.py` | Per-command test for 'Get status' (cmd 16): validates the flags and error pair, application mode, unused bits zero, and that the MOSFET-enabled bit tracks enable and disable. | `servomotor`    | No       | Yes              |
| `test_get_supply_voltage.py` | Per-command test for 'Get supply voltage' (cmd 38): reads a plausible voltage, the V and mV units agree, and the reading is stable across consecutive samples. | `servomotor`    | No       | Yes              |
| `test_getch.py`                                                                | Tests the `getch` terminal input utility using a mock motor class. No hardware interaction.                                               | `N/A`           | Yes      | No               |
| `test_glue_machine_rotation_matrix.py`                                         | Loads glue machine calibration/transform data, visualizes, and tests the transformation matrix accuracy. Application-specific.            | `N/A`           | No       | Yes              |
| `test_go_to_closed_loop.py` | Per-command test for 'Go to closed loop' (cmd 17): enters closed-loop mode on a calibrated motor, confirms status bit 2 sets, and a closed-loop move lands on target. | `servomotor`    | No       | Yes              |
| `test_go_to_closed_loop_mode.py`                                               | Detailed closed-loop entry test: captures data, runs Python Goertzel comparison, logs results, generates histograms. Uses older `communication` module. | `communication` | Yes      | No               |
| `test_go_to_closed_loop_mode_and_spin_motor.py`                                | Tests closed-loop entry, performs moves, checks final position, and gathers success statistics based on phase angle. Uses older `communication` module. | `communication` | Yes      | No               |
| `test_go_to_closed_loop_mode_and_then_get_data_many_times.py`                  | Repeatedly enters closed-loop mode and reads/saves the internal data buffer (including Goertzel results). Data collection focus. Uses older `communication` module. | `communication` | Yes      | No               |
| `test_go_to_closed_loop_mode_plot_data.py`                                     | Reads and plots data logs generated by `test_go_to_closed_loop_mode_and_then_get_data_many_times.py`. Post-processing script.             | `N/A`           | Yes      | No               |
| `test_go_to_position.py`                                                       | Tests `go_to_position` command accuracy with position specified in different units (rotations, degrees, radians, counts). Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_gradual_speed_up.py`                                                     | Detects motors and gradually increases speed using `move_with_velocity` until max speed or error. Speed limit test. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_homing.py`                                                               | Tests `homing` command accuracy with distance specified in different units and directions. Uses `servomotor` library.                     | `servomotor`    | No       | Yes              |
| `test_homing_micropython.py`                                                   | MicroPython reference homing test (ESP32-S3). Kept out of the PC auto-run suite; not expected to pass yet (no MicroPython controller).    | `servomotor (MicroPython)` | No       | No               |
| `test_identify.py` | Per-command test for 'Identify' (cmd 41), two-layer: comms layer sends identify, asserts no fatal error, and pings mid-blink + post-blink (proves RS485 stays responsive while SysTick drives the ~2.4s blink animation); visual layer (TTY only) prompts the human to confirm the rapid green-LED flash. | `servomotor`    | No       | Yes              |
| `test_host_command_encoding.py`                                                | Host-only (no hardware): verifies `gather_inputs`/`send_command` produce exact protocol bytes for every parameter type, addressing mode, and the extended size form. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_host_crc32.py`                                                           | Host-only (no hardware): verifies `calculate_crc32` and the pure-Python MicroPython CRC32 against published check vectors and each other. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_host_get_command_id.py`                                                  | Host-only (no hardware): verifies servomotor.get_command_id() returns the right enum for every command in motor_commands.json.            | `servomotor`    | No       | Yes              |
| `test_host_getch.py`                                                           | Host-only (no hardware): smoke test of the getch module — imports cleanly, exposes the expected callables, fails cleanly on non-TTY stdin.| `servomotor`    | No       | Yes              |
| `test_host_import_smoke.py`                                                    | Host-only (no hardware): verifies the package imports, M3 constructs, generated command methods exist, and command/data-type tables load. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_host_response_decoding.py`                                               | Host-only (no hardware): verifies `get_response`/`interpret_single_response` decode device byte streams (all integer widths, version numbers, strings, CRC/fatal-error paths). Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_host_terminal_formatting.py`                                             | Host-only (no hardware): verifies the terminal_formatting format_* helpers return strings that preserve the supplied text.                | `servomotor`    | No       | Yes              |
| `test_iterate_reset_and_enable.py`                                             | Repeatedly moves, disables, resets, enables. Reliability test. Uses older `communication` module. Potentially outdated/unclear.          | `communication` | Yes      | No               |
| `test_json_read.py`                                                            | Reads and prints contents of `motor_commands.json`. Basic utility script.                                                                 | `N/A`           | Yes      | No               |
| `test_leds.py`                                                                 | Manual LED visual test (Test mode 13). The firmware enters set_led_test_mode() which requires a hardware power-cycle to recover.          | `servomotor (manual)` | No       | Yes              |
| `test_motor_position_vs_hall_sensor_position_vs_external_encoder_position.py`  | Moves motor while sampling internal position, hall position, and external encoder position. Logs data and calculates deviations. Uses older `communication` module. | `communication` | Yes      | No               |
| `test_move_with_acceleration.py`                                               | Tests `move_with_acceleration` command accuracy with acceleration specified in different units. Uses `servomotor` library.                | `servomotor`    | No       | Yes              |
| `test_move_with_velocity.py`                                                   | Tests `move_with_velocity` command accuracy with velocity specified in different units. Uses `servomotor` library.                      | `servomotor`    | No       | Yes              |
| `test_multimove.py` | Per-command test for 'Multimove' (cmd 29): a two-segment acceleration multimove runs to the predicted displacement and a 33-move multimove is rejected with code 24. Driven via execute_command in internal units. | `servomotor`    | No       | Yes              |
| `test_ping.py`                                                                 | Basic communication test using `ping` command and verifying echoed data. Uses `servomotor` library.                                       | `servomotor`    | No       | Yes              |
| `test_position_telemetry.py`                                                   | Functional test: motor commanded vs hall-sensed position cross-check across moves; external-encoder check skipped if absent.              | `servomotor`    | No       | Yes              |
| `test_random_speed_stress.py`                                                  | Complex stress test: detects, assigns aliases, pings, runs random speed moves on multiple motors, monitors errors, resets periodically. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_read_multipurpose_buffer.py` | Per-command test for 'Read multipurpose buffer' (cmd 35): empty post-reset buffer times out (no-response branch); test_mode 3 in closed loop populates the buffer with PID debug data and cmd 35 returns [data_type=4][20 bytes of 5 i32 PID fields]; system_reset cleans up and the next read times out again. Avoids test_mode(0) (would lock the firmware — see WORK_CHECKLIST TODO #8). | `servomotor`    | No       | Yes              |
| `test_reset_time.py` | Per-command test for 'Reset time' (cmd 8): zeros the absolute-microsecond clock (post-reset reads near 0), the clock continues to tick afterwards (motor elapsed tracks wall elapsed), and the reset is repeatable. | `servomotor`    | No       | Yes              |
| `test_safety_limit.py`                                                         | Low-level test using direct serial communication and special command to verify internal firmware motion calculations.                     | `serial`        | Yes      | Yes              |
| `test_servomotor_module.py`                                                    | Simple script using potentially outdated `servomotor.execute_command` to enable/reset. No verification.                                   | `servomotor`    | Yes      | No               |
| `test_servomotor_module_get_command_id.py`                                     | Unit test for the `servomotor.get_command_id()` function. No hardware interaction.                                                        | `servomotor`    | Yes      | Yes              |
| `test_set_device_alias.py`                                                     | Per-command test for 'Set device alias' (cmd 21): sets a random valid alias, rejects invalid aliases (254/253/252), removes the alias.    | `servomotor`    | No       | Yes              |
| `test_set_max_allowable_position_deviation.py` | Per-command test for 'Set max allowable position deviation' (cmd 44): generous limit lets a normal move complete; tight positive limit traps a fast move with ERROR_POSITION_DEVIATION_TOO_LARGE (45); a negative input is folded to |value| (same trip). | `servomotor`    | No       | Yes              |
| `test_set_maximum_acceleration.py` | Per-command test for 'Set maximum acceleration' (cmd 5): an over-cap move is rejected with ERROR_ACCEL_TOO_HIGH (15) and an under-cap move is accepted. | `servomotor`    | No       | Yes              |
| `test_set_maximum_motor_current.py` | Per-command test for 'Set maximum motor current' (cmd 28): valid currents are accepted, an excessive current is rejected with code 23, and a move still works at a normal current. | `servomotor`    | No       | Yes              |
| `test_set_maximum_velocity.py` | Per-command test for 'Set maximum velocity' (cmd 3): an over-cap move is rejected with ERROR_VEL_TOO_HIGH (16) and an under-cap move is accepted. | `servomotor`    | No       | Yes              |
| `test_set_pid_constants.py` | Per-command test for 'Set PID constants' (cmd 43): good gains land a closed-loop move on target with a small max-PID-error window; deliberately weak gains either miss target by much more, register much larger PID error, or trip a control fatal error — proving the set changes control behavior. | `servomotor`    | No       | Yes              |
| `test_set_safety_limits.py`                                                    | Per-command test for 'Set safety limits' (cmd 30): narrow limits reject an out-of-zone move; wide limits accept an in-bounds move.        | `servomotor`    | No       | Yes              |
| `test_system_reset.py` | Per-command test for 'System reset' (cmd 27): reset returns to application mode with no fatal error, clears the MOSFET-enabled state, and clears a latched fatal error. | `servomotor`    | No       | Yes              |
| `test_terminal_formatting.py`                                                  | Tests the `terminal_formatting` utility module. No hardware interaction.                                                                  | `N/A`           | Yes      | No               |
| `test_test_mode.py`                                                            | Verifies the "Test mode" command, including invalid and fatal error scenarios. Checks for correct fatal error codes and enforces timeout behavior. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_time_sync.py`                                                            | Per-command test for 'Time sync' (cmd 10): drift -> lock-in -> measurement. Passes when the aggregate is tight (mean and p90 abs error well under threshold, catching real drift), no sample exceeds a hard ceiling, and only a small budget of rare host-jitter outliers is tolerated. | `servomotor`    | No       | Yes              |
| `test_time_sync_multiple_devices.py`                                           | Tests time synchronization across multiple specified motors, reporting min/max error. Uses older `communication` module.                  | `communication` | Yes      | No               |
| `test_trapezoid_move.py` | Per-command test for 'Trapezoid move' (cmd 2): relative moves accumulate correctly, take the requested duration, and match across position units. | `servomotor`    | No       | Yes              |
| `test_zero_position.py` | Per-command test for 'Zero position' (cmd 13): zeroing makes the position read 0 without moving the shaft, and a subsequent relative move proves the origin shifted. | `servomotor`    | No       | Yes              |
| `test_trigger_framing_error.py`                                                | Tests framing error generation and detection by sending random bytes using 8-bit + even parity mode to trigger framing errors in devices. Verifies fatal error code 36, resets devices, and confirms error clearance. Uses `servomotor` library. | `servomotor`    | No       | Yes              |
| `test_vibrate.py` | Per-command test for 'Vibrate' (cmd 40), two-layer: comms layer sends vibrate(1) then vibrate(0) and asserts both round-trip with no fatal error and the device stays responsive (works on every product variant including M17 where the vibrate function body is #ifdef'd out for M1 only); visual layer (TTY + M1 only) prompts the human to confirm the motor actually vibrated. | `servomotor`    | No       | Yes              |
| `test_get_temperature.py`                                                      | Tests the "Get temperature" command by measuring temperature changes during high-power motor operation. Displays real-time temperature, queue status, and countdown timer during testing. Validates thermal protection and step-skipping detection. Uses `servomotor` library. | `servomotor`    | No       | Yes              |

## `test_get_temperature.py`

This test verifies the "Get temperature" command functionality by measuring temperature changes during high-power motor operation. It validates temperature measurement accuracy, thermal protection behavior, and motor step-skipping detection due to overheating.

**Test Scenarios:**
1. **Baseline Temperature Reading:**
   - Takes initial temperature reading after cooling period
   - Validates temperature is within acceptable range (10°C - 80°C by default)

2. **High-Power Motor Operation:**
   - Configures motor for maximum current (390) and specified velocity
   - Queues single long-duration move followed by velocity=0 to prevent queue empty errors
   - Continuously monitors motor status, queue size, and real-time temperature during operation
   - Displays countdown timer showing estimated time remaining in current batch
   - Displays live temperature readings alongside queue status for each motor
   - Runs motor continuously for specified duration (default: 120 seconds)

3. **Final Temperature Reading:**
   - Takes final temperature reading after motor operation
   - Calculates temperature increase and validates against minimum threshold
   - Ensures final temperature stays within safe operating range

4. **Multi-Motor Support:**
   - Detects all motors on bus with collision-resistant detection
   - Tests all detected motors simultaneously
   - Reports individual results for each motor

**Key Implementation Details:**
- Uses simplified queue management: one long move + velocity=0 command to prevent error 18
- Sets proper time and velocity units for easy parameter specification
- Monitors for thermal protection events and position deviation errors
- Displays real-time temperature and queue status during motor operation
- Shows countdown timer with estimated time remaining based on motor run time parameter
- Supports configurable test parameters via command line arguments
- Provides comprehensive error handling and status reporting

**Command-line arguments:**
- `-p`, `--port`: Serial port device name (required unless `-P` is used)
- `-P`, `--PORT`: Show available ports and prompt for selection
- `--initial-sleep`: Initial cooling time in seconds (default: 30)
- `--motor-run-time`: High-power motor run time in seconds (default: 120)
- `--min-temp`: Minimum acceptable temperature in °C (default: 10)
- `--max-temp`: Maximum acceptable temperature in °C (default: 80)
- `--temp-increase`: Minimum expected temperature increase in °C (default: 5)
- `--max-current`: Maximum motor current setting (default: 390)
- `--velocity`: Motor velocity in rotations/second (default: 1.0)
- `--position-deviation`: Max allowable position deviation (default: 100000)
- `--repeat`: Number of times to repeat the test (default: 1)
- `--verbose`: Enable verbose output

**Example usage:**
```bash
# Basic test with default settings
python3 test_get_temperature.py -p /dev/ttyUSB0

# Extended test with custom parameters
python3 test_get_temperature.py -p /dev/ttyUSB0 --initial-sleep 300 --motor-run-time 1800 --temp-increase 10

# Quick test for development
python3 test_get_temperature.py -p /dev/ttyUSB0 --initial-sleep 5 --motor-run-time 60 --temp-increase 2
```

## `test_set_device_alias.py`

This test verifies the device alias setting functionality using the `servomotor` library and the `M3` class. It covers:
- Setting a random valid alias (0–251) and verifying with device detection.
- Attempting to set invalid aliases (254, 253, 252) and confirming the device enters a fatal error state.
- Removing the alias (setting to 255) and confirming the device is detected with alias 255.

**Command-line arguments:**
- `-p`, `--port`: Serial port device name (required)
- `--bootloader`: Enter bootloader mode before running the test
- `--verbose`: Enable verbose output
- `-P`, `--PORT`: Show available ports and prompt for selection
- `--repeat N`: Repeat all test scenarios N times (default: 1)

After each repeat, the script prints the number of passes and failures for each test scenario. At the end, a grand summary is printed. The script exits with code 0 if all tests pass in all repeats, or 1 if any test fails.

Example usage:
```
python3 test_set_device_alias.py -p /dev/ttyUSB0 --repeat 5
```

## `test_test_mode.py`

This test verifies the correct behavior of the "Test mode" command in the servomotor firmware/bootloader. It covers both invalid and valid (fatal error-triggering) scenarios, and ensures the device responds as expected.

**Test Scenarios:**
1. **Invalid Test Mode Parameter:**
   - Sends a test_mode value >= 74 (e.g., 255), which is not supported by the firmware.
   - Expects the device to enter the ERROR_INVALID_TEST_MODE (code 53) fatal error state.
   - The device should NOT respond to the test_mode command (timeout is expected).
   - The test then calls get_status to confirm the fatal error code is set.

2. **Valid Test Mode Parameter (Triggers Fatal Error):**
   - Randomly selects a fatal error code N in the range 0–59.
   - Sends test_mode = N + 14, which, per firmware logic, triggers fatal_error(N).
   - The device should NOT respond to the test_mode command (timeout is expected).
   - The test then calls get_status to confirm the correct fatal error code is set.

**Key Implementation Details:**
- The test uses only the device's unique ID for all commands after initial detection, ensuring robust addressing even if the alias is 255.
- After each fatal error is triggered, the test performs a system reset to return the device to a known state before the next scenario.
- The test explicitly checks that a timeout occurs after sending a test_mode command that triggers a fatal error. If no timeout occurs, the test fails.
- The test supports --bootloader and --repeat options for flexibility and repeated validation.
- The test prints clear PASS/FAIL results and exits with code 0 on success, 1 on failure.

**Firmware Mapping Reference:**
- test_mode == 0: disables test mode
- test_mode 1–9: set_motor_test_mode
- test_mode 10–13: set_led_test_mode
- test_mode 14–73: triggers fatal_error(test_mode - 14), i.e., test_mode 14 triggers fatal_error(0), 15 triggers fatal_error(1), ..., 73 triggers fatal_error(59)
- test_mode >= 74: triggers fatal_error(ERROR_INVALID_TEST_MODE)

**Command-line arguments:**
- `-p`, `--port`: Serial port device name (required)
- `--bootloader`: Enter bootloader mode before running the test
- `--verbose`: Enable verbose output
- `-P`, `--PORT`: Show available ports and prompt for selection
- `--repeat N`: Repeat all test scenarios N times (default: 1)

**Example usage:**
```
python3 test_test_mode.py -p /dev/ttyUSB0 --repeat 5 --verbose
```

## `test_get_firmware_version.py`

This test verifies the "Get firmware version" command functionality in both main firmware and bootloader modes using the `servomotor` library and the `M3` class. It covers:

- Testing the command in main firmware mode (returns firmware version with `inBootloader=0`)
- Testing the command in bootloader mode (returns bootloader version with `inBootloader=1`)
- Validating firmware version format (handles both list `[dev, patch, minor, major]` and u32 integer formats)
- Verifying the `inBootloader` flag correctly reflects the current mode
- Testing both alias and unique ID addressing methods
- Ensuring consistency across multiple repeated calls
- Comprehensive error handling and validation

**Test Scenarios:**
1. **Main Firmware Mode Test:**
   - Resets device and enters main firmware mode (long delay after reset)
   - Tests `get_firmware_version` command using alias addressing
   - Tests `get_firmware_version` command using unique ID addressing
   - Verifies both methods return identical results
   - Validates that `inBootloader` flag is 0 (false)

2. **Bootloader Mode Test:**
   - Resets device and enters bootloader mode (short delay after reset)
   - Tests `get_firmware_version` command using alias addressing
   - Tests `get_firmware_version` command using unique ID addressing
   - Verifies both methods return identical results
   - Validates that `inBootloader` flag is 1 (true)

3. **Repeat Testing:**
   - When `--repeat N` is specified, performs multiple calls to verify consistency
   - Validates all repeated calls return identical results

**Key Implementation Details:**
- Uses extended addressing (unique ID) for robust communication after device detection
- Validates firmware version format: `[development, patch, minor, major]` or u32 integer
- Converts version components to human-readable format (e.g., "0.11.0.0")
- Supports both bootloader and main firmware modes via `--bootloader` flag
- Comprehensive validation of version components and ranges
- Clear PASS/FAIL reporting with detailed error messages

**Command-line arguments:**
- `-p`, `--port`: Serial port device name (required unless `-P` is used)
- `-P`, `--PORT`: Show available ports and prompt for selection
- `--bootloader`: Enter bootloader mode before running the test
- `--repeat N`: Repeat all test scenarios N times (default: 1)
- `--verbose`: Enable verbose output

**Example usage:**
```bash
# Test in main firmware mode
python3 test_get_firmware_version.py -p /dev/ttyUSB0

# Test in bootloader mode
python3 test_get_firmware_version.py -p /dev/ttyUSB0 --bootloader

# Test with multiple repeats in bootloader mode
python3 test_get_firmware_version.py -p /dev/ttyUSB0 --bootloader --repeat 5

# Interactive port selection with verbose output
python3 test_get_firmware_version.py -P --verbose
```

## Best Practices for Writing Tests

Based on the modern tests using the `servomotor` module, follow these best practices:

*   **Use `servomotor` Library:** Import and utilize the `servomotor` library, specifically the `M3` class for motor control.
*   **Clear Initialization:** Instantiate the `M3` object, clearly defining units (`time_unit`, `position_unit`, `velocity_unit`, etc.).
*   **Command-Line Arguments:** Use `argparse` to handle common options like serial port (`-p`, `-P`), alias (`-a`), and verbosity (`-v`), making tests more flexible.
*   **Port Handling Order:** **Crucially**, ensure argument parsing (`parser.parse_args()`) and setting the port via `servomotor.set_serial_port_from_args(args)` happen *before* calling `servomotor.open_serial_port()`. This prevents the library from incorrectly falling back to reading `serial_device.txt`.
*   **Port Management:** Use `servomotor.open_serial_port()` at the beginning and `servomotor.close_serial_port()` at the end, typically within a `try...finally` block to ensure the port is closed even if errors occur.
*   **Clean State:** Start tests with `motor.system_reset()` followed by an appropriate delay (e.g., `time.sleep(1.0)`) to ensure a known starting condition.
*   **Safe Operation:** Use `motor.enable_mosfets()` before commanding movement and `motor.disable_mosfets()` during cleanup (in the `finally` block).
*   **Structured Tests:** Encapsulate test logic within functions (e.g., `def test_my_feature(motor_obj):`). Pass the initialized motor object to the test function.
*   **Standard Entry Point:** Use `if __name__ == "__main__":` to handle argument parsing, port setup, motor initialization, calling the main test function, and final port closing.
*   **Verification:** Employ helper functions (e.g., `wait_for_moves_to_complete`) and assertion functions (e.g., `verify_position`, `verify_timing`, or standard `assert`) to programmatically check expected outcomes.
*   **Clear Pass/Fail:** Conclude the test script by printing "PASSED" upon successful completion. Use assertions or explicit checks to detect failures and ensure the script exits with a non-zero status code on failure. Tests designed for continuous monitoring, data collection, or visualization might naturally omit a final PASS/FAIL message and exit code.
*   **Summarize Multi-Part Tests:** If a test performs multiple distinct checks or runs in a loop, consider tracking internal pass/fail counts and printing a summary at the end before the final "PASSED" message.

## Remaining Work (Test Framework Implementation)

1.  **Refactor Test Scripts:**
    *   Apply the best practices structure (argument parsing before port opening, `try...finally`, passing motor object) to the remaining affected test scripts:
        *   `test_set_velocity_to_specific_values.py`
    *   Review *all* non-obsolete tests to ensure they consistently exit with code 0 on success and non-zero on failure.
    *   Consider adding internal summaries for tests performing multiple checks.
2.  **Address `test_random_speed_stress.py`:**
    *   Decide whether to install the `paho-mqtt` dependency (`pip install paho-mqtt`) or modify the test to make MQTT optional/removable if not needed.
3.  **Verify Simulator Compatibility:**
    *   Investigate why tests like `test_communication_while_high_speed.py`, `test_get_comprehensive_position.py`, and `test_go_to_position.py` timed out or had read errors with the simulator. This might require adjustments to the tests or the simulator itself.
    *   Investigate the timeout for `test_fast_short_move_with_velocity.py` with the simulator.
4.  **Test Runner Refinement (Optional):**
    *   The `run_all_tests.py` script could be enhanced to parse `TEST_SUMMARY.md` more robustly to check the "Prints PASS/FAIL" column dynamically instead of hardcoding exceptions.
5.  **Run and Verify:** Execute `run_all_tests.py` against the simulator (and eventually real hardware) after fixes are applied to confirm all expected tests pass.
## Bootloader Testing

### Test Script: `test_bootloader.py` (Proposed)

This test script aims to verify the core functionalities of the STM32G031 bootloader accessed via RS485 communication. It should cover the following areas:

*   **Basic Communication & Identification:**
    *   Verify `DETECT_DEVICES_COMMAND` response (Unique ID, Alias).
    *   Verify `GET_PRODUCT_INFO_COMMAND` response (Model Code, Compatibility Code).
    *   Verify `GET_STATUS_COMMAND` response indicates "in bootloader" status.
*   **Alias Configuration:**
    *   Test setting a valid alias using `SET_DEVICE_ALIAS_COMMAND`.
    *   Verify the new alias is reported correctly via `DETECT_DEVICES_COMMAND`.
    *   Test setting invalid aliases and confirm failure.
*   **Firmware Upgrade:**
    *   Test uploading firmware pages using `FIRMWARE_UPGRADE_COMMAND`.
    *   Verify success with correct model/compatibility codes and payload size.
    *   Test failure cases: incorrect model code, incorrect compatibility code, incorrect payload size.
    *   After a simulated successful upgrade and reset, verify the device attempts to boot the application.
*   **Application Launch Control:**
    *   Verify the bootloader attempts to launch valid firmware after a delay on startup.
    *   Verify sending any command during this delay cancels the automatic launch.
*   **System Reset:**
    *   Test `SYSTEM_RESET_COMMAND` and verify the device resets.
*   **Error Handling:**
    *   Test sending commands with invalid CRC32 checksums and verify they are ignored.
    *   Test sending commands with incorrect payload sizes and verify failure.
*   **Test Mode (Optional):**
    *   If feasible, test `TEST_MODE_COMMAND` to trigger specific fatal errors and verify behavior.
### `test_bootloader.py` Status (as of 2025-04-22)

*   **Purpose:** Test STM32G031 bootloader commands via RS485.
*   **Implementation:** Uses `servomotor.communication` module functions (`send_command`, `get_response`) for packet handling, following standard test structure. Requires manual entry into bootloader mode before running.
*   **Tests Implemented:** Basic Communication & Identification, Alias Configuration.
*   **Current Issues:** Encountered `TimeoutError` during broadcast `DETECT_DEVICES` command, potentially due to timeout handling logic in `communication.send_command` for alias 0. Further investigation or workarounds within the test script needed. Other test sections (Firmware Upgrade, Reset, Error Handling) are placeholders.

## `test_trigger_framing_error.py`

This test verifies framing error generation and detection functionality by intentionally creating communication framing errors and confirming devices properly detect and report them.

**Test Scenarios:**
1. **Framing Error Generation:**
   - Sends random bytes using 8-bit + even parity mode to create 9-bit frames
   - This triggers framing errors in devices expecting standard 8-bit, no-parity communication
   - Configurable transmission frequency and duration

2. **Error Detection:**
   - Queries each device using `get_status` command to check for fatal error code 36 (ERROR_FRAMING)
   - Verifies that framing errors are properly detected and reported by the firmware

3. **Error Recovery:**
   - Resets each device individually using `system_reset` command
   - Waits 1 second after reset for device recovery
   - Verifies framing error is cleared (fatal error code returns to 0)

**Key Implementation Details:**
- Uses `detect_devices_iteratively` for automatic device discovery or accepts specific alias via `-a` parameter
- Creates `M3` motor objects with proper verbose flag handling for clean output
- Uses `verify_expected_status()` function to check for both error presence (36) and clearance (0)
- Supports both single-device and multi-device testing modes
- Provides comprehensive error handling and status reporting

**Command-line arguments:**
- `-p`, `--port`: Serial port device name (required unless `-P` is used)
- `-P`, `--PORT`: Show available ports and prompt for selection
- `-a`, `--alias`: Alias of specific device to test (if not provided, auto-detects all devices)
- `--send-frequency`: Transmission frequency in Hz (default: 1.0)
- `--duration`: Duration to send framing error data in seconds (default: 10.0)
- `--verbose`: Enable verbose output showing detailed command execution

**Example usage:**
```bash
# Test all detected devices with default settings
python test_trigger_framing_error.py -p /dev/ttyUSB0

# Test specific device with higher frequency
python test_trigger_framing_error.py -p /dev/ttyUSB0 -a 05 --send-frequency 10 --duration 2

# Verbose mode for debugging
python test_trigger_framing_error.py -p /dev/ttyUSB0 --verbose
```

**Test Flow:**
1. Detect devices (auto-detection or specific alias)
2. Send framing error data using even parity serial communication
3. For each device:
   - Check for framing error (fatal error code 36)
   - Reset device to clear error
   - Verify error is cleared (fatal error code 0)
4. Report PASSED if all devices successfully trigger and clear framing errors

This test validates the firmware's ability to detect communication framing errors and recover from them properly, ensuring robust error handling in real-world deployment scenarios.
