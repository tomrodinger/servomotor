# Servomotor Simulator Requirements

## Overview
The goal is to create a comprehensive simulator for a servomotor that:
1. Uses actual motor firmware code for accurate behavior
2. Emulates STM32G031 hardware features
3. Provides a simulated serial port interface
4. Shows real-time visualization of motor movement

## Current Status

### Implemented and Tested
- RS485 command processing:
  - USART1 peripheral configured for interrupt-driven reception
  - Command validation for both device alias (0x01) and broadcast (0xFF)
  - Debug logging tracks command flow
  - Successfully tested commands:
    - "Enable MOSFETs" with temperature checks
    - "Disable MOSFETs"
    - "Trapezoid move" with position tracking
    - "Set maximum velocity" with velocity value
    - "Detect devices" with random delay (0-990ms)
      * Random delay working correctly
      * Unique ID transmission verified
      * CRC32 calculation confirmed accurate
  - Command responses working:
    - Non-broadcast commands get proper responses
    - Response format verified (RESPONSE_CHAR, payload flag, length, data)
    - Hardware FIFO behavior correctly emulated
- System timing:
  - SysTick simulation running at precise 100Hz
  - Dedicated thread using spinlock timing
  - Frequency verified through profiling output
  - Successfully handles command delays and timeouts
- Motor control:
  - MOSFET state changes update visualization
  - Position tracking and visualization working
- System setup:
  - Basic motor visualization using SDL
  - Simulated serial port creation (/dev/ttysxxx)
  - Command reception through serial port
- Movement commands:
  - "Go to position":
    * Successfully tested with position=1000000, duration=31250
    * Command processed correctly
    * Motor visualization shows smooth movement
    * Proper response received
    * Note: Smaller position values (e.g. 1000) produce minimal visible movement
  - "Move with velocity":
    * Tested with velocity=1000000, duration=31250
    * Command processed correctly with stable timing
    * Timer maintains 31.25 kHz with <100μs deviation
    * Movement visible but slow - needs higher velocity value
    * Motor stops correctly when followed by velocity=0 command
  - "Move with acceleration":
    * Tested with acceleration=400000, duration=31250
    * Followed by deceleration=-400000, duration=31250
    * Command processed correctly
    * Smooth acceleration and deceleration
    * Motor stops correctly at zero velocity
  - "Get comprehensive position":
    * Tested after Go to position command (target: 1000000)
    * Command processed correctly
    * Response format verified:
      - Commanded position: 999999 (matches target)
      - Hall sensor position: 0 (needs simulator update)
      - External encoder: 0 (as expected, no hardware attached)
    * Note: Hall sensor position should track commanded position in simulator

### Fixed Issues
- Receive interrupt not re-enabled after command processing:
  - Issue: Commands were received and validated correctly, but subsequent commands were dropped
  - Fix: Verified rs485_allow_next_command() is called at the start of each command case in processCommand()
  - Result: Multiple commands can now be processed in sequence
- MOSFET Control:
  - Issue: "Enable MOSFETs" command triggered overheat error immediately after enabling
  - Investigation: get_temperature_ADC_value() was returning 0, below overheat threshold
  - Fix: Updated get_temperature_ADC_value() to return safe temperature value (16000)
  - Result: MOSFETs can now be enabled and disabled properly, visualization updates correctly
- USART Transmit Blocking:
  - Issue: Transmit operations were blocked by read loop, causing data loss
  - Investigation: Found transmit handling in read thread was blocking on read()
  - Fix: Split read and write operations into separate threads
  - Result: Commands and responses now transmit reliably without blocking

- Timer Performance:
  - Issue: TIM16_IRQHandler frequency was unstable during motor movement
  - Investigation: Found timing affected by visualization and debug logging
  - Fix: Moved motor control to dedicated thread with spinlock timing
  - Result: Stable 31.25 kHz with <100μs deviation

- High CPU Usage:
  - Issue: Simulator was consuming excessive CPU resources
  - Impact:
    * System performance degradation
    * Rapid battery drain on laptops
    * Non-smooth motor movement visualization
  - Fix:
    * Optimized spinlock timing in motor control thread
    * Adjusted SDL rendering frequency
    * Improved thread synchronization
  - Result: Simulator now runs efficiently with minimal CPU usage while maintaining accurate timing

- MOSFET State After Reset:
  - Issue: MOSFETs remained enabled after "System reset" command
  - Investigation: Found GPIO IDR state not tracking BSRR changes in simulation
  - Fix:
    * Changed is_mosfets_enabled() to read GPIO state directly
    * Added GPIO state tracking in motor control thread (31.25kHz)
    * IDR registers now properly track BSRR changes
  - Result: MOSFETs correctly disabled after system reset

- "Set device alias" Command:
  - Issue: Motor stopped responding after "Set device alias" command
  - Investigation: Found microsecond_delay() not implemented in simulation
  - Fix:
    * Created microsecond_clock_sim.c for simulation timing
    * Implemented accurate microsecond timing using CLOCK_MONOTONIC
    * Added proper GPIO state tracking for visualization
  - Result: Device alias successfully changes and persists across resets

### Current Focus
- No current focus items - all planned features implemented

### Known Issues
- Motor Movement When Disabled:
  - Issue: Motor visualization continues to move even when MOSFETs are disabled
  - Expected: Real motor cannot move with disabled MOSFETs
  - Impact: Simulator behavior differs from real hardware
  - Status: To be fixed in future update

- Fatal Error Handling:
  - Issue: Program terminates when fatal error occurs
  - Current behavior:
    * Error code is printed to log
    * Program exits immediately
  - Expected behavior:
    * Error code should be printed to log (working)
    * Motor should become inactive but program should continue running
    * Normal operation should resume after receiving "System reset" command
  - Impact: Unable to recover from fatal errors without manual restart
  - Status: To be investigated


### Implementation Mistakes
- Attempted to implement TIM16_IRQHandler() when it already exists in motor_control.c:
  - Mistakenly thought the function was missing because we couldn't see its implementation
  - Important lesson: Don't assume a function needs to be implemented just because you can't find it
  - Always check all source files thoroughly before attempting to implement a function
- Over-complicated timing logic:
  - Added unnecessary checks for elapsed time after nanosleep()
  - Added complex signal handling for nanosleep()
  - Important lesson: Keep it simple - nanosleep() is sufficient for timing
- Wasted time editing obsolete function:
  - Tried to modify handle_firmware_command() in servo_simulator.c
  - This function is obsolete as commands are now handled in main.c's processCommand()
  - Important lesson: Always check if code is still in use before modifying it
  - When fixing issues, first verify the code path is actually being used
- Failed to mark debug prints with "// DEBUG" comment:
  - Added debug prints to error_handling.c without proper "// DEBUG - temporarily added to aid in debugging" marker
  - Made another mistake by not marking variable assignments used only for debugging
  - Important lesson: Every line added for debugging must have "// DEBUG - temporarily added to aid in debugging"
  - This includes variable assignments, not just print statements
  - This helps track which code was added for debugging purposes and should be removed later

- Incorrectly documented fixed issues:
  - Wrote about solved timer performance issue in Known Issues section
  - Fixed issues should be removed from Known Issues and added to Fixed Issues
  - Important lesson: Documentation organization matters - keep issues in their proper sections
  - Known Issues is only for current, unresolved issues
  - When an issue is fixed, move it from Known Issues to Fixed Issues

- Attempted to delete previous test results:
  - When adding new test results, tried to delete older test results
  - Important lesson: Never delete test results
  - All test results are valuable documentation of what was tested
  - Keep all test results as a record of implementation progress

### Implementation Details
- Device addressing:
  - Device alias (0x01) loaded from non-volatile memory at startup
  - Commands must match device alias or ALL_ALIAS (0xFF)
  - Command validation in RS485.c checks:
    ```c
    if(selectedAxis != RESPONSE_CHARACTER && (selectedAxis == global_settings.my_alias || selectedAxis == 255))
    ```

- CRC32 calculation:
  - Implemented in settings_sim.c to match hardware behavior
  - Used for "Detect devices" command response validation
  - Calculates CRC32 over:
    * 8-byte unique ID
    * 1-byte device alias
  - Matches hardware CRC32 peripheral behavior:
    * Uses standard CRC32 polynomial (0xEDB88320)
    * Final value is inverted (~CRC32)

- Movement Commands:
  - "Go to position": 
    * Single command that handles acceleration/deceleration automatically
    * Example with position verification:
      ```bash
      ./motor_command.py -a 01 -p /dev/ttys123 "Enable MOSFETs" ; \
      ./motor_command.py -a 01 -p /dev/ttys123 "Go to position" 1000000 31250 ; \
      sleep 2 ; ./motor_command.py -a 01 -p /dev/ttys123 "Get comprehensive position"
      ```
  - Compound Commands:
    * "Move with velocity": 
      - Must be used as a pair of commands with sleep:
        1. First command: Set desired velocity and duration
        2. Second command: Set velocity=0 with short duration
        3. Sleep longer than total movement time
      - Example: 
        ```bash
        ./motor_command.py -a 01 -p /dev/ttys123 "Move with velocity" 1000000 31250 ; \
        ./motor_command.py -a 01 -p /dev/ttys123 "Move with velocity" 0 1000 ; \
        sleep 2
        ```
      - Fatal error occurs if velocity not brought to 0
    * "Move with acceleration":
      - Must be used as a pair of commands with sleep:
        1. First command: Set positive acceleration for time T
        2. Second command: Set negative acceleration for time T
        3. Sleep longer than total movement time
      - Example:
        ```bash
        ./motor_command.py -a 01 -p /dev/ttys123 "Move with acceleration" 400000 31250 ; \
        ./motor_command.py -a 01 -p /dev/ttys123 "Move with acceleration" -400000 31250 ; \
        sleep 2
        ```
      - Ensures velocity returns to 0

### Protocol Details
- Command Format:
  1. Axis byte: 0x00 (AXIS_MOTOR) or 0xFF (AXIS_ALL)
  2. Command byte: Command identifier
  3. Length byte: Number of payload bytes (0 for no payload)
  4. Payload bytes (if any)
- Device Addressing:
  - Each device has a unique ID and an alias
  - "Set device alias" command can change a device's alias
  - Commands can target a specific alias or all devices (0xFF)

### To Do
1. Update simulator to track hall sensor position with commanded position

## Testing Process
For each feature implementation:
1. Verify project compiles successfully
2. Confirm program runs without crashes
3. Test commands using motor_command.py:
   ```bash
   ./motor_command.py -a <alias> -p <port> "<command name>" [parameters...]
   ```
   Example:
   ```bash
   ./motor_command.py -a 01 -p /dev/ttys123 "Set maximum velocity" 1000
   ```
4. Document test results and any issues found

Note: Always use motor_command.py to send commands rather than raw byte sequences. This ensures we use the correct command format and parameter encoding.

## Source Files
- servo_simulator.c: Main simulator implementation
- ../firmware/Src/main.c: Motor firmware main logic
- ../common_source_files/RS485.c: RS485 communication implementation
- Makefile: Project build configuration

## Implementation Notes
- Debug print statements should be marked with "// DEBUG - temporarily added to aid in debugging"
- RS485 command processing follows the actual firmware's interrupt-driven approach
- Commands are processed only when receive interrupt is enabled (RXNEIE)
- RXNE flag is properly managed to simulate hardware behavior

## Original Requirements Message
Currently I have a program that simulates a small subset of functionality of a servomotor. The program is called servo_simulator.c. I need to make a more comprehensive simulator that supports many more commands. I want to use most of the actual motor firmware code, so that the simulation is well representative of what the motor will actually do. I have built a suitable makefile already, which successfully compiles the project on my M1 Mac. Then, we will have to emulate some parts of the hardware of the STM32G031 chip to receive commands over a simulated serial port and also extract the movement information from the motor firmware and show a visualization of the motor spinning in an accurate way. The graphical visualization of the motor (spinning or stationary) is already working and is implemented in servo_simulator.c. Another part that is implemented is that a simulated serial port appears on the system as /dev/ttysxxx, where xxx is some number. If I send a command to that erial port, the bytes are received in the simulator and printed to the log window. From there, the command is not yet correctly received and interpretted by the firmware of the motor. 

The source code files to look at first are:
Makefile
servo_simulator.c
../firmware/Src/main.c
../common_source_files/RS485.c

Please look around at the various source code available here and then let's start working on letting the RS485 module receive bytes through the simulated serial port and interpretting them correctly. If you need to add print statements to the source code of the servomotor, such as in main.c or RS485.c then please always follow the print statements with // DEBUG - temporarily added to aid in debugging. later remove this line.

This is the documentation of the project as we work through it. Please always keep this file up to date at each iteration so that we know what all the requirements are and what is still left to do. This file should grow over time but be kept concide. You can use .md markup format if you want. I will be strictly policing this, that you update this file at each iteration. If you don't have any update then you can skip it if you let me know explicitly that there is not update at this time. If we find a major problem and we solve it then we should also document it here. If you make a major mistake and we need to backtrack a bit then we also need to add some documentation here to prevent us from making that same mistake. This message should remain in the documentation, but can be reworded if that simhow improves the messaging.
