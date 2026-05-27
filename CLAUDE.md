# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Servomotor by Gearotons - A complete ecosystem for controlling brushless servomotors via RS485. Includes embedded firmware (STM32), Python control library (cross-platform including MicroPython), Arduino C++ library, and code autogeneration tools.

## Build Commands

### Firmware (STM32)
```bash
cd firmware
make PRODUCT_NAME=M17                    # Build for specific product
make PRODUCT_NAME=M17 PROGRAMMING_TEST_JIG_MODE  # With test jig mode
make PRODUCT_NAME=M17 program            # Build and flash via RS485 bootloader
make clean                               # Clean build artifacts
```
Products: M1, M2, M17, M23

### Desktop Simulator (C with SDL2)
```bash
cd c_test_programs
make                                     # Builds servo_simulator executable
./servo_simulator                        # Run the simulator
```
Requires SDL2 and SDL2_ttf installed (`brew install sdl2 sdl2_ttf` on macOS).

### Autogeneration
```bash
cd autogeneration
python3 autogenerate.py M3               # Generate all code for motor type M3
python3 autogenerate.py M3 --skip-hall-sensor --skip-bldc  # Skip slow computations
```
Generates: unit conversion JSON/code, command methods (Arduino), BLDC lookup tables, hall sensor constants.

### Python Tests
```bash
cd python_programs
python3 test_ping.py -p /dev/ttyUSB0     # Basic communication test
python3 test_ping.py -P                  # Interactive port selection
python3 test_go_to_position.py -p /dev/ttyUSB0 --repeat 5
```
All tests support `-p PORT`, `-P` (interactive), `--verbose`, and many support `--repeat N`.

### Firmware Upgrade
```bash
cd python_programs
python3 upgrade_firmware.py ../firmware/firmware_releases/servomotor_M17_fw0.14.2.0_scc3_hw1.5.firmware
```

## Architecture

### Command Flow
1. **JSON definitions** (`motor_commands.json`, `data_types.json`) define all 255 motor commands with parameter types and unit conversions
2. **Autogeneration** creates Python methods, Arduino C++ methods, and unit conversion code from JSON
3. **Python library** (`servomotor/`) sends commands via RS485 at 230400 baud
4. **Firmware** (`firmware/Src/main.c`) processes commands in main loop, drives motor via GC6609 IC

### Key Directories
- `firmware/` - STM32G0xx firmware (C), builds `.firmware` files
- `common_source_files/` - Shared C code (RS485, CRC32, settings, GPIO) used by firmware and bootloaders
- `python_programs/servomotor/` - Python control library (works on standard Python and MicroPython)
- `Arduino_library/` - C++ library for Arduino platforms
- `autogeneration/` - Scripts that generate code from JSON command definitions
- `bootloader_STM32G031/` - Bootloader for firmware updates via RS485
- `c_test_programs/` - Desktop simulator using SDL2 for testing without hardware

### Python Library Structure
- `M3.py` - Main motor class with dynamically-generated command methods from JSON
- `communication.py` - Low-level RS485 packet handling (standard Python)
- `communication_micropython.py` - MicroPython-specific communication
- `command_loader.py` - Loads command definitions from JSON
- `device_detection.py` - Discovers motors on RS485 bus
- `platform_detect.py` - Detects standard Python vs MicroPython
- `serial_abstraction.py` - Cross-platform serial port abstraction

### Firmware Organization
- `main.c` - Main loop, command processing, version defines (`MAJOR/MINOR/BUGFIX/DEVELOPMENT_FIRMWARE_VERSION`)
- `GC6609.c` - Motor driver IC control (3-phase BLDC)
- `motor_control.c` - Motion profiling, PID control
- `AT5833.c` / `hall_sensor_calculations.c` - Hall sensor interface
- `commutation_table_M*.h` - Motor-specific commutation patterns (auto-generated)
- `VERSIONS` file - Maps product name to hardware version and software compatibility code

### Protocol Highlights
- Packet: size byte(s) | address byte(s) | command byte | payload | optional CRC32
- Size byte: LSB always 1, actual size = byte >> 1 (extended size if decoded value = 127)
- Address 255 = broadcast, 254 = extended addressing (8-byte unique ID)
- Response indicators: 253 (with CRC), 252 (without CRC)

## Testing

Tests use the `servomotor` module (not the older `communication` module). Test files marked "Obsolete" in `TEST_SUMMARY.md` use deprecated patterns.

Standard test pattern:
```python
import servomotor
from servomotor import M3

servomotor.set_serial_port_from_args(args)  # Before open_serial_port()
servomotor.open_serial_port()
motor = M3(alias_or_unique_id, time_unit=..., position_unit=...)
motor.system_reset()
# ... test logic ...
servomotor.close_serial_port()
```

## Version Management

Firmware version in `firmware/Src/main.c`:
```c
#define MAJOR_FIRMWARE_VERSION    0
#define MINOR_FIRMWARE_VERSION    14
#define BUGFIX_FIRMWARE_VERSION   2
#define DEVELOPMENT_FIRMWARE_VERSION 0
```

Hardware version and compatibility code in `firmware/VERSIONS`:
```
M17 1.5 3
M23 0.10 1
```

Output naming: `servomotor_M17_fw0.14.2.0_scc3_hw1.5.firmware`
