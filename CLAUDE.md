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
python3 upgrade_firmware.py ../firmware/firmware_releases/servomotor_M17_fw0.15.2.0_scc3_hw1.5.firmware
```

### Publishing the `servomotor` Python library to PyPI
Releases go out via GitHub Actions trusted publishing (OIDC, no stored token), triggered by a
**python-specific** git tag `pylib-v*` (plain `v*` is reserved for firmware/bootloader versioning in
this monorepo). Workflow: `.github/workflows/publish-python.yml`.

```bash
# 1. Bump the single source of truth for the version:
#    python_programs/servomotor/__init__.py  ->  __version__ = "0.11.0"
# 2. Commit, then tag and push:
git commit -am "servomotor 0.11.0"
git tag pylib-v0.11.0 && git push --tags        # -> builds + publishes to PyPI
```
- The tag is only the trigger; PyPI rejects re-used versions, so `__version__` must be bumped first.
- Build staging (copy `servomotor` + standalone/CLI modules into `PyPi_distribution/src`, strip
  cruft) lives in **`python_programs/PyPi_distribution/prepare_src.sh`**, called by BOTH the workflow
  and the manual fallback script — edit staging logic there only, so the two can't drift.
- **Manual fallback / TestPyPI:** `cd python_programs && ./build_and_submit_to_PyPi.sh` (prompts for
  TestPyPI vs PyPI). Or dry-run the workflow: Actions → "Publish Python library to PyPI" →
  Run workflow → `target: testpypi`.
- One-time PyPI setup (already done): a trusted publisher on the `servomotor` project with workflow
  `publish-python.yml` and environment `pypi`, plus matching GitHub environments `pypi`/`testpypi`.

### Running the hardware test suite
```bash
cd python_programs
python3 run_all_tests.py -p /dev/cu.usbserial-210 -a X   # alias 'X' == byte 88
```
`run_all_tests.py` runs every non-obsolete `servomotor`-module test listed in `TEST_SUMMARY.md`. To
test the **PyPI-published** library instead of the local `servomotor/` source (which otherwise shadows
the installed package because it sits next to the tests), run from a directory that does NOT contain a
`servomotor/` dir, with `servomotor` pip-installed in the active venv, and a `firmware/` symlink so the
firmware-upgrade test's `../firmware/firmware_releases` path resolves.

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
#define MINOR_FIRMWARE_VERSION    15
#define BUGFIX_FIRMWARE_VERSION   2
#define DEVELOPMENT_FIRMWARE_VERSION 0
```

Hardware version and compatibility code in `firmware/VERSIONS`:
```
M17 1.5 3
M23 0.10 1
```

Output naming: `servomotor_M17_fw0.15.2.0_scc3_hw1.5.firmware`
