# MicroPython Deployment Guide for ESP32-S3

This guide explains how to deploy and use the servomotor library on an ESP32-S3 running MicroPython.

## Prerequisites

1. **ESP32-S3 board** with MicroPython installed
2. **USB cable** to connect ESP32-S3 to your computer
3. **MicroPython firmware** (v1.19 or later recommended)
4. **Serial terminal** program (e.g., `screen`, `minicom`, or Thonny IDE)

## Hardware Setup

### Serial Connection to Servomotor
- **TX Pin**: GPIO 4 (ESP32-S3 to Servomotor RX)
- **RX Pin**: GPIO 5 (ESP32-S3 from Servomotor TX)
- **GND**: Common ground between ESP32-S3 and servomotor
- **Baud Rate**: 230400

```
ESP32-S3          Servomotor
  GPIO 4 (TX) --> RX
  GPIO 5 (RX) <-- TX
  GND         --- GND
```

## Installation Steps

### 1. Install MicroPython on ESP32-S3

If you haven't already installed MicroPython on your ESP32-S3:

```bash
# Download MicroPython firmware for ESP32-S3
# Visit: https://micropython.org/download/ESP32_GENERIC_S3/

# Erase flash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 erase_flash

# Flash MicroPython firmware
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash -z 0x0 ESP32_GENERIC_S3-*.bin
```

### 2. Upload Servomotor Library to ESP32-S3

You need to copy the `servomotor` directory to your ESP32-S3. There are several methods:

#### Method A: Using ampy (Recommended)

```bash
# Install ampy
pip install adafruit-ampy

# Upload the entire servomotor directory
ampy --port /dev/ttyUSB0 put servomotor

# Upload JSON configuration files
ampy --port /dev/ttyUSB0 put servomotor/data_types.json servomotor/data_types.json
ampy --port /dev/ttyUSB0 put servomotor/motor_commands.json servomotor/motor_commands.json
ampy --port /dev/ttyUSB0 put servomotor/error_codes.json servomotor/error_codes.json
ampy --port /dev/ttyUSB0 put servomotor/unit_conversions_M3.json servomotor/unit_conversions_M3.json

# Upload your demo program
ampy --port /dev/ttyUSB0 put ball_juggling_demo.py
```

#### Method B: Using Thonny IDE

1. Open Thonny IDE
2. Select "MicroPython (ESP32)" from the interpreter dropdown
3. Select your ESP32-S3 port
4. Use File > Open to navigate to files
5. Right-click on `servomotor` folder and select "Upload to /"
6. Upload `ball_juggling_demo.py` to the root directory

#### Method C: Using WebREPL (if enabled)

1. Enable WebREPL on your ESP32-S3
2. Connect via web browser
3. Use the file upload feature to transfer files

### 3. Verify Installation

Connect to your ESP32-S3 via serial terminal:

```bash
# Using screen
screen /dev/ttyUSB0 115200

# Or using minicom
minicom -D /dev/ttyUSB0 -b 115200
```

In the MicroPython REPL:

```python
>>> import servomotor
>>> servomotor.get_platform()
'micropython'
>>> servomotor.is_micropython()
True
>>> servomotor.get_platform_info()
{'platform': 'micropython', 'is_micropython': True, 'is_esp32': True, ...}
```

## Running ball_juggling_demo.py

### Method 1: Direct Execution from REPL

```python
>>> import ball_juggling_demo
```

### Method 2: As main.py (Auto-run on boot)

```bash
# Rename and upload as main.py to auto-run on boot
ampy --port /dev/ttyUSB0 put ball_juggling_demo.py main.py
```

Then reset the board - the demo will run automatically.

### Method 3: Using exec()

```python
>>> with open('ball_juggling_demo.py') as f:
...     exec(f.read())
```

## Expected Output

When running successfully, you should see:

```
Running on MicroPython - file logging disabled
INFO: Opened UART on ESP32-S3 [TX=Pin(4), RX=Pin(5)]
INFO: Testing communication with device with alias X by pinging it 10 times
INFO: The device responded correctly to all the 10 pings
...
```

## Troubleshooting

### Problem: Import Error

**Error**: `ImportError: no module named 'servomotor'`

**Solution**: 
- Verify the servomotor directory was uploaded correctly
- Check that all required files are present using `import os; os.listdir('servomotor')`

### Problem: UART Not Working

**Error**: No response from servomotor

**Solution**:
- Check physical connections (TX, RX, GND)
- Verify baud rate (should be 230400)
- Test with a simple UART echo:
  ```python
  from machine import UART, Pin
  uart = UART(1, baudrate=230400, tx=Pin(4), rx=Pin(5))
  uart.write(b'test')
  ```

### Problem: Memory Error

**Error**: `MemoryError: memory allocation failed`

**Solution**:
- The ESP32-S3 has limited RAM. Try:
  - Reducing VERBOSE level
  - Importing only necessary modules
  - Running `import gc; gc.collect()` periodically
  - Consider using frozen modules (compile Python to bytecode)

### Problem: Platform Detection Issues

**Error**: Library tries to use standard Python features

**Solution**:
- Verify platform detection: `import sys; print(sys.implementation.name)`
- Should return `'micropython'`

## Memory Optimization Tips

1. **Use frozen modules**: Compile the servomotor library into the firmware
2. **Garbage collection**: Call `gc.collect()` periodically
3. **Reduce verbosity**: Set `VERBOSE = 0` in demos
4. **Selective imports**: Only import what you need

Example of memory-efficient usage:

```python
import gc
import servomotor

# Force garbage collection
gc.collect()

# Create motor instance with minimal verbosity
m = servomotor.M3(ord('X'), verbose=0)

# Your code here...

# Clean up when done
gc.collect()
```

## Performance Considerations

- **UART Speed**: 230400 baud is the standard speed
- **CPU Speed**: ESP32-S3 runs at 240MHz by default
- **Response Time**: Expect similar performance to standard Python for motor control
- **Memory**: ESP32-S3 has ~320KB RAM, sufficient for this library

## Updating the Library

To update the library on ESP32-S3:

```bash
# Remove old version
ampy --port /dev/ttyUSB0 rmdir servomotor

# Upload new version
ampy --port /dev/ttyUSB0 put servomotor
```

## Differences from Standard Python

The MicroPython version has these differences:

1. **No file logging**: Log files are not created
2. **No color terminal output**: Simple text messages only
3. **Simplified error messages**: Less verbose error reporting
4. **Fixed UART pins**: TX=Pin(4), RX=Pin(5) (cannot be changed without modifying code)
5. **No port selection menu**: UART is directly initialized

## Advanced: Creating Frozen Modules

For optimal memory usage, you can create frozen modules:

1. Clone MicroPython repository
2. Copy `servomotor` directory to `ports/esp32/modules/`
3. Rebuild firmware:
   ```bash
   cd micropython/ports/esp32
   make BOARD=ESP32_GENERIC_S3 clean
   make BOARD=ESP32_GENERIC_S3
   ```
4. Flash the new firmware

This embeds the servomotor library in ROM, freeing up RAM.

## Example: Minimal Motor Control

Here's a minimal example for testing:

```python
import servomotor
import time

# Open serial port (automatically uses correct pins)
servomotor.open_serial_port()

# Create motor instance
m = servomotor.M3(ord('X'), verbose=1)

# Reset and initialize
m.system_reset()
time.sleep(0.5)

# Test ping
test_data = b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A'
response = m.ping(test_data)
print("Ping response:", response)

# Enable motor
m.enable_mosfets()

# Get position
commanded, sensed, external = m.get_comprehensive_position()
print(f"Position - Commanded: {commanded}, Sensed: {sensed}")

# Close
servomotor.close_serial_port()
```

## Support

For issues specific to MicroPython deployment:
1. Check MicroPython documentation: https://docs.micropython.org/
2. Verify ESP32-S3 support: https://micropython.org/download/ESP32_GENERIC_S3/
3. Review this deployment guide for common solutions

For servomotor library issues:
1. Test on standard Python first to isolate platform-specific issues
2. Check that the library version supports cross-platform operation (v0.10.0+)
3. Verify all required JSON files are uploaded