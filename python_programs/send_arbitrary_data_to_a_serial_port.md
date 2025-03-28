# Serial Port Arbitrary Data Sender

This utility allows sending arbitrary bytes to a specified serial port. It's designed for testing and debugging serial communications by providing a flexible command-line interface to send precise byte sequences.

## Features

- Send arbitrary byte sequences to any serial port
- Support for both decimal (0-255) and hexadecimal (0x00-0xff) byte formats
- Input validation to ensure all bytes are within valid range
- Optional CRC32 checksum calculation and appending
- Configurable baud rate (default being 230400)

## Requirements

- Python 3.6 or higher
- `pyserial` package

## Installation

1. Install the required Python package:

```bash
pip install pyserial
```

2. Make the script executable (Unix/Linux/macOS):

```bash
chmod +x send_arbitrary_data_to_a_serial_port.py
```

## Usage

```
python send_arbitrary_data_to_a_serial_port.py -p PORT [OPTIONS] BYTE1 BYTE2 ...
```

### Parameters

- `-p, --port PORT`: Serial port to use (e.g., `/dev/ttyUSB0`, `COM3`) [required]
- `-b, --baud RATE`: Baud rate (default: 230400)
- `--crc32`: Append a 4-byte CRC32 checksum to the data
- `BYTE1 BYTE2 ...`: List of bytes to send (space-separated)

### Byte Format

Bytes can be specified in either:
- Decimal: `0` to `255`
- Hexadecimal: `0x00` to `0xff` (case-insensitive)

### Examples

Send three bytes (decimal format) to `/dev/ttyUSB0`:
```bash
python3 send_arbitrary_data_to_a_serial_port.py -p /dev/ttyUSB0 10 20 30
```

Send bytes in hexadecimal format to COM3 at 115200 baud:
```bash
python3 send_arbitrary_data_to_a_serial_port.py -p COM3 -b 115200 0x0A 0x14 0x1E
```

Send bytes with CRC32 checksum appended:
```bash
python3 send_arbitrary_data_to_a_serial_port.py -p /dev/ttyUSB0 --crc32 0x01 0x02 0x03
```

Mix decimal and hexadecimal formats:
```bash
python3 send_arbitrary_data_to_a_serial_port.py -p /dev/ttyUSB0 0 0x01 10 0x0A 255 0xFF
```

## Output

The program will:
1. Validate all input bytes
2. Calculate and append CRC32 if requested
3. Display the bytes being sent (in both hexadecimal and decimal formats)
4. Send the data to the specified serial port
5. Report success or failure

## Error Handling

The program validates all inputs and provides clear error messages for:
- Invalid byte values or formats
- Out-of-range byte values
- Serial port connection issues

## Notes

- The CRC32 checksum is appended in little-endian byte order
- No data is read back from the serial port in this utility
