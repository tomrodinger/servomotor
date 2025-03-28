#!/usr/bin/env python3
"""
Serial Port Arbitrary Data Sender

This script sends arbitrary bytes to a specified serial port. Bytes can be provided
as decimal (0-255) or hexadecimal (0x00-0xff) values. It also supports appending
a CRC32 checksum to the data if requested.
"""

import argparse
import binascii
import re
import sys
import serial


def validate_byte(byte_str):
    """
    Validate and convert a string representation of a byte to an integer.
    
    Args:
        byte_str (str): String representation of a byte (decimal or hex)
        
    Returns:
        int: The byte value as an integer
        
    Raises:
        ValueError: If the input is not a valid byte representation
    """
    try:
        # Check if it's a hex value
        if byte_str.lower().startswith('0x'):
            value = int(byte_str, 16)
        else:
            value = int(byte_str)
            
        # Validate range
        if value < 0 or value > 255:
            raise ValueError(f"Byte value {byte_str} out of range (0-255)")
        
        return value
    except ValueError:
        raise ValueError(f"Invalid byte value: {byte_str}")


def calculate_crc32(data):
    """
    Calculate CRC32 checksum for the given data.
    
    Args:
        data (bytes): Data to calculate CRC32 for
        
    Returns:
        bytes: 4-byte CRC32 value
    """
    crc = binascii.crc32(data) & 0xFFFFFFFF
    return crc.to_bytes(4, byteorder='little')


def main():
    parser = argparse.ArgumentParser(description='Send arbitrary bytes to a serial port')
    parser.add_argument('-p', '--port', required=True, help='Serial port to use (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=230400, help='Baud rate (default: 230400)')
    parser.add_argument('--crc32', action='store_true', help='Append CRC32 checksum to the data')
    parser.add_argument('bytes', nargs='+', help='Bytes to send (decimal 0-255 or hex 0x00-0xff)')
    
    args = parser.parse_args()
    
    # Validate and convert all bytes
    try:
        byte_values = [validate_byte(b) for b in args.bytes]
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Convert to bytes
    data = bytes(byte_values)
    
    # Add CRC32 if requested
    if args.crc32:
        crc = calculate_crc32(data)
        data += crc
        print(f"Appending CRC32: 0x{crc.hex()}")
    
    # Print summary of what we're sending
    print(f"Sending {len(data)} bytes to {args.port} at {args.baud} baud:")
    print("Hex: " + " ".join(f"0x{b:02x}" for b in data))
    print("Dec: " + " ".join(f"{b}" for b in data))
    
    # Send the data
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            bytes_written = ser.write(data)
            print(f"Successfully sent {bytes_written} bytes")
    except serial.SerialException as e:
        print(f"Error opening or writing to serial port: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
