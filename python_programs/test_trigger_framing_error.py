#!/usr/bin/env python3
"""
Framing Error Test Program

This script sends random bytes to a serial port using 9-bit mode with even parity.
This should trigger framing errors in devices operating in 8-bit mode.
After sending data, it queries devices for communication statistics to verify
the framing error counter increased.
"""

import argparse
import random
import sys
import time
import serial
import servomotor
from servomotor import M3
from servomotor.device_detection import detect_devices_iteratively


def get_framing_error_count(motor, reset_counter=0, verbose=False):
    """Get the current framing error count from communication statistics."""
    # Call get_communication_statistics with the specified reset_counter flag
    stats = motor.get_communication_statistics(reset_counter, verbose=verbose)
    
    if verbose:
        print(f"  Raw communication statistics response: {stats}")
        print(f"  CRC32 errors: {stats[0]}")
        print(f"  Packet decode errors: {stats[1]}")
        print(f"  Framing errors: {stats[2]}")
        print(f"  Overrun errors: {stats[3]}")
        print(f"  Noise errors: {stats[4]}")
    
    # Return the framing error count (3rd element, index 2)
    return stats[2]


def send_framing_error_data(port, frequency, duration, verbose=False):
    """Send random bytes using even parity to trigger framing errors."""
    
    # Calculate delay between transmissions
    delay = 1.0 / frequency
    
    print(f"Sending framing error data:")
    print(f"  Port: {port}")
    print(f"  Baud rate: 230400")
    print(f"  Send frequency: {frequency} Hz")
    print(f"  Duration: {duration} seconds")
    print(f"  Mode: 8-bit + even parity (creates 9-bit frames)")
    print()
    
    # Open serial port with even parity
    try:
        ser = serial.Serial(
            port=port,
            baudrate=230400,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"Successfully opened {port} in 8-bit + even parity mode")
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}", file=sys.stderr)
        return False
    
    # Send random bytes
    bytes_sent = 0
    start_time = time.time()
    
    try:
        while (time.time() - start_time) < duration:
            # Generate random byte
            random_byte = random.randint(0, 255)
            
            # Send the byte
            try:
                bytes_written = ser.write(bytes([random_byte]))
                if bytes_written == 1:
                    bytes_sent += 1
                    if verbose:
                        print(f"Sent byte #{bytes_sent}: 0x{random_byte:02x} ({random_byte})")
                    elif bytes_sent % 10 == 0:
                        print(f"Sent {bytes_sent} bytes...")
                        
            except serial.SerialException as e:
                print(f"Error writing to serial port: {e}", file=sys.stderr)
                break
            
            # Wait for next transmission
            time.sleep(delay)
            
    except KeyboardInterrupt:
        print(f"\nInterrupted by user")
    
    finally:
        ser.close()
        
    elapsed_time = time.time() - start_time
    actual_frequency = bytes_sent / elapsed_time if elapsed_time > 0 else 0
    
    print(f"\nTransmission completed:")
    print(f"  Total bytes sent: {bytes_sent}")
    print(f"  Elapsed time: {elapsed_time:.2f} seconds")
    print(f"  Actual frequency: {actual_frequency:.2f} Hz")
    print()
    
    return bytes_sent > 0


def main():
    parser = argparse.ArgumentParser(description='Send random bytes in 9-bit mode to trigger framing errors')
    parser.add_argument('-p', '--port', help='Serial port to use (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-a', '--alias', type=str, help='Alias of the device to control (if not provided, will auto-detect)')
    parser.add_argument('--send-frequency', type=float, default=10.0,
                       help='Transmission frequency in Hz (bytes per second, default: 1.0)')
    parser.add_argument('--duration', type=float, default=3.0,
                       help='Duration to run in seconds (default: 10.0)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Print each byte being sent')
    
    args = parser.parse_args()
    
    # Validate frequency
    if args.send_frequency <= 0:
        print(f"Error: Send frequency must be positive, got {args.send_frequency}", file=sys.stderr)
        sys.exit(1)
    
    # Set up serial port for servomotor communication
    servomotor.set_serial_port_from_args(args)
    
    try:
        # Open serial port for device communication
        servomotor.open_serial_port()
        
        # Detect devices or use specified alias
        if args.alias:
            print(f"Using specified alias: {args.alias}")
            # Convert alias to the appropriate format
            alias_or_unique_id = servomotor.string_to_alias_or_unique_id(args.alias)
            devices = [(alias_or_unique_id, args.alias)]  # Create a simple list for consistency
        else:
            print("Detecting devices on the bus...")
            detected_devices = detect_devices_iteratively(n_detections=3, verbose=args.verbose)
            
            if not detected_devices:
                print("No devices detected. Cannot run test.")
                sys.exit(1)
                
            print(f"Detected {len(detected_devices)} device(s):")
            for i, device in enumerate(detected_devices, 1):
                alias_str = servomotor.get_human_readable_alias_or_unique_id(device.alias)
                print(f"  {i}. Unique ID: {device.unique_id:016X}, Alias: {alias_str}")
            print()
            
            # Convert to simple format for consistency
            devices = [(device.alias, device.alias) for device in detected_devices]
        
        # Step 1: Get initial framing error counts for all devices
        print("Getting initial framing error counts for all devices...")
        initial_counts = {}
        
        for i, device_info in enumerate(devices, 1):
            if isinstance(device_info, tuple) and len(device_info) >= 2:
                alias_or_unique_id, _ = device_info
            else:
                alias_or_unique_id = device_info
                
            alias_str = servomotor.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            print(f"Getting initial count for device {i}/{len(devices)} (Alias: {alias_str})...")
            
            # Create motor object for this device
            motor = M3(alias_or_unique_id, verbose=args.verbose)
            
            initial_count = get_framing_error_count(motor, verbose=args.verbose)
            print(f"  Initial framing error count: {initial_count}")
            initial_counts[alias_or_unique_id] = initial_count
        
        # Step 2: Close servomotor serial port before sending framing error data
        servomotor.close_serial_port()
        
        # Step 3: Send framing error data using the same port
        port_name = args.port if args.port else servomotor.communication.serial_port_name
        success = send_framing_error_data(port_name, args.send_frequency, args.duration, args.verbose)
        
        if not success:
            print("Failed to send framing error data")
            sys.exit(1)
        
        # Step 4: Reopen servomotor serial port to check for errors
        servomotor.open_serial_port()
        
        # Step 5: Check for framing error count increases
        print("Checking devices for framing error count increases...")
        
        devices_with_increased_errors = 0
        total_devices = len(devices)
        
        for i, device_info in enumerate(devices, 1):
            if isinstance(device_info, tuple) and len(device_info) >= 2:
                alias_or_unique_id, _ = device_info
            else:
                alias_or_unique_id = device_info
                
            alias_str = servomotor.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            print(f"Checking device {i}/{total_devices} (Alias: {alias_str})...")
            
            initial_count = initial_counts[alias_or_unique_id]
            
            # Create motor object for this device
            motor = M3(alias_or_unique_id, verbose=args.verbose)
            
            # Get final framing error count and reset counters
            print(f"  Getting final framing error count and resetting counters...")
            final_count = get_framing_error_count(motor, reset_counter=1, verbose=args.verbose)
            print(f"  Final framing error count: {final_count}")
            print(f"  ✓ Communication statistics counters reset")
            
            # Check if framing error count increased
            error_increase = final_count - initial_count
            if error_increase > 0:
                print(f"  ✓ Framing error count increased by {error_increase} (from {initial_count} to {final_count})")
                devices_with_increased_errors += 1
                
                # Verify that counters were reset by reading them again
                print(f"  Verifying counters are reset to 0...")
                post_reset_count = get_framing_error_count(motor, reset_counter=0, verbose=args.verbose)
                if post_reset_count == 0:
                    print(f"  ✓ Framing error counter successfully reset to 0")
                else:
                    print(f"  ✗ Framing error counter was NOT reset to 0: {post_reset_count}")
                    devices_with_increased_errors -= 1  # Don't count this as a success
            else:
                print(f"  ✗ Framing error count did NOT increase (from {initial_count} to {final_count})")
        
        print(f"\nResults: {devices_with_increased_errors}/{total_devices} devices successfully detected framing error count increases")
                
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Close serial port
        servomotor.close_serial_port()

    if devices_with_increased_errors == total_devices and total_devices > 0:
        print("PASSED")
        return 0
    else:
        print("FAILED")
        return 1

if __name__ == "__main__":
    sys.exit(main())