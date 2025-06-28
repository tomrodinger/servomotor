#!/usr/bin/env python3
"""
Framing Error Test Program

This script sends random bytes to a serial port using 9-bit mode with even parity.
This should trigger framing errors in devices operating in 8-bit mode.
After sending data, it queries devices for framing errors to verify the test worked.
"""

import argparse
import random
import sys
import time
import serial
import servomotor
from servomotor import M3
from servomotor.device_detection import detect_devices_iteratively


def verify_expected_status(motor, expected_status, verbose=False):
    """Check if a device has the expected fatal error status code."""
    try:
        status = motor.get_status(verbose=verbose)
        
        if verbose:
            print(f"  Raw status response: {status} (type: {type(status)})")
        
        if status is None:
            print(f"Warning: No status response from device")
            return False
            
        if not isinstance(status, (list, tuple)):
            print(f"Warning: Status response is not a list/tuple: {status} (type: {type(status)})")
            return False
            
        if len(status) < 2:
            print(f"Warning: Status response too short: {status} (length: {len(status)})")
            return False
            
        status_flags = status[0]
        fatal_error_code = status[1]
        
        if verbose:
            print(f"  Status flags: {status_flags}, Fatal error code: {fatal_error_code}")
        
        if fatal_error_code is None:
            print(f"Warning: Fatal error code is None")
            return False
            
        return fatal_error_code == expected_status
        
    except Exception as e:
        print(f"Error querying device status: {e}")
        if verbose:
            import traceback
            traceback.print_exc()
        return False


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
        
        # Close servomotor serial port before sending framing error data
        servomotor.close_serial_port()
        
        # Send framing error data using the same port
        port_name = args.port if args.port else servomotor.communication.serial_port_name
        success = send_framing_error_data(port_name, args.send_frequency, args.duration, args.verbose)
        
        if not success:
            print("Failed to send framing error data")
            sys.exit(1)
        
        # Reopen servomotor serial port to check for errors
        servomotor.open_serial_port()
        
        # Check for framing errors
        print("Checking devices for framing errors...")
        
        devices_with_errors = 0
        total_devices = len(devices)
        
        for i, device_info in enumerate(devices, 1):
            if isinstance(device_info, tuple) and len(device_info) >= 2:
                alias_or_unique_id, _ = device_info
            else:
                alias_or_unique_id = device_info
                
            alias_str = servomotor.get_human_readable_alias_or_unique_id(alias_or_unique_id)
            print(f"Checking device {i}/{total_devices} (Alias: {alias_str})...")
            
            # Create motor object for this device
            motor = M3(alias_or_unique_id, verbose=args.verbose)
            
            # Step 1: Check for framing error (fatal error code 36)
            try:
                has_framing_error = verify_expected_status(motor, 36, verbose=args.verbose)
            except Exception as e:
                print(f"Exception in verify_expected_status: {e}")
                import traceback
                traceback.print_exc()
                has_framing_error = False
            
            if has_framing_error:
                print(f"  ✓ Device has framing error (fatal error code 36)")
                devices_with_errors += 1
                
                # Step 2: Reset the device to clear the framing error
                print(f"  Resetting device to clear framing error...")
                try:
                    motor.system_reset(verbose=args.verbose)
                    time.sleep(1.0)  # Wait 1 second after reset
                    
                    # Step 3: Verify the framing error was cleared (status should be 0)
                    print(f"  Verifying framing error was cleared...")
                    status_cleared = verify_expected_status(motor, 0, verbose=args.verbose)
                    
                    if status_cleared:
                        print(f"  ✓ Framing error successfully cleared (fatal error code 0)")
                    else:
                        print(f"  ✗ Framing error was NOT cleared properly")
                        devices_with_errors -= 1  # Don't count this as a success
                        
                except Exception as e:
                    print(f"  ✗ Error during reset/verification: {e}")
                    devices_with_errors -= 1  # Don't count this as a success
            else:
                print(f"  ✗ Device does NOT have framing error")
        
        print(f"\nResults: {devices_with_errors}/{total_devices} devices successfully triggered and cleared framing errors")
        
        if devices_with_errors == total_devices and total_devices > 0:
            print("PASSED")
            return 0
        else:
            print("FAILED")
            return 1
        
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Close serial port
        servomotor.close_serial_port()


if __name__ == "__main__":
    sys.exit(main())