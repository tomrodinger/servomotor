#!/usr/bin/env python3

"""
Test program for device detection functionality.

This program detects all devices on the RS485 bus and prints information about them.
It supports multiple detection rounds and combines the results.
"""

import argparse
import servomotor
from servomotor.device_detection import detect_devices_iteratively


def main():
    """Main function for device detection test."""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Detect all devices on the RS485 bus')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('--n-detections', type=int, default=3, help='Number of detection rounds (default: 3)')
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
    
    args = parser.parse_args()
    
    # Set up serial port
    servomotor.set_serial_port_from_args(args)
    
    try:
        # Open serial port
        servomotor.open_serial_port()
        
        print(f"Starting device detection with {args.n_detections} detection rounds...")
        print()
        
        # Detect devices
        devices = detect_devices_iteratively(n_detections=args.n_detections, verbose=args.verbose)
        
        # Print results
        print()
        print("=" * 80)
        print("DEVICE DETECTION RESULTS")
        print("=" * 80)
        print()
        
        if devices:
            print("Device Details:")
            print("-" * 50)
            for i, device in enumerate(devices, 1):
                alias_str = servomotor.get_human_readable_alias_or_unique_id(device.alias)
                print(f"{i:2d}. Unique ID: {device.unique_id:016X}")
                print(f"    Alias:     {alias_str}")
                print()
            print("=" * 80)
            print(f"Total devices detected: {len(devices)}")

        else:
            print("No devices were detected.")
        
        print("=" * 80)
        
    except Exception as e:
        print(f"Error during device detection: {e}")
        return 1
    
    finally:
        # Close serial port
        servomotor.close_serial_port()
    
    return 0


if __name__ == "__main__":
    exit(main())