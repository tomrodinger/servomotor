#!/usr/bin/env python3

"""
Test program for device detection functionality.

This program detects all devices on the RS485 bus and prints information about them.
It supports multiple detection rounds and combines the results.

This is the per-command test for the "Detect devices" command (command 20). It
passes when at least one device is found on the bus. The "Detect devices"
command itself is inherently broadcast — it has no per-device addressing — so
`-a/--alias` is not used as an address by this test. When `-a` is supplied
(as `run_all_tests.py` always does), the test additionally asserts that a
device with that alias is present in the detected list; otherwise it fails.
"""

import argparse
import servomotor
from servomotor.device_detection import detect_devices_iteratively


def alias_arg_to_int(alias):
    """Convert a -a/--alias argument (string or int) to its byte value (0-255)."""
    if alias is None:
        return None
    if isinstance(alias, int):
        return alias
    if isinstance(alias, str) and len(alias) == 1:
        return ord(alias)
    return int(alias)


def run_detection(args, required_alias):
    """Open the port, run detection, and return (status, message).

    status is 0 on success, 1 on failure. The caller prints the trailing
    PASSED / FAILED line after the serial port is closed so that PASSED
    remains the final line of stdout (which run_all_tests.py checks)."""
    try:
        servomotor.open_serial_port()

        print(f"Starting device detection with {args.n_detections} detection rounds...")
        print()

        devices = detect_devices_iteratively(n_detections=args.n_detections, verbose=args.verbose)

        print()
        print("=" * 80)
        print("DEVICE DETECTION RESULTS")
        print("=" * 80)
        print()

        if not devices:
            print("No devices were detected.")
            print("=" * 80)
            return 1, "no devices were detected on the RS485 bus."

        print("Device Details:")
        print("-" * 50)
        for i, device in enumerate(devices, 1):
            alias_str = servomotor.get_human_readable_alias_or_unique_id(device.alias)
            print(f"{i:2d}. Unique ID: {device.unique_id:016X}")
            print(f"    Alias:     {alias_str}")
            print()
        print("=" * 80)
        print(f"Total devices detected: {len(devices)}")

        # If -a was supplied (and isn't broadcast 255), assert that a device
        # with that alias is present. run_all_tests.py always supplies -a, so
        # the suite-run case requires the test motor at that alias.
        if required_alias is not None and required_alias != 255:
            found = any(device.alias == required_alias for device in devices)
            required_str = servomotor.get_human_readable_alias_or_unique_id(required_alias)
            if not found:
                print("=" * 80)
                return 1, f"no device with alias {required_str} (byte {required_alias}) is present in the detected list."
            print(f"Required alias {required_str} (byte {required_alias}) is present in the detected list.")

        print("=" * 80)
        return 0, "ok"

    except Exception as e:
        return 1, f"Error during device detection: {e}"

    finally:
        servomotor.close_serial_port()


def main():
    parser = argparse.ArgumentParser(description='Detect all devices on the RS485 bus')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-a', '--alias', default=None,
                        help='if supplied (and not 255), additionally require that a device with this alias is present in the detected list. The Detect devices command itself is broadcast and does not use the alias for addressing.')
    parser.add_argument('--n-detections', type=int, default=3, help='Number of detection rounds (default: 3)')
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')

    args = parser.parse_args()
    required_alias = alias_arg_to_int(args.alias)

    servomotor.set_serial_port_from_args(args)

    status, message = run_detection(args, required_alias)

    # Print the final verdict line AFTER close_serial_port() has run (inside
    # run_detection's finally), so PASSED is the literal last line of stdout
    # — which run_all_tests.py grades on.
    if status == 0:
        print("PASSED")
    else:
        print(f"FAILED: {message}")
    return status


if __name__ == "__main__":
    exit(main())
