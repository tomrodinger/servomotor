#!/usr/bin/env python3

import argparse
import time
import servomotor
import sys

def main():
    # Define the arguments for this program
    parser = argparse.ArgumentParser(description='Continually get the comprehensive position and print it to the screen')
    parser.add_argument('-a', '--alias', help='alias of the device to control', required=True)
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
    parser.add_argument('-d', '--duration', type=int, default=30, help='Duration for the test to run in seconds.')
    args = parser.parse_args()

    alias_str = args.alias
    if alias_str == "255" or alias_str == "254":
        print("Error: invalid alias. The alias cannot be 255 or 254.", file=sys.stderr)
        return False
    elif len(alias_str) > 1:
        print(f"Error: the alias must be just one character, not: {alias_str}", file=sys.stderr)
        return False

    alias = ord(alias_str)
    verbose_level = 2 if args.verbose else 0
    test_passed = True

    servomotor.set_serial_port_from_args(args)
    motorX = servomotor.M3(alias, verbose=verbose_level)
    
    try:
        servomotor.open_serial_port()
        
        print(f"Running position reporting for {args.duration} seconds...")
        start_time = time.time()
        while time.time() - start_time < args.duration:
            response = motorX.get_comprehensive_position()
            if response is None:
                print("Error: get_comprehensive_position() returned None", file=sys.stderr)
                test_passed = False
                break
            print(response)
            time.sleep(0.05)
            
        end_time = time.time()
        print(f"\nTest completed in {end_time - start_time:.2f} seconds.")

    except Exception as e:
        print(f"An error occurred: {e}", file=sys.stderr)
        test_passed = False
    finally:
        print("Closing serial port...")
        servomotor.close_serial_port()
    
    return test_passed

if __name__ == '__main__':
    if main():
        print("\nPASSED")
        sys.exit(0)
    else:
        print("\nFAILED")
        sys.exit(1)