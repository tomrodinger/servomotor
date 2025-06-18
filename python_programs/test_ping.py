#!/usr/bin/env python3
import servomotor
import time
import argparse
import sys

N_PINGS = 1000


def generate_random_10_byte_string():
    import random
    return bytes([random.randint(0, 255) for i in range(10)])


def main():
    parser = argparse.ArgumentParser(description='Test servomotor ping functionality')
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Set verbosity level
    verbose_level = 2 if args.verbose else 0
    
    # Set up serial port
    servomotor.set_serial_port_from_args(args)
    
    # Create motor instance with provided alias
    motorX = servomotor.M3(args.alias,
                          time_unit="seconds",
                          position_unit="degrees",
                          velocity_unit="degrees_per_second",
                          acceleration_unit="degrees_per_second_squared",
                          current_unit="milliamps",
                          voltage_unit="volts",
                          temperature_unit="celsius",
                          verbose=verbose_level)

    servomotor.open_serial_port()
    motorX.system_reset()
    time.sleep(1.2)

    start_time = time.time()
    last_progress_time = start_time

    for i in range(N_PINGS):
        current_time = time.time()
        # Print progress every 3 seconds
        if current_time - last_progress_time >= 3.0:
            print(f"Ping progress: {i}/{N_PINGS} ({(i/N_PINGS)*100:.1f}%)")
            last_progress_time = current_time
        
        random_10_byte_string = generate_random_10_byte_string()
        response = motorX.ping(random_10_byte_string)
        # Make sure the response is the same as the random 10 byte string
        if response != random_10_byte_string:
            print("Ping failed")
            print("Sent:", random_10_byte_string)
            print("Received:", response)
            print("FAILED")
            sys.exit(1)

    # Print final summary
    total_time = time.time() - start_time
    print(f"Completed {N_PINGS} pings in {total_time:.2f} seconds")

    servomotor.close_serial_port()
    del motorX

    print("PASSED")


if __name__ == "__main__":
    main()
