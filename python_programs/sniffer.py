#!/usr/bin/env python3

import servomotor
import time
import argparse

def print_program_description():
    print("This program will read raw bytes from the servomotor and print them to the console.")
    print("The program will print the time since the start of the program, the byte read, and the byte in hexadecimal format.")
    print("The program will run indefinitely until it is stopped by the user.")

def main():
    parser = argparse.ArgumentParser(description='Servomotor random speed stress test')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
    args = parser.parse_args()

    print_program_description()

    if args.verbose:
        verbose_level = 2
    else:
        verbose_level = 0

    # Initialize communication
    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port()

    start_time = time.time()
    while 1:
        b = servomotor.read_raw_byte()
        time_since_start = time.time() - start_time
        if b != None:
            ord_b = ord(b)
            print(f"Time: {time_since_start} Read: {b} ({ord_b}) (0x{ord_b:02X})")

if __name__ == "__main__":
    main()
