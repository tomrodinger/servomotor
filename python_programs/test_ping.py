#!/usr/bin/env python3
import servomotor
import time
import argparse
import sys


def generate_random_10_byte_string():
    import random
    return bytes([random.randint(0, 255) for i in range(10)])


def main():
    parser = argparse.ArgumentParser(description='Test servomotor ping functionality')
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--duration', type=float, default=3.0, help='Duration to run pings in seconds (default: 3.0)')
    parser.add_argument('--send-frequency', type=float, help='Frequency to send pings in Hz (if not specified, sends as fast as possible)')
    
    args = parser.parse_args()
    
    # Validate duration
    if args.duration <= 0:
        print(f"Error: Duration must be positive, got {args.duration}", file=sys.stderr)
        sys.exit(1)
    
    # Validate send frequency if provided
    if args.send_frequency is not None and args.send_frequency <= 0:
        print(f"Error: Send frequency must be positive, got {args.send_frequency}", file=sys.stderr)
        sys.exit(1)
    
    # Set verbosity level
    verbose_level = 2 if args.verbose else 0
    
    # Set up serial port
    servomotor.set_serial_port_from_args(args)
    
    # Create motor instance with provided alias
    motor = servomotor.M3(args.alias, verbose=verbose_level)

    servomotor.open_serial_port()
    motor.system_reset()
    time.sleep(1.2)

    start_time = time.time()
    last_progress_time = start_time
    ping_count = 0
    successful_pings = 0
    failed_pings = 0
    
    # Calculate delay between pings if frequency is specified
    ping_delay = 1.0 / args.send_frequency if args.send_frequency else 0

    print(f"Starting ping test:")
    print(f"  Duration: {args.duration} seconds")
    if args.send_frequency:
        print(f"  Send frequency: {args.send_frequency} Hz")
    else:
        print(f"  Send frequency: as fast as possible")
    print()

    try:
        while (time.time() - start_time) < args.duration:
            current_time = time.time()
            
            # Print progress every 3 seconds
            if current_time - last_progress_time >= 3.0:
                elapsed = current_time - start_time
                remaining = args.duration - elapsed
                print(f"Ping progress: {ping_count} pings sent, {successful_pings} successful, {failed_pings} failed, {remaining:.1f}s remaining")
                last_progress_time = current_time
            
            ping_count += 1
            random_10_byte_string = generate_random_10_byte_string()
            
            try:
                response = motor.ping(random_10_byte_string)
                # Make sure the response is the same as the random 10 byte string
                if response == random_10_byte_string:
                    successful_pings += 1
                else:
                    failed_pings += 1
                    if args.verbose:
                        print(f"Ping {ping_count} failed - response mismatch")
                        print("Sent:", random_10_byte_string)
                        print("Received:", response)
            except servomotor.communication.TimeoutError:
                failed_pings += 1
                if args.verbose:
                    print(f"Ping {ping_count} failed with a timeout")
            except Exception as e:
                failed_pings += 1
                if args.verbose:
                    print(f"Ping {ping_count} failed with exception: {e}")
            
            # Wait for next ping if frequency is specified
            if ping_delay > 0:
                time.sleep(ping_delay)

    except KeyboardInterrupt:
        print(f"\nTest interrupted by user")

    # Calculate final statistics
    total_time = time.time() - start_time
    actual_ping_rate = ping_count / total_time if total_time > 0 else 0

    # Print statistics
    print(f"\nPing Test Statistics:")
    print(f"  Total pings sent: {ping_count}")
    print(f"  Successful pings: {successful_pings}")
    print(f"  Failed pings: {failed_pings}")
    print(f"  Total time: {total_time:.2f} seconds")
    print(f"  Actual ping rate: {actual_ping_rate:.2f} pings/second")

    servomotor.close_serial_port()
    del motor

    # Test passes only if all pings succeeded
    if failed_pings == 0 and successful_pings > 0:
        print("PASSED")
        return 0
    else:
        print("FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
