#!/usr/bin/env python3

import servomotor
import time
import math
import random
import string
import argparse
import sys
from servomotor import communication

# Constants
MAX_MOTOR_CURRENT = 200  # Maximum MOSFET current in milliamps

def generate_random_string(length=10):
    """Generate a random string of fixed length"""
    return ''.join(random.choices(string.ascii_letters + string.digits, k=length))

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while True:
        queue_size = motor.get_n_queued_items()
        if queue_size == 0:
            break
        time.sleep(0.01)

def verify_timing(elapsed_time, expected_time, tolerance=0.5):
    """Verify elapsed time is within tolerance of expected time"""
    diff = abs(elapsed_time - expected_time)
    print(f"Expected time (seconds): {expected_time}")
    print(f"Actual time (seconds):   {elapsed_time:.3f}")
    print(f"Difference (seconds):    {diff:.3f}")
    assert diff <= tolerance, f"Timing error ({diff:.3f} seconds) exceeds tolerance ({tolerance} seconds)"

def print_statistics(elapsed_time, ping_count, failed_pings, queued_moves):
    """Print current test statistics"""
    success_rate = ((ping_count - failed_pings) / ping_count * 100) if ping_count > 0 else 0
    print(f"Time: {elapsed_time:.1f}s | "
          f"Pings: {ping_count} | "
          f"Failed: {failed_pings} | "
          f"Success Rate: {success_rate:.2f}% | "
          f"Queued Moves: {queued_moves}")

def test_communication_while_high_speed(motor_alias):
    # Initialize motor with RPM as velocity unit for easier speed control
    motorX = servomotor.M3(motor_alias,
                          time_unit="seconds",
                          position_unit="encoder_counts",
                          velocity_unit="rpm",
                          acceleration_unit="rpm_per_second",
                          current_unit="milliamps",
                          voltage_unit="volts",
                          temperature_unit="celsius",
                          verbose=0)

    try:
        servomotor.open_serial_port()
        print("\nResetting motor...")
        motorX.system_reset()
        time.sleep(1.5)  # Required delay after reset
        
        print("\nEnabling MOSFETs...")
        motorX.enable_mosfets()
        time.sleep(0.5)

        print("\nSetting maximum motor current...")
        motorX.set_maximum_motor_current(MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT)

        print("\nQueuing three moves:")
        print("1. Accelerate to 580 RPM over 30 seconds")
        print("2. Coast at 580 RPM for 10 seconds")
        print("3. Decelerate to 0 RPM over 30 seconds")

        # Calculate acceleration rate
        accel_rate = 580 / 30  # RPM per second to reach 580 RPM in 30 seconds
        decel_rate = -580 / 30  # RPM per second to stop from 580 RPM in 30 seconds

        # Queue the three moves
        motorX.move_with_acceleration(accel_rate, 30)    # Accelerate
        motorX.move_with_acceleration(0, 10)             # Coast
        motorX.move_with_acceleration(decel_rate, 30)    # Decelerate

        print("\nStarting continuous ping test while moves execute...")
        print("\nTest Statistics (updated every second):")
        start_time = time.time()
        last_stats_time = start_time
        ping_count = 0
        failed_pings = 0

        # Monitor moves and ping continuously
        while motorX.get_n_queued_items() > 0:
            try:
                # Generate random 10-char string and ping
                test_string = generate_random_string(10)
                response = motorX.ping(test_string)
                
                # Decode bytes response before comparing
                if isinstance(response, bytes):
                    response = response.decode('utf-8')
                
                # Verify response matches sent string
                if response != test_string:
                    print(f"Ping mismatch: sent '{test_string}', received '{response}'")
                    failed_pings += 1
                ping_count += 1

                # Print statistics every second
                current_time = time.time()
                if current_time - last_stats_time >= 1.0:
                    elapsed_time = current_time - start_time
                    queued_moves = motorX.get_n_queued_items()
                    print_statistics(elapsed_time, ping_count, failed_pings, queued_moves)
                    last_stats_time = current_time

            except Exception as e:
                print(f"Ping failed: {e}")
                failed_pings += 1
            time.sleep(0.01)  # Small delay between pings

        elapsed_time = time.time() - start_time
        expected_time = 70  # 30s accelerate + 10s coast + 30s decelerate

        print(f"\nTest completed!")
        print(f"Total pings sent: {ping_count}")
        print(f"Failed pings: {failed_pings}")
        print(f"Ping success rate: {((ping_count - failed_pings) / ping_count) * 100:.2f}%")

        # Verify timing
        verify_timing(elapsed_time, expected_time)

        # Verify all pings succeeded
        assert failed_pings == 0, f"Communication test failed: {failed_pings} pings failed"

    except communication.TimeoutError as e:
        print(f"\nTest failed: {e}", file=sys.stderr)
        test_passed = False
    except Exception as e:
        print(f"\nTest failed: {e}", file=sys.stderr)
        test_passed = False
    finally:
        print("\nDisabling MOSFETs...")
        try:
            motorX.disable_mosfets()
        except communication.TimeoutError:
            print("Timeout while disabling motor. Motor is likely unresponsive.", file=sys.stderr)
        servomotor.close_serial_port()
        del motorX

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test communication while motor runs at high speed.")
    parser.add_argument('-p', '--port', type=str, help='Serial port device name (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', type=str, default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    
    # Set up serial port
    servomotor.set_serial_port_from_args(args)
    
    test_passed = True
    try:
        test_communication_while_high_speed(args.alias)
    except Exception:
        test_passed = False

    if test_passed:
        print("\nPASSED")
        sys.exit(0)
    else:
        print("\nFAILED")
        sys.exit(1)
