#!/usr/bin/env python3

import servomotor
import time
import math
from servomotor.communication import TimeoutError
import argparse
import sys

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while True:
        queue_size = motor.get_n_queued_items()
        if queue_size == 0:
            break
        time.sleep(0.01)  # Reduced sleep time for more accurate timing

def verify_position(motor, expected_position, tolerance=10):
    """Verify motor position is within tolerance of expected position using multiple units"""
    # Get position in different units
    motor.set_position_unit("encoder_counts")
    counts_reading = motor.get_position()
    
    motor.set_position_unit("shaft_rotations")
    rotations_reading = motor.get_position()
    
    motor.set_position_unit("degrees")
    degrees_reading = motor.get_position()
    
    motor.set_position_unit("radians")
    radians_reading = motor.get_position()
    
    # Expected values in each unit
    expected_rotations = expected_position / 3276800  # 1 rotation = 3,276,800 counts
    expected_degrees = (expected_position / 3276800) * 360  # Convert to rotations then to degrees
    expected_radians = (expected_position / 3276800) * (2 * math.pi)  # Convert to rotations then to radians
    
    # Print all readings for comparison
    print(f"Expected values:")
    print(f"  encoder_counts:  {expected_position}")
    print(f"  shaft_rotations: {expected_rotations}")
    print(f"  degrees:         {expected_degrees}")
    print(f"  radians:         {expected_radians}")
    print(f"Actual readings:")
    print(f"  encoder_counts:  {counts_reading}")
    print(f"  shaft_rotations: {rotations_reading}")
    print(f"  degrees:         {degrees_reading}")
    print(f"  radians:         {radians_reading}")
    
    # Calculate differences in encoder counts
    diff_counts = abs(counts_reading - expected_position)
    diff_rotations = abs(rotations_reading - expected_rotations)
    diff_degrees = abs(degrees_reading - expected_degrees)
    diff_radians = abs(radians_reading - expected_radians)
    
    # Convert differences to counts for consistent comparison
    diff_rotations_counts = diff_rotations * 3276800
    diff_degrees_counts = (diff_degrees / 360) * 3276800
    diff_radians_counts = (diff_radians / (2 * math.pi)) * 3276800
    
    print(f"Differences (in encoder counts):")
    print(f"  From encoder_counts:  {diff_counts}")
    print(f"  From shaft_rotations: {diff_rotations_counts}")
    print(f"  From degrees:         {diff_degrees_counts}")
    print(f"  From radians:         {diff_radians_counts}")
    
    max_diff = max(diff_counts, diff_rotations_counts, diff_degrees_counts, diff_radians_counts)
    print(f"Maximum difference (counts): {max_diff}")
    
    assert max_diff <= tolerance, f"Position error ({max_diff} counts) exceeds tolerance ({tolerance} counts)"

def main(args):
    test_passed = True
    motorX = None
    try:
        servomotor.set_serial_port_from_args(args)
        servomotor.open_serial_port()

        # Initialize motor with encoder counts as default position unit for precise verification
        motorX = servomotor.M3(args.alias,
                               time_unit="seconds",
                               position_unit="encoder_counts",
                               velocity_unit="rpm",
                               acceleration_unit="degrees_per_second_squared",
                               current_unit="amps",
                               voltage_unit="volts",
                               temperature_unit="celsius",
                               verbose=0)  # Set verbose=0 to reduce output

        print("\nResetting motor...")
        motorX.system_reset()
        time.sleep(1.5)  # Required delay after reset
        
        print("\nEnabling MOSFETs...")
        motorX.enable_mosfets()

        # Test position in shaft rotations (1 rotation)
        print("\nTesting position in shaft rotations...")
        print("Moving to position 1 rotation...")
        start_time = time.time()
        motorX.set_position_unit("shaft_rotations")
        motorX.go_to_position(1, duration=2)  # Move to 1 rotation in 2 seconds
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Move completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 3276800)  # 1 rotation = 3,276,800 counts
        
        print("\nReturning to zero...")
        start_time = time.time()
        motorX.go_to_position(0, duration=0.25)  # Return to zero quickly
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Return completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 0)

        # Test position in degrees (360 degrees = 1 rotation)
        print("\nTesting position in degrees...")
        print("Moving to position 360 degrees...")
        start_time = time.time()
        motorX.set_position_unit("degrees")
        motorX.go_to_position(360, duration=2)  # Move to 360 degrees in 2 seconds
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Move completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 3276800)  # 360 degrees = 3,276,800 counts
        
        print("\nReturning to zero...")
        start_time = time.time()
        motorX.go_to_position(0, duration=0.25)  # Return to zero quickly
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Return completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 0)

        # Test position in radians (2π radians = 1 rotation)
        print("\nTesting position in radians...")
        print("Moving to position 2π radians...")
        start_time = time.time()
        motorX.set_position_unit("radians")
        motorX.go_to_position(2 * math.pi, duration=2)  # Move to 2π radians in 2 seconds
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Move completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 3276800)  # 2π radians = 3,276,800 counts
        
        print("\nReturning to zero...")
        start_time = time.time()
        motorX.go_to_position(0, duration=0.25)  # Return to zero quickly
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Return completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 0)

        # Test position in encoder counts
        print("\nTesting position in encoder counts...")
        print("Moving to position 3276800 counts (1 rotation)...")
        start_time = time.time()
        motorX.set_position_unit("encoder_counts")
        motorX.go_to_position(3276800, duration=2)  # Move to 3276800 counts in 2 seconds
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Move completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 3276800)
        
        print("\nReturning to zero...")
        start_time = time.time()
        motorX.go_to_position(0, duration=0.25)  # Return to zero quickly
        wait_for_moves_to_complete(motorX)
        elapsed_time = time.time() - start_time
        print(f"Return completed in {elapsed_time:.3f} seconds")
        verify_position(motorX, 0)

        print("\nTest complete!")

    except Exception as e:
        print(f"\nTest failed: {e}", file=sys.stderr)
        test_passed = False
    finally:
        if motorX:
            # Attempt to disable mosfets, but don't let it crash the script
            try:
                motorX.disable_mosfets()
            except Exception as e:
                print(f"Failed to disable mosfets: {e}", file=sys.stderr)
        servomotor.close_serial_port()
        print("Closed the serial port")

    if test_passed:
        print("\nPASSED")
        sys.exit(0)
    else:
        print("\nFAILED")
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test go_to_position command.')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', help='Alias of the device to control', default='X')
    args = parser.parse_args()
    main(args)
