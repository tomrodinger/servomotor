#!/usr/bin/env python3

import servomotor
import time
import math
import argparse # Import argparse
from servomotor.communication import TimeoutError

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while motor.get_n_queued_items() > 0:
        time.sleep(0.01)  # Reduced sleep time for more accurate timing

def verify_position(motor, expected_position, tolerance=100):
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

def verify_timing(elapsed_time, expected_time, tolerance=0.1):
    """Verify elapsed time is within tolerance of expected time"""
    diff = abs(elapsed_time - expected_time)
    print(f"Expected time (seconds): {expected_time}")
    print(f"Actual time (seconds):   {elapsed_time:.3f}")
    print(f"Difference (seconds):    {diff:.3f}")
    assert diff <= tolerance, f"Timing error ({diff:.3f} seconds) exceeds tolerance ({tolerance} seconds)"

def test_velocity_units(motorX): # Accept motor object as argument
    """Test velocity in different units. Assumes motor object is initialized and port is open."""
    print("\nResetting motor...")
    motorX.system_reset() # Use the passed motor object
    time.sleep(1.5)  # Required delay after reset

    print("\nEnabling MOSFETs...")
    motorX.enable_mosfets()
    time.sleep(0.5)

    # Test velocity in rotations per second
    print("\nTesting velocity in rotations per second...")
    motorX.set_velocity_unit("rotations_per_second")
    print("Moving at 1 rotation per second for 2 seconds...")
    start_time = time.time()
    motorX.move_with_velocity(velocity=1, duration=2)  # 2 seconds at 1 rotation/s
    motorX.move_with_velocity(velocity=0, duration=0.001)  # Stop with minimal duration
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Move completed in {elapsed_time:.3f} seconds")
    verify_timing(elapsed_time, 2.001)  # Expected time: 2s velocity + 0.001s stop
    # At 1 rotation per second for 2 seconds, we expect 2 rotations
    expected_position = 2 * 3276800  # 2 rotations * counts per rotation
    verify_position(motorX, expected_position)

    # Test velocity in RPM (1 rotation per second = 60 RPM)
    print("\nTesting velocity in RPM...")
    motorX.set_velocity_unit("rpm")
    print("Moving at 60 RPM (1 rotation per second) for 2 seconds...")
    start_time = time.time()
    motorX.move_with_velocity(velocity=60, duration=2)  # 2 seconds at 60 RPM
    motorX.move_with_velocity(velocity=0, duration=0.001)  # Stop with minimal duration
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Move completed in {elapsed_time:.3f} seconds")
    verify_timing(elapsed_time, 2.001)  # Expected time: 2s velocity + 0.001s stop
    # At 60 RPM (1 rotation per second) for 2 seconds, we expect 2 more rotations
    expected_position += 2 * 3276800  # Add 2 more rotations
    verify_position(motorX, expected_position)

    # Test velocity in degrees per second (360 deg/s = 1 rotation per second)
    print("\nTesting velocity in degrees per second...")
    motorX.set_velocity_unit("degrees_per_second")
    print("Moving at 360 degrees/second (1 rotation per second) for 2 seconds...")
    start_time = time.time()
    motorX.move_with_velocity(velocity=360, duration=2)  # 2 seconds at 360 deg/s
    motorX.move_with_velocity(velocity=0, duration=0.001)  # Stop with minimal duration
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Move completed in {elapsed_time:.3f} seconds")
    verify_timing(elapsed_time, 2.001)  # Expected time: 2s velocity + 0.001s stop
    # At 360 deg/s (1 rotation per second) for 2 seconds, we expect 2 more rotations
    expected_position += 2 * 3276800  # Add 2 more rotations
    verify_position(motorX, expected_position)

    # Test velocity in radians per second (2π rad/s = 1 rotation per second)
    print("\nTesting velocity in radians per second...")
    motorX.set_velocity_unit("radians_per_second")
    print("Moving at 2π radians/second (1 rotation per second) for 2 seconds...")
    start_time = time.time()
    motorX.move_with_velocity(velocity=2 * math.pi, duration=2)  # 2 seconds at 2π rad/s
    motorX.move_with_velocity(velocity=0, duration=0.001)  # Stop with minimal duration
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Move completed in {elapsed_time:.3f} seconds")
    verify_timing(elapsed_time, 2.001)  # Expected time: 2s velocity + 0.001s stop
    # At 2π rad/s (1 rotation per second) for 2 seconds, we expect 2 more rotations
    expected_position += 2 * 3276800  # Add 2 more rotations
    verify_position(motorX, expected_position)

    # Test velocity in encoder counts per second
    print("\nTesting velocity in encoder counts per second...")
    motorX.set_velocity_unit("counts_per_second")
    print("Moving at 3276800 counts/second (1 rotation per second) for 2 seconds...")
    start_time = time.time()
    motorX.move_with_velocity(velocity=3276800, duration=2)  # 2 seconds at 1 rotation/s
    motorX.move_with_velocity(velocity=0, duration=0.001)  # Stop with minimal duration
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Move completed in {elapsed_time:.3f} seconds")
    verify_timing(elapsed_time, 2.001)  # Expected time: 2s velocity + 0.001s stop
    # At 3276800 counts/s (1 rotation per second) for 2 seconds, we expect 2 more rotations
    expected_position += 2 * 3276800  # Add 2 more rotations
    verify_position(motorX, expected_position)

    print("\nTest complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test servomotor move_with_velocity.')
    parser.add_argument('-p', '--port', help='Serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='Show available ports and prompt for selection', action="store_true")
    parser.add_argument('-a', '--alias', help='Alias of the device to control', required=True)
    args = parser.parse_args()

    # Set port before opening
    servomotor.set_serial_port_from_args(args)

    motorX = None
    try:
        servomotor.open_serial_port()
        motorX = servomotor.M3(args.alias,
                              time_unit="seconds",
                              position_unit="encoder_counts", # Start with counts for verification
                              velocity_unit="rpm", # Default velocity unit
                              acceleration_unit="degrees_per_second_squared",
                              verbose=0)
        test_velocity_units(motorX)
        print("\nPASSED")
    finally:
        servomotor.close_serial_port()
