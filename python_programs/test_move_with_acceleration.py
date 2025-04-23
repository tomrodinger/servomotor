#!/usr/bin/env python3

import servomotor
import time
import math
import argparse # Import argparse

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while True:
        queue_size = motor.get_n_queued_items()
        if queue_size == 0:
            break
        time.sleep(0.01)

def verify_timing(elapsed_time, expected_time, tolerance=0.1):
    """Verify elapsed time is within tolerance of expected time"""
    diff = abs(elapsed_time - expected_time)
    print(f"Expected time (seconds): {expected_time}")
    print(f"Actual time (seconds):   {elapsed_time:.3f}")
    print(f"Difference (seconds):    {diff:.3f}")
    assert diff <= tolerance, f"Timing error ({diff:.3f} seconds) exceeds tolerance ({tolerance} seconds)"

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

def test_acceleration_units(motorX): # Accept motor object as argument
    """Test acceleration in different units. Assumes motor object is initialized and port is open."""
    print("\nResetting motor...")
    motorX.system_reset()
    time.sleep(1.5)  # Wait 1.5 seconds for reset to complete

    print("\nEnabling MOSFETs...")
    motorX.enable_mosfets()

    # Test acceleration in rotations/second²
    print("\nTesting acceleration in rotations/second²...")
    motorX.set_acceleration_unit('rotations_per_second_squared')
    start_time = time.time()
    # Queue all three commands: accelerate, coast, decelerate
    motorX.move_with_acceleration(1.0, 1)    # Accelerate to 1 rotation/sec over 1s
    motorX.move_with_acceleration(0, 3)      # Maintain velocity for 3s
    motorX.move_with_acceleration(-1.0, 1)   # Decelerate to stop over 1s
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    verify_timing(elapsed_time, 5.0)  # Total time = 1s + 3s + 1s
    verify_position(motorX, 13107200)  # Final position = 4 rotations

    # Test acceleration in RPM/second
    print("\nTesting acceleration in RPM/second...")
    motorX.set_acceleration_unit('rpm_per_second')
    start_time = time.time()
    # Queue all three commands: accelerate, coast, decelerate
    motorX.move_with_acceleration(60, 1)     # Accelerate to 60 RPM over 1s
    motorX.move_with_acceleration(0, 3)      # Maintain velocity for 3s
    motorX.move_with_acceleration(-60, 1)    # Decelerate to stop over 1s
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    verify_timing(elapsed_time, 5.0)  # Total time = 1s + 3s + 1s
    verify_position(motorX, 26214400)  # Final position = 8 rotations

    # Test acceleration in degrees/second²
    print("\nTesting acceleration in degrees/second²...")
    motorX.set_acceleration_unit('degrees_per_second_squared')
    start_time = time.time()
    # Queue all three commands: accelerate, coast, decelerate
    motorX.move_with_acceleration(360, 1)    # Accelerate to 360 deg/s over 1s
    motorX.move_with_acceleration(0, 3)      # Maintain velocity for 3s
    motorX.move_with_acceleration(-360, 1)   # Decelerate to stop over 1s
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    verify_timing(elapsed_time, 5.0)  # Total time = 1s + 3s + 1s
    verify_position(motorX, 39321600)  # Final position = 12 rotations

    # Test acceleration in radians/second²
    print("\nTesting acceleration in radians/second²...")
    motorX.set_acceleration_unit('radians_per_second_squared')
    start_time = time.time()
    # Queue all three commands: accelerate, coast, decelerate
    motorX.move_with_acceleration(2*math.pi, 1)  # Accelerate to 2π rad/s over 1s
    motorX.move_with_acceleration(0, 3)          # Maintain velocity for 3s
    motorX.move_with_acceleration(-2*math.pi, 1) # Decelerate to stop over 1s
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    verify_timing(elapsed_time, 5.0)  # Total time = 1s + 3s + 1s
    verify_position(motorX, 52428800)  # Final position = 16 rotations

    # Test acceleration in encoder counts/second²
    print("\nTesting acceleration in encoder counts/second²...")
    motorX.set_acceleration_unit('counts_per_second_squared')
    start_time = time.time()
    # Queue all three commands: accelerate, coast, decelerate
    motorX.move_with_acceleration(3276800, 1)    # Accelerate to 3,276,800 counts/s over 1s
    motorX.move_with_acceleration(0, 3)          # Maintain velocity for 3s
    motorX.move_with_acceleration(-3276800, 1)   # Decelerate to stop over 1s
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    verify_timing(elapsed_time, 5.0)  # Total time = 1s + 3s + 1s
    verify_position(motorX, 65536000)  # Final position = 20 rotations

    print("\nTest complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test servomotor move_with_acceleration.')
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
                              time_unit='seconds',
                              position_unit='encoder_counts',
                              velocity_unit='counts_per_second',
                              acceleration_unit='counts_per_second_squared',
                              verbose=0)
        test_acceleration_units(motorX)
        print("\nPASSED")
    finally:
        servomotor.close_serial_port()
