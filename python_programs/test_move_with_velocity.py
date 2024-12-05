#!/usr/bin/env python3

import servomotor
import time
import math
from servomotor.communication import TimeoutError

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while motor.get_n_queued_items() > 0:
        time.sleep(0.01)  # Reduced sleep time for more accurate timing

def verify_position(motor, expected_position, tolerance=100):
    """Verify motor position is within tolerance of expected position"""
    actual_position = motor.get_position()
    diff = abs(actual_position - expected_position)
    print(f"Expected position (counts): {expected_position}")
    print(f"Actual position (counts):   {actual_position}")
    print(f"Difference (counts):        {diff}")
    assert diff <= tolerance, f"Position error ({diff} counts) exceeds tolerance ({tolerance} counts)"

def verify_timing(elapsed_time, expected_time, tolerance=0.1):
    """Verify elapsed time is within tolerance of expected time"""
    diff = abs(elapsed_time - expected_time)
    print(f"Expected time (seconds): {expected_time}")
    print(f"Actual time (seconds):   {elapsed_time:.3f}")
    print(f"Difference (seconds):    {diff:.3f}")
    assert diff <= tolerance, f"Timing error ({diff:.3f} seconds) exceeds tolerance ({tolerance} seconds)"

def test_velocity_units():
    # Initialize motor with encoder counts as default position unit for precise verification
    motorX = servomotor.M3("X",
                          time_unit="seconds", 
                          position_unit="encoder_counts", 
                          velocity_unit="rpm", 
                          acceleration_unit="degrees_per_second_squared", 
                          current_unit="amps", 
                          voltage_unit="volts", 
                          temperature_unit="celsius", 
                          verbose=0)  # Set verbose=0 to reduce output

    try:
        servomotor.open_serial_port()
        print("\nResetting motor...")
        motorX.system_reset()
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

    except Exception as e:
        print(f"\nTest failed: {e}")
        raise
    finally:
        servomotor.close_serial_port()
        del motorX

if __name__ == "__main__":
    test_velocity_units()
    print("\nPASSED")
