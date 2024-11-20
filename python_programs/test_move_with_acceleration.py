#!/usr/bin/env python3

import servomotor
import time
import math

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
    """Verify motor position is within tolerance of expected position"""
    actual_position = motor.get_position()
    diff = abs(actual_position - expected_position)
    print(f"Expected position (counts): {expected_position}")
    print(f"Actual position (counts):   {actual_position}")
    print(f"Difference (counts):        {diff}")
    assert diff <= tolerance, f"Position error ({diff} counts) exceeds tolerance ({tolerance} counts)"

def test_acceleration_units():
    """Test acceleration in different units"""
    motorX = servomotor.M3(
        alias='X',
        motor_type='M3',
        time_unit='seconds',
        position_unit='encoder_counts',
        velocity_unit='counts_per_second',
        acceleration_unit='counts_per_second_squared',
        current_unit='amps',
        voltage_unit='volts',
        temperature_unit='celsius',
        verbose=0
    )

    try:
        servomotor.open_serial_port()

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

    except Exception as e:
        print(f"\nTest failed: {e}")
        raise
    finally:
        servomotor.close_serial_port()
        del motorX

if __name__ == "__main__":
    test_acceleration_units()
    print("\nPASSED")
