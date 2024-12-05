#!/usr/bin/env python3

import servomotor
import time
import math
from servomotor.communication import TimeoutError

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while True:
        queue_size = motor.get_n_queued_items()
        if queue_size == 0:
            break
        time.sleep(0.01)  # Reduced sleep time for more accurate timing

def verify_position(motor, expected_position, tolerance=10):
    """Verify motor position is within tolerance of expected position"""
    # Get raw position in encoder counts
    motor.set_position_unit("encoder_counts")
    actual_position = motor.get_position()
    diff = abs(actual_position - expected_position)
    print(f"Expected position (counts): {expected_position}")
    print(f"Actual position (counts):   {actual_position}")
    print(f"Difference (counts):        {diff}")
    assert diff <= tolerance, f"Position error ({diff} counts) exceeds tolerance ({tolerance} counts)"

def test_position_units():
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
        print(f"\nTest failed: {e}")
        raise
    finally:
        servomotor.close_serial_port()
        del motorX

if __name__ == "__main__":
    test_position_units()
    print("\nPASSED")
