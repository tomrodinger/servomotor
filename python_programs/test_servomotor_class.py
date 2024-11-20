#!/usr/bin/env python3

import servomotor
import time
from servomotor.communication import TimeoutError

def test_different_position_units():
    # Initialize motor with degrees as default unit
    motorX = servomotor.M3("X", motor_type="M3", 
                          time_unit="seconds", 
                          position_unit="degrees", 
                          velocity_unit="degrees_per_second", 
                          acceleration_unit="degrees_per_second_squared", 
                          current_unit="amps", 
                          voltage_unit="volts", 
                          temperature_unit="celsius", 
                          verbose=2)

    try:
        servomotor.open_serial_port()
        motorX.system_reset()
        time.sleep(1.2)
        motorX.enable_mosfets()

        # Test movement in degrees
        print("\nTesting movement in degrees...")
        motorX.go_to_position(position=90, duration=2)  # Move 90 degrees in 2 seconds
        time.sleep(2)

        # Change to shaft rotations and test
        print("\nChanging to shaft rotations and testing...")
        motorX.set_position_unit("shaft_rotations")
        motorX.go_to_position(position=0.25, duration=2)  # Move 1/4 rotation (90 degrees)
        time.sleep(2)

        # Change to radians and test
        print("\nChanging to radians and testing...")
        motorX.set_position_unit("radians")
        motorX.go_to_position(position=3.14159/2, duration=2)  # Move 90 degrees (π/2 radians)
        time.sleep(2)

        # Change to encoder counts and test
        print("\nChanging to encoder counts and testing...")
        motorX.set_position_unit("encoder_counts")
        motorX.go_to_position(position=819200, duration=2)  # Move 90 degrees worth of counts
        time.sleep(2)

        # Test different time units
        print("\nTesting different time units...")
        motorX.set_position_unit("degrees")  # Back to degrees
        motorX.set_time_unit("milliseconds")
        motorX.go_to_position(position=180, duration=2000)  # Move 180 degrees in 2000 milliseconds
        time.sleep(3)

        # Get and display position in different units
        print("\nGetting current position in different units...")
        motorX.set_position_unit("degrees")
        pos_deg = motorX.get_position()
        print(f"Position in degrees: {pos_deg}")

        motorX.set_position_unit("shaft_rotations")
        pos_rot = motorX.get_position()
        print(f"Position in rotations: {pos_rot}")

        motorX.set_position_unit("radians")
        pos_rad = motorX.get_position()
        print(f"Position in radians: {pos_rad}")

        # Get temperature in different units
        print("\nGetting temperature in different units...")
        motorX.set_temperature_unit("celsius")
        temp_c = motorX.get_temperature()
        print(f"Temperature in Celsius: {temp_c}")

        motorX.set_temperature_unit("fahrenheit")
        temp_f = motorX.get_temperature()
        print(f"Temperature in Fahrenheit: {temp_f}")

    except Exception as e:
        print(f"\nTest failed: {e}")
        raise
    finally:
        servomotor.close_serial_port()
        del motorX

if __name__ == "__main__":
    test_different_position_units()
