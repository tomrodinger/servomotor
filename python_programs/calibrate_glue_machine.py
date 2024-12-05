#!/usr/bin/env python3

import argparse
import time
import json
import os
import servomotor

# Motor aliases
X_ALIAS = ord('X')
Y_ALIAS = ord('Y')
Z_ALIAS = ord('Z')
ALL_MOTORS_ALIAS = 255

# Motor settings (copied from glue_machine.py for consistency)
X_Y_MOTOR_CURRENT_SETTING = 50
Z_MOTOR_CURRENT_SETTING = 100
X_Y_REDUCTION_RATIO = 28 / 14
Z_REDUCTION_RATIO = 18.0 / 14
PULLEY_N_TEETH = 14
BELT_TOOTH_PITCH = 2.0
MM_PER_PULLEY_ROTATION = BELT_TOOTH_PITCH * PULLEY_N_TEETH / X_Y_REDUCTION_RATIO
X_Y_SPAN_MM = 227.0
X_Y_SPAN_ROTATIONS = X_Y_SPAN_MM / MM_PER_PULLEY_ROTATION * X_Y_REDUCTION_RATIO
Z_SPAN_MM = 120.0
Z_LEAD_SCREW_MM_PER_ROTATION = 4.0 / Z_REDUCTION_RATIO
Z_SPAN_ROTATIONS = Z_SPAN_MM / Z_LEAD_SCREW_MM_PER_ROTATION
HOMING_MAX_TIME_S = 10.0
REQUIRED_SUCCESSFUL_PINGS = 30

# Calibration settings
CALIBRATION_FILE = "glue_machine_calibration.json"
MOVE_TIME = 5.0  # Time to move to positions
MOSFET_ENABLE_DELAY = 1.0  # Delay between enabling Z and X/Y MOSFETs
Z_SAFETY_OFFSET_MM = 10.0  # Distance to subtract from Z position during movements

# Default positions if no calibration file exists
DEFAULT_POSITIONS = {
    "corner1": {"x": 50.0, "y": 50.0, "z": 90.0},
    "corner2": {"x": 200.0, "y": 200.0, "z": 90.0},
    "corner3": {"x": 50.0, "y": 200.0, "z": 90.0},
    "corner4": {"x": 200.0, "y": 50.0, "z": 90.0}
}

def load_calibration_data():
    """Load calibration data from file or return defaults if file doesn't exist."""
    try:
        if os.path.exists(CALIBRATION_FILE):
            with open(CALIBRATION_FILE, 'r') as f:
                data = json.load(f)
                print(f"Loaded existing calibration data from {CALIBRATION_FILE}")
                return data
    except Exception as e:
        print(f"Error reading calibration file: {e}")
    
    print("Using default calibration positions")
    return DEFAULT_POSITIONS

def check_device_with_ping(motor, required_successful_pings):
    """Check device communication with multiple pings."""
    import random
    for ping_number in range(required_successful_pings):
        try:
            random_10_bytes = bytes([random.randint(0, 255) for _ in range(10)])
            response = motor.ping(random_10_bytes)
            if response == random_10_bytes:
                print(f"Ping {ping_number + 1} successful")
            else:
                print(f"Error: Ping {ping_number + 1} failed")
                exit(1)
        except Exception as e:
            print(f"Communication error: {e}")
            exit(1)
    print(f"Successfully pinged device with alias {motor.alias} {required_successful_pings} times")

def do_homing(motor, motor_current_setting, max_displacement, max_time):
    """Perform homing sequence for a motor."""
    human_readable_alias = servomotor.get_human_readable_alias(motor.alias)
    print(f"Setting current of motor {human_readable_alias} to {motor_current_setting}")
    response = motor.set_maximum_motor_current(motor_current_setting, motor_current_setting)
    print("Going into closed loop mode")
    response = motor.go_to_closed_loop()
    print(f"Homing motor {human_readable_alias}")
    response = motor.homing(max_displacement, max_time)
    
    while True:
        response = motor.get_status()
        homing = (response[0] & 0b00010000) != 0
        if not homing:
            break
        time.sleep(0.2)
    print("Motor finished homing")

def get_position_mm(motor, is_z_axis=False):
    """Get motor position in millimeters using hall sensor position."""
    response = motor.get_hall_sensor_position()
    if is_z_axis:
        return response * Z_LEAD_SCREW_MM_PER_ROTATION
    return response * MM_PER_PULLEY_ROTATION

def move_to_position_mm(motor, position_mm, move_time, is_z_axis=False):
    """Move motor to position in millimeters."""
    if is_z_axis:
        # Subtract safety offset from Z position during movement
        position_mm -= Z_SAFETY_OFFSET_MM
        position_rotations = position_mm / Z_LEAD_SCREW_MM_PER_ROTATION
    else:
        position_rotations = position_mm / MM_PER_PULLEY_ROTATION
    response = motor.go_to_position(position_rotations, move_time)
    return response

def wait_for_move_complete(motors):
    """Wait for all motors to complete their moves."""
    while True:
        all_complete = True
        for motor in motors:
            if motor.get_n_queued_items() > 0:
                all_complete = False
                break
        if all_complete:
            break
        time.sleep(0.1)

def capture_position(motorX, motorY, motorZ):
    """Capture current position of all axes."""
    # Capture positions while motors are still disabled
    x_pos = get_position_mm(motorX)
    y_pos = get_position_mm(motorY)
    z_pos = get_position_mm(motorZ, True)
    
    print(f"Captured position - X: {x_pos:.2f}mm, Y: {y_pos:.2f}mm, Z: {z_pos:.2f}mm")
    
    # Now re-enable motors in sequence to prevent collisions
    print("Re-enabling Z axis...")
    motorZ.enable_mosfets()
    motorZ.go_to_closed_loop()
    time.sleep(MOSFET_ENABLE_DELAY)  # Wait before enabling X/Y
    
    print("Re-enabling X and Y axes...")
    motorX.enable_mosfets()
    motorY.enable_mosfets()
    motorX.go_to_closed_loop()
    motorY.go_to_closed_loop()
    time.sleep(0.5)  # Wait for motors to stabilize
    
    return {"x": x_pos, "y": y_pos, "z": z_pos}

def calculate_opposite_corners(corner1, corner2):
    """Calculate the other two corners based on two known corners."""
    corner3 = {"x": corner1["x"], "y": corner2["y"], "z": corner1["z"]}
    corner4 = {"x": corner2["x"], "y": corner1["y"], "z": corner1["z"]}
    return corner3, corner4

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Calibrate glue machine positions')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports and select from menu', action="store_true")
    parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
    args = parser.parse_args()

    servomotor.set_serial_port_from_args(args)

    # Load existing calibration data or use defaults
    calibration_data = load_calibration_data()

    # Initialize motors
    motor255 = servomotor.M3(ALL_MOTORS_ALIAS, verbose=args.verbose)
    motorX = servomotor.M3(X_ALIAS, verbose=args.verbose)
    motorY = servomotor.M3(Y_ALIAS, verbose=args.verbose)
    motorZ = servomotor.M3(Z_ALIAS, verbose=args.verbose)
    servomotor.open_serial_port()

    try:
        # Reset all devices
        motor255.system_reset()
        time.sleep(1.5)

        # Check communication
        check_device_with_ping(motorX, REQUIRED_SUCCESSFUL_PINGS)
        check_device_with_ping(motorY, REQUIRED_SUCCESSFUL_PINGS)
        check_device_with_ping(motorZ, REQUIRED_SUCCESSFUL_PINGS)

        # Perform homing
        print("\nPerforming homing sequence...")
        do_homing(motorZ, Z_MOTOR_CURRENT_SETTING, -Z_SPAN_ROTATIONS + 1, HOMING_MAX_TIME_S)
        do_homing(motorX, X_Y_MOTOR_CURRENT_SETTING, -X_Y_SPAN_ROTATIONS + 0.25, HOMING_MAX_TIME_S)
        do_homing(motorY, X_Y_MOTOR_CURRENT_SETTING, -X_Y_SPAN_ROTATIONS + 0.25, HOMING_MAX_TIME_S)

        # Zero positions
        motorX.zero_position()
        motorY.zero_position()
        motorZ.zero_position()

        # Move to first corner position using calibration data
        print("\nMoving to first corner position...")
        move_to_position_mm(motorX, calibration_data["corner1"]["x"], MOVE_TIME)
        move_to_position_mm(motorY, calibration_data["corner1"]["y"], MOVE_TIME)
        move_to_position_mm(motorZ, calibration_data["corner1"]["z"], MOVE_TIME, True)
        wait_for_move_complete([motorX, motorY, motorZ])

        # Disable motors for manual adjustment
        print("\nDisabling motors for manual adjustment of first corner...")
        motorX.disable_mosfets()
        motorY.disable_mosfets()
        motorZ.disable_mosfets()
        input("Adjust position manually and press Enter when done...")

        # Capture first corner position
        corner1 = capture_position(motorX, motorY, motorZ)
        print(f"First corner position: {corner1}")

        # Move to second corner using calibration data
        print("\nMoving to second corner position...")
        move_to_position_mm(motorX, calibration_data["corner2"]["x"], MOVE_TIME)
        move_to_position_mm(motorY, calibration_data["corner2"]["y"], MOVE_TIME)
        wait_for_move_complete([motorX, motorY, motorZ])

        # Disable motors for manual adjustment
        print("\nDisabling motors for manual adjustment of second corner...")
        motorX.disable_mosfets()
        motorY.disable_mosfets()
        motorZ.disable_mosfets()
        input("Adjust position manually and press Enter when done...")

        # Capture second corner position
        corner2 = capture_position(motorX, motorY, motorZ)
        print(f"Second corner position: {corner2}")

        # Calculate other corners
        corner3, corner4 = calculate_opposite_corners(corner1, corner2)

        # Verify corner 3
        print("\nMoving to third corner for verification...")
        move_to_position_mm(motorX, corner3["x"], MOVE_TIME)
        move_to_position_mm(motorY, corner3["y"], MOVE_TIME)
        wait_for_move_complete([motorX, motorY, motorZ])

        print("\nDisabling motors for manual adjustment of third corner...")
        motorX.disable_mosfets()
        motorY.disable_mosfets()
        motorZ.disable_mosfets()
        input("Adjust position manually and press Enter when done...")
        corner3 = capture_position(motorX, motorY, motorZ)

        # Verify corner 4
        print("\nMoving to fourth corner for verification...")
        move_to_position_mm(motorX, corner4["x"], MOVE_TIME)
        move_to_position_mm(motorY, corner4["y"], MOVE_TIME)
        wait_for_move_complete([motorX, motorY, motorZ])

        print("\nDisabling motors for manual adjustment of fourth corner...")
        motorX.disable_mosfets()
        motorY.disable_mosfets()
        motorZ.disable_mosfets()
        input("Adjust position manually and press Enter when done...")
        corner4 = capture_position(motorX, motorY, motorZ)

        # Save calibration data
        calibration_data = {
            "corner1": corner1,
            "corner2": corner2,
            "corner3": corner3,
            "corner4": corner4
        }

        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(calibration_data, f, indent=4)
        print(f"\nCalibration data saved to {CALIBRATION_FILE}")

    finally:
        # Clean up
        servomotor.close_serial_port()
        del motorX
        del motorY
        del motorZ
        del motor255

if __name__ == "__main__":
    main()
