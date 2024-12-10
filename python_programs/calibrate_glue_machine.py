#!/usr/bin/env python3

import argparse
import time
import json
import os
import servomotor
import math

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
MOVE_TIME_COLLISION_AVOIDANCE = 3.0
MOSFET_ENABLE_DELAY = 0.1  # Delay between enabling Z and X/Y MOSFETs
INTERPOLATION_XY_DISTANCE_MM = 6.0  # Distance from corner points to the estimated zref points in the XY plane
COLLISION_AVOIDANCE_Z_OFFSET = 30.0  # Distance to move Z before moveing X and Y to avoid collisions during long "across the board" moves
Z_OFFSET_BEFORE_MANUAL_CAPTURE_MM = 10.0  # Distance to subtract from Z position during movements
ZREF_HEIGHT_OFFSET_MM = 8.0  # Additional Z height added to zref points to account for height difference between corner points and the zref plane 

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
                # Only use the first 4 points if file contains 8 points
                return {k: data[k] for k in ["corner1", "corner2", "corner3", "corner4"]}
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
    return {"x": x_pos, "y": y_pos, "z": z_pos}

def calculate_opposite_corners(corner1, corner2):
    """Calculate the other two corners based on two known corners."""
    corner3 = {"x": corner1["x"], "y": corner2["y"], "z": corner1["z"]}
    corner4 = {"x": corner2["x"], "y": corner1["y"], "z": corner1["z"]}
    return corner3, corner4

def calculate_center_point(corners):
    """Calculate the center point from the four corner points."""
    x_sum = sum(corner["x"] for corner in corners.values())
    y_sum = sum(corner["y"] for corner in corners.values())
    z_sum = sum(corner["z"] for corner in corners.values())
    return {
        "x": x_sum / 4,
        "y": y_sum / 4,
        "z": z_sum / 4
    }

def calculate_interpolated_point(center, corner):
    """Calculate a point between center and corner at specified distance from corner."""
    # Calculate vector from corner to center
    dx = corner["x"] - center["x"]
    dy = corner["y"] - center["y"]
    dz = corner["z"] - center["z"]
    
    # Calculate distance between points
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    
    # Calculate ratio for interpolation (12mm from corner)
    ratio = INTERPOLATION_XY_DISTANCE_MM / distance
    
    # Calculate interpolated point
    return {
        "x": corner["x"] + dx * ratio,
        "y": corner["y"] + dy * ratio,
        "z": corner["z"] + dz * ratio - ZREF_HEIGHT_OFFSET_MM
    }

def calculate_zref_points(corners):
    """Calculate the four zref points based on corners and center point."""
    center = calculate_center_point(corners)
    zref_points = {}
    
    for i, corner_key in enumerate(["corner1", "corner2", "corner3", "corner4"], 1):
        zref_points[f"zref{i}"] = calculate_interpolated_point(center, corners[corner_key])
    
    return zref_points

def move_to_a_corner_and_disable_mosfets_and_get_user_input_and_capture_position(position, motorX, motorY, motorZ, move_time):
    # Now re-enable motors in sequence to prevent collisions
    print("Enabling Z axis...")
    motorZ.enable_mosfets()
    time.sleep(MOSFET_ENABLE_DELAY)  # Wait before enabling X/Y
    move_to_position_mm(motorZ, position["z"] - COLLISION_AVOIDANCE_Z_OFFSET, MOVE_TIME_COLLISION_AVOIDANCE, True)
    wait_for_move_complete([motorZ])
    
    print("Enabling X and Y axes...")
    motorX.enable_mosfets()
    motorY.enable_mosfets()
    time.sleep(0.5)  # Wait for motors to stabilize

    move_to_position_mm(motorX, position["x"], MOVE_TIME)
    move_to_position_mm(motorY, position["y"], MOVE_TIME)
    wait_for_move_complete([motorX, motorY])

    move_to_position_mm(motorZ, position["z"] - Z_OFFSET_BEFORE_MANUAL_CAPTURE_MM, MOVE_TIME_COLLISION_AVOIDANCE, True)
    wait_for_move_complete([motorZ])

    # Disable motors for manual adjustment
    print("\nDisabling motors for manual adjustment of first corner...")
    motorX.disable_mosfets()
    motorY.disable_mosfets()
    motorZ.disable_mosfets()
    input("Adjust position manually and press Enter when done...")

    # Capture first corner position
    position = capture_position(motorX, motorY, motorZ)
    print(f"Captured position: {position}")
    return position


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

        motorX.set_max_allowable_position_deviation(20.0)
        motorY.set_max_allowable_position_deviation(20.0)
        motorZ.set_max_allowable_position_deviation(20.0)

        # Move to corners one at a time and capture positions
        captured_corners = []
        first_z_move = True
        for corner_key in ["corner1", "corner2", "corner3", "corner4"]:
            if corner_key == "corner3":
                calibration_data["corner3"], calibration_data["corner4"] = calculate_opposite_corners(captured_corners[0], captured_corners[1])
            print(f"\nMoving to {corner_key} position...")
            if first_z_move:
                move_time = 10.0
                first_z_move = False
            else:
                move_time = MOVE_TIME
            print(f"Move time: {move_time}")
            corner = move_to_a_corner_and_disable_mosfets_and_get_user_input_and_capture_position(calibration_data[corner_key], motorX, motorY, motorZ, move_time)
            captured_corners.append(corner)

        # Store corner positions
        corners = {
            "corner1": captured_corners[0],
            "corner2": captured_corners[1],
            "corner3": captured_corners[2],
            "corner4": captured_corners[3]
        }

        # Calculate zref points
        zref_points = calculate_zref_points(corners)

        # Move to corners one at a time and capture positions
        captured_zref_points = []
        for zref_point in ["zref1", "zref2", "zref3", "zref4"]:
            print(f"\nMoving to {zref_point} position...")
            zref = move_to_a_corner_and_disable_mosfets_and_get_user_input_and_capture_position(zref_points[zref_point], motorX, motorY, motorZ, MOVE_TIME)
            captured_zref_points.append(zref)

        # Store zref positions
        zref_points = {
            "zref1": captured_zref_points[0],
            "zref2": captured_zref_points[1],
            "zref3": captured_zref_points[2],
            "zref4": captured_zref_points[3]
        }

        print("The final corners are:")
        print(corners)
        print("The final zref points are:")
        print(zref_points)

        # Combine all points and save calibration data
        calibration_data = {**corners, **zref_points}

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
