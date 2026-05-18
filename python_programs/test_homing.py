#!/usr/bin/env python3

import servomotor
import time
import math
import argparse # Import argparse
from servomotor.communication import TimeoutError

# Maximum allowed difference between expected and actual movement
MAX_POSITION_TOLERANCE_PERCENT = 0.01

# Settings for the simulated-hard-stop homing test. The test motor has no
# physical end stop, so a hard stop is simulated by dropping the motor current
# mid-homing until the motor is too weak to follow the motion profile; the
# firmware then detects the resulting following error as a crash and stops.
HARD_STOP_CURRENT_NORMAL = 200  # current at which the motor homes normally
HARD_STOP_CURRENT_WEAK = 10     # current too weak to continue homing (experimentally determined)
HARD_STOP_DROP_FRACTION = 0.25  # fraction of maxDuration to home normally before dropping the current

def wait_for_moves_to_complete(motor):
    """Wait until all queued moves are complete"""
    while True:
        queue_size = motor.get_n_queued_items()
        if queue_size == 0:
            break
        time.sleep(0.01)  # Reduced sleep time for more accurate timing

def verify_position_in_all_units(motor, expected_position, tolerance=100):
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

def verify_homing_movement(motor, start_position, expected_direction, expected_counts):
    """Verify the homing movement was in the correct direction and moved approximately the expected distance"""
    # Always get position in encoder counts for consistent comparison
    motor.set_position_unit("encoder_counts")
    end_position = motor.get_position()
    movement = end_position - start_position
    print(f"Start position (counts):  {start_position}")
    print(f"End position (counts):    {end_position}")
    print(f"Total movement (counts):  {movement}")
    print(f"Expected movement:        {expected_counts}")
    
    # Verify direction
    if expected_direction > 0:
        assert movement > 0, "Expected positive movement but got negative"
    else:
        assert movement < 0, "Expected negative movement but got positive"

    # Verify magnitude of movement
    actual_magnitude = abs(movement)
    expected_magnitude = abs(expected_counts)
    difference_percent = abs(actual_magnitude - expected_magnitude) / expected_magnitude * 100
    print(f"Movement difference:      {difference_percent:.3f}%")
    assert difference_percent <= MAX_POSITION_TOLERANCE_PERCENT, f"Movement error ({difference_percent:.1f}%) exceeds tolerance ({MAX_POSITION_TOLERANCE_PERCENT}%)"

    # Verify final position in all units
    print("\nVerifying final position in all units:")
    verify_position_in_all_units(motor, end_position)

def run_homing_with_simulated_hard_stop(motor, max_distance_counts, max_duration):
    """Run a homing move and simulate hitting a hard stop partway through.

    The test motor has no physical end stop. To exercise the firmware's crash
    detection, homing is started at a normal motor current, and once the motor
    is moving the current is dropped so low that the motor can no longer follow
    the motion profile. The commanded position then runs ahead of the measured
    position and the firmware stops homing, reporting a crash.

    Returns (start_position_counts, elapsed_time_seconds).
    """
    motor.set_position_unit("encoder_counts")
    motor.set_maximum_motor_current(HARD_STOP_CURRENT_NORMAL, HARD_STOP_CURRENT_NORMAL)
    start_position = motor.get_position()

    start_time = time.time()
    motor.homing(maxDistance=max_distance_counts, maxDuration=max_duration)
    # Let the motor home normally for part of the move, then make it too weak to continue.
    time.sleep(max_duration * HARD_STOP_DROP_FRACTION)
    motor.set_maximum_motor_current(HARD_STOP_CURRENT_WEAK, HARD_STOP_CURRENT_WEAK)
    wait_for_moves_to_complete(motor)
    elapsed_time = time.time() - start_time

    # Restore a normal current so anything that runs afterwards is unaffected.
    motor.set_maximum_motor_current(HARD_STOP_CURRENT_NORMAL, HARD_STOP_CURRENT_NORMAL)
    return start_position, elapsed_time

def verify_homing_hard_stop(motor, start_position, expected_direction, max_distance_counts, elapsed_time, max_duration):
    """Verify that homing detected the simulated hard stop and stopped early."""
    motor.set_position_unit("encoder_counts")
    end_position = motor.get_position()
    movement = end_position - start_position
    print(f"Start position (counts):  {start_position}")
    print(f"End position (counts):    {end_position}")
    print(f"Total movement (counts):  {movement}")
    print(f"Commanded max distance:   {max_distance_counts}")
    print(f"Elapsed time:             {elapsed_time:.3f}s (maxDuration was {max_duration}s)")

    # Direction must match the commanded direction.
    if expected_direction > 0:
        assert movement > 0, "Expected positive movement but got negative"
    else:
        assert movement < 0, "Expected negative movement but got positive"

    # The motor must have genuinely been homing before the hard stop, i.e. it
    # moved well beyond the firmware's crash-detection threshold (50000 counts).
    assert abs(movement) > 100000, \
        f"Expected real homing motion before the hard stop, but only moved {abs(movement)} counts"

    # A hard stop must end homing before the full commanded distance is covered...
    assert abs(movement) < abs(max_distance_counts) * 0.9, \
        f"Expected homing to stop early, but it covered {abs(movement)} of {abs(max_distance_counts)} counts"

    # ...and before the full maxDuration elapses.
    assert elapsed_time < max_duration * 0.9, \
        f"Expected homing to end early, but it ran {elapsed_time:.3f}s of the {max_duration}s maxDuration"

    # The reported position must still be consistent across all units.
    print("\nVerifying final position in all units:")
    verify_position_in_all_units(motor, end_position)

def test_homing(motorX): # Accept motor object as argument
    """Runs the homing tests. Assumes motor object is initialized and port is open."""
    print("\nResetting motor...")
    motorX.system_reset() # Use the passed motor object
    time.sleep(1.5)  # Required delay after reset

    print("\nEnabling MOSFETs...")
    motorX.enable_mosfets()

    print("\nGoing to closed loop mode...")
    motorX.go_to_closed_loop()
    time.sleep(0.5)  # Give time to enter closed loop mode

    # ========================================================================
    # Part 1: Homing that finishes by covering the full commanded distance
    # (no hard stop). Verifies direction, distance accuracy and unit handling.
    # ========================================================================
    # Test homing in shaft rotations (2 rotations)
    print("\nTesting homing in shaft rotations...")
    print("Homing with distance of +2 rotations...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Get start position in encoder counts
    start_position = motorX.get_position()
    motorX.set_position_unit("shaft_rotations")  # Set to shaft_rotations for homing
    motorX.homing(maxDistance=2, maxDuration=2)  # Home with +2 rotation distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Positive direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, 1, 2 * 3276800)  # 2 rotations = 2 * 3,276,800 counts

    print("\nHoming with distance of -2 rotations...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Get start position in encoder counts
    start_position = motorX.get_position()
    motorX.set_position_unit("shaft_rotations")  # Set to shaft_rotations for homing
    motorX.homing(maxDistance=-2, maxDuration=2)  # Home with -2 rotation distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Negative direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, -1, -2 * 3276800)  # -2 rotations = -2 * 3,276,800 counts

    # Test homing in degrees (720 degrees = 2 rotations)
    print("\nTesting homing in degrees...")
    print("Homing with distance of +720 degrees...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Get start position in encoder counts
    start_position = motorX.get_position()
    motorX.set_position_unit("degrees")  # Set to degrees for homing
    motorX.homing(maxDistance=720, maxDuration=2)  # Home with +720 degrees distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Positive direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, 1, 2 * 3276800)  # 720 degrees = 2 rotations = 2 * 3,276,800 counts

    print("\nHoming with distance of -720 degrees...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Get start position in encoder counts
    start_position = motorX.get_position()
    motorX.set_position_unit("degrees")  # Set to degrees for homing
    motorX.homing(maxDistance=-720, maxDuration=2)  # Home with -720 degrees distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Negative direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, -1, -2 * 3276800)  # -720 degrees = -2 rotations = -2 * 3,276,800 counts

    # Test homing in radians (4π radians = 2 rotations)
    print("\nTesting homing in radians...")
    print("Homing with distance of +4π radians...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Get start position in encoder counts
    start_position = motorX.get_position()
    motorX.set_position_unit("radians")  # Set to radians for homing
    motorX.homing(maxDistance=4 * math.pi, maxDuration=2)  # Home with +4π radians distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Positive direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, 1, 2 * 3276800)  # 4π radians = 2 rotations = 2 * 3,276,800 counts

    print("\nHoming with distance of -4π radians...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Get start position in encoder counts
    start_position = motorX.get_position()
    motorX.set_position_unit("radians")  # Set to radians for homing
    motorX.homing(maxDistance=-4 * math.pi, maxDuration=2)  # Home with -4π radians distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Negative direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, -1, -2 * 3276800)  # -4π radians = -2 rotations = -2 * 3,276,800 counts

    # Test homing in encoder counts (6553600 counts = 2 rotations)
    print("\nTesting homing in encoder counts...")
    print("Homing with distance of +2 rotations in encoder counts...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Already in encoder counts
    start_position = motorX.get_position()
    motorX.homing(maxDistance=2 * 3276800, maxDuration=2)  # Home with +2 rotations distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Positive direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, 1, 2 * 3276800)  # 2 rotations = 2 * 3,276,800 counts

    print("\nHoming with distance of -2 rotations in encoder counts...")
    start_time = time.time()
    motorX.set_position_unit("encoder_counts")  # Ensure we're in encoder counts
    start_position = motorX.get_position()  # Already in encoder counts
    motorX.homing(maxDistance=-2 * 3276800, maxDuration=2)  # Home with -2 rotations distance, 2 second limit
    wait_for_moves_to_complete(motorX)
    elapsed_time = time.time() - start_time
    print(f"Negative direction homing completed in {elapsed_time:.3f} seconds")
    verify_homing_movement(motorX, start_position, -1, -2 * 3276800)  # -2 rotations = -2 * 3,276,800 counts

    # ========================================================================
    # Part 2: Homing that finishes by a (simulated) hard stop, before the full
    # commanded distance is reached. Verifies the firmware's crash detection.
    # ========================================================================
    print("\nTesting homing that ends on a simulated hard stop...")
    HARD_STOP_DISTANCE = 2 * 3276800  # command 2 rotations; the simulated hard stop happens first
    HARD_STOP_DURATION = 2

    print("\nHoming +2 rotations into a simulated hard stop...")
    start_position, elapsed_time = run_homing_with_simulated_hard_stop(motorX, HARD_STOP_DISTANCE, HARD_STOP_DURATION)
    print(f"Homing stopped after {elapsed_time:.3f} seconds")
    verify_homing_hard_stop(motorX, start_position, 1, HARD_STOP_DISTANCE, elapsed_time, HARD_STOP_DURATION)

    print("\nHoming -2 rotations into a simulated hard stop...")
    start_position, elapsed_time = run_homing_with_simulated_hard_stop(motorX, -HARD_STOP_DISTANCE, HARD_STOP_DURATION)
    print(f"Homing stopped after {elapsed_time:.3f} seconds")
    verify_homing_hard_stop(motorX, start_position, -1, -HARD_STOP_DISTANCE, elapsed_time, HARD_STOP_DURATION)

    print("\nTest complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test servomotor homing.')
    parser.add_argument('-p', '--port', help='Serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='Show available ports and prompt for selection', action="store_true")
    parser.add_argument('-a', '--alias', help='Alias of the device to control', required=True)
    # Add verbosity later if needed, pass it to M3 constructor
    args = parser.parse_args()

    # Set port before opening
    servomotor.set_serial_port_from_args(args)

    motorX = None # Initialize motorX to None
    try:
        servomotor.open_serial_port()
        # Initialize motor object after opening port and getting alias
        motorX = servomotor.M3(args.alias,
                              time_unit="seconds",
                              position_unit="encoder_counts", # Start with counts for verification
                              velocity_unit="rpm",
                              acceleration_unit="degrees_per_second_squared",
                              verbose=0) # Set verbose=0 to reduce output
        test_homing(motorX) # Pass the motor object to the test function
        servomotor.close_serial_port()
        print("\nPASSED")
    except:
        servomotor.close_serial_port()
        raise
