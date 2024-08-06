#!/usr/bin/env python3

import sys
import redirect_motor_commands as motor_commands
import redirect_communication as communication
import time
import os
import random
import math


VERBOSE = False

N_POLES = 50

MOSFET_CURRENT = 500
HIGHEST_MOSFET_CURRENT = 1024

# you can characterize the needed MOSFET current using the tool called characterize_homing.py
UP_DOWN_MOSFET_CURRENT = 200
SLIDER_MOSFET_CURRENT = 350
DISK_SPIN_MOSFET_CURRENT = 200
DISK_TRAY_MOSFET_CURRENT = 200

DISK_PULLER_MOSFET_CURRENT = 200
DISK_PULLER_PROBE_MOSFET_CURRENT1 = 70
DISK_PULLER_PROBE_MOSFET_CURRENT2 = 200
PID_P = 3000
PID_I = 2
PID_D = 1000000
#PID_P = 500 # DEBUG
#PID_I = 0 # DEBUG
#PID_D = 0 # DEBUG

N_COMMUTATION_STEPS               = 64
N_COMMUTATION_SUB_STEPS           = 1350
ONE_REVOLUTION_ELECTRICAL_CYCLES  = 50

CENTRING_PUSHER_REST_ANGLE = 24
CENTRING_PUSHER_START_ANGLE = 130
CENTRING_PUSHER_END_ANGLE = CENTRING_PUSHER_START_ANGLE - 7
CENTERING_TIME = 12
CENTERING_ROTATIONS = 50
INITIAL_DISK_PUSHER_TIME = 2
ALTERNATE_POLE_ADJUSTMENT_ANGLE = 180.0 + 7.2
POLE_MOVE_PERCENTAGE = 0.35  # we want to move the pole so that it is nearly touching the disk
POLE_MOVE_PERCENTAGE_BACKUP = POLE_MOVE_PERCENTAGE - 0.1  # we want to back up a little to let the pole get pulled down to the magnet that is holding it parallel to the disk
DISK_PULLER_START_ANGLE = 360 * 1.8
DISK_PULLER_PROBE_ANGLE1 = 100
DISK_PULLER_PROBE_ANGLE2 = 360
DISK_PULLER_PROBE_POSITION_TOLERANCE = 1
DISK_PULLER_PICKUP_ANGLE = 480
DISK_PULLER_DROP_OFF_ANGLE = -612


MAX_RPM                 = 360
MAX_RPS                 = MAX_RPM / 60
MICROSTEPS_PER_ROTATION = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES
ONE_DEGREE_MICROSTEPS = MICROSTEPS_PER_ROTATION / 360
TIME_STEPS_PER_SECOND   = 31250
THROW_VELOCITY          = int(MAX_RPS * MICROSTEPS_PER_ROTATION * (1 << 32) / TIME_STEPS_PER_SECOND / (1 << 12))

MM_PER_ROTATION                                   = 20
MAX_ACCELERATION_MM_PER_SECOND_SQUARED            = 10000
MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED     = (MAX_ACCELERATION_MM_PER_SECOND_SQUARED / MM_PER_ROTATION)
MAX_ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED    = (MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED * MICROSTEPS_PER_ROTATION)
MAX_ACCELERATION_MICROSTEPS_PER_TIME_STEP_SQUARED = (MAX_ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED / (TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND))
THROW_ACCELERATION                                = int(MAX_ACCELERATION_MICROSTEPS_PER_TIME_STEP_SQUARED * (1 << 32) / (1 << 8))


OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "ball_throwing_demo"

DISK_SPIN_ALIAS = ord("S")
SLIDER_ALIAS = ord("X")
DISK_TRAY_ALIAS = ord("Y")
UP_DOWN_ALIAS = ord("Z")

N_PINGS_TO_TEST_COMMUNICATION = 10
MOVE_TIME = 2.0
N_UP_DOWN_CYCLES = 3
N_DISK_TRAY_CYCLES = 3
ROTATIONAL_VELOCITY_OF_TRAY_MOTOR = 360.0 # degrees per second
DEFAULT_TRAY_FIRST_POSITION = 64.45 # unit is degrees of the shaft rotation. to find the best value for this, you should run the characterize_first_disk_location_on_tray.py program
TRAY_MOTOR_PULLEY_TEETH_NUMBER = 14
DISTANCE_BETWEEN_TEETH_MM = 2.0
DISTANCE_BETWEEN_DISKS_MM = 220 / 5
SPACING_BETWEEN_DISKS_DEGREES = 360 * DISTANCE_BETWEEN_DISKS_MM / (TRAY_MOTOR_PULLEY_TEETH_NUMBER * DISTANCE_BETWEEN_TEETH_MM)
UP_DOWN_RANGE_MIN = 300 # unit is degrees of the shaft rotation
EXPECTED_UP_DOWN_RANGE = 485
EXPECTED_DECREASED_RANGE_DUE_TO_DISK = 8
UP_DOWN_RANGE_TOLERANCE = 15
UP_DOWN_POSITION_TOLERANCE = 10 # unit is degrees of the shaft rotation
DISK_PICKUP_FIRST_POSITION = EXPECTED_UP_DOWN_RANGE * 0.6 # some of the way up but not all the way
DISK_PICKUP_FIRST_TIME = 2.0
DISK_PICKUP_SECOND_TIME = 2.0
DISK_PICKUP_SPIN_ROTATIONS = 10
N_DISKS = 8
POLE_PLACE_ANGLE_FUDGE_FACTOR = -2.0
EXPECTED_SLIDER_RANGE = 118.9
SLIDER_RANGE_TOLERANCE = 3.0
SLIDER_CENTRE_FINE_TUNE_OFFSET = -0.0
POLE_PLACE_ANGLE = EXPECTED_SLIDER_RANGE / 2 * 0.99 # a certain fraction of the way to the end stop



def reset_all_and_fatal_error():
    # reset all motors
    parsed_response = communication.execute_command(255, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(0.5)
    exit(1)


def do_homing_and_get_position_in_degrees(alias, max_degrees, max_time_s, homing_n_times = 1, relief_degrees = None, homing_tolerance_degrees = None, start_position = None,
                                          bidirectional = False, expected_bidirectional_range = None, zero_after_homing = True, zero_after_start_position_reached = False):
    # make sure all numbers are positive that need to be positive
    if max_time_s < 0 or homing_n_times < 0 or (relief_degrees is not None and relief_degrees < 0) or (homing_tolerance_degrees is not None and homing_tolerance_degrees < 0) or (expected_bidirectional_range is not None and expected_bidirectional_range < 0):
        print("ERROR: All numerical inputs (except max_degrees and start_position) must be positive")
        reset_all_and_fatal_error()
    rotation_speed_degrees_per_second = abs(max_degrees) / max_time_s
    print("The rotation speed is", rotation_speed_degrees_per_second, "degrees per second")
    homing_positions_degrees = []
    for i in range(homing_n_times):
        # do homing
        rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * max_degrees)
        movement_time_device_units = int(31250 * max_time_s)
        parsed_response = communication.execute_command(alias, "HOMING_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)

        # wait for the motor to finish homing
        while True:
            parsed_response = communication.execute_command(alias, "GET_STATUS_COMMAND", [], verbose=VERBOSE)
            print("The motor status is", parsed_response[0], "and the fatal error code is", parsed_response[1])
            # extract bit number 3 from the motor status byte, which is the homing bit
            homing = (parsed_response[0] & 0b00010000) != 0
            if not homing:
                break
            time.sleep(0.2)
        print("The motor has finished homing")

        # we should be at one end stop now. get the hall sensor position
        parsed_response = communication.execute_command(alias, "GET_HALL_SENSOR_POSITION_COMMAND", [], verbose=VERBOSE)
        position_motor_units = parsed_response[0]
        position_degrees = position_motor_units / ONE_DEGREE_MICROSTEPS
        homing_positions_degrees.append(position_degrees)
        print("The motor is at rotational position in degrees after this homing:", position_degrees)
        if relief_degrees is not None and i < homing_n_times - 1:
            # move the motor away from the end stop by the relief amount
            move_time_s = relief_degrees / rotation_speed_degrees_per_second
            rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * relief_degrees)
            if max_degrees > 0:
                rotation_motor_units = -rotation_motor_units
            movement_time_device_units = int(31250 * move_time_s)
            parsed_response = communication.execute_command(alias, "GO_TO_POSITION_COMMAND", [position_motor_units + rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
            time.sleep(0.5)
    # calculate the average position for all homing runs
    average_position_degrees = sum(homing_positions_degrees) / len(homing_positions_degrees)
    # determine the maximum deviation from the average position of any of the runs
    max_deviation_degrees = max([abs(position_degrees - average_position_degrees) for position_degrees in homing_positions_degrees])
    print(f"The average position is {average_position_degrees} and the maximum deviation is {max_deviation_degrees} degrees")
    if homing_tolerance_degrees is not None:
        # check that all homing runs are within the tolerance
        print(f"Our tolerance is {homing_tolerance_degrees} degrees")
        if max_deviation_degrees > homing_tolerance_degrees:
            print("ERROR: The homing position is not within the tolerance")
            exit(1)
        else:
            print("The homing position is within the tolerance")
    if zero_after_homing:
        # zero out the motor
        print("Zeroing out the motor right after homing")
        parsed_response = communication.execute_command(alias, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)
    if start_position is not None:
        print("Moving the motor to the start position")
        parsed_response = communication.execute_command(alias, "GET_HALL_SENSOR_POSITION_COMMAND", [], verbose=VERBOSE)
        position_motor_units = parsed_response[0]
        current_position_degrees = position_motor_units / ONE_DEGREE_MICROSTEPS
        distance_to_start_position = start_position - current_position_degrees
        print("Moving the motor to the start position, which is at", start_position, "degrees")
        move_time_s = abs(distance_to_start_position) / rotation_speed_degrees_per_second
        print("The move time is", move_time_s, "seconds")
        movement_time_device_units = int(31250 * move_time_s)
        rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * start_position)
        parsed_response = communication.execute_command(alias, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)
        # read the position and see if we are where we expect to be. if not then we exit
        parsed_response = communication.execute_command(alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        position_motor_units = parsed_response[0]
        motor_start_position_tolerance_degrees = 1.5
        if abs(position_motor_units - rotation_motor_units) > motor_start_position_tolerance_degrees * ONE_DEGREE_MICROSTEPS:
            print("ERROR: The motor did not reach the start position")
            print("The motor is at", position_motor_units / ONE_DEGREE_MICROSTEPS, "degrees")
            print("We wanted to move it to", start_position, "degrees")
            print(f"Our error is {abs(position_motor_units - rotation_motor_units) / ONE_DEGREE_MICROSTEPS} degrees and our tolerance is {motor_start_position_tolerance_degrees} degrees")
            reset_all_and_fatal_error()
        if zero_after_start_position_reached:
            print("Zeroing out the motor and moving it to the start position")
            # zero out the centring pusher motor
            parsed_response = communication.execute_command(alias, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)

    print("The motor has finished the first homing")
    return average_position_degrees


def disable_mosfets_and_read_positions():
    # disable the MOSFETs in all three motors
    parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(SLIDER_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    while 1:
        # get the positions of all three motors
        parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        disk_rotator_position = parsed_response[0]
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        centring_position = parsed_response[0]
        parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        slider_position = parsed_response[0]
        parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        disk_puller_position = parsed_response[0]
        disk_rotor_position_angle = disk_rotator_position / ONE_DEGREE_MICROSTEPS
        centring_position_angle = centring_position / ONE_DEGREE_MICROSTEPS
        slider_position_angle = slider_position / ONE_DEGREE_MICROSTEPS
        disk_puller_position_angle = disk_puller_position / ONE_DEGREE_MICROSTEPS
        # print out all these positions in the units of shaft angle with three decimal places
        print(f"Positions: disk rotator: {disk_rotor_position_angle:.3f}, centring: {centring_position_angle:.3f}, slider: {slider_position_angle:.3f}, disk puller: {disk_puller_position_angle:.3f}")
        time.sleep(0.1)

def assemble_magents_on_one_disk():
    cumulative_disk_angle = 0.0
    for i in range(N_POLES):
        print("Pole number ", i + 1, "of", N_POLES, "is next")
        user_input = input("Press Enter when you are ready to assemble the next pole or press p + enter to pull out the disk and end this disk assembly or press r + enter to abort all this, disable the motors, and read the positions of the motors...")
        if user_input == "p":
            break
        elif user_input == "r":
            disable_mosfets_and_read_positions()
            exit(0)

        # now, move the motor to pick up one pole. we will do this onlt the first time. all other times, we will pick up a pole while placing another
        move_time_s = 0.3
        # we will either go to the positive or negative position depending on the pole number
        pole_place_angle = POLE_PLACE_ANGLE

#        if i == 0:
#            # now, move the motor to place that pole onto the disk
#            parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_place_angle, 31250 * move_time_s], verbose=VERBOSE)
#            time.sleep(move_time_s * 1.1)

        if i % 2 == 1:
            pole_place_angle = -pole_place_angle

        # now, move the motor to place that pole onto the disk
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_place_angle, 31250 * move_time_s], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)

        # now, move the motor to the centre
        move_time_s = 0.3
        rotation = 0
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * rotation, 31250 * move_time_s], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)

        # rotate one one pole
        move_time_s = 0.3
        movement_time_device_units = int(31250 * move_time_s)
        if i % 2 == 0:
            cumulative_disk_angle += -ALTERNATE_POLE_ADJUSTMENT_ANGLE - POLE_PLACE_ANGLE_FUDGE_FACTOR + 360.0 / N_POLES * 2
        else:
            cumulative_disk_angle += ALTERNATE_POLE_ADJUSTMENT_ANGLE + POLE_PLACE_ANGLE_FUDGE_FACTOR
        print("Cumulative angle is:", cumulative_disk_angle)
        cumulative_position_motor_units = int(ONE_DEGREE_MICROSTEPS * cumulative_disk_angle + 0.5)
        print("Cumulative position in motor units is:", cumulative_position_motor_units)
        parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "GO_TO_POSITION_COMMAND", [cumulative_position_motor_units, movement_time_device_units], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)


# create the directory for saving the data logs if it does not already exist
if not os.path.exists(OUTPUT_LOG_FILE_DIRECTORY):
    try:
        os.makedirs(OUTPUT_LOG_FILE_DIRECTORY)
    except OSError as e:
        print("Could not create directory for saving the log files: %s: %s" % (OUTPUT_LOG_FILE_DIRECTORY, e))
        exit(1)
output_log_file = OUTPUT_LOG_FILE_DIRECTORY + "/" + OUTPUT_LOG_FILE_NAME + ".log"

# open the log file for writing
try:
    log_fh = open(output_log_file, "w")
except IOError as e:
    print("Could not open the log file for writing: %s: %s" % (output_log_file, e))
    exit(1)


communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

#communication.serial_port = "/dev/cu.usbserial-1140"
communication.open_serial_port()

# reset all motors
parsed_response = communication.execute_command(255, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
time.sleep(2.5)

# let's ping the device many times to make sure it is there and that communication is working flawlessly
all_devices_responsed = True
for alias in [DISK_SPIN_ALIAS, SLIDER_ALIAS, UP_DOWN_ALIAS]:
    print("Testing communication with device with alias %d by pinging it %d times" % (alias, N_PINGS_TO_TEST_COMMUNICATION))
    for i in range(N_PINGS_TO_TEST_COMMUNICATION):
        # generate a bytearray with 10 completely random bytes
        random_10_bytes = bytearray(random.getrandbits(8) for _ in range(10))
        parsed_response = communication.execute_command(alias, "PING_COMMAND", [random_10_bytes], verbose=VERBOSE)
        if len(parsed_response) != 1 or parsed_response[0] != random_10_bytes:
            print("ERROR: The device with alias", alias, "did not respond to the PING_COMMAND")
            all_devices_responsed = False
            break
if not all_devices_responsed:
    print("ERROR: The device did not respond to the PING_COMMAND")
    exit(1)
print("The device responded correctly to all the %d pings" % (N_PINGS_TO_TEST_COMMUNICATION))

# set the PID constants for all three motors
parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)

# enable MOSFETs in all three motors
parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# go to close loop position control mode in all three motors
parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)

# set the MOSFET current
parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [DISK_SPIN_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [SLIDER_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [UP_DOWN_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [DISK_TRAY_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
time.sleep(0.2) # allow the motor to stabilize it's position


# Home the Up/Down motor to the bottom position
do_homing_and_get_position_in_degrees(UP_DOWN_ALIAS, max_degrees = 360 * 4, max_time_s = 4, homing_n_times = 3, relief_degrees = 50, homing_tolerance_degrees = 10, start_position = 0,
                                          bidirectional = False, expected_bidirectional_range = None, zero_after_start_position_reached = True)

#print("First we will home the disk tray. Make sure that the Up/Down motor is not in the way of the disk tray. Push it all the way down manually.")
# prompt the user to press enter to start the test
#print("Press enter to start the first procedure")
#input()

do_homing_and_get_position_in_degrees(DISK_TRAY_ALIAS, max_degrees = -360 * 15, max_time_s = 15, homing_n_times = 3, relief_degrees = 30, homing_tolerance_degrees = 5, start_position = 0,
                                          bidirectional = False, expected_bidirectional_range = None, zero_after_homing = True, zero_after_start_position_reached = False)

for disk_number in range(N_DISKS):
    # Move the Disk Tray to the next position position. We will now actually start assembing disks (the first disk won't be assembled correctly). The first position may have caused magnets to be placed during homing and they will be pushed to far onto the disk
    current_disk_position = DEFAULT_TRAY_FIRST_POSITION + SPACING_BETWEEN_DISKS_DEGREES * disk_number
    print(f"Moving the disk tray to the next position and this is at position {current_disk_position} degrees")
    input("Please press enter to confirm")

    move_time_s = 2.0
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * current_disk_position)
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)

    # Now, move the Up/Down motor to part of the way to the top position (at least it must be past the point where it picks up the disk)
    move_time_s = 2.0
    rotation_motor_units = -int(ONE_DEGREE_MICROSTEPS * DISK_PICKUP_FIRST_POSITION)
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
    # check if it attained the position we wanted
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
    position_motor_units = parsed_response[0]
    position_degrees = position_motor_units / ONE_DEGREE_MICROSTEPS
    if abs(position_degrees - (-DISK_PICKUP_FIRST_POSITION)) > UP_DOWN_POSITION_TOLERANCE:
        print("ERROR: The Up/Down motor did not reach the position we wanted. It may be that something is blocking it like a disk or the tray")
        reset_all_and_fatal_error()
    # now start to spin the disk rotator motor (to help center the disk) and move up the rest of the way at the same time
    move_time_s = DISK_PICKUP_SECOND_TIME
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * 360 * DISK_PICKUP_SPIN_ROTATIONS)
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    rotation_motor_units = -int(ONE_DEGREE_MICROSTEPS * EXPECTED_UP_DOWN_RANGE - EXPECTED_DECREASED_RANGE_DUE_TO_DISK)
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
    # and spin the disk the other way back to zero using the same amount of time
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "GO_TO_POSITION_COMMAND", [0, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)

    # check the position that the Up/Down motor is at
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
    position_motor_units = parsed_response[0]
    position_degrees = position_motor_units / ONE_DEGREE_MICROSTEPS
    print("************* After moving up to place the disk Up/Down motor is at position", position_degrees, "degrees")
    expected_position_degrees = -(EXPECTED_UP_DOWN_RANGE - EXPECTED_DECREASED_RANGE_DUE_TO_DISK)
    position_error = position_degrees - expected_position_degrees
    if abs(position_error) > UP_DOWN_POSITION_TOLERANCE:
        print("ERROR: The Up/Down motor did not reach the position we expected. It may be that something is blocking it like a disk or the tray")
        print("The position is", position_degrees, "degrees")
        print("We expect to be at:", expected_position_degrees, "degrees")
        print("The position error is", position_error, "degrees")
        reset_all_and_fatal_error()

    if disk_number == 0: # we will use the first disk for homing of the slider. if magents are loaded then one or two magnets will be pushed onto the disk. since they won't be positioned right, we will not full assemble the disk
        # Home the Slider in the first direction
        slider_homing_position1 = do_homing_and_get_position_in_degrees(SLIDER_ALIAS, max_degrees = 360 * 0.6, max_time_s = 2, homing_n_times = 3, relief_degrees = 10, homing_tolerance_degrees = 10, start_position = None,
                                                                        bidirectional = False, expected_bidirectional_range = None, zero_after_homing = False, zero_after_start_position_reached = False)
        # Home the Slider in the second direction
        slider_homing_position2 = do_homing_and_get_position_in_degrees(SLIDER_ALIAS, max_degrees = -360 * 0.6, max_time_s = 2, homing_n_times = 3, relief_degrees = 10, homing_tolerance_degrees = 10, start_position = None,
                                                                        bidirectional = False, expected_bidirectional_range = None, zero_after_homing = False, zero_after_start_position_reached = False)
        print("The first homing position is", slider_homing_position1, "degrees")
        print("The second homing position is", slider_homing_position2, "degrees")
        slider_range = abs(slider_homing_position2 - slider_homing_position1)
        print("The range of the slider is", slider_range, "degrees")
        # let's check that the range is what we expect
        if abs(slider_range - EXPECTED_SLIDER_RANGE) > SLIDER_RANGE_TOLERANCE:
            print("ERROR: The slider range is not what we expect")
            print("The range we measured is", slider_range, "degrees")
            print("The range we expect is", EXPECTED_SLIDER_RANGE, "degrees")
            print(f"Consider updating this variable: EXPECTED_SLIDER_RANGE = {slider_range}")
            reset_all_and_fatal_error()
        # now, move the slider motor to the centre and zero out the position
        move_time_s = 0.5
        movement_time_device_units = int(31250 * move_time_s)
        rotation_motor_units = int(((slider_homing_position1 + slider_homing_position2) / 2 + SLIDER_CENTRE_FINE_TUNE_OFFSET) * ONE_DEGREE_MICROSTEPS)
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)
        parsed_response = communication.execute_command(SLIDER_ALIAS, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)
    else:
        assemble_magents_on_one_disk()



    # Move the Up/Down motor to the bottom position (the 0 position)
    move_time_s = 2.0
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * 0)
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)


# disable MOSFETs
parsed_response = communication.execute_command(DISK_SPIN_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# close the log file
log_fh.close()

time.sleep(0.2)
