#!/usr/bin/env python3

import sys
import redirect_motor_commands as motor_commands
import redirect_communication as communication
import time
import os
import random
import math
import struct


VERBOSE = False

N_POLES = 50

MOSFET_CURRENT = 500
HIGHEST_MOSFET_CURRENT = 1024
MOSFET_CURRENT = 200
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

DISK_TRAY_ALIAS = ord("Y")
UP_DOWN_ALIAS = ord("Z")
DISK_TRAY_MOSFET_CURRENT = 200
UP_DOWN_MOSFET_CURRENT = 200
OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "characterize_homing"
N_PINGS_TO_TEST_COMMUNICATION = 10
MOVE_TIME = 2.0
N_UP_DOWN_CYCLES = 3
N_DISK_TRAY_CYCLES = 3
ROTATIONAL_VELOCITY_OF_TRAY_MOTOR = 360.0 # degrees per second
DEFAULT_TRAY_FIRST_POSITION = 64.45 # unit is degrees of the shaft rotation
UP_DOWN_RANGE_MIN = 300 # unit is degrees of the shaft rotation
EXPECTED_UP_DOWN_RANGE = 479
UP_DOWN_RANGE_TOLERANCE = 15

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

def read_position_continuously(alias):
    while 1:
        # get the position
        parsed_response = communication.execute_command(alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        disk_tray_position = parsed_response[0]
        disk_tray_position_angle = disk_tray_position / ONE_DEGREE_MICROSTEPS
        # print out all these positions in the units of shaft angle with three decimal places
        print(f"Position of disk tray: {disk_tray_position_angle:.3f} (after you figure out what is the best first disk tray position, you can set it in the code in the DEFAULT_TRAY_FIRST_POSITION variable)")
        time.sleep(0.5)


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
for alias in [DISK_TRAY_ALIAS, UP_DOWN_ALIAS]:
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

# enable MOSFETs
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# go to close loop position control mode
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)

# set the MOSFET current
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [UP_DOWN_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [DISK_TRAY_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)

# Home the Up/Down motor to the bottom position
do_homing_and_get_position_in_degrees(UP_DOWN_ALIAS, max_degrees = 360 * 4, max_time_s = 4, homing_n_times = 3, relief_degrees = 50, homing_tolerance_degrees = 10, start_position = 0,
                                          bidirectional = False, expected_bidirectional_range = None, zero_after_start_position_reached = True)

#print("First we will home the disk tray. Make sure that the Up/Down motor is not in the way of the disk tray. Push it all the way down manually.")
# prompt the user to press enter to start the test
#print("Press enter to start the first procedure")
#input()

do_homing_and_get_position_in_degrees(DISK_TRAY_ALIAS, max_degrees = -360 * 15, max_time_s = 15, homing_n_times = 3, relief_degrees = 30, homing_tolerance_degrees = 5, start_position = DEFAULT_TRAY_FIRST_POSITION,
                                          bidirectional = False, expected_bidirectional_range = None, zero_after_homing = True, zero_after_start_position_reached = False)


# Now, home the Up/Down motor to the top position
up_down_upper_limit = do_homing_and_get_position_in_degrees(UP_DOWN_ALIAS, max_degrees = -360 * 4, max_time_s = 4, homing_n_times = 3, relief_degrees = 50, homing_tolerance_degrees = 15, start_position = None,
                                                            bidirectional = False, expected_bidirectional_range = None, zero_after_homing = False, zero_after_start_position_reached = False)
up_down_range = -up_down_upper_limit

print("The up/down range is", up_down_range, "degrees")
if up_down_range < UP_DOWN_RANGE_MIN:
    # disable the MOSFETs in all three motors
    parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(UP_DOWN_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    # At this point, we should read back the position of the tray and advise the user what new value to set to the DEFAULT_TRAY_FIRST_POSITION
    read_position_continuously(DISK_TRAY_ALIAS)

# check if the up/down range is what we expect winin some tolerance
if abs(up_down_range - EXPECTED_UP_DOWN_RANGE) > UP_DOWN_RANGE_TOLERANCE:
    print("ERROR: The up/down range is not what we expect. The expected range is", EXPECTED_UP_DOWN_RANGE, "degrees")
    reset_all_and_fatal_error()

# Now, let's move the tray back and forth while the up/down axis is at the top. This will find the extremes of the disk position. We will take the middle of this to know the location of the centre of the disk position
print("**************** Starting to move the tray back and forth to find the centre of the disk position")
tray_homing_position1 = do_homing_and_get_position_in_degrees(DISK_TRAY_ALIAS, max_degrees = -360 * 1, max_time_s = 1, homing_n_times = 3, relief_degrees = 10, homing_tolerance_degrees = 5, start_position = None,
                                                              bidirectional = False, expected_bidirectional_range = None, zero_after_homing = False, zero_after_start_position_reached = False)
tray_homing_position2 = do_homing_and_get_position_in_degrees(DISK_TRAY_ALIAS, max_degrees = 360 * 1, max_time_s = 1, homing_n_times = 3, relief_degrees = 10, homing_tolerance_degrees = 5, start_position = None,
                                                              bidirectional = False, expected_bidirectional_range = None, zero_after_homing = False, zero_after_start_position_reached = False)
print
tray_homing_midpoint = (tray_homing_position1 + tray_homing_position2) / 2
print("The midpoint of the disk tray homing is", tray_homing_midpoint, "degrees. You should set this value to the DEFAULT_TRAY_FIRST_POSITION variable in the code")
print("********************************************************************")
print(f"DEFAULT_TRAY_FIRST_POSITION = {tray_homing_midpoint}")
print("********************************************************************")

# Move that tray to that found midpoint
move_time_s = 1.0
rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * tray_homing_midpoint)
movement_time_device_units = int(31250 * move_time_s)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
time.sleep(move_time_s * 1.1)

# Move the Up/Down motor to the bottom position (the 0 position)
move_time_s = 3.0
rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * 0)
movement_time_device_units = int(31250 * move_time_s)
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
time.sleep(move_time_s * 1.1)

# disable MOSFETs
parsed_response = communication.execute_command(UP_DOWN_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_TRAY_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# close the log file
log_fh.close()
time.sleep(0.2)
exit(0)

start_time = time.time()
for i in range(N_UP_DOWN_CYCLES):
    for j in range(N_DISK_TRAY_CYCLES):
        # collect the position data and save it to a file as two columns of data
        position_data = move_one_cycle_and_collect_position(DISK_TRAY_ALIAS, move_velocity = 140000000, move_time_s = MOVE_TIME, start_time = start_time)
        for position in position_data:
            log_fh.write(f"{position[0]} {position[1]} {position[2]} {position[3]}\n")
        # look at the third column of the data and fomr those numbers, find the maximum and minimum values, subtract them to get the range. Let's write out the range vs. the mosfet current
        # to another file whose filename is in RANGE_VS_MOSFET_CURRENT_FILENAME
        max_position = -1000000000000
        min_position = 1000000000000
        for position in position_data:
            if position[2] > max_position:
                max_position = position[2]
            if position[2] < min_position:
                min_position = position[2]
        position_range = max_position - min_position


# disable the MOSFETs at the end
parsed_response = communication.execute_command(ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# close the log file
log_fh.close()

time.sleep(0.2)
