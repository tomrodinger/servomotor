#!/usr/bin/env python3

import sys
import motor_commands
import communication
import time
import os
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

VERBOSE = 0

GET_CURRENT_TIME_COMMAND_INTERVAL = 60
STATISTIC_PRINT_INTERVAL_SECONDS = 10
TIME_OUT_OF_SYNC_THRESHOLD_US = 4000
GET_PICOAMP_SECONDS_COUNT_COMMAND_INTERVAL = 60
ONE_ROTATION_MOTOR_UNITS = 357 * 256 * 7
GOERTZEL_ALGORITHM_N_SAMPLES = 80
TEST_MOVE_TIME = 1 # in seconds
TEST_MOVE_ROTATIONS = 6 # in rotations
TEST_MOVE_ROTATIONS_ERROR_TOLERANCE = 1 # in rotations

TEST_MOVE_TIME_MOTOR_UNITS = int(31250 * TEST_MOVE_TIME)
TEST_MOVE_ROTATIONS_MOTOR_UNITS = int(ONE_ROTATION_MOTOR_UNITS * TEST_MOVE_ROTATIONS)
TEST_MOVE_ROTATIONS_ERROR_TOLERANCE_MOTOR_UNITS = int(TEST_MOVE_ROTATIONS_ERROR_TOLERANCE * ONE_ROTATION_MOTOR_UNITS)

ALIAS1 = ord("X")
ALIAS2 = ord("Y")
ALIAS3 = ord("Z")
ALIAS_LIST = [ord("0")]
ALL_ALIASES = 255

N_PINGS_TO_TEST_COMMUNICATION = 100


def write_data(filename, int32_list):
    with open(filename, "w") as fh:
        for i in range(len(int32_list)):
            fh.write(str(i) + " " + str(int32_list[i]) + "\n")


def execute_command(alias, command_str, inputs, verbose=2):
    communication.alias = alias
    command_id = communication.get_command_id(command_str)
    if command_id == None:
        print("ERROR: The command", command_str, "is not supported")
        exit(1)
    if verbose == 2:
        print("The command is: %s and it has ID %d" % (command_str, command_id))
    gathered_inputs = communication.gather_inputs(command_id, inputs, verbose=verbose)
    response = communication.send_command(command_id, gathered_inputs, verbose=verbose)
    parsed_response = communication.interpret_response(command_id, response, verbose=verbose)
    return parsed_response


communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

communication.serial_port = "/dev/tty.usbserial-1140"
communication.open_serial_port()

# let's reset all devices to start from a clean state
for alias in ALIAS_LIST:
    execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

# give time for the devices to reset and boot up
time.sleep(1.0)

# let's ping the devices many times to make sure they are there and that communication is working flawlessly
all_devices_responsed = True
for alias in ALIAS_LIST:
    print("Testing communication with device with alias %d by pinging it %d times" % (alias, N_PINGS_TO_TEST_COMMUNICATION))
    for i in range(N_PINGS_TO_TEST_COMMUNICATION):
        # generate a bytearray with 10 completely random bytes
        random_10_bytes = bytearray(random.getrandbits(8) for _ in range(10))
        parsed_response = execute_command(alias, "PING_COMMAND", [random_10_bytes], verbose=VERBOSE)
        if len(parsed_response) != 1 or parsed_response[0] != random_10_bytes:
            print("ERROR: The device with alias", alias, "did not respond to the PING_COMMAND")
            all_devices_responsed = False
            break
if not all_devices_responsed:
    print("ERROR: Some devices did not respond to the PING_COMMAND")
    exit(1)
print("All devices responded correctly to all the %d pings" % (N_PINGS_TO_TEST_COMMUNICATION))

# Set up the dictionaries for gathering the statistics
statistics_failed_go_to_closed_loop = {}
statistics_total_go_to_closed_loop_attempts = 0
for alias in ALIAS_LIST:
    statistics_failed_go_to_closed_loop[alias] = 0
phase_angle_success_count = {}
phase_angle_total_count = {}
total_rotation_motor_units = 0
while 1:
    # enable MOSFETs on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the ENABLE_MOSFETS_COMMAND")
            exit(1)

    # set the MOSFET current on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "SET_MAXIMUM_MOTOR_CURRENT", [150, 150], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
            exit(1)

    # rotate all devices a random amount between -357 * 256 * 7 and 357 * 256 * 7
    max_movement_time = 0
    for alias in ALIAS_LIST:
        if(total_rotation_motor_units) >= 0:
            random_rotation_motor_units = random.randint(-ONE_ROTATION_MOTOR_UNITS, 0)
        else:
            random_rotation_motor_units = random.randint(0, ONE_ROTATION_MOTOR_UNITS)
        total_rotation_motor_units += random_rotation_motor_units
        print("Random rotation motor units:", random_rotation_motor_units)
        print("Total rotation motor units:", total_rotation_motor_units)
        movement_time = 0.5 # do the rotation over half a second
        if movement_time > max_movement_time:
            max_movement_time = movement_time
        movement_time_motor_units = int(31250 * movement_time) 
        parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [random_rotation_motor_units, movement_time_motor_units], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
            exit(1)
    time.sleep(max_movement_time + 0.3)

    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)

    # set the MOSFET current on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "SET_MAXIMUM_MOTOR_CURRENT", [300, 300], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
            exit(1)

    # now lets go to closed loop mode on all the axes
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the GO_TO_CLOSED_LOOP_COMMAND")
            exit(1)

    # wait for the motor to finish going to closed loop mode, which takes several seconds
    for alias in ALIAS_LIST:
        while True:
            parsed_response = execute_command(alias, "GET_STATUS_COMMAND", [], verbose=VERBOSE)
            if len(parsed_response) != 2:
                print("ERROR: The device with alias", alias, "did not respond correctly to the GET_STATUS_COMMAND")
                exit(1)
            if VERBOSE == 2:
                print("The motor status is", parsed_response[0], "and the fatal error code is", parsed_response[1])
            # extract bit number 2 from the motor status byte, which is the closed loop bit
            is_motor_busy = (parsed_response[0] & 0b01000000) != 0
            if not is_motor_busy:
                break
            time.sleep(0.2)
        print("Successfully entered closed loop mode on the device with alias", alias)


    # get data that was captured while going into closed loop mode
    phase_angle = {}
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "READ_MULTIPURPOSE_BUFFER_COMMAND", [], verbose=VERBOSE)
        parsed_response = parsed_response[0]
        if len(parsed_response) < 1:
            print("Error: The device with alias {alias} did not respond correctly to the READ_MULTIPURPOSE_BUFFER_COMMAND and did not return at least one byte")
            exit(1)
        data_type = parsed_response[0]
        parsed_response = parsed_response[1:]
        print("Go to closed loop mode number of data elements:", len(parsed_response))
        parsed_response = bytearray(parsed_response)
        int32_list = [int.from_bytes(parsed_response[i:i+4], byteorder='little', signed=True) for i in range(0, len(parsed_response), 4)]
        if len(int32_list) != GOERTZEL_ALGORITHM_N_SAMPLES + 2:
            print("Error: The device with alias {alias} did not respond correctly to the READ_MULTIPURPOSE_BUFFER_COMMAND and did not return the expected number of data elements")
            exit(1)
        real_part = int32_list[-2]
        imaginary_part = int32_list[-1]
        print("The on-device goertzel result is: {real_part} {imaginary_part}")
        if abs(real_part) >= abs(imaginary_part):
            if real_part >= 0.0:
                print("Angle determined to be very roughly 0 degrees")
                phase_angle[alias] = 0
            else:
                print("Angle determined to be very roughly 180 degrees")
                phase_angle[alias] = 180
        else:
            if imaginary_part >= 0.0:
                print("Angle determined to be very roughly 90 degrees")
                phase_angle[alias] = 90
            else:
                print("Angle determined to be very roughly 270 degrees")
                phase_angle[alias] = 270

    time.sleep(0.2)

    expected_position_after_move = 0
    failed_to_go_to_closed_loop = {}
    for alias in ALIAS_LIST:
        failed_to_go_to_closed_loop[alias] = False
    for j in range(2): # we will move in the forward and reverse direction
        # Rotate all devices a fixed amount quickly. We will check to see if they achieved the movement in the expected time.
        rotation_motor_units = TEST_MOVE_ROTATIONS_MOTOR_UNITS
        if j == 1:
            rotation_motor_units = -rotation_motor_units
        expected_position_after_move += rotation_motor_units
        for alias in ALIAS_LIST:
            parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, TEST_MOVE_TIME_MOTOR_UNITS], verbose=VERBOSE)
            if len(parsed_response) != 0:
                print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
                exit(1)
        time.sleep(TEST_MOVE_TIME)

        # Check the final position to see if the motor arrived there
        for alias in ALIAS_LIST:
            parsed_response = execute_command(alias, "GET_HALL_SENSOR_POSITION_COMMAND", [], verbose=VERBOSE)
            if len(parsed_response) != 1:
                print("ERROR: The device with alias", alias, "did not respond correctly to the GET_POSITION_COMMAND")
                exit(1)
            sensor_position_after_move = parsed_response[0]
            print("The sensor position after the move is:", sensor_position_after_move)
            position_error = abs(sensor_position_after_move - expected_position_after_move)
            print("The position error is:", position_error, "motor units")
            if position_error > TEST_MOVE_ROTATIONS_ERROR_TOLERANCE_MOTOR_UNITS:
                print("ERROR: The final position is too far from the expected one. The motor did not move correctly.")
                failed_to_go_to_closed_loop[alias] = True

    # Now, udate the statistics in the main dictionary
    statistics_total_go_to_closed_loop_attempts += 1
    for alias in ALIAS_LIST:
        if failed_to_go_to_closed_loop[alias]:
            statistics_failed_go_to_closed_loop[alias] += 1

    # Update the statistics for the phase angles
    for alias in ALIAS_LIST:
        if phase_angle[alias] not in phase_angle_success_count:
            phase_angle_success_count[phase_angle[alias]] = 0
            phase_angle_total_count[phase_angle[alias]] = 0
        phase_angle_total_count[phase_angle[alias]] += 1
        if failed_to_go_to_closed_loop[alias] == False:
            phase_angle_success_count[phase_angle[alias]] += 1

    # Print out all statistics for all aliases
    print("\n=== STATISTICS ===============================================================================================")
    print("   Total go to closed loop attempts:", statistics_total_go_to_closed_loop_attempts)
    for alias in ALIAS_LIST:
        print("   Alias", alias, ":")
        print("      Failed go to closed loop attempts:", statistics_failed_go_to_closed_loop[alias])
        print("      Success perentage:", (statistics_total_go_to_closed_loop_attempts - statistics_failed_go_to_closed_loop[alias]) / statistics_total_go_to_closed_loop_attempts * 100)
    for phase_angle in phase_angle_success_count:
        print("   Phase angle", phase_angle, "success percentage:", phase_angle_success_count[phase_angle] / phase_angle_total_count[phase_angle] * 100)
    print("==============================================================================================================\n")

    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)
