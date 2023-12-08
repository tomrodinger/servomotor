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

N_ITERATIONS = 1000
OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "test_go_to_closed_loop_mode"
GOERTZEL_ALGORITHM_RESULTS_FILENAME = OUTPUT_LOG_FILE_DIRECTORY + "goertzel_algorithm_results.txt"
ON_DEVICE_GOERTZEL_ALGORITHM_RESULTS_FILENAME = OUTPUT_LOG_FILE_DIRECTORY + "on_device_goertzel_algorithm_results.txt"

VERBOSE = True

POSITION_ERROR_TOLERANCE = 10000
GET_CURRENT_TIME_COMMAND_INTERVAL = 60
TIME_SYNC_COMMAND_INTERVAL = 0.5
STATISTIC_PRINT_INTERVAL_SECONDS = 10
TIME_OUT_OF_SYNC_THRESHOLD_US = 4000
GET_PICOAMP_SECONDS_COUNT_COMMAND_INTERVAL = 60
ONE_ROTATION_MOTOR_UNITS = 357 * 256 * 7
GOERTZEL_ALGORITHM_N_SAMPLES = 80

ALIAS1 = ord("X")
ALIAS2 = ord("Y")
ALIAS3 = ord("Z")
ALIAS_LIST = [ALIAS1]
ALL_ALIASES = 255

N_PINGS_TO_TEST_COMMUNICATION = 100


def write_data(filename, int32_list):
    with open(filename, "w") as fh:
        for i in range(len(int32_list)):
            fh.write(str(i) + " " + str(int32_list[i]) + "\n")


def execute_command(alias, command_str, inputs, verbose=True):
    communication.alias = alias
    command_id = communication.get_command_id(command_str)
    if command_id == None:
        print("ERROR: The command", command_str, "is not supported")
        exit(1)
    if verbose:
        print("The command is: %s and it has ID %d" % (command_str, command_id))
    gathered_inputs = communication.gather_inputs(command_id, inputs, verbose=verbose)
    response = communication.send_command(command_id, gathered_inputs, verbose=verbose)
    parsed_response = communication.interpret_response(command_id, response, verbose=verbose)
    return parsed_response


# create the directory for saving the data logs if it does not already exist
if not os.path.exists(OUTPUT_LOG_FILE_DIRECTORY):
    try:
        os.makedirs(OUTPUT_LOG_FILE_DIRECTORY)
    except OSError as e:
        print("Could not create directory for saving the log files: %s: %s" % (OUTPUT_LOG_FILE_DIRECTORY, e))
        exit(1)
log_file_timestamp = str(int(time.time()))
output_log_file = OUTPUT_LOG_FILE_DIRECTORY + "/" + OUTPUT_LOG_FILE_NAME + "." + log_file_timestamp + ".log"

# open the log file for writing
try:
    log_fh = open(output_log_file, "w")
except IOError as e:
    print("Could not open the log file for writing: %s: %s" % (output_log_file, e))
    exit(1)



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
statistics_total_go_to_closed_loop_attempts = {}
for alias in ALIAS_LIST:
    statistics_failed_go_to_closed_loop[alias] = 0
    statistics_total_go_to_closed_loop_attempts[alias] = 0

total_rotation_motor_units = 0
for iteration_number in range(N_ITERATIONS):
    # enable MOSFETs on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the ENABLE_MOSFETS_COMMAND")
            exit(1)

    # set the MOSFET current on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "SET_MAXIMUM_MOTOR_CURRENT", [200, 200], verbose=VERBOSE)
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
        print("***************************************************************************** random_rotation_motor_units", random_rotation_motor_units)
        print("***************************************************************************** Total rotation motor units:", total_rotation_motor_units)
        movement_time = 0.5 # do the rotation over half a second
        if movement_time > max_movement_time:
            max_movement_time = movement_time
        movement_time_motor_units = int(32150 * movement_time) 
        parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [random_rotation_motor_units, movement_time_motor_units], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
            exit(1)
    time.sleep(max_movement_time + 0.3)

    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)

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
            print("The motor status is", parsed_response[0], "and the fatal error code is", parsed_response[1])
            # extract bit number 2 from the motor status byte, which is the closed loop bit
            is_motor_busy = (parsed_response[0] & 0b01000000) != 0
            if not is_motor_busy:
                break
            time.sleep(0.2)
        print("Successfully entered closed loop mode on the device with alias", alias)

    expected_position_after_move = 0
    for j in range(2): # we will move in the forward and reverse direction
        # Rotate all devices a fixed amount quickly. We will check to see if they achieved the movement in the expected time.
        movement_time = 0.2 # do the rotation in this amount of time
        movement_time_motor_units = int(32150 * movement_time)
        rotation_motor_units = int(ONE_ROTATION_MOTOR_UNITS * 0.5)
        if j == 1:
            rotation_motor_units = -rotation_motor_units
        expected_position_after_move += rotation_motor_units
        for alias in ALIAS_LIST:
            parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_motor_units], verbose=VERBOSE)
            if len(parsed_response) != 0:
                print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
                exit(1)
        time.sleep(max_movement_time)

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
            if position_error > POSITION_ERROR_TOLERANCE:
                print("ERROR: The final position is too far from the expected one. The motor did not move correctly.")
                statistics_failed_go_to_closed_loop[alias] += 1
            statistics_total_go_to_closed_loop_attempts[alias] += 1

    # Print out all statistics for all aliases
    print("\n=== STATISTICS ===============================================================================================")
    for alias in ALIAS_LIST:
        print("   Alias", alias, ":")
        print("      Total go to closed loop attempts:", statistics_total_go_to_closed_loop_attempts[alias])
        print("      Failed go to closed loop attempts:", statistics_failed_go_to_closed_loop[alias])
    print("==============================================================================================================\n")

    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)
