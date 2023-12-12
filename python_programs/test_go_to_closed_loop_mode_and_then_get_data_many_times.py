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

N_ITERATIONS = 100000
OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "test_go_to_closed_loop_mode_and_get_data_many_times"

VERBOSE = True

GET_CURRENT_TIME_COMMAND_INTERVAL = 60
STATISTIC_PRINT_INTERVAL_SECONDS = 10
TIME_OUT_OF_SYNC_THRESHOLD_US = 4000
GET_PICOAMP_SECONDS_COUNT_COMMAND_INTERVAL = 60
ONE_ROTATION_MOTOR_UNITS = 357 * 256 * 7
GOERTZEL_ALGORITHM_N_SAMPLES = 80

ALIAS1 = ord("X")
ALIAS2 = ord("Y")
ALIAS3 = ord("Z")
ALIAS_LIST = [ALIAS2]
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

communication.serial_port = "/dev/cu.usbserial-0001"
communication.open_serial_port()

# let's reset all devices to start from a clean state
for alias in ALIAS_LIST:
    execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

# give time for the devices to reset and boot up
time.sleep(1.5)

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
total_rotation_motor_units = 0
for alias in ALIAS_LIST:
    if(total_rotation_motor_units) >= 0:
        random_rotation_motor_units = random.randint(-ONE_ROTATION_MOTOR_UNITS, 0)
    else:
        random_rotation_motor_units = random.randint(0, ONE_ROTATION_MOTOR_UNITS)
    total_rotation_motor_units += random_rotation_motor_units
    print("Total rotation motor units:", total_rotation_motor_units)
    movement_time = 0.5 # do the rotation over half a second
    if movement_time > max_movement_time:
        max_movement_time = movement_time
    movement_time_device_units = int(32150 * movement_time) 
    parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [random_rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
        exit(1)
time.sleep(max_movement_time + 0.3)

# let's reset all devices to start from a clean state
for alias in ALIAS_LIST:
    execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
time.sleep(1.5)

# now lets set test mode number 2 for all devices, which is the test mode for the go to closed loop mode
for alias in ALIAS_LIST:
    parsed_response = execute_command(alias, "TEST_MODE_COMMAND", [2], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", alias, "did not respond correctly to the GO_TO_CLOSED_LOOP_COMMAND")
        exit(1)


for iteration_number in range(N_ITERATIONS):
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

    # get data that was captured while going into closed loop mode
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "READ_MULTIPURPOSE_BUFFER_COMMAND", [], verbose=VERBOSE)
        parsed_response = parsed_response[0]
        if len(parsed_response) < 1:
            print("Error: The device with alias", alias, "did not respond correctly to the READ_MULTIPURPOSE_BUFFER_COMMAND and did not return at least one byte")
            exit(1)
        data_type = parsed_response[0]
        parsed_response = parsed_response[1:]
        print("Go to closed loop mode number of data elements:", len(parsed_response))
        parsed_response = bytearray(parsed_response)
        int32_list = [int.from_bytes(parsed_response[i:i+4], byteorder='little', signed=True) for i in range(0, len(parsed_response), 4)]
        if len(int32_list) != GOERTZEL_ALGORITHM_N_SAMPLES + 2:
            print("Error: The device with alias", alias, "did not respond correctly to the READ_MULTIPURPOSE_BUFFER_COMMAND and did not return the expected number of data elements")
            exit(1)
        on_device_goertzel_result = (int32_list[-2], int32_list[-1])
        int32_list = int32_list[:-2]
        print("Received this list of int32 data:", int32_list)
        print("There are this many values in the list:", len(int32_list))
        print("The on-device goertzel result is:", on_device_goertzel_result)
        filename = chr(alias) + "_data_last_captured"
        path_and_filename = OUTPUT_LOG_FILE_DIRECTORY + "/" + filename
        write_data(path_and_filename, int32_list)

exit(0)

for alias in ALIAS_LIST:
    homing_distance_motor_units = HOMING_DISTANCE_MOTOR_UNITS_DICT[alias]
    # now lets do homing to find the end stop so we can get to a known position
    parsed_response = execute_command(alias, "HOMING_COMMAND", [homing_distance_motor_units, 31250 * Y_AXIS_HOMING_TIME_SECONDS], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", alias, "did not respond correctly to the HOME_COMMAND")
        exit(1)

    # wait for the motor to finish homing, which can take several seconds if the motor is far from the end stop
    while True:
        parsed_response = execute_command(alias, "GET_STATUS_COMMAND", [], verbose=VERBOSE)
        if len(parsed_response) != 2:
            print("ERROR: The device with alias", alias, "did not respond correctly to the GET_STATUS_COMMAND")
            exit(1)
        print("The motor status is", parsed_response[0], "and the fatal error code is", parsed_response[1])
        # extract bit number 3 from the motor status byte, which is the homing bit
        homing = (parsed_response[0] & 0b00010000) != 0
        if not homing:
            break
        time.sleep(0.2)

exit(0)


# now, set the position to zero
parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the ZERO_POSITION_COMMAND")
    exit(1)
desired_pogo_pin_position = 0

# lets make sure we are able to travel to at least half the programming position
movement_time = 2 # seconds
movement_time_device_units = 31250 * movement_time
new_desired_pogo_pin_position = int(PROGRAMMING_POGO_PIN_MOVE_DISTANCE / 2)
movement_distance = new_desired_pogo_pin_position - desired_pogo_pin_position
parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "TRAPEZOID_MOVE_COMMAND", [movement_distance, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)
time.sleep(movement_time + 0.3)
desired_pogo_pin_position = new_desired_pogo_pin_position
time.sleep(1)

# read the position and make sure it is close to the expected value, within 50000 device units
parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 1:
    print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the GET_POSITION_COMMAND")
    exit(1)
actual_position = parsed_response[0]
print("The position is", actual_position)
if abs(actual_position - desired_pogo_pin_position) > 50000:
    print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not move to the expected position")
    exit(1)
print("Passed that sanity check")

# and move back up
new_desired_pogo_pin_position = 0
movement_distance = new_desired_pogo_pin_position - desired_pogo_pin_position
parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "TRAPEZOID_MOVE_COMMAND", [movement_distance, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)
time.sleep(movement_time + 0.3)
desired_pogo_pin_position = new_desired_pogo_pin_position
time.sleep(1)


# now lets go to closed loop mode on the X device (the moving table)
parsed_response = execute_command(MOVING_TABLE_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the GO_TO_CLOSED_LOOP_COMMAND")
    exit(1)

# wait for the motor to finish going to closed loop mode, which takes several seconds
while True:
    parsed_response = execute_command(MOVING_TABLE_ALIAS, "GET_STATUS_COMMAND", [], verbose=VERBOSE)
    if len(parsed_response) != 2:
        print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the GET_STATUS_COMMAND")
        exit(1)
    print("The motor status is", parsed_response[0], "and the fatal error code is", parsed_response[1])
    # extract bit number 2 from the motor status byte, which is the closed loop bit
    in_closed_loop = (parsed_response[0] & 0b00000100) != 0
    if in_closed_loop:
        break
    time.sleep(0.2)
print("Successfully entered closed loop mode on the device with alias", MOVING_TABLE_ALIAS)

# now lets do homing to find the end stop so we can get to a known position
parsed_response = execute_command(MOVING_TABLE_ALIAS, "HOMING_COMMAND", [3225600 * 8, 156250 * 2], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the HOME_COMMAND")
    exit(1)

# wait for the motor to finish homing, which can take several seconds if the motor is far from the end stop
while True:
    parsed_response = execute_command(MOVING_TABLE_ALIAS, "GET_STATUS_COMMAND", [], verbose=VERBOSE)
    if len(parsed_response) != 2:
        print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the GET_STATUS_COMMAND")
        exit(1)
    print("The motor status is", parsed_response[0], "and the fatal error code is", parsed_response[1])
    # extract bit number 3 from the motor status byte, which is the homing bit
    homing = (parsed_response[0] & 0b00010000) != 0
    if not homing:
        break
    time.sleep(0.2)

# now, set the position to zero
parsed_response = execute_command(MOVING_TABLE_ALIAS, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the ZERO_POSITION_COMMAND")
    exit(1)

# lets travel to all of the positions
movement_time = 1 # seconds
movement_time_device_units = 31250 * movement_time
for go_to_position_index in range(len(PROGRAMMING_POSITIONS)):
    # we need to read the current position first, so we can calculate the distance to travel
    parsed_response = execute_command(MOVING_TABLE_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
    if len(parsed_response) != 1:
        print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the GET_POSITION_COMMAND")
        exit(1)
    current_position = parsed_response[0]
    distance_to_travel = PROGRAMMING_POSITIONS[go_to_position_index] - current_position
    parsed_response = execute_command(MOVING_TABLE_ALIAS, "TRAPEZOID_MOVE_COMMAND", [distance_to_travel, movement_time_device_units], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", MOVING_TABLE_ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
        exit(1)
    time.sleep(movement_time + 0.3)

    # lets travel to the programming position
    movement_time = 3 # seconds
    movement_time_device_units = 31250 * movement_time
    new_desired_pogo_pin_position = PROGRAMMING_POGO_PIN_MOVE_DISTANCE
    movement_distance = new_desired_pogo_pin_position - desired_pogo_pin_position
    parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "TRAPEZOID_MOVE_COMMAND", [movement_distance, movement_time_device_units], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
        exit(1)
    time.sleep(movement_time + 0.3)
    desired_pogo_pin_position = new_desired_pogo_pin_position
    time.sleep(1)
    # and move back up
    new_desired_pogo_pin_position = 0
    movement_distance = new_desired_pogo_pin_position - desired_pogo_pin_position
    parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "TRAPEZOID_MOVE_COMMAND", [movement_distance, movement_time_device_units], verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
        exit(1)
    time.sleep(movement_time + 0.3)
    desired_pogo_pin_position = new_desired_pogo_pin_position
    time.sleep(1)

# we are done all movements. let's put everything into a safe position
# move the pogo pins down to the rest position
new_desired_pogo_pin_position = int(PROGRAMMING_POGO_PIN_MOVE_DISTANCE * 0.7)
movement_distance = new_desired_pogo_pin_position - desired_pogo_pin_position
parsed_response = execute_command(MOVING_POGO_PINS_ALIAS, "TRAPEZOID_MOVE_COMMAND", [movement_distance, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", MOVING_POGO_PINS_ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)
time.sleep(movement_time + 0.3)
desired_pogo_pin_position = new_desired_pogo_pin_position
time.sleep(1)

# reset all devices, because we are done testing
for alias in ALIAS_LIST:
    execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

