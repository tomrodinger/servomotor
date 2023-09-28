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

N_ITERATIONS = 1
N_ROTATIONS = 2.5
TIME_FOR_N_ROTATIONS = 5
OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "position_vs_hall_sensor_position_vs_external_encoder_position"
RESULTS_FILENAME = OUTPUT_LOG_FILE_DIRECTORY + "position.txt"

VERBOSE = True

GET_CURRENT_TIME_COMMAND_INTERVAL = 60
TIME_SYNC_COMMAND_INTERVAL = 0.5
STATISTIC_PRINT_INTERVAL_SECONDS = 10
TIME_OUT_OF_SYNC_THRESHOLD_US = 4000
GET_PICOAMP_SECONDS_COUNT_COMMAND_INTERVAL = 60
ONE_ROTATION_MOTOR_UNITS = 357 * 256 * 7
ONE_ROTATION_ENCODER_UNITS = 2500

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

motor_position_encoder_units_list = []
hall_sensor_deviation_list = []
external_encoder_deviation_list = []
hall_position_vs_encoder_position_list = []

total_rotation_motor_units = 0
for iteration_number in range(N_ITERATIONS):
    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)

    # enable MOSFETs on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the ENABLE_MOSFETS_COMMAND")
            exit(1)

    # set the MOSFET current on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "SET_MAXIMUM_MOTOR_CURRENT", [300, 300], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
            exit(1)

    # we will go forwared direction and then do the same in the backward direction, collect data, then we will analyse the hysteresis and so on
    for direction in [1, -1]:
        for alias in ALIAS_LIST:
            movement_time_device_units = int(32150 * TIME_FOR_N_ROTATIONS)
            rotation_motor_units = int(ONE_ROTATION_MOTOR_UNITS * N_ROTATIONS) * direction
            parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
            if len(parsed_response) != 0:
                print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
                exit(1)

        start_time = time.time()
        stop_time = start_time + TIME_FOR_N_ROTATIONS + 0.3
        while time.time() < stop_time:
            for alias in ALIAS_LIST:
                parsed_response = execute_command(alias, "GET_COMPREHENSIVE_POSITION_COMMAND", [], verbose=VERBOSE)
                if len(parsed_response) != 3:
                    print("ERROR: The device with alias", alias, "did not respond correctly to the GET_COMPREHENSIVE_POSITION_COMMAND")
                    exit(1)
                motor_position = parsed_response[0]
                hall_sensor_position = parsed_response[1]
                external_encoder_position = parsed_response[2]
                hall_sensor_position_encoder_units = hall_sensor_position * (float(ONE_ROTATION_ENCODER_UNITS) / float(ONE_ROTATION_MOTOR_UNITS))
                motor_position_encoder_units = motor_position * (float(ONE_ROTATION_ENCODER_UNITS) / float(ONE_ROTATION_MOTOR_UNITS))
                motor_position_encoder_units_list.append(motor_position_encoder_units)
                hall_sensor_deviation_list.append(hall_sensor_position_encoder_units - motor_position_encoder_units)
                external_encoder_deviation_list.append(external_encoder_position - motor_position_encoder_units)
                hall_position_vs_encoder_position_list.append(hall_sensor_position_encoder_units - external_encoder_position)
                log_fh.write(str(time.time()) + " " + str(motor_position) + " "  + str(motor_position_encoder_units) + " " + str(hall_sensor_position) + " " + str(hall_sensor_position_encoder_units) + " " + str(external_encoder_position) + " " + str(motor_position_encoder_units) + "\n")
                print("Motor position:", motor_position, "Motor position encoder units:", motor_position_encoder_units, "Hall sensor position:", hall_sensor_position, "External encoder position:", external_encoder_position,
                    "Theoretical encoder position:", motor_position_encoder_units)

# let's reset all devices to start from a clean state
for alias in ALIAS_LIST:
    execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
time.sleep(2)

log_fh.close()

# let's compute the mean and standard deviation of the hall sensor and external encoder deviations ass well as the deviation of the hall sensor position from the external encoder position
print("")
hall_sensor_deviation_mean = sum(hall_sensor_deviation_list) / len(hall_sensor_deviation_list)
hall_sensor_deviation_standard_deviation = math.sqrt(sum([(x - hall_sensor_deviation_mean) ** 2 for x in hall_sensor_deviation_list]) / len(hall_sensor_deviation_list))
external_encoder_deviation_mean = sum(external_encoder_deviation_list) / len(external_encoder_deviation_list)
external_encoder_deviation_standard_deviation = math.sqrt(sum([(x - external_encoder_deviation_mean) ** 2 for x in external_encoder_deviation_list]) / len(external_encoder_deviation_list))
hall_position_vs_external_encoder_position_mean = sum(hall_position_vs_encoder_position_list) / len(hall_position_vs_encoder_position_list)
hall_position_vs_external_encoder_position_deviation = math.sqrt(sum([(x - hall_position_vs_external_encoder_position_mean) ** 2 for x in hall_position_vs_encoder_position_list]) / len(hall_position_vs_encoder_position_list))
print("Hall sensor deviation mean:", hall_sensor_deviation_mean, "Hall sensor deviation standard deviation:", hall_sensor_deviation_standard_deviation)
print("External encoder deviation mean:", external_encoder_deviation_mean, "External encoder deviation standard deviation:", external_encoder_deviation_standard_deviation)
print("Hall position vs external encoder position mean:", hall_position_vs_external_encoder_position_mean, "Hall position vs external encoder position deviation:", hall_position_vs_external_encoder_position_deviation)

# calculate the maximum and minimum deviations of the hall sensor and external encoder from their means as well as the maximum and minimum deviations of the hall sensor position from the external encoder position
hall_sensor_deviation_max = max(hall_sensor_deviation_list) - hall_sensor_deviation_mean
hall_sensor_deviation_min = min(hall_sensor_deviation_list) - hall_sensor_deviation_mean
external_encoder_deviation_max = max(external_encoder_deviation_list) - external_encoder_deviation_mean
external_encoder_deviation_min = min(external_encoder_deviation_list) - external_encoder_deviation_mean
hall_position_vs_external_encoder_position_max = max(hall_position_vs_encoder_position_list) - hall_position_vs_external_encoder_position_mean
hall_position_vs_external_encoder_position_min = min(hall_position_vs_encoder_position_list) - hall_position_vs_external_encoder_position_mean
print("Hall sensor deviation max:", hall_sensor_deviation_max, "Hall sensor deviation min:", hall_sensor_deviation_min)
print("External encoder deviation max:", external_encoder_deviation_max, "External encoder deviation min:", external_encoder_deviation_min)
print("Hall position vs external encoder position max:", hall_position_vs_external_encoder_position_max, "Hall position vs external encoder position min:", hall_position_vs_external_encoder_position_min)

print("")
print("The log file is saved to", output_log_file)
