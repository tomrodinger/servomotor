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

TIME_FOR_N_ROTATIONS = 0.03
N_ROTATIONS = 0.15
THROW_MOSFET_CURRENT = 1024
THROW_ACCELERATON = 37108517
THROW_VELOCITY = 1000000000

N_COMMUTATION_STEPS                 = 64
N_COMMUTATION_SUB_STEPS             = 1350
ONE_REVOLUTION_ELECTRICAL_CYCLES    = 50
ONE_REVOLUTION_HALL_SENSOR_CYCLES   = 25

OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "ball_throwing_demo"


VERBOSE = True

ONE_ROTATION_MOTOR_UNITS = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES

ALIAS = ord("X")

N_PINGS_TO_TEST_COMMUNICATION = 10


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

communication.serial_port = "/dev/cu.usbserial-0001"
communication.open_serial_port()


# let's ping the devices many times to make sure they are there and that communication is working flawlessly
all_devices_responsed = True
print("Testing communication with device with alias %d by pinging it %d times" % (ALIAS, N_PINGS_TO_TEST_COMMUNICATION))
for i in range(N_PINGS_TO_TEST_COMMUNICATION):
    # generate a bytearray with 10 completely random bytes
    random_10_bytes = bytearray(random.getrandbits(8) for _ in range(10))
    parsed_response = execute_command(ALIAS, "PING_COMMAND", [random_10_bytes], verbose=VERBOSE)
    if len(parsed_response) != 1 or parsed_response[0] != random_10_bytes:
        print("ERROR: The device with alias", ALIAS, "did not respond to the PING_COMMAND")
        all_devices_responsed = False
        break
if not all_devices_responsed:
    print("ERROR: The device did not respond to the PING_COMMAND")
    exit(1)
print("The device responded correctly to all the %d pings" % (N_PINGS_TO_TEST_COMMUNICATION))

# enable MOSFETs
parsed_response = execute_command(ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the ENABLE_MOSFETS_COMMAND")
    exit(1)

# set the maximum acceleration of the throw
parsed_response = execute_command(ALIAS, "SET_MAX_ACCELERATION_COMMAND", [THROW_ACCELERATON], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SET_MAX_ACCELERATION_COMMAND")
    exit(1)

# set the maximul velocity of the throw
parsed_response = execute_command(ALIAS, "SET_MAX_VELOCITY_COMMAND", [THROW_VELOCITY], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SET_MAX_VELOCITY_COMMAND")
    exit(1)

time.sleep(0.2)

# set the MOSFET current
parsed_response = execute_command(ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [THROW_MOSFET_CURRENT, THROW_MOSFET_CURRENT], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
    exit(1)

time.sleep(0.3)

# we will move the motor in the forward direction quickly to throw the ball
movement_time_device_units = int(32150 * TIME_FOR_N_ROTATIONS)
rotation_motor_units = int(ONE_ROTATION_MOTOR_UNITS * N_ROTATIONS)
parsed_response = execute_command(ALIAS, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)

motor_position_list = []
hall_sensor_position_list = []
position_deviation_list = []
start_time = time.time()
stop_time = start_time + TIME_FOR_N_ROTATIONS + 0.3
while time.time() < stop_time:
    parsed_response = execute_command(ALIAS, "GET_COMPREHENSIVE_POSITION_COMMAND", [], verbose=VERBOSE)
    if len(parsed_response) != 3:
        print("ERROR: The device with alias", ALIAS, "did not respond correctly to the GET_COMPREHENSIVE_POSITION_COMMAND")
        exit(1)
    motor_position = parsed_response[0]
    hall_sensor_position = parsed_response[1]
    position_deviation = hall_sensor_position - motor_position
    motor_position_list.append(motor_position)
    hall_sensor_position_list.append(hall_sensor_position)
    position_deviation_list.append(hall_sensor_position - motor_position)
    log_fh.write(str(time.time()) + " " + str(motor_position) + " " + str(hall_sensor_position) + " " + str(position_deviation) + "\n")
    print("Motor position:", motor_position, "Hall sensor position:", hall_sensor_position, "Position deviation:", position_deviation)

# set the MOSFET current
parsed_response = execute_command(ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [300, 300], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
    exit(1)

time.sleep(0.1)

# we will return the arm back to the original position (slowly) so that we can load the next ball
TIME_FOR_N_ROTATIONS = 1.0
N_ROTATIONS = -N_ROTATIONS
movement_time_device_units = int(32150 * TIME_FOR_N_ROTATIONS)
rotation_motor_units = int(ONE_ROTATION_MOTOR_UNITS * N_ROTATIONS)
parsed_response = execute_command(ALIAS, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)

log_fh.close()