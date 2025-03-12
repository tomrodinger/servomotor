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

VERBOSE = 2

GET_CURRENT_TIME_COMMAND_INTERVAL = 60
STATISTIC_PRINT_INTERVAL_SECONDS = 10
TIME_OUT_OF_SYNC_THRESHOLD_US = 4000
GET_PICOAMP_SECONDS_COUNT_COMMAND_INTERVAL = 60
ONE_ROTATION_MOTOR_UNITS = 357 * 256 * 7
GOERTZEL_ALGORITHM_N_SAMPLES = 80

ALIAS1 = ord("X")
ALIAS2 = ord("Y")
ALIAS3 = ord("Z")
ALIAS_LIST = [ALIAS1]


N_PINGS_TO_TEST_COMMUNICATION = 100


class calculate_constants:
    def calculate(self, n_samples):
        print("Calculating the constants:")
        self.n_samples = n_samples
        sample_rate = n_samples
        frequency = 1.0
        f = frequency / sample_rate
        print("f: " + str(f))
        w_real = 2.0 * math.cos(2.0 * math.pi * f)
        w_imag = math.sin(2.0 * math.pi * f)
        print("   w_real: " + str(w_real))
        print("   w_imag: " + str(w_imag))
        self.w_real_shift = 20
        self.w_real_multiplier = int(w_real * (1 << self.w_real_shift) + 0.5)
        self.w_imag_shift = 20
        self.w_imag_multiplier = int(w_imag * (1 << self.w_imag_shift) + 0.5)
        print("w_real_multiplier:", self.w_real_multiplier)
        print("w_real_shift:", self.w_real_shift)
        print("w_imag_multiplier:", self.w_imag_multiplier)
        print("w_imag_shift:", self.w_imag_shift)

    def get_constants(self):
        return (self.w_real_multiplier, self.w_real_shift, self.w_imag_multiplier, self.w_imag_shift)


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

cc = calculate_constants()
cc.calculate(GOERTZEL_ALGORITHM_N_SAMPLES)


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


total_rotation_motor_units = 0
for iteration_number in range(N_ITERATIONS):
    # enable MOSFETs on all devices
#    for alias in ALIAS_LIST:
#        parsed_response = execute_command(alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
#        if len(parsed_response) != 0:
#            print("ERROR: The device with alias", alias, "did not respond correctly to the ENABLE_MOSFETS_COMMAND")
#            exit(1)

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
        movement_time_device_units = int(31250 * movement_time) 
        parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [random_rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
            exit(1)
    time.sleep(max_movement_time + 0.3)

   # disable MOSFETs on all devices
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the DISABLE_MOSFETS_COMMAND")
            exit(1)

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

    time.sleep(1.0)
