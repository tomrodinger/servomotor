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
ALL_ALIASES = 255

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


def goertzel_algorithm_integer_math(samples, constants):
    (w_real_multiplier, w_real_shift, w_imag_multiplier, w_imag_shift) = constants
    n_samples = len(samples)
    print("n_samples: " + str(n_samples))
    d1 = 0
    d2 = 0
    for n in range(n_samples):
        d1_times_w_real_multiplier_64bit = d1 * w_real_multiplier
        y = samples[n] + (d1_times_w_real_multiplier_64bit >> w_real_shift) - d2
        d2 = d1
        d1 = y
        print("n: %d, d1_times_w_real_multiplier_64bit: %d, y: %d, d1: %d, d2: %d" % (n, d1_times_w_real_multiplier_64bit, y, d1, d2))
    d1_times_w_real_multiplier_64bit = w_real_multiplier * d1
    d1_times_w_imag_multiplier_64bit = d1 * w_imag_multiplier
    d1_times_w_real_multiplier_32bit = (d1_times_w_real_multiplier_64bit >> w_real_shift)
    d1_times_w_imag_multiplier_32bit = (d1_times_w_imag_multiplier_64bit >> w_imag_shift)
    result = ((d1_times_w_real_multiplier_32bit >> 1) - d2, d1_times_w_imag_multiplier_32bit)
    # append the result to a datafile
    with open(GOERTZEL_ALGORITHM_RESULTS_FILENAME, "a") as fh:
        fh.write(str(result[0]) + " " + str(result[1]) + "\n")
    return result

# here we compute the magnitude and phase shift of the input waveform using the goertzel algorithm
# we use floating point math and we calculate the constants every time. This is not efficient, but
# it is easy to understand and there should not be bugs in it.
def goertzel_algorithm_floating_point_math(samples):
    print("Goertzel Algotithm: Calculating using floating point math:")
    sample_rate = float(n_samples)
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

    (w_real_multiplier, w_real_shift, w_imag_multiplier, w_imag_shift) = constants
    n_samples = len(samples)
    print("n_samples: " + str(n_samples))
    d1 = 0
    d2 = 0
    for n in range(n_samples):
        d1_times_w_real_multiplier_64bit = d1 * w_real_multiplier
        y = samples[n] + (d1_times_w_real_multiplier_64bit >> w_real_shift) - d2
        d2 = d1
        d1 = y
        print("n: %d, d1_times_w_real_multiplier_64bit: %d, y: %d, d1: %d, d2: %d" % (n, d1_times_w_real_multiplier_64bit, y, d1, d2))
    d1_times_w_real_multiplier_64bit = w_real_multiplier * d1
    d1_times_w_imag_multiplier_64bit = d1 * w_imag_multiplier
    d1_times_w_real_multiplier_32bit = (d1_times_w_real_multiplier_64bit >> w_real_shift)
    d1_times_w_imag_multiplier_32bit = (d1_times_w_imag_multiplier_64bit >> w_imag_shift)
    result = ((d1_times_w_real_multiplier_32bit >> 1) - d2, d1_times_w_imag_multiplier_32bit)
    # append the result to a datafile
    with open(GOERTZEL_ALGORITHM_RESULTS_FILENAME, "a") as fh:
        fh.write(str(result[0]) + " " + str(result[1]) + "\n")
    return result


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

cc = calculate_constants()
cc.calculate(GOERTZEL_ALGORITHM_N_SAMPLES)


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
        movement_time_device_units = int(32150 * movement_time) 
        parsed_response = execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [random_rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
            exit(1)
    time.sleep(max_movement_time + 0.3)

    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)

    # now lets set test mode number 2 for all devices, which is the test mode for the go to closed loop mode
    for alias in ALIAS_LIST:
        parsed_response = execute_command(alias, "TEST_MODE_COMMAND", [2], verbose=VERBOSE)
        if len(parsed_response) != 0:
            print("ERROR: The device with alias", alias, "did not respond correctly to the GO_TO_CLOSED_LOOP_COMMAND")
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
        # Reinterpret bytearray as a list of 16-bit signed integers
        #int16_list = [struct.unpack('<h', parsed_response[i:i+2])[0] for i in range(0, len(parsed_response), 2)]
        # Reinterpret bytearray as a list of 32-bit signed integers
        #int32_list = [struct.unpack('<i', parsed_response[i:i+4])[0] for i in range(0, len(parsed_response), 4)]

        # Reinterpret bytearray as a list of 16-bit signed integers
        #int16_list = [int.from_bytes(parsed_response[i:i+2], byteorder='little', signed=True) for i in range(0, len(parsed_response), 2)]
        # Reinterpret bytearray as a list of 32-bit signed integers
        int32_list = [int.from_bytes(parsed_response[i:i+4], byteorder='little', signed=True) for i in range(0, len(parsed_response), 4)]
        if len(int32_list) != GOERTZEL_ALGORITHM_N_SAMPLES + 2:
            print("Error: The device with alias", alias, "did not respond correctly to the READ_MULTIPURPOSE_BUFFER_COMMAND and did not return the expected number of data elements")
            exit(1)
        on_device_goertzel_result = (int32_list[-2], int32_list[-1])
        int32_list = int32_list[:-2]
        print("Received this list of int32 data:", int32_list)
        print("There are this many values in the list:", len(int32_list))
        print("The on-device goertzel result is:", on_device_goertzel_result)
        filename = chr(alias) + "_data_" + str(iteration_number)
        path_and_filename = OUTPUT_LOG_FILE_DIRECTORY + "/" + filename
        write_data(path_and_filename, int32_list)
        result = goertzel_algorithm_integer_math(int32_list, cc.get_constants())
        print("The result of the goertzel algorithm is", result)
        # append the result to a datafile
        with open(ON_DEVICE_GOERTZEL_ALGORITHM_RESULTS_FILENAME, "a") as fh:
            fh.write(str(on_device_goertzel_result[0]) + " " + str(on_device_goertzel_result[1]) + "\n")

    # let's reset all devices to start from a clean state
    for alias in ALIAS_LIST:
        execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(2)
