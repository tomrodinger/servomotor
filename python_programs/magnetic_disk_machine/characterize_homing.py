#!/usr/bin/env python3

import sys
import motor_commands
import communication
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


OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "characterize_homing"
RANGE_VS_MOSFET_CURRENT_FILENAME = "range_vs_mosfet_current"
N_PINGS_TO_TEST_COMMUNICATION = 10

def set_global_parameters(alias):
    global ALIAS
    global START_MOSFET_CURRENT
    global END_MOSFET_CURRENT
    global N_MOSFET_CURRENT_STEPS
    global MOVE_TIME
    if alias == "X":
        # For the Slider Axis of the magnet placement machine
        ALIAS = ord("X")
        START_MOSFET_CURRENT = 50
        END_MOSFET_CURRENT = 350
        N_MOSFET_CURRENT_STEPS = 30 + 1
        MOVE_TIME = 2.0
    elif alias == "Y":
        # For the Disk Tray Axis of the magnet placement machine
        ALIAS = ord("Y")
        START_MOSFET_CURRENT = 20
        END_MOSFET_CURRENT = 200
        N_MOSFET_CURRENT_STEPS = 10 + 1
        MOVE_TIME = 10.0
    elif alias == "Z":
        # For the Up/Down Axis of the magnet placement machine
        ALIAS = ord("Z")
        START_MOSFET_CURRENT = 50
        END_MOSFET_CURRENT = 350
        N_MOSFET_CURRENT_STEPS = 30 + 1
        MOVE_TIME = 2.0
    elif alias == "S":
        # For the Disk Spin Axis of the magnet placement machine
        ALIAS = ord("S")
        START_MOSFET_CURRENT = 20
        END_MOSFET_CURRENT = 200
        N_MOSFET_CURRENT_STEPS = 20 + 1
        MOVE_TIME = 2.0
    else:
        # all other aliases are an error
        print("The alias", alias, "is not recognized.")
        exit(1)

def reset_all_and_fatal_error():
    # reset all motors
    parsed_response = communication.execute_command(255, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(0.5)
    exit(1)


def execute_multimove_command(alias, multi_moves):
    # Define all the constants for this command
    MOVE_DISPLACEMENT_MOTOR_UNITS_PER_ROTATION = 4320000
    MOVE_TIME_MOTOR_UNITS_PER_SECOND = 31250
    move_types = "" # move with velocity will one a 1 bit appended for each move
    multi_moves_converted = []
    for i in range(len(multi_moves)):
        velocity_motor_units = int(multi_moves[i][0])
        time_motor_units = int(multi_moves[i][1] * MOVE_TIME_MOTOR_UNITS_PER_SECOND + 0.5)
        multi_moves_converted.append([velocity_motor_units, time_motor_units])
        move_types += "1"
    multi_moves_str = str(multi_moves_converted)
    move_types_int = int(move_types, 2)

    if len(move_types) != len(multi_moves):
        print('Please specify the same number of bits as the number of moves.')
        exit(1)

    print("number of moves:", len(multi_moves))
    print("Moves types:", move_types)
    print("Move types int:", move_types_int)
    print("Multi-moves string:", multi_moves_str)
    parsed_response = communication.execute_command(alias, "MULTI_MOVE_COMMAND", [len(multi_moves), move_types_int, multi_moves_str], verbose = VERBOSE)


def move_one_cycle_and_collect_position(alias, move_velocity, move_time_s, mosfet_current, start_time = 0):
    # set the MOSFET current
    parsed_response = communication.execute_command(alias, "SET_MAXIMUM_MOTOR_CURRENT", [mosfet_current, mosfet_current], verbose=VERBOSE)
    multi_moves = [ [move_velocity, move_time_s], [-move_velocity, move_time_s], [0, 0.01] ]
    execute_multimove_command(ALIAS, multi_moves)
    delay_start_time = time.time()
    # wait the required amount of time for the moves to happen and iterate many time to read the desired and measured positions
    position_data = []
    while time.time() - delay_start_time < move_time_s * 2 * 1.1:
        parsed_response = communication.execute_command(alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        position_motor_units = parsed_response[0]
        commanded_position_degrees = position_motor_units / ONE_DEGREE_MICROSTEPS
        parsed_response = communication.execute_command(alias, "GET_HALL_SENSOR_POSITION_COMMAND", [], verbose=VERBOSE)
        position_motor_units = parsed_response[0]
        measured_position_degrees = position_motor_units / ONE_DEGREE_MICROSTEPS
        print("The motor is at rotational angle position in degrees (commanded, measured):", commanded_position_degrees, measured_position_degrees)
        current_time_from_start = time.time() - start_time
        position_data.append([current_time_from_start, commanded_position_degrees, measured_position_degrees, mosfet_current])
        time.sleep(0.01)
    return position_data

# The alias that we will test will be read from the command line. If it is not provided, we will exit with a descriptive error message.
if len(sys.argv) != 2:
    print("Usage: python3 characterize_homing.py ALIAS")
    exit(1)
alias = sys.argv[1]
set_global_parameters(alias)

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
for alias in [ALIAS]:
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

# set the PID constants
parsed_response = communication.execute_command(ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)

# enable MOSFETs
parsed_response = communication.execute_command(ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

mosfet_current_step = (END_MOSFET_CURRENT - START_MOSFET_CURRENT) / (N_MOSFET_CURRENT_STEPS - 1)
start_time = time.time()
range_vs_mosfet_current_full_filename = OUTPUT_LOG_FILE_DIRECTORY + "/" + RANGE_VS_MOSFET_CURRENT_FILENAME + ".log"
with open(range_vs_mosfet_current_full_filename, "w") as range_vs_mosfet_current_fh:
    for i in range(N_MOSFET_CURRENT_STEPS):
        # calculate the MOSFET current for this iteration
        mosfet_current = START_MOSFET_CURRENT + i * mosfet_current_step
        # collect the position data and save it to a file as two columns of data
        position_data = move_one_cycle_and_collect_position(alias, move_velocity = 140000000, move_time_s = MOVE_TIME, mosfet_current = mosfet_current, start_time = start_time)
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
        range_vs_mosfet_current_fh.write(f"{mosfet_current} {position_range}\n")


# disable the MOSFETs at the end
parsed_response = communication.execute_command(ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# close the log file
log_fh.close()

time.sleep(0.2)
