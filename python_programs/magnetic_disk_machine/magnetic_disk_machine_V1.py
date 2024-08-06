#!/usr/bin/env python3

import sys
import motor_commands
import communication
import time
import os
import random
import math


VERBOSE = False

N_POLES = 50

MOSFET_CURRENT = 500
HIGHEST_MOSFET_CURRENT = 1024
SLIDER_MOSFET_CURRENT = 700
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
EXPECTED_SLIDER_RANGE = 116.7
SLIDER_RANGE_TOLERANCE = 3.0
SLIDER_CENTRE_FINE_TUNE_OFFSET = -0.0
POLE_PLACE_ANGLE = EXPECTED_SLIDER_RANGE / 2 * 0.82 # a certain fraction of the way to the end stop
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

DISK_ROTATE_ALIAS = ord("X")
CENTRING_ALIAS = ord("Y")
SLIDER_ALIAS = ord("Z")
DISK_PULLER_ALIAS = ord("P")

N_PINGS_TO_TEST_COMMUNICATION = 10


def reset_all_and_fatal_error():
    # reset all motors
    parsed_response = communication.execute_command(255, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)
    time.sleep(0.5)
    exit(1)


def do_homing_and_get_position_in_degrees(alias, max_degrees, max_time_s, homing_n_times = 1, relief_degrees = None, homing_tolerance_degrees = None, start_position = None,
                                          bidirectional = False, expected_bidirectional_range = None, zero_after_start_position_reached = False):
    rotation_speed_degrees_per_second = max_degrees / max_time_s
    print("The rotation speed is", rotation_speed_degrees_per_second, "degrees per second")
    homing_positions_degrees = []
    for i in range(homing_n_times):
        # do homing
        rotation_motor_units = -int(ONE_DEGREE_MICROSTEPS * max_degrees)
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
        if relief_degrees is not None:
            # move the motor away from the end stop by the relief amount
            move_time_s = relief_degrees / rotation_speed_degrees_per_second
            rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * relief_degrees)
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
    if start_position is not None:
        print("Zeroing out the motor and moving it to the start position")
        # zero out the centring pusher motor
        parsed_response = communication.execute_command(alias, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)
        # now, move the centring pusher to the start location
        print("Moving the motor to the start position, which is at", start_position, "degrees")
        move_time_s = start_position / rotation_speed_degrees_per_second
        print("The move time is", move_time_s, "seconds")
        movement_time_device_units = int(31250 * move_time_s)
        rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * start_position)
        parsed_response = communication.execute_command(alias, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)
        if zero_after_start_position_reached:
            # read the position and see if we are where we expect to be. if not then we exit
            parsed_response = communication.execute_command(alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
            position_motor_units = parsed_response[0]
            if abs(position_motor_units - rotation_motor_units) > 0.5 * ONE_DEGREE_MICROSTEPS:
                print("ERROR: The motor did not reach the start position")
                reset_all_and_fatal_error()
            # zero out the disk puller motor position
            parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)

    print("The motor has finished the first homing")
    return average_position_degrees


def correctly_position_disk_rotator(disk_rotator_alias, disk_puller_alias):
    # what we need to do here is to disable the disk rotator motor, get its position, then probe in with the disk puller motor
    # either the disk puller won't go in or it will and the disk rotator will move a bit into the correctly aligned position
    # if the disk puller goes in, then we need to zero out the position of the disk rotator, and we are done.
    # if the disk puller does not go in, then we need to enable the disk rotator motor and move the disk rotator a bit and try
    # again to probe
    # first, disable the disk rotator motor
    parsed_response = communication.execute_command(disk_rotator_alias, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    # get the position of the disk rotator
    parsed_response = communication.execute_command(disk_rotator_alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
    disk_rotator_position = parsed_response[0]
    print("The disk rotator position is", disk_rotator_position)
    # now, probe in with the disk puller motor using a fairly small force
    parsed_response = communication.execute_command(disk_puller_alias, "SET_MAXIMUM_MOTOR_CURRENT", [DISK_PULLER_PROBE_MOSFET_CURRENT1, MOSFET_CURRENT], verbose=VERBOSE)
    move_time_s = 1.5
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * DISK_PULLER_PROBE_ANGLE1)
    parsed_response = communication.execute_command(disk_puller_alias, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
    # now see if the disk puller motor went in by checking its position and comparing it to the commanded position
    parsed_response = communication.execute_command(disk_puller_alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
    disk_puller_position = parsed_response[0]
    print("The disk puller position is", disk_puller_position)
    if abs(disk_puller_position - rotation_motor_units) >= DISK_PULLER_PROBE_POSITION_TOLERANCE * ONE_DEGREE_MICROSTEPS:
        print("The disk puller motor did not go in correctly. Probe failed.")
        print(f"We commanded the disk puller to go to position {rotation_motor_units} but it went to {disk_puller_position}")
        print(f"This is an error of {abs(disk_puller_position - rotation_motor_units)} microsteps and our tolerance is {DISK_PULLER_PROBE_POSITION_TOLERANCE * ONE_DEGREE_MICROSTEPS} microsteps")
        reset_all_and_fatal_error()
        return
    print("The disk puller motor went in correctly")
    # now probe in further with the disk puller motor using a medium force
    parsed_response = communication.execute_command(disk_puller_alias, "SET_MAXIMUM_MOTOR_CURRENT", [DISK_PULLER_PROBE_MOSFET_CURRENT2, MOSFET_CURRENT], verbose=VERBOSE)
    move_time_s = 1.0
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * DISK_PULLER_PROBE_ANGLE2)
    parsed_response = communication.execute_command(disk_puller_alias, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
    # now see if the disk puller motor went in by checking its position and comparing it to the commanded position
    parsed_response = communication.execute_command(disk_puller_alias, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
    disk_puller_position = parsed_response[0]
    print("The disk puller position is", disk_puller_position)
    if abs(disk_puller_position - rotation_motor_units) >= DISK_PULLER_PROBE_POSITION_TOLERANCE * ONE_DEGREE_MICROSTEPS:
        print("The disk puller motor did not go in correctly. Probe failed.")
        print(f"We commanded the disk puller to go to position {rotation_motor_units} but it went to {disk_puller_position}")
        print(f"This is an error of {abs(disk_puller_position - rotation_motor_units)} microsteps and our tolerance is {DISK_PULLER_PROBE_POSITION_TOLERANCE * ONE_DEGREE_MICROSTEPS} microsteps")
        reset_all_and_fatal_error()
        return
    print("The disk puller motor went in correctly the second time too")
    parsed_response = communication.execute_command(disk_puller_alias, "GO_TO_POSITION_COMMAND", [0, movement_time_device_units], verbose=VERBOSE)
    # zero out the disk rotator position
    parsed_response = communication.execute_command(disk_rotator_alias, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)
    # enable the disk rotator motor
    parsed_response = communication.execute_command(disk_rotator_alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

    time.sleep(move_time_s * 1.1)

def disable_mosfets_and_read_positions():
    # disable the MOSFETs in all three motors
    parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(CENTRING_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(SLIDER_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
    while 1:
        # get the positions of all three motors
        parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        disk_rotator_position = parsed_response[0]
        parsed_response = communication.execute_command(CENTRING_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        centring_position = parsed_response[0]
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        slider_position = parsed_response[0]
        parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "GET_POSITION_COMMAND", [], verbose=VERBOSE)
        disk_puller_position = parsed_response[0]
        disk_rotor_position_angle = disk_rotator_position / ONE_DEGREE_MICROSTEPS
        centring_position_angle = centring_position / ONE_DEGREE_MICROSTEPS
        slider_position_angle = slider_position / ONE_DEGREE_MICROSTEPS
        disk_puller_position_angle = disk_puller_position / ONE_DEGREE_MICROSTEPS
        # print out all these positions in the units of shaft angle with three decimal places
        print(f"Positions: disk rotator: {disk_rotor_position_angle:.3f}, centring: {centring_position_angle:.3f}, slider: {slider_position_angle:.3f}, disk puller: {disk_puller_position_angle:.3f}")
        time.sleep(0.1)

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
for alias in [DISK_ROTATE_ALIAS, CENTRING_ALIAS, SLIDER_ALIAS]:
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
parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)
parsed_response = communication.execute_command(CENTRING_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "SET_PID_CONSTANTS_COMMAND", [PID_P, PID_I, PID_D], verbose=VERBOSE)

# enable MOSFETs in all three motors
parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(CENTRING_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

# go to close loop position control mode in all three motors
parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(CENTRING_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "GO_TO_CLOSED_LOOP_COMMAND", [], verbose=VERBOSE)

# set the MOSFET current
parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(CENTRING_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(SLIDER_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [SLIDER_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [DISK_PULLER_MOSFET_CURRENT, MOSFET_CURRENT], verbose=VERBOSE)
time.sleep(0.2) # allow the motor to stabilize it's position

# do the homing of the centring pusher and get the position in degrees
degrees_at_first_stop = do_homing_and_get_position_in_degrees(CENTRING_ALIAS, 180, 2, homing_n_times = 3, relief_degrees = 5,
                                                              homing_tolerance_degrees = 1, start_position = CENTRING_PUSHER_REST_ANGLE)


# do the homing of the disk puller
degrees_at_first_stop = do_homing_and_get_position_in_degrees(DISK_PULLER_ALIAS, 360 * 4, 2 * 4, homing_n_times = 3, relief_degrees = 5,
                                                              homing_tolerance_degrees = 6, start_position = DISK_PULLER_START_ANGLE,
                                                              zero_after_start_position_reached = True)

# do the homing of the slider and get the position in degrees
degrees_at_first_stop = do_homing_and_get_position_in_degrees(SLIDER_ALIAS, 180, 4)
print("The motor has finished homing in the counter-clockwise direction")
print("The motor is at rotational position in degrees:", degrees_at_first_stop)
# do the homing again and find the other stop
degrees_at_other_stop = do_homing_and_get_position_in_degrees(SLIDER_ALIAS, -180, 4)
print("The motor has finished homing in the clockwise direction")
print("The motor is at rotational position in degrees:", degrees_at_other_stop)
slider_range_in_degrees = degrees_at_other_stop - degrees_at_first_stop
print("The stop to stop distance in degrees is:", slider_range_in_degrees)
# check that the stop to stop distance is what we expect
if abs(slider_range_in_degrees - EXPECTED_SLIDER_RANGE) > SLIDER_RANGE_TOLERANCE:
    print("ERROR: The stop to stop distance is not what we expect")
    exit(1)
# now, move the slider motor to the centre
move_time_s = 0.5
movement_time_device_units = int(31250 * move_time_s)
rotation_motor_units = int(((degrees_at_first_stop + degrees_at_other_stop) / 2 + SLIDER_CENTRE_FINE_TUNE_OFFSET) * ONE_DEGREE_MICROSTEPS)
parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
time.sleep(move_time_s + 0.2)
# zero out the slider motor position
parsed_response = communication.execute_command(SLIDER_ALIAS, "ZERO_POSITION_COMMAND", [], verbose=VERBOSE)

correctly_position_disk_rotator(DISK_ROTATE_ALIAS, DISK_PULLER_ALIAS)

# set the MOSFET current of the centring pusher motor to the highest level
parsed_response = communication.execute_command(CENTRING_ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [HIGHEST_MOSFET_CURRENT, HIGHEST_MOSFET_CURRENT], verbose=VERBOSE)

# disable the mosfets of the centring pusher motor. we will keep them disabled unless we are centering a disk
parsed_response = communication.execute_command(CENTRING_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)


while 1:
    # ask the user to insert the metal disk
    input("Please insert the metal disk and press Enter when you are ready to continue...")

    # enable the mosfets of the centring pusher motor
    parsed_response = communication.execute_command(CENTRING_ALIAS, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

    # rotate a few revolutions. we will be centering the metal disk now
    move_time_s = CENTERING_TIME
    rotation_motor_units = -int(MICROSTEPS_PER_ROTATION * CENTERING_ROTATIONS)
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    # while the disk is rotating, quickly move the centring pusher intot he disk to begin centering it
    move_time_s = INITIAL_DISK_PUSHER_TIME
    rotation_motor_units = -int(MICROSTEPS_PER_ROTATION * 3)
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * CENTRING_PUSHER_START_ANGLE)
    parsed_response = communication.execute_command(CENTRING_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
    # and then move the centring pusher away slowly so the disk can settle in a centred position
    move_time_s = CENTERING_TIME - INITIAL_DISK_PUSHER_TIME
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * CENTRING_PUSHER_END_ANGLE)
    parsed_response = communication.execute_command(CENTRING_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
    # let the disk rotator go back to the start position quickly
    move_time_s = 3.0
    movement_time_device_units = int(31250 * move_time_s)
    parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "GO_TO_POSITION_COMMAND", [0, movement_time_device_units], verbose=VERBOSE)

    # now, move the centring pusher to the resting location
    move_time_s = 0.2
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = 0
    parsed_response = communication.execute_command(CENTRING_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)
            
    # disable the mosfets of the centring pusher motor
    parsed_response = communication.execute_command(CENTRING_ALIAS, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

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
        pole_touch_disk_angle = POLE_PLACE_ANGLE * POLE_MOVE_PERCENTAGE
        pole_backup_angle = POLE_PLACE_ANGLE * POLE_MOVE_PERCENTAGE_BACKUP
        pole_place_angle = POLE_PLACE_ANGLE

        if i == 0:
            # we will push the magnet to where the disk starts, which is a certain percentage of the way to its final resting place
            parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_touch_disk_angle, 31250 * move_time_s], verbose=VERBOSE)
            time.sleep(move_time_s * 1.1)
            # and back up a little to let the pole get pulled down to the magnet that is holding it parallel to the disk
            parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_backup_angle, 31250 * move_time_s], verbose=VERBOSE)
            time.sleep(move_time_s * 1.1)
            # now, move the motor to place that pole onto the disk
            parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_place_angle, 31250 * move_time_s], verbose=VERBOSE)
            time.sleep(move_time_s * 1.1)

        if i % 2 == 0:
            pole_touch_disk_angle = -pole_touch_disk_angle
            pole_backup_angle = -pole_backup_angle
            pole_place_angle = -pole_place_angle

        # we will push the magnet to where the disk starts, which is a certain percentage of the way to its final resting place
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_touch_disk_angle, 31250 * move_time_s], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)
        # and back up a little to let the pole get pulled down to the magnet that is holding it parallel to the disk
        parsed_response = communication.execute_command(SLIDER_ALIAS, "GO_TO_POSITION_COMMAND", [ONE_DEGREE_MICROSTEPS * pole_backup_angle, 31250 * move_time_s], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)
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
            cumulative_disk_angle += -ALTERNATE_POLE_ADJUSTMENT_ANGLE + 360.0 / N_POLES * 2
        else:
            cumulative_disk_angle += ALTERNATE_POLE_ADJUSTMENT_ANGLE
        print("Cumulative angle is:", cumulative_disk_angle)
        cumulative_position_motor_units = int(ONE_DEGREE_MICROSTEPS * cumulative_disk_angle + 0.5)
        print("Cumulative position in motor units is:", cumulative_position_motor_units)
        parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "GO_TO_POSITION_COMMAND", [cumulative_position_motor_units, movement_time_device_units], verbose=VERBOSE)
        time.sleep(move_time_s * 1.1)

    # return the disk rotator to the start position where it can be picked up by the disk puller
    move_time_s = 0.3
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * 0)
    parsed_response = communication.execute_command(DISK_ROTATE_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)

    # now reach in and hook onto the disk
    move_time_s = 3.0
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * DISK_PULLER_PICKUP_ANGLE)
    parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)

    # and extract the disk
    move_time_s = 5.0
    movement_time_device_units = int(31250 * move_time_s)
    rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * DISK_PULLER_DROP_OFF_ANGLE)
    parsed_response = communication.execute_command(DISK_PULLER_ALIAS, "GO_TO_POSITION_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
    time.sleep(move_time_s * 1.1)

time.sleep(0.2)

log_fh.close()
