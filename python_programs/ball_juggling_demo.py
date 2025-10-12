#!/usr/bin/env python3

import sys
import servomotor
import time
import os
import random
import math

THROW_TIME = 0.12
THROW_ANGLE = 15.5
MAX_HOMING_TIME = 3
WIND_UP_ANGLE = 25
WIND_UP_ROTATIONS = WIND_UP_ANGLE / 360
START_POSITION_ROTATIONS = 0.25
WIND_UP_TIME = 0.3
THROW_ROTATIONS = THROW_ANGLE / 360
CATCH_START_ANGLE = 150
PAUSE_AFTER_THROW_TIME = 0.1
TO_CATCH_POSITION_TIME = 0.2
CATCH_ANGLE = 50
CATCH_TIME = 0.2
GO_BACK_TO_START_TIME = 1.0

VERBOSE = 0

THROW_MOSFET_CURRENT = 300
CATCH_CURRENT = 45
THROW_ACCELERATION = 37108500

N_COMMUTATION_STEPS               = 64
N_COMMUTATION_SUB_STEPS           = 1350
ONE_REVOLUTION_ELECTRICAL_CYCLES  = 55
ONE_REVOLUTION_HALL_SENSOR_CYCLES = 25

MAX_RPM                 = 360
MAX_RPS                 = MAX_RPM / 60
MICROSTEPS_PER_ROTATION = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES
ONE_DEGREE_MICROSTEPS = MICROSTEPS_PER_ROTATION / 360
TIME_STEPS_PER_SECOND   = 31250
THROW_VELOCITY          = int(MAX_RPS * MICROSTEPS_PER_ROTATION * (1 << 32) / TIME_STEPS_PER_SECOND / (1 << 12))
THROW_VELOCITY          = 2.96

MM_PER_ROTATION                                   = 20
MAX_ACCELERATION_MM_PER_SECOND_SQUARED            = 10000
MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED     = (MAX_ACCELERATION_MM_PER_SECOND_SQUARED / MM_PER_ROTATION)
MAX_ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED    = (MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED * MICROSTEPS_PER_ROTATION)
MAX_ACCELERATION_MICROSTEPS_PER_TIME_STEP_SQUARED = (MAX_ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED / (TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND))
THROW_ACCELERATION                                = int(MAX_ACCELERATION_MICROSTEPS_PER_TIME_STEP_SQUARED * (1 << 32) / (1 << 8))
THROW_ACCELERATION = 60


OUTPUT_LOG_FILE_DIRECTORY = "./logs/"
OUTPUT_LOG_FILE_NAME = "ball_throwing_demo"

ALIAS = ord('X')

N_PINGS_TO_TEST_COMMUNICATION = 10


def do_a_throw(direction):
    m.set_maximum_motor_current(THROW_MOSFET_CURRENT, THROW_MOSFET_CURRENT, verbose=VERBOSE) # set the MOSFET current

#    time.sleep(0.3)

#    m.go_to_position(-START_POSITION_ROTATIONS * direction, 0.5, verbose=VERBOSE) # wind up just before the throw
    m.go_to_position(-(START_POSITION_ROTATIONS + WIND_UP_ROTATIONS) * direction, 0.5, verbose=VERBOSE) # wind up just before the throw
    time.sleep(0.6)

#    m.trapezoid_move(-WIND_UP_ROTATIONS * direction, 0.4, verbose=VERBOSE) # wind up just before the throw

    time.sleep(1.5)

    m.trapezoid_move((WIND_UP_ROTATIONS + THROW_ROTATIONS) * direction, THROW_TIME, verbose=VERBOSE) # throw the ball
    m.trapezoid_move(0, PAUSE_AFTER_THROW_TIME, verbose=VERBOSE) # pause for a short time without movement to wait for the ball to leave the arm
    m.trapezoid_move(((CATCH_START_ANGLE - THROW_ANGLE) / 360) * direction, TO_CATCH_POSITION_TIME, verbose=VERBOSE) # move the motor very quickly to the catch position
    while 1:   # wait for all these moves to finish, whish will be at the time when the arm is in the catch position
        if m.get_n_queued_items() == 0:
            break
    parsed_response = m.set_maximum_motor_current(CATCH_CURRENT, CATCH_CURRENT, verbose=VERBOSE)   # let's make the motor much more weak so that it has flex and causes the ball to be caught without bouncing out

    t = time.time()
    while 1:
        commanded, sensed, external = m.get_comprehensive_position(verbose=VERBOSE)
        expected_position_after_catch = (full_range / 2) * direction
        error_from_expected_position = abs(sensed - expected_position_after_catch)
        print(f"Error from expected position: {error_from_expected_position}")
        if error_from_expected_position < 0.01:
            print("CAUGHT IT")
            break
        if time.time() - t > 120:
            print("FAILED TO CATCH")
            return -1
        time.sleep(0.01)

    start_current = 15
    parsed_response = m.set_maximum_motor_current(start_current, start_current, verbose=VERBOSE)   # let's make the motor super weak so it can no longer push the ball up against gravity
    print(f"Set the current to {start_current}")

    m.go_to_position((START_POSITION_ROTATIONS + WIND_UP_ROTATIONS) * direction, 1, verbose=VERBOSE) # wind up just before the throw

    n_steps = 40
    for i in range(n_steps):
        final_current = start_current * 3
        current_step = (final_current - start_current) / n_steps
        current_at_this_step = (i + 1) * current_step + start_current
        print(f"Setting current to {current_at_this_step}")
        m.set_maximum_motor_current(current_at_this_step, current_at_this_step, verbose=VERBOSE)   # let's make the motor super weak so it can no longer push the ball up against gravity
        time.sleep(0.01)
    #m.trapezoid_move(CATCH_ANGLE / 360, CATCH_TIME, verbose=VERBOSE) # we will move the motor as the ball is coming down to ease it into the catch

    time.sleep(1)


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


servomotor.open_serial_port()

m = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations", verbose=VERBOSE)

try:
    # do a system reset to make sure we are in a known state
    parsed_response = m.system_reset(verbose=VERBOSE)
    if len(parsed_response) != 0:
        print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SYSTEM_RESET_COMMAND")
        exit(1)
    time.sleep(0.5)

    # let's ping the device many times to make sure it is there and that communication is working flawlessly
    all_devices_responsed = True
    print(f"Testing communication with device with alias {ALIAS} by pinging it {N_PINGS_TO_TEST_COMMUNICATION} times")
    for i in range(N_PINGS_TO_TEST_COMMUNICATION):
        # generate a bytearray with 10 completely random bytes
        random_10_bytes = bytearray(random.getrandbits(8) for _ in range(10))
        parsed_response = m.ping(random_10_bytes, verbose=VERBOSE)
        if len(parsed_response) != 10 or parsed_response != random_10_bytes:
            print("ERROR: The device with alias", ALIAS, "did not respond to the PING_COMMAND")
            all_devices_responsed = False
            break
    if not all_devices_responsed:
        print("ERROR: The device did not respond to the PING_COMMAND")
        exit(1)
    print("The device responded correctly to all the %d pings" % (N_PINGS_TO_TEST_COMMUNICATION))

#    parsed_response = m.set_pid_constants(4000, 2, 1000000, verbose=VERBOSE) # set the PID constants
    parsed_response = m.enable_mosfets(verbose=VERBOSE)                      # enable MOSFETs
    parsed_response = m.set_maximum_motor_current(50, 50, verbose=VERBOSE)   # let's make the motor very weak first, and then do homing
    parsed_response = m.set_pid_constants(3000, 2, 175000, verbose=VERBOSE)  # set the PID constants
    parsed_response = m.go_to_closed_loop(verbose=VERBOSE)                   # go to closed loop position control mode
    time.sleep(0.2)

    m.homing(1, MAX_HOMING_TIME, verbose=VERBOSE)                       # do homing
    # wait until homing is done
    while True:
        status_flags, error_code = m.get_status(verbose=VERBOSE)
        if (status_flags & (1 << 4)) == 0:
            break
        time.sleep(0.5)
    commanded, sensed, external = m.get_comprehensive_position(verbose=VERBOSE)
    print(f"Initial positions: commanded={commanded}, sensed={sensed}")
    rightmost_positon = sensed

    m.homing(-1, MAX_HOMING_TIME, verbose=VERBOSE)                       # do homing
    # wait until homing is done
    while True:
        status_flags, error_code = m.get_status(verbose=VERBOSE)
        if (status_flags & (1 << 4)) == 0:
            break
        time.sleep(0.5)
    commanded, sensed, external = m.get_comprehensive_position(verbose=VERBOSE)
    print(f"Initial positions: commanded={commanded}, sensed={sensed}")
    leftmost_positon = sensed

    # we will now calculate the center position between the two extreme positions
    full_range = rightmost_positon - leftmost_positon
    print(f"The full range of motion is {full_range} rotations:")
    center_position = (leftmost_positon + rightmost_positon) / 2
    print("The center position between the two extreme positions is:", center_position)
    # and travel to that position
    m.go_to_position(center_position, 0.5, verbose=VERBOSE)
    time.sleep(1.2)
    # zero the positon at the center position
    m.zero_position(verbose=VERBOSE)
    m.set_maximum_acceleration(THROW_ACCELERATION, verbose=VERBOSE) # set the maximum acceleration of the throw
    m.set_maximum_velocity(THROW_VELOCITY, verbose=VERBOSE) # set the maximum velocity of the throw

except Exception as e:
    print("ERROR: There was an exception while trying to communicate with the device with alias", ALIAS)
    print("       The exception was:", e)
    exit(0)

while 1:
    if do_a_throw(1) == -1:
        break
    if do_a_throw(-1) == -1:
        break

m.system_reset(verbose=VERBOSE)

time.sleep(0.2)

exit(0)


time.sleep(0.3)



movement_time_device_units = int(31250 * CATCH_TIME)
rotation_motor_units = -int(ONE_DEGREE_MICROSTEPS * CATCH_ANGLE)
parsed_response = communication.execute_command(ALIAS, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)

time.sleep(2)
# we will move the motor back to the start position
movement_time_device_units = int(31250 * GO_BACK_TO_START_TIME)
rotation_motor_units = int(ONE_DEGREE_MICROSTEPS * (CATCH_START_ANGLE + CATCH_ANGLE))
parsed_response = communication.execute_command(ALIAS, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)

time.sleep(GO_BACK_TO_START_TIME + 0.1)

# decrease the MOSFET current so motor does not get hot
parsed_response = communication.execute_command(ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [400, 400], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
    exit(1)

time.sleep(0.1)

exit(0)

motor_position_list = []
hall_sensor_position_list = []
position_deviation_list = []
start_time = time.time()
stop_time = start_time + TIME_FOR_N_ROTATIONS + 0.3
while time.time() < stop_time:
    parsed_response = communication.execute_command(ALIAS, "GET_COMPREHENSIVE_POSITION_COMMAND", [], verbose=VERBOSE)
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
parsed_response = communication.execute_command(ALIAS, "SET_MAXIMUM_MOTOR_CURRENT", [400, 400], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the SET_MAXIMUM_MOTOR_CURRENT command")
    exit(1)

time.sleep(0.1)

# we will return the arm back to the original position (slowly) so that we can load the next ball
TIME_FOR_N_ROTATIONS = 1.0
N_ROTATIONS = -(N_ROTATIONS + N_ROTATIONS_FOR_PREPARE_TO_THROW)
movement_time_device_units = int(31250 * TIME_FOR_N_ROTATIONS)
rotation_motor_units = int(MICROSTEPS_PER_ROTATION * N_ROTATIONS)
parsed_response = communication.execute_command(ALIAS, "TRAPEZOID_MOVE_COMMAND", [rotation_motor_units, movement_time_device_units], verbose=VERBOSE)
if len(parsed_response) != 0:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the TRAPEZOID_MOVE_COMMAND")
    exit(1)

parsed_response = communication.execute_command(ALIAS, "GET_MAX_PID_ERROR_COMMAND", [], verbose=VERBOSE)
if len(parsed_response) != 2:
    print("ERROR: The device with alias", ALIAS, "did not respond correctly to the GET_MAX_PID_ERROR_COMMAND")
    exit(1)

print("The minimum and maximum PID error is:", parsed_response[0], parsed_response[1])

log_fh.close()
