#!/usr/bin/env python3

import random
import struct
import argparse
import servomotor


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Add some random moves to the queue to test the calculations of the safety limits')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
parser.add_argument('n_moves', help='The number of moves to add to the queue (this should be less than the queue size)')
args = parser.parse_args()

serial_port = args.port
alias = args.alias
if alias == None:
    print("Error: you must specify an alias with the -a option. Run this command with -h to get more detailed help.")
    exit(1)
elif alias == "255":
    alias = 255
elif len(alias) == 1:
    alias = ord(alias)
else:
    print("Error: the alias nust be just one character, not:", alias)
    print("The alias can also be 255 (which means no alias)")
    exit(1)

n_moves = int(args.n_moves)

ADD_TO_QUEUE_TEST_COMMAND = 253

MOVE_WITH_ACCELERATION = 0
MOVE_WITH_VELOCITY = 1

def send_add_to_queue_test_command(ser, alias, parameter, time_steps, movement_type):
    command = bytearray([alias, ADD_TO_QUEUE_TEST_COMMAND, 9])
    command = command + parameter.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little")
    command = command + movement_type.to_bytes(1, "little")
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)

def get_response(ser):
    response = ser.read(3)
    if len(response) != 3:
        print("Error: didn't receive enough bytes in the response. Got %d bytes" % (len(response)))
        exit(1)
    print("Received a response: ", response)
    if response[0] != servomotor.RESPONSE_CHARACTER:
        print(f"Error: the first is not the expected {servomotor.RESPONSE_CHARACTER}")
        exit(1)
    payload_size = response[2]
    if payload_size == 0:
        if response[1] != 0:
            print("Error: the second byte should be 0 if there is no payload")
            exit(1)
    else:
        if response[1] != 1:
            print("Error: the second byte should be 1 if there is a payload")
            exit(1)

    payload = ser.read(payload_size)
    if len(payload) != payload_size:
        print("Error: didn't receive the right length payload")
        print("Received this payload: ", payload)
        exit(1)
    print("Got a valid payload:", payload)
    return payload

def print_data(data):
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))


TIME_STEPS_PER_SECOND = 31250
mm_per_rotation = 20
microsteps_per_rotation = 360 * 256 * 7
microsteps_per_mm = microsteps_per_rotation / mm_per_rotation
max_rpm = 1980
max_rps = max_rpm / 60
max_microsteps_per_second = max_rps * microsteps_per_rotation
max_velocity_microsteps_per_time_step = max_microsteps_per_second / TIME_STEPS_PER_SECOND
max_velocity_mm_per_second = max_rps * mm_per_rotation
MAX_ACCELERATION_MM_PER_SECOND_SQUARED = 10000
max_acceleration_rotations_per_second_squared = MAX_ACCELERATION_MM_PER_SECOND_SQUARED / mm_per_rotation
max_acceleration_microsteps_per_second_squared = max_acceleration_rotations_per_second_squared * microsteps_per_rotation
max_acceleration_microsteps_per_time_step_squared = max_acceleration_microsteps_per_second_squared / (TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND)
max_acceleration = max_acceleration_microsteps_per_time_step_squared
acceleration_conversion_factor = max_acceleration_microsteps_per_time_step_squared / MAX_ACCELERATION_MM_PER_SECOND_SQUARED
velocity_conversion_factor = max_velocity_microsteps_per_time_step / max_velocity_mm_per_second
print("The maximum microsteps per time step (velocity) is:", max_velocity_microsteps_per_time_step)
print("The maximum microsteps per time step squared (max acceleration) is:", max_acceleration_microsteps_per_time_step_squared)
print("The acceleration conversion factor is:", acceleration_conversion_factor)
print("The velocity conversion factor is:", velocity_conversion_factor)

max_velocity_microsteps_per_time_step = int(max_velocity_microsteps_per_time_step)
velocity = 0
position = 0
previous_velocity = velocity
DATA_WRITE_INTERVAL = 1000
data_write_counter = 0

max_discrepancy1 = 0
max_discrepancy2 = 0
max_discrepancy3 = 0
max_discrepancy4 = 0
max_discrepancy5 = 0

TIME_AT_TURN_POINT_SHIFT = 4

def do_a_move(acceleration, time_steps):
    global velocity
    global position
    global previous_velocity
    global fh
    global data_write_counter
    global max_discrepancy1
    global max_discrepancy2
    global max_discrepancy3
    global max_discrepancy4
    global max_discrepancy5

    acceleration = (acceleration << 8) # this is what the motor does when it receives the command to accelerate

    print("Acceleration:", acceleration)
    print("Time steps:", time_steps)
    print("Starting velocity:", velocity // (2**32))
    print("Starting position:", position // (2**32))

    predicted_final_velocity = velocity + acceleration * time_steps
    predicted_final_position = position + velocity * time_steps + acceleration * ((time_steps * (time_steps + 1)) >> 1)
    print("Predicted final velocity:", predicted_final_velocity // (2**32))
    print("Predicted final position:", predicted_final_position // (2**32))

    if (predicted_final_velocity > (max_velocity_microsteps_per_time_step << 32)) or (predicted_final_velocity < -(max_velocity_microsteps_per_time_step << 32)):
        print("Predicted final velocity is too high, aborting")
        return -1

#    time_at_turn_point = (-velocity - 0.5 * acceleration) / acceleration
    if acceleration == 0:
        time_at_turn_point = -1
        time_at_turn_point_shifted = 0
        relative_position_at_turn_point2 = 0
    else:
        time_at_turn_point = int(-velocity / acceleration)
        time_at_turn_point_shifted = -int((velocity << TIME_AT_TURN_POINT_SHIFT) / acceleration)
        print("Time step at turn point:", time_at_turn_point_shifted >> TIME_AT_TURN_POINT_SHIFT)
    if(time_at_turn_point <= 0) or (time_at_turn_point >= time_steps):
        print("No turn point in this move")
        relative_position_at_turn_point1 = None
        relative_position_at_turn_point2 = 0
        relative_position_at_turn_point3 = None
        relative_position_at_turn_point4 = None
        relative_position_at_turn_point5 = None
        position_at_turn_point1 = None
        position_at_turn_point2 = None
        position_at_turn_point3 = None
        position_at_turn_point4 = None
        position_at_turn_point5 = None
        print("Initial and final positions: %f, %f" % (position, predicted_final_position))
    else:
        relative_position_at_turn_point1 = velocity * time_at_turn_point + acceleration * ((time_at_turn_point * (time_at_turn_point + 1)) >> 1)
        relative_position_at_turn_point2 = int(velocity * (time_at_turn_point_shifted - (1 << TIME_AT_TURN_POINT_SHIFT))) >> (TIME_AT_TURN_POINT_SHIFT + 1)
        relative_position_at_turn_point3 = velocity * (time_at_turn_point - 1) / 2
        half_velocity = velocity >> 1
        relative_position_at_turn_point4 = half_velocity * time_at_turn_point - half_velocity
        relative_position_at_turn_point5 = half_velocity * (time_at_turn_point - 1)
        
        print("Relative position at turn point 1:", relative_position_at_turn_point1 // (2**32))
        print("Relative position at turn point 2:", relative_position_at_turn_point2 // (2**32))
        print("Relative position at turn point 3:", relative_position_at_turn_point3 // (2**32))
        print("Relative position at turn point 4:", relative_position_at_turn_point4 // (2**32))
        print("Relative position at turn point 5:", relative_position_at_turn_point5 // (2**32))
        position_at_turn_point1 = position + relative_position_at_turn_point1
        position_at_turn_point2 = position + relative_position_at_turn_point2
        position_at_turn_point3 = position + relative_position_at_turn_point3
        position_at_turn_point4 = position + relative_position_at_turn_point4
        position_at_turn_point5 = position + relative_position_at_turn_point5
        print("Position at turn point 1:", position_at_turn_point1 // (2**32))
        print("Position at turn point 2:", position_at_turn_point2 // (2**32))
        print("Position at turn point 3:", position_at_turn_point3 // (2**32))
        print("Position at turn point 4:", position_at_turn_point4 // (2**32))
        print("Position at turn point 5:", position_at_turn_point5 // (2**32))
        print("Initial and final positions: %f, %f   The turn point should be outside this range: %f" % (position // (2**32), predicted_final_position // (2**32), (position + relative_position_at_turn_point1) // (2**32)))
        fraction = (relative_position_at_turn_point1) / (predicted_final_position - position)
        print("The fraction is: %f" % fraction)
        if fraction > 0.0 and fraction < 1.0:
            print("The turn point is in an unexpected position. The math is wrong.")
            exit(1)

    if velocity == 0:
        if acceleration > 0:
            previous_velocity = 0
        else:
            previous_velocity = -1
    found_turn_point = False
    for i in range(time_steps):
        velocity = velocity + acceleration
        if velocity > (int(max_velocity_microsteps_per_time_step) << 32):
            velocity = (int(max_velocity_microsteps_per_time_step) << 32)
        elif velocity < -(int(max_velocity_microsteps_per_time_step) << 32):
            velocity = -(int(max_velocity_microsteps_per_time_step) << 32)
        position = position + velocity
        position_shifted = position >> 32
        velocity_shifted = velocity >> 32

        if position_shifted  < -2000000000 or position_shifted > 2000000000:
            print("Position is outside the range of a 32 bit integer")
            exit(1)

        if (velocity < 0) != (previous_velocity < 0):
            found_turn_point = True
            print("Detected a turn point:", position)
            if position_at_turn_point1 is None:
                print("Error: we detected a turn point in reality but not in the math.")
                exit(1)
            discrepency1 = abs(position_at_turn_point1 - position) / 2**32
            discrepency2 = abs(position_at_turn_point2 - position) / 2**32
            discrepency3 = abs(position_at_turn_point3 - position) / 2**32
            discrepency4 = abs(position_at_turn_point4 - position) / 2**32
            discrepency5 = abs(position_at_turn_point5 - position) / 2**32
            print("Turn point discrepency 1:", discrepency1)
            print("Turn point discrepency 2:", discrepency2)
            print("Turn point discrepency 3:", discrepency3)
            print("Turn point discrepency 4:", discrepency4)
            print("Turn point discrepency 5:", discrepency5)
            if discrepency1 > max_discrepancy1:
                max_discrepancy1 = discrepency1
            if discrepency2 > max_discrepancy2:
                max_discrepancy2 = discrepency2
            if discrepency3 > max_discrepancy3:
                max_discrepancy3 = discrepency3
            if discrepency4 > max_discrepancy4:
                max_discrepancy4 = discrepency4
            if discrepency5 > max_discrepancy5:
                max_discrepancy5 = discrepency5
        previous_velocity = velocity

    if (found_turn_point == False) and (position_at_turn_point1 != None):
        print("We did not detect a turn point in reality but we did in the math.")
        exit(1)

    print("Max discrepancies: %f, %f, %f, %f, %f" % (max_discrepancy1, max_discrepancy2, max_discrepancy3, max_discrepancy4, max_discrepancy5))

    print("Final velocity:", velocity // (2**32))
    if(predicted_final_velocity == velocity):
        print("Predicted final velocity matches final velocity: %d %d" % (predicted_final_velocity, velocity))
    else:
        print("Predicted final velocity does not match final velocity")
        exit(1)
    print("Final position:", position // (2**32))
    if(predicted_final_position == position):
        print("Predicted final position matches final position")
    else:
        print("Predicted final position does not match final position")
        print("The predicted final position:", predicted_final_position)
        print("The final position:", position)
        exit(1)

    return (predicted_final_velocity >> 32, predicted_final_position >> 32, time_at_turn_point_shifted >> TIME_AT_TURN_POINT_SHIFT, relative_position_at_turn_point2 >> 32)

def convert_to_motor_units(acceleration_mm_per_second_squared, time_seconds):
    acceleration_microsteps_per_time_step_squared = acceleration_mm_per_second_squared * acceleration_conversion_factor
    acceleration_to_send = int(acceleration_microsteps_per_time_step_squared * (1 << 24) + 0.5)
    time_steps = int(time_seconds * TIME_STEPS_PER_SECOND + 0.5)
    print("The microsteps per time step squared (acceleration) is: %f   The acceleration to send: %d   The time steps is: %d" % (acceleration_microsteps_per_time_step_squared, acceleration_to_send, time_steps))
    return acceleration_to_send, time_steps


ser = servomotor.serial_functions.open_serial_port(serial_port, 230400, 0.5)

for i in range(n_moves):
    while 1:
        if (position >> 32) >= 1900000000:
            acceleration = -MAX_ACCELERATION_MM_PER_SECOND_SQUARED
        elif (position >> 32) <= -1900000000:
            acceleration = MAX_ACCELERATION_MM_PER_SECOND_SQUARED
        else:
            acceleration = random.randint(-(MAX_ACCELERATION_MM_PER_SECOND_SQUARED - 1), (MAX_ACCELERATION_MM_PER_SECOND_SQUARED - 1))
        # set the time_steps to a random floating point number between 0.01 and 3.0
        time_seconds = random.uniform(0.01, 3.0)
        # before doing the move, convert to the right units for the motor
        acceleration_to_send, time_steps = convert_to_motor_units(acceleration, time_seconds)
        # now do the move
        ret = do_a_move(acceleration_to_send, time_steps)
        if ret == -1:
            continue
        break
    (predicted_final_velocity, predicted_final_position, time_step_at_turn_point, relative_position_at_turn_point) = ret

    send_add_to_queue_test_command(ser, alias, acceleration_to_send, time_steps, MOVE_WITH_ACCELERATION)
    payload = get_response(ser)
    if (payload == None) or (len(payload) != 20):
        print("Received an invalid response")
        exit(1)
    print("Command succeeded")
    # now unpack the response. the equivalent C values in there are:
    #   int32_t predicted_final_velocity;
	#   int32_t predicted_final_position;
	#   uint32_t time_step_at_turn_point;
	#   int32_t relative_position_at_turn_point;
    (predicted_final_velocity_from_motor, predicted_final_position_from_motor, time_step_at_turn_point_from_motor, relative_position_at_turn_point_from_motor) = struct.unpack("<iqii", payload)

    print("")
    print("Predicted final velocity:", predicted_final_velocity)
    print("Predicted final position:", predicted_final_position)
    print("Time step at turn point:", time_step_at_turn_point)
    print("Relative position at turn point:", relative_position_at_turn_point)
    print("")
    print("Predicted final velocity_from_motor:", predicted_final_velocity_from_motor)
    print("Predicted final position_from_motor:", predicted_final_position_from_motor)
    print("Time step at turn point_from_motor:", time_step_at_turn_point_from_motor)
    print("Relative position at turn point_from_motor:", relative_position_at_turn_point_from_motor)
    print("")

    if (predicted_final_velocity != predicted_final_velocity_from_motor):
        print("Predicted final velocity does not match predicted final velocity from motor")
        exit(1)
    if (predicted_final_position != predicted_final_position_from_motor):
        print("Predicted final position does not match predicted final position from motor")
        exit(1)
    if (time_step_at_turn_point != time_step_at_turn_point_from_motor):
        print("Time step at turn point does not match time step at turn point from motor")
        exit(1)
    if(relative_position_at_turn_point != relative_position_at_turn_point_from_motor):
        print("Relative position at turn point does not match relative position at turn point from motor")
        exit(1)

ser.close()

print("PASSED")
