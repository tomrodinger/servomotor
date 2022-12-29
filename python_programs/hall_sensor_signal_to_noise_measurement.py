#!/usr/bin/env python3

import time
import struct
import argparse
import motor_commands
import communication


def print_statistics(response):
    #motor_commands.interpret_response(command_id, response)
    #print("The response is:", response)
    # unpack this response into a tuple
    all_statistics = struct.unpack("<HHHHHHQQQI", response)
    hall1_max = all_statistics[0]
    hall2_max = all_statistics[1]
    hall3_max = all_statistics[2]
    hall1_min = all_statistics[3]
    hall2_min = all_statistics[4]
    hall3_min = all_statistics[5]
    hall1_mean = all_statistics[6] / all_statistics[9]
    hall2_mean = all_statistics[7] / all_statistics[9]
    hall3_mean = all_statistics[8] / all_statistics[9]
    hall1_delta = hall1_max - hall1_min
    hall2_delta = hall2_max - hall2_min
    hall3_delta = hall3_max - hall3_min
    hall1_delta_fraction = hall1_delta / hall1_mean
    hall2_delta_fraction = hall2_delta / hall2_mean
    hall3_delta_fraction = hall3_delta / hall3_mean
    print("The statistics are:")
    print("   Hall 1 maximum value: ", hall1_max)
    print("   Hall 2 maximum value: ", hall2_max)
    print("   Hall 3 maximum value: ", hall3_max)
    print("   Hall 1 minimum value: ", hall1_min)
    print("   Hall 2 minimum value: ", hall2_min)
    print("   Hall 3 minimum value: ", hall3_min)
    print("   Hall 1 mean value:    ", hall1_mean)
    print("   Hall 2 mean value:    ", hall2_mean)
    print("   Hall 3 mean value:    ", hall3_mean)
    print("   Hall 1 delta:         ", hall1_delta)
    print("   Hall 2 delta:         ", hall2_delta)
    print("   Hall 3 delta:         ", hall3_delta)
    print("   Hall 1 delta fraction:", hall1_delta_fraction)
    print("   Hall 2 delta fraction:", hall2_delta_fraction)
    print("   Hall 3 delta fraction:", hall3_delta_fraction)


communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-a', '--alias', help='alias of the device to control', default=None)
args = parser.parse_args()

communication.set_standard_options_from_args(args) # This will find out the port to use and the alias of the device and store those in the motor_commands module
communication.open_serial_port()

#command_text = "SET_MAXIMUM_MOTOR_CURRENT"
#print("Running the command:", command_text)
#command_id = motor_commands.get_command_id(command_text)
#assert(command_id != None)
#gathered_inputs = motor_commands.gather_inputs(command_id, [150, 150])
#response = motor_commands.send_command(command_id, gathered_inputs)
#motor_commands.interpret_response(command_id, response)

command_text = "ZERO_POSITION_COMMAND"
print("Running the command:", command_text)
command_id = communication.get_command_id(command_text)
assert(command_id != None)
gathered_inputs = communication.gather_inputs(command_id, [])
response = communication.send_command(command_id, gathered_inputs)
communication.interpret_response(command_id, response)

for j in range(2):
    if j == 0:
        print("Running cycle 1 with MOSFETS enabled during collection of statistics")
    else:
        print("Running cycle 2 with MOSFETS disabled during collection of statistics")
    for i in range(5):
        command_text = "ENABLE_MOSFETS_COMMAND"
        print("Running the command:", command_text)
        command_id = communication.get_command_id(command_text)
        assert(command_id != None)
        gathered_inputs = communication.gather_inputs(command_id, [])
        response = communication.send_command(command_id, gathered_inputs)
        communication.interpret_response(command_id, response)
        time.sleep(0.5)

        command_text = "TRAPEZOID_MOVE_COMMAND"
        print("Running the command:", command_text)
        command_id = communication.get_command_id(command_text)
        assert(command_id != None)
        gathered_inputs = communication.gather_inputs(command_id, [int(645000 / 20), int(32250 / 2)])
        response = communication.send_command(command_id, gathered_inputs)
        communication.interpret_response(command_id, response)
        time.sleep(0.6)

        if j == 1:
            command_text = "DISABLE_MOSFETS_COMMAND"
            print("Running the command:", command_text)
            command_id = communication.get_command_id(command_text)
            assert(command_id != None)
            gathered_inputs = communication.gather_inputs(command_id, [])
            response = communication.send_command(command_id, gathered_inputs)
            communication.interpret_response(command_id, response)
            time.sleep(0.5)

        command_text = "CONTROL_HALL_SENSOR_STATISTICS_COMMAND"
        print("Running the command:", command_text)
        command_id = communication.get_command_id(command_text)
        assert(command_id != None)
        gathered_inputs = communication.gather_inputs(command_id, [1]) # reset the statistics
        response = communication.send_command(command_id, gathered_inputs)
        communication.interpret_response(command_id, response)
        time.sleep(3)

        command_text = "GET_HALL_SENSOR_STATISTICS_COMMAND"
        print("Running the command:", command_text)
        command_id = communication.get_command_id(command_text)
        assert(command_id != None)
        gathered_inputs = communication.gather_inputs(command_id, [])
        response = communication.send_command(command_id, gathered_inputs)
        print_statistics(response)

communication.close_serial_port()
