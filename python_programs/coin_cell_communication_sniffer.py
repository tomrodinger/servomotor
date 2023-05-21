#!/usr/bin/env python3

import sys
import argparse
import coin_cell_tester_commands
import communication

communication.set_command_data(coin_cell_tester_commands.PROTOCOL_VERSION, coin_cell_tester_commands.registered_commands, coin_cell_tester_commands.command_data_types, coin_cell_tester_commands.data_type_dict,
                               coin_cell_tester_commands.data_type_to_size_dict, coin_cell_tester_commands.data_type_min_value_dict, coin_cell_tester_commands.data_type_max_value_dict,
                               coin_cell_tester_commands.data_type_is_integer_dict, coin_cell_tester_commands.data_type_description_dict)

# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
args = parser.parse_args()

communication.print_protocol_version()
communication.print_data_type_descriptions()
communication.print_registered_commands()


communication.set_serial_port_from_args(args) # This will find out the port to use and the alias of the device and store those in the communication module
communication.open_serial_port(timeout=0.05)
while True:
    alias, command_id, payload = communication.sniffer()
    if alias == None or command_id == None or payload == None:
        continue
    if alias >= 33 and alias <= 126:
        alias = chr(alias)
    else:
        alias = str(alias)
    if alias == "R":
        if ((command_id == 0) and (len(payload) != 0)) or ((command_id != 0) and (len(payload) == 0)) or (command_id > 1):
            print("ERROR: The response is not valid: Command ID: %d, Payload: %s" % (command_id, payload))
        else:
            print("Response: Payload: %s" % (payload))
    else:
        if command_id not in communication.registered_commands:
            command_id_string = "UNKNOWN COMMAND"
        else:
            command_id_string = str(communication.registered_commands[command_id][0])
        print("Alias: %s, Command ID: %d (%s), Payload: %s" % (alias, command_id, command_id_string, payload))
#    communication.interpret_response(command_id, payload)
#communication.close_serial_port()
