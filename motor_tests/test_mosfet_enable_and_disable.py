#!/usr/bin/env python3

import redirect_motor_commands as motor_commands
import redirect_communication as communication
import time
import common_functions

test_description = '''
Here we will test the ENABLE_MOSFETS_COMMAND and DISABLE_MOSFETS_COMMAND. All connected devices will be detected using the 
DETECT_DEVICES_COMMAND first and then all of the devices will undergo the test. First, the ENABLE_MOSFETS_COMMAND will be
issued, the position will be read, then the motor will be moved a set distance, the position will be read again and checked
to see if the motor actually moved. Then the DISABLE_MOSFETS_COMMAND will be issued and the motor will be commanded to 
move a certain distance and then the position will be read again. In this second case, the position is expected not to
change. The test will pass if all the above conditions are met.
'''

VERBOSE = False
DISTANCE_TO_MOVE_MOTOR_UNITS = 1000000
MOVE_DISTANCE_TOLERANCE = 30000
TIME_TO_MOVE_SECONDS = 2


common_functions.print_test_description(test_description)

communication.set_command_data(motor_commands.PROTOCOL_VERSION, motor_commands.registered_commands, motor_commands.command_data_types, motor_commands.data_type_dict,
                               motor_commands.data_type_to_size_dict, motor_commands.data_type_min_value_dict, motor_commands.data_type_max_value_dict,
                               motor_commands.data_type_is_integer_dict, motor_commands.data_type_description_dict)

communication.serial_port = "/dev/tty.usbserial-140"
communication.open_serial_port()

# Let's reset all devices to start from a clean state
print("Resetting all devices")
communication.execute_command(communication.ALL_ALIAS, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

# give time for the devices to reset and boot up
time.sleep(1.5)

print("Detecting devices")
parsed_response = communication.execute_command(communication.ALL_ALIAS, "DETECT_DEVICES_COMMAND", [], verbose=VERBOSE)
alias_list = common_functions.get_alias_list(parsed_response)
common_functions.print_alias_list(alias_list)
if len(alias_list) == 0:
    print("Error: No devices were detected")
    common_functions.test_failed()

for alias in alias_list:
    human_readable_alias = common_functions.get_human_readable_alias(alias)
    print(f"Testing device with alias {human_readable_alias}")

    for action in range(2):
        commanded_move_distance = DISTANCE_TO_MOVE_MOTOR_UNITS
        if action == 0:
            expected_move_distance = commanded_move_distance
            # Enable the MOSFETs
            print("Enabling the MOSFETs")
            parsed_response = communication.execute_command(alias, "ENABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)
        else:
            expected_move_distance = 0
            # Disable the MOSFETs
            print("Disabling the MOSFETs")
            parsed_response = communication.execute_command(alias, "DISABLE_MOSFETS_COMMAND", [], verbose=VERBOSE)

        # Give a short delay to let the motor stabilize its position after energizing it
        time.sleep(0.3)

        # Read the position of the motor
        print("Reading the initial position of the motor")
        parsed_response = communication.execute_command(alias, "GET_HALL_SENSOR_POSITION_COMMAND", [], verbose=VERBOSE)
        initial_position = parsed_response[0]

        # Move the motor a set distance
        time_to_move_motor_units = int(TIME_TO_MOVE_SECONDS * 64000000 / 1024 / 2)
        print(f"Commanding the motor to moving a set distance of {commanded_move_distance} motor units")
        parsed_response = communication.execute_command(alias, "TRAPEZOID_MOVE_COMMAND", [commanded_move_distance, time_to_move_motor_units], verbose=VERBOSE)

        # Give a short delay to let the motor stabilize its position after energizing it
        print(f"Waiting for {TIME_TO_MOVE_SECONDS} seconds for the motor to move")
        time.sleep(TIME_TO_MOVE_SECONDS + 0.2)

        # Read the position of the motor
        print("Reading the final position of the motor")
        parsed_response = communication.execute_command(alias, "GET_HALL_SENSOR_POSITION_COMMAND", [], verbose=VERBOSE)

        # Do calculations and print the summary and test result
        final_position = parsed_response[0]
        actual_move_distance = final_position - initial_position
        error_in_move_distance = actual_move_distance - expected_move_distance
        print(f"Move summary:")
        print(f"   Initial position: {initial_position}")
        print(f"   Final position: {final_position}")
        print(f"   Commanded move distance: {commanded_move_distance}")
        print(f"   Expected move distance: {expected_move_distance}")
        print(f"   Actual move distance: {actual_move_distance}")
        print(f"   Error in the move distance: {error_in_move_distance}")
        print(f"   Tolerance to pass this test: {MOVE_DISTANCE_TOLERANCE}")
        if abs(error_in_move_distance) <= MOVE_DISTANCE_TOLERANCE:
            print("Yes, the motor moved the expected distance")
        else:
            print("ERROR: The motor did not move the expected distance")
            common_functions.test_failed()

common_functions.test_passed()
