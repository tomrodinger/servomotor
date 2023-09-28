
PROTOCOL_VERSION = 19
registered_commands = {}

def register_command(command_id, command_name, description, inputs, response, multiple_responses = False):
    if command_id in registered_commands.keys():
        print("ERROR: The command ID", command_id, "is already registered")
        exit(1)
    if inputs == None:
        inputs = []
    elif isinstance(inputs, tuple):
        inputs = [inputs]
    if response == None:
        response = []
    elif isinstance(response, tuple):
        response = [response]
    registered_commands[command_id] = (command_name, description, inputs, response, multiple_responses)

def get_registered_commands():
    return registered_commands

# These are the various data types that can be used in the inputs and outputs of a command
i8  = 0  # 8-bit signed integer
u8  = 1  # 8-bit unsigned integer
i16 = 2  # 16-bit signed integer
u16 = 3  # 16-bit unsigned integer
i24 = 4  # 24-bit signed integer
u24 = 5  # 24-bit unsigned integer
i32 = 6  # 32-bit signed integer
u32 = 7  # 32-bit unsigned integer
i48 = 8  # 48-bit signed integer
u48 = 9  # 48-bit unsigned integer
i64 = 10 # 64-bit signed integer
u64 = 11 # 64-bit unsigned integer
string8 = 12            # 8 byte long string with null termination if it is shorter than 8 bytes
u24_version_number = 13 # 3 byte version number. the order is patch, minor, major
u32_version_number = 14 # 4 byte version number. the order is development number, patch, minor, major
u64_unique_id = 15      # the unique id of the device
u8_alias = 16           # the alias of the device
crc32 = 17              # 32-bit CRC
u8_alias = 18           # This can hold an ASCII character where the value is represented as an ASCII character if it is in the range 33 to 126, otherwise it is represented as a number up to 255
buf10 = 19              # 10 byte long buffer containing any binary data
list_2d = 200           # a two dimensional list in a Python style format, for example: [[1, 2], [3, 4]]
string_null_term = 201  # this is a string with a variable length and must be null terminated
unknown_data = 202      # this is an unknown data type (work in progress; will be corrected and documented later)
general_data = 203      # this is a general data type whose size is determined by the payload size
success_response = 240  # indicates that the command was received successfully and is being executed. the next command can be transmitted without causing a command overflow situation.

class command_data_types:
    i8  = 0  # 8-bit signed integer
    u8  = 1  # 8-bit unsigned integer
    i16 = 2  # 16-bit signed integer
    u16 = 3  # 16-bit unsigned integer
    i24 = 4  # 24-bit signed integer
    u24 = 5  # 24-bit unsigned integer
    i32 = 6  # 32-bit signed integer
    u32 = 7  # 32-bit unsigned integer
    i48 = 8  # 48-bit signed integer
    u48 = 9  # 48-bit unsigned integer
    i64 = 10 # 64-bit signed integer
    u64 = 11 # 64-bit unsigned integer
    string8 = 12            # 8 byte long string with null termination if it is shorter than 8 bytes
    u24_version_number = 13 # 3 byte version number. the order is patch, minor, major
    u32_version_number = 14 # 4 byte version number. the order is development number, patch, minor, major
    u64_unique_id = 15      # the unique id of the device
    u8_alias = 16           # the alias of the device
    crc32 = 17              # 32-bit CRC
    u8_alias = 18           # This can hold an ASCII character where the value is represented as an ASCII character if it is in the range 33 to 126, otherwise it is represented as a number up to 255
    buf10 = 19              # 10 byte long buffer containing any binary data
    list_2d = 200           # a two dimensional list in a Python style format, for example: [[1, 2], [3, 4]]
    string_null_term = 201  # this is a string with a variable length and must be null terminated
    unknown_data = 202      # this is an unknown data type (work in progress; will be corrected and documented later)
    general_data = 203      # this is data whose length is determined by the payload length
    success_response = 240  # indicates that the command was received successfully and is being executed. the next command can be transmitted without causing a command overflow situation.
    


data_type_dict = {
    i8  : "i8",
    u8  : "u8",
    i16 : "i16",
    u16 : "u16",
    i24 : "i24",
    u24 : "u24",
    i32 : "i32",
    u32 : "u32",
    i48 : "i48",
    u48 : "u48",
    i64 : "i64",
    u64 : "u64",
    string8 : "string8",
    u24_version_number : "u24_version_number",
    u32_version_number : "u32_version_number",
    u64_unique_id : "u64_unique_id",
    u8_alias : "u8_alias",
    crc32 : "crc32",
    buf10 : "buf10",
    list_2d : "list_2d",
    string_null_term : "string_null_term",
    unknown_data : "unknown_data",
    general_data : "general_data",
    success_response : "success_response"
}

data_type_to_size_dict = {
    i8  : 1,
    u8  : 1,
    i16 : 2,
    u16 : 2,
    i24 : 3,
    u24 : 3,
    i32 : 4,
    u32 : 4,
    i48 : 6,
    u48 : 6,
    i64 : 8,
    u64 : 8,
    string8 : 8,
    u24_version_number : 3,
    u32_version_number : 4,
    u8_alias : 1,
    u64_unique_id : 8,
    crc32 : 4,
    buf10 : 10,
    list_2d : None,
    string_null_term : None,
    unknown_data : None,
    general_data : None,
    success_response : None,
}

data_type_min_value_dict = {
    i8  : -128,
    u8  : 0,
    i16 : -32768,
    u16 : 0,
    i24 : -8388608,
    u24 : 0,
    i32 : -2147483648,
    u32 : 0,
    i48 : -549755813888,
    u48 : 0,
    i64 : -9223372036854775808,
    u64 : 0,
    string8 : None,
    u24_version_number : None,
    u32_version_number : None,
    u8_alias : None,
    u64_unique_id : None,
    crc32 : None,
    buf10 : None,
    list_2d : None,
    string_null_term : None,
    unknown_data : None,
    general_data : None,
    success_response : None,
}

data_type_max_value_dict = {
    i8  : 127,
    u8  : 255,
    i16 : 32767,
    u16 : 65535,
    i24 : 8388607,
    u24 : 16777215,
    i32 : 2147483647,
    u32 : 4294967295,
    i48 : 549755813887,
    u48 : 1099511627775,
    i64 : 9223372036854775807,
    u64 : 18446744073709551615,
    string8 : None,
    u24_version_number : None,
    u32_version_number : None,
    u8_alias : None,
    u64_unique_id : None,
    crc32 : None,
    buf10 : None,
    list_2d : None,
    string_null_term : None,
    unknown_data : None,
    general_data : None,
    success_response : None,
}

data_type_is_integer_dict = {
    i8  : True,
    u8  : True,
    i16 : True,
    u16 : True,
    i24 : True,
    u24 : True,
    i32 : True,
    u32 : True,
    i48 : True,
    u48 : True,
    i64 : True,
    u64 : True,
    string8 : False,
    u24_version_number : False,
    u32_version_number : False,
    u64_unique_id : False,
    u8_alias : False, # This is sometimes a character, sometimes a number, but for this purpose we will say it is not an integer  
    crc32 : False,   
    buf10 : False,
    list_2d : False,
    string_null_term : False,
    unknown_data : False,
    general_data : False,
    success_response : False,
}

data_type_description_dict = {
    i8  : "8-bit signed integer",
    u8  : "8-bit unsigned integer",
    i16 : "16-bit signed integer",
    u16 : "16-bit unsigned integer",
    i24 : "24-bit signed integer",
    u24 : "24-bit unsigned integer",
    i32 : "32-bit signed integer",
    u32 : "32-bit unsigned integer",
    i48 : "48-bit signed integer",
    u48 : "48-bit unsigned integer",
    i64 : "64-bit signed integer",
    u64 : "64-bit unsigned integer",
    string8 : "8 byte long string with null termination if it is shorter than 8 bytes",
    u24_version_number : "3 byte version number. the order is patch, minor, major",
    u32_version_number : "4 byte version number. the order is development number, patch, minor, major",
    u64_unique_id : "The unique ID of the device (8-bytes long)",
    u8_alias : "This can hold an ASCII character where the value is represented as an ASCII character if it is in the range 33 to 126, otherwise it is represented as a number from 0 to 255",
    crc32 : "32-bit CRC",
    buf10 : "10 byte long buffer containing any binary data",
    list_2d : "A two dimensional list in a Python style format, for example: [[1, 2], [3, 4]]",
    string_null_term : "This is a string with a variable length and must be null terminated",
    unknown_data : "This is an unknown data type (work in progress; will be corrected and documented later)",
    general_data : "This is a general data type whose size is determined by the payload size",
    success_response : "Indicates that the command was received successfully and is being executed. the next command can be immediately transmitted without causing a command overflow situation."
}

# === Below are the commands that are defined in the protocol ===

command_id   = 0
command_name = "DISABLE_MOSFETS_COMMAND"
description  = "Disable MOSFETS (note that MOSFETs are disabled after initial power on)"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 1
command_name = "ENABLE_MOSFETS_COMMAND"
description  = "Enable MOSFETS"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 2
command_name = "TRAPEZOID_MOVE_COMMAND"
description  = "Move immediately to the given position using the currently set speed (the speed is set by a separate command)"
inputs       = [(i32, "The displacement to travel. Can be positiove or negative."),
                (u32, "The time over which to do the move")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 3
command_name = "SET_MAX_VELOCITY_COMMAND"
description  = "Set maximum velocity (this is not used at this time)"
inputs       = (u32, "Maximum velocity")
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 4
command_name = "SET_POSITION_AND_FINISH_TIME_COMMAND"
description  = "Move to this new given position and finish the move at the given absolution time"
inputs       = [(i32, "Position value"),
                (u32, "Time value")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 5
command_name = "SET_MAX_ACCELERATION_COMMAND"
description  = "Set max acceleration"
inputs       = (u32, "The maximum acceleration")
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 6
command_name = "START_CALIBRATION_COMMAND"
description  = "Start a calibration, which will determine the average values of the hall sensors and will determine if they are working correctly"
inputs       = []
response     = None
register_command(command_id, command_name, description, inputs, response)

command_id   = 7
command_name = "CAPTURE_HALL_SENSOR_DATA_COMMAND"
description  = "Start sending hall sensor data (work in progress; don't send this command)"
inputs       = (u8, "Indicates the type of data to read. Currently 1 to 4 are valid. 0 indicates turning off the reading.")
response     = (unknown_data, "Various data. This is work in progress.")
register_command(command_id, command_name, description, inputs, response)

command_id   = 8
command_name = "RESET_TIME_COMMAND"
description  = "Reset the absolute time to zero (call this first before issuing any movement commands"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 9
command_name = "GET_CURRENT_TIME_COMMAND"
description  = "Get the current absolute time"
inputs       = []
response     = (u48, "The current absolute time")
register_command(command_id, command_name, description, inputs, response)

command_id   = 10
command_name = "TIME_SYNC_COMMAND"
description  = "Send the master time to the motor so that it can sync its own clock (do this 10 times per second)"
inputs       = (u48, "The motor absolute time that the motor should sync to (in microseconds)")
response     = [(i32, "The error in the motor's time compared to the master time"),
                (u16, "The contents of the RCC-ICSCR register (holds the HSICAL and HSITRIM settings)")]
register_command(command_id, command_name, description, inputs, response)

command_id   = 11
command_name = "GET_N_ITEMS_IN_QUEUE_COMMAND"
description  = "Get the number of items currently in the movement queue (if this gets too large, don't queue any more movement commands)"
inputs       = []
response     = (u8, "The number of items in the movement queue. This command will return between 0 and 32. If less than 32, you can add more items to the queue to continue the movements in order without stopping.")
register_command(command_id, command_name, description, inputs, response)

command_id   = 12
command_name = "EMERGENCY_STOP_COMMAND"
description  = "Emergency stop (stop all movement, disable MOSFETS, clear the queue)"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 13
command_name = "ZERO_POSITION_COMMAND"
description  = "Make the current position the position zero (origin)"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 14
command_name = "HOMING_COMMAND"
description  = "Homing (or in other words, move until a crash and then stop immediately)"
inputs       = [(i32, "The maximum distance to move (if a crash does not occur). This can be positive or negative. the sign determines the direction of movement."),
                (u32, "The maximum time to allow for homing. Make sure to give enough time for the motor to cover the maximum distance or the motor may move too fast or throw a fatal error.")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 15
command_name = "GET_HALL_SENSOR_POSITION_COMMAND"
description  = "Get the position as measured by the hall sensors (this should be the actual position of the motor and if everything is ok then it will be about the same as the desired position)"
inputs       = []
response     = (i32, "The current hall sensor position")
register_command(command_id, command_name, description, inputs, response)

command_id   = 16
command_name = "GET_STATUS_COMMAND"
description  = "Get the status"
inputs       = []
response     = [(u8,
'''A series of flags which are 1 bit each. These are:
   Bit 0: In the bootloader (if this flag is set then the other flags below will all be 0)
   Bit 1: MOSFETs are enabled
   Bit 2: Motor is in closed loop mode
   Bit 3: Motor is currently executing the calibration command
   Bit 4: Motor is currently executing a homing command
   Bit 5: Motor is currently executing the procedure to go to closed loop mode
   Bit 6: Motor is currently busy doing a time consuming task and is not ready to take another command
   Bit 7: Not used, set to 0'''),
                (u8, "The fatal error code. If 0 then there is no fatal error. Once a fatal error happens, the motor becomes disabled and cannot do much anymore until reset. You can press the reset button on the motor or you can execute the SYSTEM_RESET_COMMAND to get out of the fatal error state.")]
register_command(command_id, command_name, description, inputs, response)

command_id   = 17
command_name = "GO_TO_CLOSED_LOOP_COMMAND"
description  = "Go to closed loop position control mode"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 18
command_name = "GET_UPDATE_FREQUENCY_COMMAND"
description  = "Get the update frequency (reciprocal of the time step)"
inputs       = []
response     = (u32, "Update frequency in Hz. This is how often the motor executes all calculations for hall sensor position, movement, PID loop, safety, etc.")
register_command(command_id, command_name, description, inputs, response)

command_id   = 19
command_name = "MOVE_WITH_ACCELERATION_COMMAND"
description  = "Move with acceleration"
inputs       = [(i32, "The acceleration (the unit is microsteps per time step per time step * 2^24)"),
                (u32, "The number of time steps to apply this acceleration. Use command 18 to get the frequency of the time steps. After this many time steps, the acceleration will go to zero and velocity will be maintained.")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 20
command_name = "DETECT_DEVICES_COMMAND"
description  = "Detect devices"
inputs       = []
response     = [(u64_unique_id, "A unique ID (unique among all devices manufactured). The response is sent after a random delay of between 0 and 1 seconds."),
                (u8_alias, "The alias of the device that has this unique ID"),
                (crc32, "A CRC32 value for this packet. This is used to verify that the response is correct. However, currently this is hardcoded as 0x04030201")]
register_command(command_id, command_name, description, inputs, response, multiple_responses = True)

command_id   = 21
command_name = "SET_DEVICE_ALIAS_COMMAND"
description  = "Set device alias"
inputs       = [(u64_unique_id, "Unique ID of the target device"),
                (u8_alias, "The alias (short one byte ID) such as X, Y, Z, E, etc. Cannot be R because this is reserved for a response message.")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 22
command_name = "GET_PRODUCT_INFO_COMMAND"
description  = "Get product information"
inputs       = []
response     = [(string8, "The product code / model number (when doing a firmware upgrade, this must match between the firmware file and the target device)"),
                (u8, "A firmware compatibility code (when doing a firmware upgrade, this must match between the firmware file and the target device)"),
                (u24_version_number, "The hardware version stored as 3 bytes. The first byte is the patch version, followed by the minor and major versions."),
                (u32, "The serial number"),
                (u64_unique_id, "The unique ID for the product"),
                (u32, "Not currently used")]
register_command(command_id, command_name, description, inputs, response)

command_id   = 23
command_name = "FIRMWARE_UPGRADE_COMMAND"
description  = "Upgrade one page of flash memory (several of these are needed to do a full firmware upgrade). Documentation to be done later."
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 24
command_name = "GET_PRODUCT_DESCRIPTION_COMMAND"
description  = "Get the product description. Documentation to be done later."
inputs       = []
response     = (string_null_term, "This is a brief description of the product")
register_command(command_id, command_name, description, inputs, response)

command_id   = 25
command_name = "GET_FIRMWARE_VERSION_COMMAND"
description  = "Get the firmware version. Documentation to be done latre."
inputs       = []
response     = (u32_version_number, "The firmware version stored as 4 bytes. The first byte is the development number, then patch version, followed by the minor and major versions.")
register_command(command_id, command_name, description, inputs, response)

command_id   = 26
command_name = "MOVE_WITH_VELOCITY_COMMAND"
description  = "Move with velocity"
inputs       = [(i32, "The velocity (the unit is microsteps per time step * 2^20)"),
                (u32, "The number of time steps to maintain this velocity. Use command 18 to get the frequency of the time steps. After this many time steps, If the queue becomes empty, the motor will maintain the last velocity indefinitely. The velocity will take affect immediately if the queue is empty or will take affect immediately when this queued item is reached.")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 27
command_name = "SYSTEM_RESET_COMMAND"
description  = "System reset / go to the bootloader. The motor will reset immediately and will enter the bootloader. If there is no command sent within a short time, the motor will exit the bootloader and run the application from the beginning."
inputs       = []
response     = None
register_command(command_id, command_name, description, inputs, response)

command_id   = 28
command_name = "SET_MAXIMUM_MOTOR_CURRENT"
description  = "Set the maximum motor current and maximum regeneration current. The values are stored in non-volatile memory and survive a reset."
inputs       = [(u16, "The motor current.  The units are some arbitrary units and not amps. A value of 50 or 100 is suitable."),
                (u16, "The motor regeneration current (while it is braking). This parameter is currently not used for anything.")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 29
command_name = "MULTI_MOVE_COMMAND"
description  = "Multi-move command"
inputs       = [(u8, "Specify how many moves are being communicated in this one shot"),
                (u32, "Each bit specifies if the move is a (bit = 0) MOVE_WITH_ACCELERATION_COMMAND or a (bit = 1) MOVE_WITH_VELOCITY_COMMAND"),
                (list_2d, "A 2D list in Python format (list of lists). Each item in the list is of type [i32, u32] representing a series of move commands. Each move command specifies the acceleration to move at or the velocity to instantly change to (according to the bits baove) and the number of time steps over which this command is to be executed. For example: '[[100, 30000], [-200, 60000]]'. There is a limit of 32 move commands that can be listed in this one multi-move command. Each of the moves takes up one queue spot, so make sure there is enough space in the queue to store all of the commands.")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 30
command_name = "SET_SAFETY_LIMITS_COMMAND"
description  = "Set safety limits (to prevent motion from exceeding set bounds)"
inputs       = [(i32, "The lower limit in microsteps"),
                (i32, "The upper limit in microsteps")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 31
command_name = "PING_COMMAND"
description  = "Send a payload containing any data and the device will respond with the same data back"
inputs       = (buf10, "Any binary data payload to send to the device")
response     = (buf10, "The same data that was sent to the device will be returned if all went well")
register_command(command_id, command_name, description, inputs, response)

command_id   = 32
command_name = "CONTROL_HALL_SENSOR_STATISTICS_COMMAND"
description  = "Turn on or off the gathering of statistics for the hall sensors and reset the statistics"
inputs       = [(u8, "0 = turn off statistics gathering, 1 = reset statistics and turn on gathering")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 33
command_name = "GET_HALL_SENSOR_STATISTICS_COMMAND"
description  = "Read back the statistics gathered from the hall sensors. Useful for checking the hall sensor health and noise in the system."
inputs       = []
response     = [(u16, "The maximum value of hall sensor 1 encoutered since the last statistics reset"),
                (u16, "The maximum value of hall sensor 2 encoutered since the last statistics reset"),
                (u16, "The maximum value of hall sensor 3 encoutered since the last statistics reset"),
                (u16, "The minimum value of hall sensor 1 encoutered since the last statistics reset"),
                (u16, "The minimum value of hall sensor 2 encoutered since the last statistics reset"),
                (u16, "The minimum value of hall sensor 3 encoutered since the last statistics reset"),
                (u64, "The sum of hall sensor 1 values collected since the last statistics reset"),
                (u64, "The sum of hall sensor 2 values collected since the last statistics reset"),
                (u64, "The sum of hall sensor 3 values collected since the last statistics reset"),
                (u32, "The number of times the hall sensors were measured since the last statistics reset")]
register_command(command_id, command_name, description, inputs, response)

command_id   = 34
command_name = "GET_POSITION_COMMAND"
description  = "Get the current desired position (which may not always be the actual position as measured by the hall sensors)"
inputs       = []
response     = (i32, "The current desired position")
register_command(command_id, command_name, description, inputs, response)

command_id   = 35
command_name = "READ_MULTIPURPOSE_BUFFER_COMMAND"
description  = "Read whatever is in the multipurpose buffer (the buffer is used for data generated during calibration, going to closed loop mode, and when capturing hall sensor data)"
inputs       = []
response     = (general_data, "The data in the buffer (the format and length of the data depends on what was put in the buffer)")
register_command(command_id, command_name, description, inputs, response)

command_id   = 36
command_name = "TEST_MODE_COMMAND"
description  = "Set a test mode. Set this to 0 for the default operation."
inputs       = [(u8, "The test mode to use (0 = normal operation).")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 37
command_name = "GET_COMPREHENSIVE_POSITION_COMMAND"
description  = "Get the electrical commutation position, hall sensor position, and external encoder position all in one shot"
inputs       = []
response     = [(i32, "The electrical commutation position"),
                (i32, "The hall sensor position"),
                (i32, "The external encoder position")]
register_command(command_id, command_name, description, inputs, response)

command_id   = 38
command_name = "GET_SUPPLY_VOLTAGE_COMMAND"
description  = "Get the measured voltage of the power supply"
inputs       = []
response     = (u16, "The voltage. Divide this number by 10 to get the actual voltage in volts.")
register_command(command_id, command_name, description, inputs, response)

command_id   = 254
command_name = "ADD_TO_QUEUE_TEST_COMMAND"
description  = "This is used for testing of some calculations that predict of the motion will go out of the set safety limits"
inputs       = []
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

