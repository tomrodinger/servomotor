
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
    success_response : "Indicates that the command was received successfully and is being executed. the next command can be immediately transmitted without causing a command overflow situation."
}

# === Below are the commands that are defined in the protocol ===

command_id   = 0
command_name = "ADD_NEW_INTERVAL_COMMAND"
description  = "Add a new coin cell test interval where a specified resistive load is applied (or no load) for a specified amount of time"
inputs       = [(u16, "Interval index of the interval to set up. Index starts at zero. If the index is already in use, the interval will be overwritten."),
                (u8,  "The loads to apply where each bit signifies if the load should be on (1) or off (0) during this interval"),
                (u32, "The duration in milliseconds that the load should be applied for")]
response     = (success_response, "Indicates success")
register_command(command_id, command_name, description, inputs, response)

command_id   = 1
command_name = "SET_INTERVAL_RUN_MODE_COMMAND"
description  = "Controls how the intervals are processed, wether they are run in a loop or only once, etc."
inputs       = [(u8, "0 = Stop running all intervals and delete them. 1 = Don't run the intervals. 2 = Hold the current state of loads indefinitely. 3. Finish the current interval and then stop. 4. Finish all of the intervals and then stop. 5 = Run all intervals in a loop forever.")]
response     = (success_response, "Indicates success")
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
   Bit 5: Not used, set to 0
   Bit 6: Not used, set to 0
   Bit 7: Not used, set to 0'''),
                (u8, "The fatal error code. If 0 then there is no fatal error. Once a fatal error happens, the motor becomes disabled and cannot do much anymore until reset. You can press the reset button on the motor or you can execute the SYSTEM_RESET_COMMAND to get out of the fatal error state.")]
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

command_id   = 27
command_name = "SYSTEM_RESET_COMMAND"
description  = "System reset / go to the bootloader. The motor will reset immediately and will enter the bootloader. If there is no command sent within a short time, the motor will exit the bootloader and run the application from the beginning."
inputs       = []
response     = None
register_command(command_id, command_name, description, inputs, response)

command_id   = 31
command_name = "PING_COMMAND"
description  = "Send a payload containing any data and the device will respond with the same data back"
inputs       = (buf10, "Any binary data payload to send to the device")
response     = (buf10, "The same data that was sent to the device will be returned if all went well")
register_command(command_id, command_name, description, inputs, response)

command_id   = 32
command_name = "GET_DATA_RECORD_COMMAND"
description  = "Read back the data record from one interval. This works on a FIFO basis. You should keep reading the records to prevent overflow of the buffer and data loss."
inputs       = []
response     = [(u16, "The maximum millivolts of the coin cell observed during the interval"),
                (u16, "The minimum millivolts of the coin cell observed during the interval"),
                (u8,  "The interval index number of this interval data record"),
                (u16, "The sequence count. Starts at 0 and increments each time that the full programmed cycle of intervals is completes"),
                (u16, "The number of records available immediately to be read (not including the one being read in this command)"),
                (u8,  "Overflow flag: 0 = no overflow, 1 = overflow of records. You did not read the records fast enough and some record(s) were lost.")]
register_command(command_id, command_name, description, inputs, response)

