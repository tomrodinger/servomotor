# Guide to Updating main.c Files with Structured Data Formats

This document provides detailed instructions for updating the `process_packet()` function in various firmware `main.c` files to use structured formats for returning data and to properly invoke the `rs485_finalize_and_transmit_packet` function for transmission or for returning a no error response
with the `transmit_no_error_response` function.

## Overview of Changes

The primary goal is to modify command handling in the `process_packet()` function to:

1. Replace direct `rs485_transmit()` calls with structured data formats
2. Use the `rs485_finalize_and_transmit_packet()` function for packet transmission
3. Use the `transmit_no_error_response` function to transmit a packet that has no returned data
4. Ensure consistent structure across all command responses
5. Include proper CRC32 handling in the packet structure

## Commands That Need Updating

The following commands should be updated to use structured data formats:

- `GET_CURRENT_TIME_COMMAND`
- `TIME_SYNC_COMMAND`
- `GET_UPDATE_FREQUENCY_COMMAND`
- `GET_PRODUCT_INFO_COMMAND`
- `GET_PRODUCT_DESCRIPTION_COMMAND`
- `GET_FIRMWARE_VERSION_COMMAND`
- `PING_COMMAND`
- `GET_SUPPLY_VOLTAGE_COMMAND`
- likely several others

## Step-by-Step Update Process

For each command in the `process_packet()` function, follow these steps:

### 1. Identify the Current Implementation

Look for the command case in the switch statement and identify:
- What data is being returned
- How the data is currently being transmitted
- Any special handling for the data

### 2. Create a Structured Format

For each command, create a packed structure that includes:
- A header field (3 bytes) that will be filled by `rs485_finalize_and_transmit_packet()`
- Fields for all the data that needs to be transmitted
- A CRC32 field (4 bytes) that will be filled by `rs485_finalize_and_transmit_packet()`

Example structure format:
```c
struct __attribute__((__packed__)) {
    uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
    // Data fields specific to this command
    uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
} command_reply;
```

### 3. Populate the Structure

Fill the structure with the appropriate data:
- Set all data fields with the values that need to be transmitted
- Do not set the header or CRC32 fields (these will be handled by `rs485_finalize_and_transmit_packet()`)

### 4. Call rs485_finalize_and_transmit_packet()

Replace the `rs485_transmit()` calls with a single call to `rs485_finalize_and_transmit_packet()`:
```c
rs485_finalize_and_transmit_packet(&command_reply, sizeof(command_reply), crc32_enabled);
```

## Detailed Examples for Each Command

### GET_CURRENT_TIME_COMMAND

**Before:**
```c
case GET_CURRENT_TIME_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        local_time = get_microsecond_time();
        rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
        rs485_transmit(&local_time, 6);
    }
    break;
```

**After:**
```c
case GET_CURRENT_TIME_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint64_t local_time;
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } time_reply;
        time_reply.local_time = get_microsecond_time();
        rs485_finalize_and_transmit_packet(&time_reply, sizeof(time_reply), crc32_enabled);
    }
    break;
```

### TIME_SYNC_COMMAND

**Before:**
```c
case TIME_SYNC_COMMAND:
    memcpy(&time_from_master, payload, 6);
    rs485_done_with_this_packet();
    int32_t time_error = time_sync(time_from_master);
    uint16_t clock_calibration_value = get_clock_calibration_value();
    if(!is_broadcast) {
        rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
        rs485_transmit(&time_error, 4);
        rs485_transmit(&clock_calibration_value, 2);
    }
    break;
```

**After:**
```c
case TIME_SYNC_COMMAND:
    memcpy(&time_from_master, payload, 6);
    rs485_done_with_this_packet();
    int32_t time_error = time_sync(time_from_master);
    uint16_t clock_calibration_value = get_clock_calibration_value();
    if(!is_broadcast) {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            int32_t time_error;
            uint16_t clock_calibration_value;
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } time_sync_reply;
        time_sync_reply.time_error = time_error;
        time_sync_reply.clock_calibration_value = clock_calibration_value;
        rs485_finalize_and_transmit_packet(&time_sync_reply, sizeof(time_sync_reply), crc32_enabled);
    }
    break;
```

### GET_UPDATE_FREQUENCY_COMMAND

**Before:**
```c
case GET_UPDATE_FREQUENCY_COMMAND:
    rs485_done_with_this_packet();
    frequency = get_update_frequency();
    if(!is_broadcast) {
        rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x04", 3);
        rs485_transmit(&frequency, 4);
    }
    break;
```

**After:**
```c
case GET_UPDATE_FREQUENCY_COMMAND:
    rs485_done_with_this_packet();
    frequency = get_update_frequency();
    if(!is_broadcast) {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint32_t frequency;
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } frequency_reply;
        frequency_reply.frequency = frequency;
        rs485_finalize_and_transmit_packet(&frequency_reply, sizeof(frequency_reply), crc32_enabled);
    }
    break;
```

### GET_PRODUCT_INFO_COMMAND

**Before:**
```c
case GET_PRODUCT_INFO_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
        uint8_t product_info_length = sizeof(struct product_info_struct);
        struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
        rs485_transmit(&product_info_length, 1);
        rs485_transmit(product_info, sizeof(struct product_info_struct));
    }
    break;
```

**After:**
```c
case GET_PRODUCT_INFO_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint8_t product_info_length;
            struct product_info_struct product_info;
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } product_info_reply;
        product_info_reply.product_info_length = sizeof(struct product_info_struct);
        memcpy(&product_info_reply.product_info, (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION), sizeof(struct product_info_struct));
        rs485_finalize_and_transmit_packet(&product_info_reply, sizeof(product_info_reply), crc32_enabled);
    }
    break;
```

### GET_PRODUCT_DESCRIPTION_COMMAND

**Before:**
```c
case GET_PRODUCT_DESCRIPTION_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
        uint8_t product_description_length = sizeof(PRODUCT_DESCRIPTION);
        rs485_transmit(&product_description_length, 1);
        rs485_transmit(&PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
    }
    break;
```

**After:**
```c
case GET_PRODUCT_DESCRIPTION_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint8_t product_description_length;
            char product_description[sizeof(PRODUCT_DESCRIPTION)];
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } product_description_reply;
        product_description_reply.product_description_length = sizeof(PRODUCT_DESCRIPTION);
        memcpy(product_description_reply.product_description, &PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
        rs485_finalize_and_transmit_packet(&product_description_reply, sizeof(product_description_reply), crc32_enabled);
    }
    break;
```

### GET_FIRMWARE_VERSION_COMMAND

**Before:**
```c
case GET_FIRMWARE_VERSION_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
        uint8_t firmware_version_length = sizeof(firmware_version);
        rs485_transmit(&firmware_version_length, 1);
        rs485_transmit(&firmware_version, sizeof(firmware_version));
    }
    break;
```

**After:**
```c
case GET_FIRMWARE_VERSION_COMMAND:
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint8_t firmware_version_length;
            char firmware_version_data[sizeof(firmware_version)];
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } firmware_version_reply;
        firmware_version_reply.firmware_version_length = sizeof(firmware_version);
        memcpy(firmware_version_reply.firmware_version_data, &firmware_version, sizeof(firmware_version));
        rs485_finalize_and_transmit_packet(&firmware_version_reply, sizeof(firmware_version_reply), crc32_enabled);
    }
    break;
```

### PING_COMMAND

**Before:**
```c
case PING_COMMAND:
    memcpy(ping_response_buffer + 3, payload, PING_PAYLOAD_SIZE);
    rs485_done_with_this_packet();
    if(!is_broadcast) {
        ping_response_buffer[0] = RESPONSE_CHARACTER;
        ping_response_buffer[1] = '\x01';
        ping_response_buffer[2] = PING_PAYLOAD_SIZE;
        rs485_transmit(ping_response_buffer, PING_PAYLOAD_SIZE + 3);
    }
    break;
```

**After:**
```c
case PING_COMMAND:
    {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint8_t ping_payload[PING_PAYLOAD_SIZE];
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } ping_reply;
        memcpy(ping_reply.ping_payload, payload, PING_PAYLOAD_SIZE);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            rs485_finalize_and_transmit_packet(&ping_reply, sizeof(ping_reply), crc32_enabled);
        }
    }
    break;
```

**IMPORTANT**: Notice that we extract data from the payload BEFORE calling `rs485_done_with_this_packet()`. This is critical because `rs485_done_with_this_packet()` may invalidate the payload buffer.

### GET_SUPPLY_VOLTAGE_COMMAND

**Before:**
```c
case GET_SUPPLY_VOLTAGE_COMMAND:
    rs485_done_with_this_packet();
    {
        struct __attribute__((__packed__)) {
            uint8_t header[3];
            uint16_t supply_voltage;
        } supply_voltage_reply;
        if(!is_broadcast) {
            supply_voltage_reply.header[0] = RESPONSE_CHARACTER;
            supply_voltage_reply.header[1] = 1;
            supply_voltage_reply.header[2] = sizeof(supply_voltage_reply) - 3;
            supply_voltage_reply.supply_voltage = get_supply_voltage_volts_time_10();
            rs485_transmit(&supply_voltage_reply, sizeof(supply_voltage_reply));
        }
    }
    break;
```

**After:**
```c
case GET_SUPPLY_VOLTAGE_COMMAND:
    rs485_done_with_this_packet();
    {
        struct __attribute__((__packed__)) {
            uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
            uint16_t supply_voltage;
            uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
        } supply_voltage_reply;
        if(!is_broadcast) {
            supply_voltage_reply.supply_voltage = get_supply_voltage_volts_time_10();
            rs485_finalize_and_transmit_packet(&supply_voltage_reply, sizeof(supply_voltage_reply), crc32_enabled);
        }
    }
    break;
```

## Important Notes

1. **Structure Packing**: Always use the `__attribute__((__packed__))` attribute to ensure the structure is packed without any padding between fields.

2. **Header and CRC32 Fields**: Always include these fields in the structure, but do not set their values. They will be filled in by the `rs485_finalize_and_transmit_packet()` function.

3. **Broadcast Check**: Maintain the existing `if(!is_broadcast)` checks to ensure responses are only sent for non-broadcast commands in the case that you are using `rs485_finalize_and_transmit_packet()` to send out the data. 

4. **Memory Handling**: For commands that involve copying data (like product info, descriptions, etc.), use `memcpy()` to copy the data into the structure.

5. **CRC32 Enabling**: Always pass the `crc32_enabled` flag to the `rs485_finalize_and_transmit_packet()` function to ensure consistent CRC32 handling.

6. **CRITICAL: Payload Access Order**: Always extract all needed data from the payload buffer BEFORE calling `rs485_done_with_this_packet()`. The `rs485_done_with_this_packet()` function may invalidate the payload buffer, so you must copy any needed data before calling it.

## Correct sending of the "No error" response

1. **Broadcast Check**: If the command has no data
to return and instead `rs485_transmit_no_error_packet` is used to send a "No error" response packet,then  you should not enclose it in a `if(!is_broadcast)` condition because that function does the check internally. Here is an example:

**Before:**
```c
            if(!is_broadcast) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
```

**After:**
```c
            transmit_no_error_response(is_broadcast, crc32_enabled); // nothing will be transmitted if is_broadcast is true
```



## Variable Scope in process_packet

When updating the `process_packet()` function, it's important to follow proper variable scoping practices:

1. **Local Variables**: All variables that are only used within a specific case block should be declared locally within that block, not at the top of the function.

2. **Block Scope**: Use curly braces `{}` to create a block scope for each case statement, allowing for proper local variable declarations.

### Before (Poor Practice):
```c
void process_packet(void)
{
    uint8_t command;
    uint16_t payload_size;
    void *payload;
    uint8_t is_broadcast;

    // These variables should NOT be declared here
    uint64_t local_time;
    uint64_t time_from_master = 0;
    uint8_t capture_length;
    uint32_t frequency;
    uint64_t unique_id;
    uint8_t new_alias;
    uint8_t ping_response_buffer[PING_PAYLOAD_SIZE + 3];
    
    // Function body...
    
    switch(command) {
    case CAPTURE_HALL_SENSOR_DATA_COMMAND:
        capture_length = payload[0];
        // Rest of case...
        break;
    }
}
```

### After (Good Practice):
```c
void process_packet(void)
{
    uint8_t command;
    uint16_t payload_size;
    void *payload;
    uint8_t is_broadcast;
    
    // Function body...
    
    switch(command) {
    case CAPTURE_HALL_SENSOR_DATA_COMMAND:
        {
            uint8_t capture_length = payload[0];
            // Rest of case...
        }
        break;
    case TIME_SYNC_COMMAND:
        {
            uint64_t time_from_master = 0;
            // Rest of case...
        }
        break;
    }
}
```

## Payload Size Checking and Alignment Fixes

In addition to the structured data format changes, two other critical areas need attention:

1. **Ensuring proper payload size checking for all commands**
2. **Fixing alignment issues for commands with multiple parameters**

### Payload Size Checking

Every command in the `process_packet()` function should include explicit payload size validation through one of the following mechanisms:

1. **For commands with no payload**: Add calls to `check_payload_size_is_zero(payload_size)` to verify that no unexpected data was received.
   ```c
   case IDENTIFY_COMMAND:
       check_payload_size_is_zero(payload_size);
       rs485_done_with_this_packet();
       // Command implementation...
   ```

2. **For commands with fixed-size payloads**: Replace direct `memcpy` calls with `copy_input_parameters_and_check_size()` to ensure the received payload matches the expected size.
   ```c
   case SET_MAX_VELOCITY_COMMAND:
   {
       uint32_t max_velocity;
       copy_input_parameters_and_check_size(&max_velocity, payload, sizeof(max_velocity), payload_size);
       rs485_done_with_this_packet();
       // Command implementation...
   }
   ```

3. **For commands with variable-sized payloads**: Implement custom size validation logic that calculates the expected payload size based on command parameters.
   ```c
   case MULTIMOVE_COMMAND:
   {
       // First check if we have at least one byte for n_moves_in_this_command
       if (payload_size < 1) {
           fatal_error(ERROR_COMMAND_SIZE_WRONG);
       }
       
       uint8_t n_moves_in_this_command = ((int8_t*)payload)[0];
       
       // Calculate the expected payload size
       uint16_t expected_size = 1 + sizeof(uint32_t) + 
           n_moves_in_this_command * (sizeof(int32_t) + sizeof(int32_t));
       
       if (payload_size != expected_size) {
           fatal_error(ERROR_COMMAND_SIZE_WRONG);
       }
       
       // Command implementation...
   }
   ```

### Alignment Fixes

Alignment fixes address potential issues with data structure alignment on the microcontroller:

1. **Single-parameter commands**: Change from using structs to direct variables for better efficiency and clarity.
   ```c
   // Before:
   struct __attribute__((__packed__)) {
       uint32_t max_velocity;
   } velocity_input;
   copy_input_parameters_and_check_size(&velocity_input, payload, sizeof(velocity_input), payload_size);
   set_max_velocity(velocity_input.max_velocity);
   
   // After:
   uint32_t max_velocity;
   copy_input_parameters_and_check_size(&max_velocity, payload, sizeof(max_velocity), payload_size);
   set_max_velocity(max_velocity);
   ```

2. **Multi-parameter commands**: Ensure consistent use of packed structs to prevent alignment issues.
   ```c
   struct __attribute__((__packed__)) {
       int32_t displacement;
       uint32_t time;
   } trapezoid_move_inputs;
   copy_input_parameters_and_check_size(&trapezoid_move_inputs, payload, sizeof(trapezoid_move_inputs), payload_size);
   ```

3. **Response structures**: Maintain consistent structure with header, data fields, and CRC32 fields for all command responses.
   ```c
   struct __attribute__((__packed__)) {
       uint8_t header[3]; // filled by rs485_finalize_and_transmit_packet()
       int64_t motor_position;
       uint32_t crc32; // filled by rs485_finalize_and_transmit_packet()
   } position_reply;
   ```

These changes ensure that all commands properly validate their input parameters, prevent potential buffer overflows or misaligned data access, and maintain a consistent structure for command handling throughout the firmware.

## Testing After Updates

After making these changes, it's important to test each command to ensure:

1. The correct data is being transmitted
2. The CRC32 is being properly calculated and included
3. The receiving end can correctly parse the structured data
4. There are no memory alignment issues with the packed structures

By following this guide, you can ensure consistent command handling across all firmware files, improving code organization and maintainability.
