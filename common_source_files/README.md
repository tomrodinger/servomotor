# Servo Motor RS485 Communication Protocol

## Overview

This document describes the RS485-based communication protocol used in the servo motor control system. The protocol enables reliable communication between a host controller and multiple servo motor devices on a shared RS485 bus.

## Protocol Structure

### Packet Format

Each command packet follows this structure:

1. **Size Bytes**:
   - **Byte 1**: Encoded first byte containing the packet size
     - The LSB (bit 0) is always set to 1 for validation purposes
     - The actual size value is stored in bits 1-7 (size = encoded_byte >> 1)
     - If the decoded size value is 127 (DECODED_FIRST_BYTE_EXTENDED_SIZE), then the next two bytes form a 16-bit size value
     - Otherwise, this is the direct size of the entire packet including all fields
   
2. **Address Bytes**:
   - **Standard Addressing**: Single byte containing device alias
     - `255` (ALL_ALIAS): Broadcasts to all devices on the bus
     - `254` (EXTENDED_ADDRESSING): Indicates extended addressing mode
     - `253` (RESPONSE_CHARACTER): Indicates a response from a device
     - `0-252`: Normal device aliases
   - **Extended Addressing**: When the address byte is `254` (EXTENDED_ADDRESSING), the next 8 bytes contain the 64-bit Unique ID of the target device

3. **Command Byte**: 8-bit value specifying the operation to be performed

4. **Payload Data**: Variable length data according to the command requirements

5. **CRC32 (4 bytes)**: When CRC32 is enabled, the last 4 bytes of the packet contain a CRC32 checksum of all preceding bytes (including size and address bytes)

### Response Format

Responses from devices follow a similar format but always begin with the address byte set to `RESPONSE_CHARACTER` (253) to indicate a reply from a device.

## Command Processing Flow

1. **Initialization**: 
   - The system initializes UART1 for RS485 communication at 230400 baud
   - Hardware flow control is enabled using the DE pin for direction control (transmit/receive switching)

2. **Reception Process**:
   - When data arrives, the USART1 interrupt handler processes it byte by byte
   - Bytes are collected until a complete packet is formed based on the size information
   - Timeout detection resets the reception process if there's a gap in transmission
   - Error detection for framing, overrun, and noise errors

3. **Packet Validation**:
   - Validates that the first byte has LSB=1 (part of the encoding scheme)
   - Decodes the first byte to determine packet size
   - If extended size is indicated, reads the next two bytes for the full packet size
   - Checks if the packet is addressed to this device (matching alias, broadcast, or matching Unique ID)
   - If CRC32 is enabled, validates the CRC32 checksum
   - Buffers the packet for processing if valid

4. **Command Processing**:
   - Main program loop processes commands when `rs485_has_a_packet()` returns true
   - `rs485_get_next_packet()` extracts command, payload size, payload pointer, and broadcast flag
   - This function handles all addressing modes and returns only packets addressed to this device
   - After processing, `rs485_done_with_this_packet()` must be called to clear the receive buffer

5. **Response Transmission**:
   - `rs485_transmit()` sends basic data back to the host
   - `rs485_finalize_and_transmit_packet()` handles CRC32 calculation and packet finalization before transmission
   - Responses include the CRC32 checksum if CRC32 is enabled

## CRC32 Implementation

The protocol includes CRC32 error detection:

1. **Calculation**:
   - Uses hardware CRC32 unit for efficient processing
   - Calculated over the entire packet (including size and address bytes)
   - 4-byte CRC32 value appended to the end of the packet

2. **Control**:
   - Enabled by default after reset or power cycle
   - Can be disabled/enabled using `CRC32_CONTROL_COMMAND`
   - Error statistics tracked and retrievable via `GET_CRC32_ERROR_COUNT_COMMAND`

3. **Key Functions**:
   - `crc32_init()`: Initializes the CRC32 hardware unit
   - `calculate_crc32_buffer()`: Calculates CRC32 for a data buffer
   - `rs485_validate_packet_crc32()`: Validates received packet CRC32
   - `rs485_finalize_and_transmit_packet()`: Adds CRC32 to outgoing packets if enabled

## Error Handling

The protocol includes robust error handling mechanisms:

- **Framing errors**: Incorrect stop/start bits
- **Overrun errors**: Data received before previous byte processed
- **Noise errors**: Electrical interference
- **Command overflow**: New command received before previous one processed
- **Command too long**: Exceeding buffer capacity
- **CRC32 errors**: Data corruption during transmission

## Visual Indicators

- The red LED is turned on when bytes are being received and off when reception is idle

## Special Features

- **Timeout detection**: Resets the reception state if transmission is interrupted
- **Broadcast support**: Commands can be sent to all devices simultaneously
- **Hardware flow control**: Automatic direction switching for transmit/receive
- **Simulation support**: Protocol can be used in both real hardware and simulation environments

## Key Functions

- `rs485_init()`: Initializes the RS485 communication interface
- `rs485_has_a_packet()`: Checks if a valid packet is available for processing
- `rs485_get_next_packet()`: Retrieves the next packet for processing
- `rs485_validate_packet_crc32()`: Validates the CRC32 checksum of a packet
- `rs485_done_with_this_packet()`: Clears processed packet and enables reception of the next packet
- `rs485_transmit()`: Sends raw data over the RS485 bus without modification
- `rs485_finalize_and_transmit_packet()`: Sets packet size, adds CRC32 if enabled, and transmits the packet
- `encode_first_byte()`: Encodes a byte by shifting left and setting LSB to 1
- `decode_first_byte()`: Decodes a byte by shifting right

## Protocol Bit Structure

1. **First Byte Encoding**:
   - The least significant bit (LSB) of the first byte is always set to 1
   - This enables future automatic baud rate detection by ensuring a predicatble first low pulse

2. **Validation**:
   - The system validates that the first byte of a packet has LSB=1
   - Any packet with LSB=0 in the first byte is rejected
   - Reception resets after a timeout (10ms without UART traffic)

## Extended Addressing

The protocol supports addressing devices by their 64-bit Unique ID:

1. **Addressing Mode**:
   - When address byte is `EXTENDED_ADDRESSING (254)`, the next 8 bytes contain the device's 64-bit Unique ID

2. **Implementation Details**:
   - Firmware recognizes and processes extended addressing commands
   - Python library supports both standard addressing (by alias) and extended addressing (by Unique ID)
   - The `-a` option in the command-line interface accepts both aliases and Unique IDs (16-character hex strings)
   - The system automatically detects whether to use standard or extended addressing based on the input format

3. **Usage Example**:
   - Standard addressing: `python3 motor_command.py -a 1 ENABLE_MOSFETS`
   - Extended addressing: `python3 motor_command.py -a 0123456789ABCDEF ENABLE_MOSFETS`

## CRC32-Related Commands

1. **CRC32_CONTROL_COMMAND**:
   - Enables or disables CRC32 checking
   - Payload: Single byte (CRC32_ENABLE or CRC32_DISABLE)
   - Default: Enabled after reset or power cycle

2. **GET_CRC32_ERROR_COUNT_COMMAND**:
   - Retrieves the current CRC32 error count
   - Payload: Single byte (CRC32_ERROR_READ_ONLY or CRC32_ERROR_RESET)
   - Response: 32-bit error count
   - Can optionally reset the error counter

## Future Enhancements

Planned enhancements for the communication protocol:

1. **Support extended addressing in the Arduino module**
   - Enable specifying the unique ID when creating a motor object
   - Add function to change the unique ID of the class that is commanding the device

2. **Change the way that we set the alias**
   - Use extended addressing mode to address devices specifically
   - Simplify the "Set device alias" command to require only the new alias parameter
   