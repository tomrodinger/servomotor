# Servo Motor RS485 Communication Protocol

## Overview

This document describes the RS485-based communication protocol used in the servo motor control system. The protocol enables reliable communication between a host controller and multiple servo motor devices on a shared RS485 bus.

## Protocol Structure

### Packet Format

Each command packet follows this structure:

1. **Byte 1**: Encoded Axis/Device ID (8-bit)
   - `127` (encoded ALL_ALIAS): Broadcasts to all devices on the bus
   - `126` (encoded RESPONSE_CHARACTER): Indicates a response from a device
   - `125` (encoded EXTENDED_ADDRESSING): Used for extended addressing mode
   - Other values: Encoded specific device address/alias (always with LSB=1)

2. **Byte 2**: Command ID (8-bit)
   - Specifies the operation to be performed

3. **Byte 3**: Value Length (8-bit)
   - If value is `255`, then the next two bytes form a 16-bit length value
   - Otherwise, this is the direct length of the payload

4. **Bytes 4+**: Payload/Value Data
   - Variable length data according to the specified value length
   - Maximum size is defined by `MAX_VALUE_BUFFER_LENGTH`

### Response Format

Responses from devices follow a similar format but always begin with the encoded `RESPONSE_CHARACTER` (126) to indicate a reply from a device.

## Command Processing Flow

1. **Initialization**: 
   - The system initializes UART1 for RS485 communication at 230400 baud
   - Hardware flow control is enabled using the DE pin for direction control (transmit/receive switching)

2. **Reception Process**:
   - When data arrives, the USART1 interrupt handler processes it byte by byte
   - Bytes are collected until a complete command is formed
   - Timeout detection resets the reception process if there's a gap in transmission
   - Error detection for framing, overrun, and noise errors

3. **Command Validation**:
   - The system validates that the received byte has LSB=1 (part of the encoding scheme)
   - Decodes the first byte of the packet (which contains the packet size) by shifting right by 1 bit
   - Checks if this is a extended size, and if yes, then it gets the size from the next two bytes
   - Checks if the command is addressed to this device (matching alias or broadcast or if extended addressing mode then do further decoding)
   - Checks if this is extended addressing mode, and if yes, gets the unique ID from the next 8 bytes
   - Validates that the command length doesn't exceed buffer capacity
   - Buffers the packet information for subsequent further decoding and interpretation and execution (if valid and address to the device)
   - Disables further reception until the current packet is processed and the `rs485_done_with_this_packet()` function is called

4. **Command Processing**:
   - Main program loop processes commands when `rs485_has_a_packet()` returns true
   - After processing, `rs485_done_with_this_packet()` must be called to clear the receive buffer so that it can be used again, otherwise there will be a command overflow fatal error

5. **Response Transmission**:
   - `rs485_transmit()` sends data back to the host
   - Uses a buffer and interrupt-driven transmission
   - Responses start with the encoded `RESPONSE_CHARACTER` (126) to indicate a reply

## Error Handling

The protocol includes robust error handling mechanisms:

- **Framing errors**: Incorrect stop/start bits
- **Overrun errors**: Data received before previous byte processed
- **Noise errors**: Electrical interference
- **Command overflow**: New command received before previous one processed
- **Command too long**: Exceeding buffer capacity
- **Too many bytes**: Received without forming a valid command

## Visual Indicators

- The red LED is turned on when bytes are being received and off when reception is idle

## Special Features

- **Timeout detection**: Resets the reception state if transmission is interrupted
- **Broadcast support**: Commands can be sent to all devices simultaneously
- **Hardware flow control**: Automatic direction switching for transmit/receive
- **Simulation support**: Protocol can be used in both real hardware and simulation environments

## Key Functions

- `rs485_init()`: Initializes the RS485 communication interface
- `rs485_done_with_this_packet()`: Enables reception of the next packet
- `rs485_transmit()`: Sends data over the RS485 bus
- `rs485_wait_for_transmit_done()`: Waits for transmission completion
- `encode_first_byte()`: Encodes the first byte of a packet by shifting left and setting LSB to 1
- `decode_first_byte()`: Decodes the first byte of a packet by shifting right

## Constants

- `ALL_ALIAS (127)`: Address for broadcasting to all devices
- `RESPONSE_CHARACTER (126)`: Indicates a response from a device
- `EXTENDED_ADDRESSING (125)`: Used for extended addressing mode

## Implementation Notes

The protocol is implemented in the RS485.c and RS485.h files, which handle the low-level communication details. The implementation uses interrupt-driven I/O for efficient operation and minimal CPU overhead during data transfer.

## Protocol Bit Structure

The communication protocol implements the following bit structure for device addressing:

1. **Device ID Bit Structure**:
   - All Device ID bytes have the least significant bit (LSB) set to 1
   - This constraint enables automatic baud rate detection (which is not yet implemented)
   - The actual Device ID is determined by shifting the received byte right by one bit
   - This provides a 7-bit address space (0-127) for device identification

2. **Special Address Values**:
   - `ALL_ALIAS (127)`: Broadcast address to communicate with all devices
   - `RESPONSE_CHARACTER (126)`: Indicates a response from a device
   - `EXTENDED_ADDRESSING (125)`: Reserved for extended addressing mode
   - Normal device IDs range from 0-124

3. **Encoding/Decoding Implementation**:
   - Device IDs are encoded before transmission: `encoded_id = (device_id << 1) | 1`
   - Device IDs are decoded upon reception: `device_id = encoded_id >> 1`
   - The encoded values are used in transmission:
     - Encoded ALL_ALIAS: 255 (binary 11111111)
     - Encoded RESPONSE_CHARACTER: 253 (binary 11111101)
     - Encoded EXTENDED_ADDRESSING: 251 (binary 11111011)
   - All normal device IDs (0-124) are encoded with LSB=1 in transmission
     - Example: Device ID 10 → Encoded as 21 (binary 00010101)
     - Example: Device ID 64 → Encoded as 129 (binary 10000001)

4. **Validation**:
   - The system validates that all received device ID bytes have LSB=1
   - Any device ID byte with LSB=0 is rejected as invalid

## Extended Addressing Implementation

The protocol now supports extended addressing mode, which allows addressing devices by their 64-bit Unique ID:

1. **Extended Addressing Mode**:
   - Devices can now be addressed by their Unique ID (64-bit number)
   - Uses the `EXTENDED_ADDRESSING (125)` value to indicate extended addressing mode
   - When first byte is encoded EXTENDED_ADDRESSING, the next 8 bytes contain the 64-bit Unique ID in little-endian format
   - Command ID and payload follow the Unique ID bytes

2. **Implementation Details**:
   - Firmware has been updated to recognize and process extended addressing commands
   - Python library supports both standard addressing (by alias) and extended addressing (by Unique ID)
   - The `-a` option in the command-line interface accepts both aliases and Unique IDs (16-character hex strings)
   - The system automatically detects whether to use standard or extended addressing based on the input format

3. **Usage Example**:
   - Standard addressing: `python motor_command.py -a 1 ENABLE_MOSFETS_COMMAND`
   - Extended addressing: `python motor_command.py -a 0123456789ABCDEF ENABLE_MOSFETS_COMMAND`

## Future Enhancements

Planned enhancements for the communication protocol:

1. **Support extended addressing in the Arduino module**
   - At the moment we specify the alias when we create a motor object, but we should also be able to specify the unique ID
   - There is also a function to change the alias. We should also be able to change the unique ID with some other new functon

2. **Change the way that we set the alias**
   - The "Set device alias" command currently takes the Unique ID as one of the input parameters (along with the new alias), but we do not need to do it this way anymore and instead we can use extended addressing mode to address it specifically
   - Therefore, we need just one input parameter, which is the new alias

3. **Add CRC32 checking to every command**
   - By default the CRC32 will be enabled and checked (and always after reset or power cycle)
   - Add a command that will disable the CRC32 checking to save bytes and complexity for certain use cases
   - Add a statistics counter that will count the number of CRC32 errors
   - Add a command to be able to retrieve and optionally reset the CRC32 error counter
   