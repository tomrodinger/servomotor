# Servo Motor RS485 Communication Protocol

## Overview

This document describes the RS485-based communication protocol used in the servo motor control system. The protocol enables reliable communication between a host controller and multiple servo motor devices on a shared RS485 bus.

## Protocol Structure

### Packet Format

Each command packet follows this structure:

1. **Size Byte(s)**:
   - **7-bit size**: Encoded first byte containing the packet size
     - The LSB (bit 0) is always set to 1 for validation purposes and to enable automatic baud rate detection in the future
     - The actual size value is stored in bits 1-7 (size = encoded_byte >> 1)
     - This is the direct size of the entire packet including all fields and even including the size byte, unless the decoded size is 127, which indicated extended size
   - **Extended 16-bit size**: Encoded first byte contains 127 and two more bytes contain the actual 16-bit size
     - If the decoded size value is 127 (DECODED_FIRST_BYTE_EXTENDED_SIZE), then the next two bytes form a 16-bit size value. It is little endian.
   
2. **Address Byte(s)**:
   - **Standard Addressing**: Single byte containing device alias
     - `255` (ALL_ALIAS): Broadcasts to all devices on the bus
     - `254` (EXTENDED_ADDRESSING): Indicates extended addressing mode
     - `253` (RESPONSE_CHARACTER_CRC32_ENABLED): Indicates a response from a device where the response HAS a 4-byte CRC32 checksum at the end
     - `252` (RESPONSE_CHARACTER_CRC32_DISABLED): Indicates a response from a device where the response DOES NOT HAVE a 4-byte CRC32 checksum at the end
     - `0-251`: Normal device aliases
   - **Extended Addressing**: When the address byte is `254` (EXTENDED_ADDRESSING), the next 8 bytes contain the 64-bit Unique ID of the target device

3. **Command Byte**: 8-bit value specifying the operation to be performed

4. **Payload Data**: Variable length data according to the command requirements

5. **CRC32 (4 bytes, optional)**: When CRC32 is enabled, the last 4 bytes of the packet contain a CRC32 checksum of all preceding bytes (including size and address bytes)

### Response Format

Responses from devices follow a similar format: they start with the size byte, and are followed by an address byte, which is set to eitiher `253` (RESPONSE_CHARACTER_CRC32_ENABLED) to indicate a reply from a device with a CRC32 checksum appended, or to `252` (RESPONSE_CHARACTER_CRC32_DISABLED) to indicate a reply from a device which does not have the 4-byte CRC32.

### Immediate position targets — commands 49 (`0x31`) and 50 (`0x32`)

Stable **0.17.0.0** adds these commands to 0.16.0.0. It promotes validated
development 0.17.0.6 with only its development byte changed to zero. The exact
release image passed 35,148+ canary updates with zero communication errors.
Release evidence and limitations: `firmware/immediate_target_validation/2026-11-11/release_0.17.0.0/hardware/HARDWARE_RESULTS.txt`.
The wire schema introduced by development .3 and the software compatibility
code are unchanged; .1/.2 used the legacy 24-byte reply described below.

Both requests contain exactly one signed 64-bit little-endian position, in
encoder counts relative to the current session zero. There is no duration.
In stable 0.17.0.0 (and development 0.17.0.6), the command handler validates with interrupts enabled.
Validation takes its own atomic copy of the cached Hall value for the deviation
check. Once an empty queue, idle operation and zero commanded/planned velocity
are established, an ISR cannot start a new queued or busy operation on its own;
foreground command dispatch is serialized.

The reference and empty-queue planned end position are then updated together
in a short interrupt-masked section containing only stores. They must remain
consistent because the empty-queue ISR compares them. No validation, condition,
function call or Hall read occurs inside this write mask.

Both commands return without waiting for a control tick or shaft arrival.
Command 49 (`Set immediate position target`) retains its compact success ACK.
Command 50 (`Set immediate position target and get position`) reads the latest
stored Hall value after applying the target, using a separate atomic copy in
`get_hall_position()`, the same getter used by `Get comprehensive position`.
The getter's mask contains only the cached-variable loads; it does no ADC read
or position recalculation. An ISR may run between the target write and this
read, so the returned value may come from a tick before or after the update.
The write and read do not form one atomic snapshot.

| Payload offset | Type | Meaning |
|---|---|---|
| 0 | i64 | Latest stored Hall position, read after applying the target |

The successful payload is exactly **8 bytes**, signed little endian, in encoder
counts relative to session zero. The response is **15 bytes with CRC32 or 11
without it**. The normal header error code remains zero on success; fatal-error
framing is unchanged. The accepted reference is exactly the requested signed-i64
target: invalid targets raise an error rather than being clamped. The reply
confirms reference application, not shaft arrival. It contains no target echo,
sample counter, status flags, mode, queue depth, current or torque.

The compact ACK for command 49 remains size + response marker + optional CRC
(six bytes with CRC, two without), with no error/payload byte. Command 37 is
unchanged.

**Version boundary:** development 0.17.0.1 and 0.17.0.2 returned a 24-byte
command-50 payload: accepted target i64, measured position i64, sample counter
u32, status flags u16, control mode u8, queue depth u8. Query firmware version
before motion. These source Python/Arduino bindings support stable 0.17.0.0
and development 0.17.0.3+ for command 50; they reject an old reply's unexpected size.
ServoTune selects legacy for development 0.17.0.2, and compact for stable
0.17.0.0 or development 0.17.0.3+. Stable .0 must be explicitly recognized;
numeric comparison with .3 alone would reject it. Targets remain disabled
on development 0.17.0.1 because of its historical ISR overhead.
Versions 0.17.0.3 and 0.17.0.4 wait for the first control tick and return its
Hall position; 0.17.0.5 reads the cache in the same mask as the target update,
and 0.17.0.6 reads it afterwards with a separate mask. Neither waits for a tick.
This timing change does not alter payload size. Never resend motion to guess
a schema after an unexpected reply.

CRC-enabled examples for alias X and target 1000 encoder counts:

```text
Command 49 request: 1f 58 31 e8 03 00 00 00 00 00 00 85 b7 dd a4
Success ACK:        0d fd 13 e2 7b 37
Command 50 request: 1f 58 32 e8 03 00 00 00 00 00 00 40 8b 50 9d
```

Example command 50 response with measured position 900:

```text
1f fd 00 84 03 00 00 00 00 00 00 75 17 a2 55
```

The command must find an empty trajectory queue and enabled closed-loop
position control. Preconditions are checked with interrupts enabled before
either position is changed. Invalid requests leave both positions unchanged.
There is no deferred command validation or pending transaction. Later Hall
samples remain subject to the normal running protections; a subsequent fault
can occur after this command has returned success. Validation never clears
the queue. Fatal errors disable outputs and require reset.

| Code | Rejection condition (in validation order) |
|---|---|
| 51 | Request payload is not exactly eight bytes |
| 8 | Any active/queued trajectory item remains |
| 19 | Calibration, homing, closed-loop entry, capture/test mode, or residual commanded/planned velocity is active |
| 13 | Control mode is not closed-loop position control |
| 55 | MOSFET outputs are disabled |
| 27 | Target lies outside the configured safety limits |
| 45 | Distance from measured position exceeds maximum allowable position deviation |

Unsigned distance checking handles the full i64 range without overflowing a
signed subtraction. Normal PID and deviation checks also reject an
unrepresentable signed position error before subtraction or absolute-value
arithmetic can overflow. Actual usable targets remain bounded by the configured
safety/deviation limits. Existing faults and emergency stops retain priority.
Requests do not enable outputs, clear faults, reset PID history, change current
limits, or write flash. A successful target updates the empty-queue planner
baseline, so later legacy timed moves start from that reference. Later target
updates replace the reference without consuming queue entries. If updates stop,
the enabled motor retains its last reference.

Immediate reference changes have no trajectory velocity/acceleration profile;
do not assume trajectory-limit setters bound physical catch-up motion. Old
timed commands and their zero-duration semantics are unchanged. Broadcasts
execute on each eligible device with no response, including errors; use unicast
for validation and for command 50's state. Keep one request outstanding until
its full response arrives. Never retry motion commands automatically after a
timeout; first reestablish a synchronized connection and inspect device state.

Python (source bindings, no conversion loss in raw encoder counts):

```python
m = servomotor.M3("X", position_unit="encoder_counts", verbose=0)
m.set_immediate_position_target(1000)
measured = m.set_immediate_position_target_and_get_position(2000)
```

Other selected position units convert the request target and measured result
normally. Arduino returns a measured-position float from the converted methods
and an exact i64 from `...Raw`, with the usual unique-ID overloads for both. Test
the real C handlers/ISR/queue without hardware using
`python3 python_programs/test_host_immediate_target.py`. The local
`servotune/` browser bench supports both commands and a clearly labelled demo.

### Get PID constants — command 48 (`0x30`)

Released in firmware **0.16.0.0** (also present in development build **0.15.12.1**). The request has **no payload**.
The success response contains a zero status/error byte followed by exactly
12 payload bytes. Each gain is an unsigned 32-bit little-endian integer:

| Payload offset | Width | Value |
|---|---|---|
| 0 | 4 bytes | P, raw configured proportional gain |
| 4 | 4 bytes | I, raw configured integral gain |
| 8 | 4 bytes | D, raw configured derivative gain |

These are the last values supplied to `Set PID constants` (43), or the compiled
defaults loaded at startup. All raw `uint32` values, including `0xffffffff`,
round-trip unchanged. The existing setter caps **working** gains at `INT32_MAX`,
can further cap I for overflow safety, and scales/quantizes D for derivative
averaging. Readback returns the original requested D and raw gains before those
limits; it does not report the derived effective coefficients. The setter's
behavior is unchanged.

The triplet is snapshotted coherently using a short interrupt-masked read in the
serialized foreground command handler. Reading does not alter PID state, motion
queues, position, or output enablement and is supported while moving. Gains
remain RAM-only: reset or power cycling restores product defaults, including
P=2000, I=5, D=175000 on M17. Broadcast produces no reply. Any non-empty request
payload raises the existing `ERROR_COMMAND_SIZE_WRONG` (51); existing fatal-error
handling applies. Response byte 2 is the status/error code, not an echoed command
ID. Keep one request outstanding on the bus at a time, as with existing reads.

For alias `X` (`0x58`), the complete packets with CRC32 enabled are:

```text
Request:  0f 58 30 df 79 a6 a0
Response: 27 fd 00 d0 07 00 00 05 00 00 00 98 ab 02 00 dc 29 79 b8
```

That response contains the M17 startup defaults. With CRC32 disabled, the same
transaction is:

```text
Request:  07 58 30
Response: 1f fc 00 d0 07 00 00 05 00 00 00 98 ab 02 00
```

The generated Python method is `p, i, d = motor.get_pid_constants()`; the
generated Arduino method is `getPidConstantsResponse gains = motor.getPidConstants()`
with `uint32_t` fields `kP`, `kI`, and `kD`. Neither applies unit conversion.
Earlier firmware has no handler for command 48; check `Get firmware version`
before using it on existing devices.

To build the development firmware using the existing toolchain:

```sh
cd firmware
make PRODUCT_NAME=M17
```

This creates `build/servomotor_M17_fw0.16.0.0_scc3_hw1.5.firmware` for the current
M17 hardware/software-compatibility configuration in `VERSIONS`. This change
adds only PID readback, its bindings and tests; it adds no flash persistence.
Use the existing `python_programs/upgrade_firmware.py` updater with that file
for a bench reflash. The same updater can restore the previous compatible
`.firmware` file if rollback is needed; keep its product and software
compatibility code matched to the device.
Development-build validation on 2026-10-29 tested 0.15.12.1 on 27 M17 motors (hardware 1.5,
software compatibility code 3). All passed raw gain round trips, exact wire-byte
checks with and without CRC32, malformed-request and framing recovery, broadcast
silence, reset defaults, and readback during a slow 45-degree move and return.
The Python fleet test recorded 14,931 checks, 7,101 gain triplets, 108 captured
wire exchanges, and 1,902 reads during motion. Both generated Arduino getter overloads also passed the native C++ hardware
test on all 27 motors. Physical power cycling was not part of this run. Evidence is under
[`firmware/pid_readback_validation/2026-10-29`](../firmware/pid_readback_validation/2026-10-29/);
the exact 0.16.0.0 release-image canary results are retained separately in
[`firmware/pid_readback_validation/2026-10-29_release_0.16.0.0`](../firmware/pid_readback_validation/2026-10-29_release_0.16.0.0/).
Reproduction commands are in
[`python_programs/TEST_SUMMARY.md`](../python_programs/TEST_SUMMARY.md#pid-readback-hardware-validation).

## Command Processing Flow

1. **Initialization**: 
   - The system initializes UART1 for RS485 communication at 230400 baud
   - Hardware is configured to use the hardware based communication direction control through the DE pin (transmit/receive switching)

2. **Reception Process**:
   - When data arrives, the USART1 interrupt handler processes it byte by byte
   - Bytes are collected until a complete packet is formed based on the size information
   - Timeout detection resets the reception process if there's a gap in transmission
   - Error detection for framing, overrun, and noise errors
   - Statistics are gathered for any of these arrors
   - These types of errors do not cause a fatal error and a command is available for the user to fetch the statistics from the device

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
   - 4-byte CRC32 value is placed at the very end of the packet in all cases

2. **CRC32 Enable or Disable Control**:
   - Enabled by default after reset or power cycle
   - Can be disabled/enabled using `CRC32_CONTROL_COMMAND`
   - CRC32 error statistics tracked and retrievable via `GET_CRC32_ERROR_COUNT_COMMAND`
   - A device will check or not check the CRC32 of a received packet based on whether or not the CRC32 checking is enabled through this command
   - A response will have or not have a CRC32 checksome appended based on whether or not the CRC32 checking is enabled through this command
   - User needs to be careful that if he/she disables CRC32 (not the default) then all commands following should have no CRC32 bytes and also the responses from the device will have no CRC32 calculated or appended
   - The main advantage of disabling CRC32 checking is that it makes communication a bit faster (less bytes to send). The user is advised that they should first try their application with CRC32 enabled and periodically look at the communication statistics to make sure that communication is happening with 100% reliablilty. Only then should they consider to disable CRC32 checking.

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
- **CRC32 errors**: Data corruption during transmission or invalid packet format

## Visual Indicators

- The green LED is turned on when bytes are being received and off when reception is idle

## Special Features

- **Timeout detection**: Resets the reception state if transmission is interrupted
- **Broadcast support**: Commands can be sent to all devices simultaneously
- **Communication direction control**: Automatic direction switching for transmit/receive using DE pin of the hardware
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

## Packet Validation
   - The system validates that the first byte of a packet has LSB=1
   - Any packet with LSB=0 in the first byte is rejected by attempting to receive it despite the error and then discarding the packet
   - Packets are also validated based on the CRC32 checksum that is at the end of every packet (although this can be disabled to save bytes)
   - Reception resets after a timeout (10ms without UART traffic), so one way to make sure that a corrupt packet gets flushed it to wait the timeout before sending another packet

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
   - Standard addressing: `python3 servomotor_command.py -a 1 ENABLE_MOSFETS`
   - Extended addressing: `python3 servomotor_command.py -a 0123456789ABCDEF ENABLE_MOSFETS`

## CRC32-Related Commands

1. **CRC32_CONTROL_COMMAND**:
   - Enables or disables CRC32 checking
   - Payload: Single byte (CRC32_ENABLE or CRC32_DISABLE)
   - Default: Enabled after reset or power cycle

2. **Get communication statistics command**:
   - Retrieves the current CRC32 error count, packet decode error count, first bit error count, framing error count, overrun error count, and noise error count
   - Payload: Single byte (CRC32_ERROR_READ_ONLY or CRC32_ERROR_RESET)
   - Response: 32-bit numbers for each type of error
   - Can optionally reset the error counter after getting the counts
   - See the motor_commands.json file for more details or run `python3 servomotor_command.py -c`

## Future Enhancements

Planned enhancements for the communication protocol:

1. **Support extended addressing in the Arduino module**
   - Enable specifying the unique ID when creating a motor object
   - Add function to change the unique ID of the class that is commanding the device

2. **Change the way that we set the alias**
   - Use extended addressing mode to address devices specifically
   - Simplify the "Set device alias" command to require only the new alias parameter
   
3. **Automatic baud rate detection**
   - The first bit of the first byte of a packet is purposely always set to 1, which forces the start bit to be clearly define with a falling edge and a rising edge after one bit time. Theoretically we can calculate the baud rate from this.
