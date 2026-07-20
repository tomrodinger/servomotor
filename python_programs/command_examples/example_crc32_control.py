#!/usr/bin/env python3
"""
Example: CRC32 control -- turn the protocol's CRC32 layer off and back on.
Shows that with CRC32 disabled the device accepts CRC-less packets, then
restores the CRC-enabled default. The state changes take effect immediately,
so every command must be framed to match the device's CURRENT CRC state.
"""
import time
import servomotor
from servomotor import communication

ALIAS = 'X'                             # Device alias; change if needed
SYSTEM_RESET_COMMAND_ID = 27            # used by the dual-framing recovery in the finally block
PING_PAYLOAD = b"hello12345"            # 'Ping' requires exactly 10 bytes
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
servomotor.set_serial_port(SERIAL_PORT)

servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
                                        #  (a reset also restores the CRC-enabled default)

    # The motor object's outgoing framing must always match the device's CRC32
    # state: with CRC32 enabled on the device, a CRC-less or corrupted packet is
    # dropped silently (symptom: TimeoutError). The reverse mismatch is worse:
    # while the device has CRC32 disabled, a packet sent WITH a CRC has its 4 CRC
    # bytes counted as payload, which latches fatal error 51
    # (ERROR_COMMAND_SIZE_WRONG), disabling the device until 'System reset'.
    # Keep the two in lockstep: send 'CRC32 control', then set_crc32_enabled().

    # NOTE: the library prints a framing warning after each CRC32-control ack --
    # the device applies the change first, so the ack already arrives in the NEW
    # format; the warnings are expected and harmless.
    print("Disabling CRC32 (this command itself is sent WITH a CRC)...")
    motor.crc32_control(0)
    motor.set_crc32_enabled(False)      # from now on this object sends without a CRC

    print("Pinging WITHOUT a CRC to prove the device now accepts CRC-less packets...")
    echoed = motor.ping(PING_PAYLOAD)
    print(f"  sent {PING_PAYLOAD}, got {echoed}: {'match' if echoed == PING_PAYLOAD else 'MISMATCH'}")

    print("Re-enabling CRC32 (sent WITHOUT a CRC to match the current device state)...")
    motor.crc32_control(1)
    motor.set_crc32_enabled(True)       # back to the CRC-enabled default

    echoed = motor.ping(PING_PAYLOAD)   # normal M3 call; a CRC is appended again
    print(f"  normal ping echoed {echoed}: {'match' if echoed == PING_PAYLOAD else 'MISMATCH'}")
    print("Device is back at its CRC-enabled default.")
finally:
    # We may abort in either CRC state. A CRC-less reset is silently ignored by a
    # CRC-enabled device (harmless); after the mandatory bus-silent wait, a normal
    # CRC-framed reset covers the other case. Reset also disables the MOSFETs.
    try:
        communication.execute_command(SYSTEM_RESET_COMMAND_ID, [],
                                      alias_or_unique_id=motor.alias_or_unique_id,
                                      crc32_enabled=False, verbose=0)
    except Exception:
        pass
    time.sleep(1.5)
    try:
        motor.system_reset()
        time.sleep(1.5)
    except Exception:
        pass
    servomotor.close_serial_port()
