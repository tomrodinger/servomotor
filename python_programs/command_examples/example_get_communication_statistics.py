#!/usr/bin/env python3
"""
Example: Get communication statistics -- read the RS485 error counters.
Reads the six error counters without resetting them (flag 0) and prints each
one with its label. On a healthy bus every counter should be 0.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
RESET_FLAG = 0                          # 0 = just read; nonzero = clear all six counters after reading

# The six u32 counters, in wire order.
LABELS = [
    "crc32ErrorCount        (packets dropped by CRC32 validation)",
    "packetDecodeErrorCount (inconsistent packet size)",
    "firstBitErrorCount     (first byte's LSB was not 1)",
    "framingErrorCount      (RS485 receiver framing errors)",
    "overrunErrorCount      (RS485 receiver overrun errors)",
    "noiseErrorCount        (RS485 receiver noise errors)",
]

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # These counters are the only way to observe silently-dropped packets: with
    # CRC32 enabled, a corrupted packet gets NO error reply -- it just increments
    # crc32ErrorCount and the sender sees a timeout. Counters are RAM-only and
    # restart at zero on every reset/power-up.
    stats = motor.get_communication_statistics(RESET_FLAG)  # -> flat list of 6 counters
    print("Communication error counters since the last reset:")
    for label, count in zip(LABELS, stats):
        print(f"  {label} = {count}")
    if all(c == 0 for c in stats):
        print("All counters are 0 -- the bus is healthy.")
    else:
        print("Nonzero counters mean garbled/dropped packets -- check wiring,")
        print("termination, baud rate, and that both ends agree on the CRC32 setting.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
