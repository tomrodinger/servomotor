#!/usr/bin/env python3
"""
Example: Ping -- verify the communication link by echoing 10 random bytes.
The device must return exactly the bytes sent. A correct echo proves the
port, wiring, addressing, and CRC32 framing are all working.
"""
import os
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
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

    payload = os.urandom(10)            # 'Ping' requires EXACTLY 10 bytes; any other length
                                        #  trips fatal error 51 and disables the device
    echoed = motor.ping(payload)

    print(f"Sent:     {payload.hex()}")
    print(f"Received: {bytes(echoed).hex()}")
    if bytes(echoed) == payload:
        print("PASS: echo matches exactly -- the communication link is healthy.")
    else:
        print("FAIL: echo mismatch -- check wiring, serial port, alias, and CRC32 state.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
