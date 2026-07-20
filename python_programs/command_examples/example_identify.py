#!/usr/bin/env python3
"""
Example: Identify -- flash the green LED rapidly to visually locate one motor.
The LED flashes for about 2.7 s, then the normal 1 s heartbeat blink resumes.
The command is purely cosmetic, so the device stays fully responsive: this
example proves it by pinging mid-blink.
"""
import os
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
BLINK_TIME = 3.0                        # s: the flash sequence lasts ~2.7 s; watch it finish
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

    print("Sending 'Identify' -- watch for the rapid green LED flashing (~2.7 s).")
    motor.identify()

    # Identify does not touch the motion queue, motion state, or MOSFETs, so the
    # device keeps answering commands while the LED flashes -- prove it with a ping:
    payload = os.urandom(10)            # 'Ping' requires exactly 10 bytes
    echoed = motor.ping(payload)
    if bytes(echoed) == payload:
        print("Pinged the device mid-blink: echo matches -- still fully responsive.")
    else:
        print("Pinged the device mid-blink: ECHO MISMATCH -- check the bus.")

    time.sleep(BLINK_TIME)              # let the flash sequence finish before cleanup
    print("Identification finished; the normal 1 s heartbeat blink has resumed.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
