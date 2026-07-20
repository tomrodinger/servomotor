#!/usr/bin/env python3
"""
Example: Get position -- read the current commanded (desired) position.
Zeroes the position, performs a small move, and reads the position before and
after. Expect 0 before the move and about +1.0 shaft rotations after it.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
MOVE_ROTATIONS = 1.0                    # Small, safe test move (shaft rotations)
MOVE_SECONDS = 2.0                      # Duration of the test move (seconds)

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    motor.enable_mosfets()
    time.sleep(0.3)                     # settle after energizing before zeroing the position
    motor.zero_position()

    print(f"Position after zeroing:  {motor.get_position():+.4f} shaft rotations")

    motor.trapezoid_move(MOVE_ROTATIONS, MOVE_SECONDS)
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # poll with sleeps -- never busy-poll the bus
    time.sleep(0.2)                     # brief mechanical settling before reading

    print(f"Position after the move: {motor.get_position():+.4f} shaft rotations")
    # NOTE: 'Get position' returns the COMMANDED position -- the motion profile's
    # internal target, not a measurement. It can be read at any time, including
    # during motion. For the measured shaft position use 'Get hall sensor
    # position' and compare the two to detect stalls or missed motion.
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
