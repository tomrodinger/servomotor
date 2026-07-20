#!/usr/bin/env python3
"""
Example: Zero position -- redefine the current shaft position as the origin (position 0).
Zeroes and reads back ~0, makes a small move, zeroes again, and reads back ~0 once more.
The shaft never moves when zeroing; only the coordinate system shifts.
"""
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

    motor.enable_mosfets()
    time.sleep(0.3)                     # the motor is ready immediately, but it may twitch as
                                        #  the rotor snaps to a commutation step; we settle
                                        #  ONLY because we zero precisely next -- zeroing
                                        #  during the twitch corrupts the origin
    motor.zero_position()               # the current spot is now position 0
    print(f"After zero:    commanded = {motor.get_position():.4f}, "
          f"measured = {motor.get_hall_sensor_position():.4f} rotations (both ~0)")

    motor.trapezoid_move(0.5, 1.0)      # move +0.5 rotation away from the origin in 1 s
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # wait for the motion queue to drain
    time.sleep(0.2)                     # brief mechanical settle before reading
    print(f"After move:    commanded = {motor.get_position():.4f} rotations (~0.5)")

    # Zero again: the origin is redefined at the CURRENT spot, without any motion.
    # The queue must be empty when zeroing -- the wait above guarantees that.
    motor.zero_position()
    print(f"After re-zero: commanded = {motor.get_position():.4f}, "
          f"measured = {motor.get_hall_sensor_position():.4f} rotations (both ~0)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
