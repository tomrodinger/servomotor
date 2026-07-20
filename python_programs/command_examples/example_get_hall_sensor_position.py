#!/usr/bin/env python3
"""
Example: Get hall sensor position -- read the measured (actual) shaft position.
Performs a small move, then prints the commanded position, the hall sensor
(measured) position, and their difference. A large or growing difference
means the motor stalled or missed motion -- the basis of stall detection.
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
    time.sleep(0.3)                     # energizing snaps the rotor to a commutation step;
                                        #  zeroing during that transient corrupts the zero
    motor.zero_position()               # zeroes the commanded AND hall sensor positions together

    motor.trapezoid_move(MOVE_ROTATIONS, MOVE_SECONDS)
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # poll with sleeps -- never busy-poll the bus
    time.sleep(0.2)                     # brief mechanical settling before reading positions

    commanded = motor.get_position()             # where the firmware thinks the shaft is
    measured = motor.get_hall_sensor_position()  # where the shaft actually is
    print(f"Commanded position:       {commanded:+.4f} shaft rotations")
    print(f"Hall sensor position:     {measured:+.4f} shaft rotations")
    print(f"Difference (meas - cmd):  {measured - commanded:+.4f} shaft rotations")
    # At rest the two should agree within a few hundred encoder counts (about
    # 0.0001 rotations). If the difference exceeds the max allowable position
    # deviation (default 2 rotations), the firmware latches fatal error 45.
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
