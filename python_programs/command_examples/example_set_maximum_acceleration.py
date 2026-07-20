#!/usr/bin/env python3
"""
Example: Set maximum acceleration -- cap the acceleration used to plan moves.
Sets a 100 rotations/s^2 limit, then runs a move that complies with it.
The firmware REJECTS a move that would exceed the limit with a latched fatal
error (it does not clamp), so set limits first and plan moves within them.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
MAX_ACCELERATION = 100.0                # rotations/s^2. Do not set 0: firmware >= 0.15.4.0 rejects it
                                        #  with latched fatal error 34 (trapezoid planning divides by this limit).
MOVE_ROTATIONS = 1.0                    # shaft rotations (relative move)
MOVE_DURATION = 2.0                     # seconds; generous enough that the planned peak
                                        #  acceleration stays well under MAX_ACCELERATION --
                                        #  a too-short duration trips fatal error 15

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Set the limit BEFORE queueing moves: planning uses the value in effect at
    # queue time. The setting is RAM-only and reverts to default on any reset.
    motor.set_maximum_acceleration(MAX_ACCELERATION)
    print(f"Maximum acceleration set to {MAX_ACCELERATION} rotations/s^2")

    motor.enable_mosfets()
    time.sleep(0.3)                     # let the rotor settle onto a commutation step
    motor.zero_position()

    motor.trapezoid_move(MOVE_ROTATIONS, MOVE_DURATION)   # complies with the new limit
    while motor.get_n_queued_items() > 0:                 # wait for motion to finish
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading position
    print(f"Move complete under the acceleration limit; "
          f"position = {motor.get_position():.4f} rotations (expected {MOVE_ROTATIONS})")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
