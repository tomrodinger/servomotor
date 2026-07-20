#!/usr/bin/env python3
"""
Example: Move with velocity -- spin at a constant velocity for a fixed time.
Spins the shaft at 1 rotation/second for 2 seconds, stops with a queued
zero-velocity segment, waits for the queue to drain, then prints the final
position (expect ~2 shaft rotations).
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
VELOCITY = 1.0                          # rotations/second
MOVE_TIME = 2.0                         # seconds
STOP_TIME = 0.1                         # seconds (duration of the zero-velocity stop segment)

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    motor.enable_mosfets()
    time.sleep(0.3)                     # let the rotor settle on a commutation step
    motor.zero_position()

    motor.move_with_velocity(VELOCITY, MOVE_TIME)   # 1 rot/s for 2 s
    # ---------------------------------------------------------------------
    # MANDATORY STOP SEGMENT: a velocity move does NOT stop on its own.
    # When the 2 s expire the motor keeps the last commanded velocity, and
    # if the queue is empty at that instant the firmware raises fatal
    # error 18 (ERROR_RUN_OUT_OF_QUEUE_ITEMS) and disables itself until
    # reset. Queue the zero-velocity segment back-to-back with the moving
    # one, so it is already waiting in the queue when the move time expires.
    # ---------------------------------------------------------------------
    motor.move_with_velocity(0.0, STOP_TIME)

    while motor.get_n_queued_items() > 0:   # wait for both segments to execute
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading

    position = motor.get_position()
    print(f"Final position: {position:.3f} shaft rotations (expected ~2.0)")
finally:
    try:
        motor.emergency_stop()          # clears queue + disables MOSFETs, safe mid-motion
    except Exception:
        pass
    servomotor.close_serial_port()
