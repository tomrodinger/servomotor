#!/usr/bin/env python3
"""
Example: Move with acceleration -- queue constant-acceleration segments.
Queues +2 rot/s^2 for 1 s (speed up), 0 for 1 s (coast at 2 rot/s), and
-2 rot/s^2 for 1 s (slow down) back-to-back, waits for the queue to drain,
then prints the final position (expect ~4 shaft rotations).
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
ACCELERATION = 2.0                      # rotations/second^2
SEGMENT_TIME = 1.0                      # seconds per segment

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

    # Queue the whole plan back-to-back, BEFORE the first segment finishes.
    # An acceleration segment does not stop by itself: the motor keeps the
    # velocity it reached, and if the queue empties while velocity is nonzero
    # the firmware raises fatal error 18 (ERROR_RUN_OUT_OF_QUEUE_ITEMS) and
    # disables itself until reset. The mirrored -2 rot/s^2 segment brings the
    # velocity back to exactly zero before the queue runs dry.
    motor.move_with_acceleration(ACCELERATION, SEGMENT_TIME)    # 0 -> 2 rot/s
    motor.move_with_acceleration(0.0, SEGMENT_TIME)             # coast at 2 rot/s
    motor.move_with_acceleration(-ACCELERATION, SEGMENT_TIME)   # 2 rot/s -> rest

    while motor.get_n_queued_items() > 0:   # wait for the whole plan to execute
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading

    position = motor.get_position()
    print(f"Final position: {position:.3f} shaft rotations (expected ~4.0)")
finally:
    try:
        motor.emergency_stop()          # clears queue + disables MOSFETs, safe mid-motion
    except Exception:
        pass
    servomotor.close_serial_port()
