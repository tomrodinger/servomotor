#!/usr/bin/env python3
"""
Example: Go to position -- queue a smooth ABSOLUTE move to a target position.
Moves the shaft to +0.5 rotations in 2 s, then back to absolute position 0.
You should see the shaft rotate out and return, with both positions printed.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
servomotor.set_serial_port(SERIAL_PORT)

TARGET_POSITION = 0.5                   # shaft rotations (ABSOLUTE target, not a displacement)
MOVE_DURATION = 2.0                     # seconds allowed for each move

servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    motor.enable_mosfets()
    time.sleep(0.3)                     # energizing snaps the rotor to a commutation step;
                                        #  let it settle before establishing the zero
    motor.zero_position()               # make absolute position 0 mean "right here"

    motor.go_to_position(TARGET_POSITION, MOVE_DURATION)
    # The success response only confirms the move was queued, not that motion
    # finished -- poll the queue until it empties (always sleep between polls).
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading position
    print(f"Position after first move: {motor.get_position():.4f} rotations "
          f"(expected {TARGET_POSITION})")

    motor.go_to_position(0.0, MOVE_DURATION)   # absolute move: back to the origin
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)
    time.sleep(0.2)
    print(f"Position after second move: {motor.get_position():.4f} rotations (expected 0.0)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
