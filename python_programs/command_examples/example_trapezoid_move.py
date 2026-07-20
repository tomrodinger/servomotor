#!/usr/bin/env python3
"""
Example: Trapezoid move -- rotate the shaft +1 rotation over 2 seconds.
The displacement is RELATIVE (a signed offset from the end of previously queued
motion), not an absolute target. Watch the shaft make one full turn, then see
the position read back close to 1.0 rotations.
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

DISPLACEMENT = 1.0                      # shaft rotations (signed, RELATIVE displacement)
DURATION = 2.0                          # seconds for the whole move, ramps included

try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    motor.enable_mosfets()
    time.sleep(0.3)                     # settle the commutation-snap transient before zeroing
    motor.zero_position()               # known origin so the readback below is meaningful

    # Queued, not immediate: the success response only confirms validation + queueing.
    motor.trapezoid_move(DISPLACEMENT, DURATION)
    print(f"Queued a relative move of {DISPLACEMENT:+.1f} rotation(s) over {DURATION} s...")

    # Wait for completion by polling the motion queue (always sleep between polls).
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading position

    position = motor.get_position()
    print(f"Final position: {position:.4f} shaft rotations (expected about {DISPLACEMENT})")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
