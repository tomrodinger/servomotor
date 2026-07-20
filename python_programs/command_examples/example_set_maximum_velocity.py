#!/usr/bin/env python3
"""
Example: Set maximum velocity -- cap how fast the motor may spin.
Sets a 5 rotations/s ceiling, then runs a move that stays well below it.
A move that would exceed the limit is REJECTED with a latched fatal error,
not slowed down to fit.
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

MAX_VELOCITY = 5.0                      # rotations per second -- the new velocity ceiling
DISPLACEMENT = 1.0                      # shaft rotations for the demo move
DURATION = 2.0                          # seconds; peak velocity ~0.5 rot/s, well under the limit

try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Set limits BEFORE queueing moves: planning uses the values in effect at queue time,
    # and a violating move latches a fatal error (it is not clamped to fit). Never LOWER
    # this limit while a move is executing -- it applies immediately and trips fatal
    # error 16 if the in-flight velocity now exceeds it; change it only while stopped
    # with an empty queue (as here, right after reset).
    motor.set_maximum_velocity(MAX_VELOCITY)
    print(f"Maximum velocity set to {MAX_VELOCITY} rotations/s")

    motor.enable_mosfets()
    time.sleep(0.3)                     # settle the commutation-snap transient before zeroing
    motor.zero_position()

    motor.trapezoid_move(DISPLACEMENT, DURATION)   # compliant: stays below the new ceiling
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # always sleep between queue polls
    time.sleep(0.2)                     # brief mechanical settling before reading position

    position = motor.get_position()
    print(f"Move accepted and completed within the limit; "
          f"final position {position:.4f} shaft rotations")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
