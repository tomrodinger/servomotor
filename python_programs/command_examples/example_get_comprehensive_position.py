#!/usr/bin/env python3
"""
Example: Get comprehensive position -- commanded, measured, and external
encoder positions in one round trip. Performs a small move, then prints all
three values from a single query (safe to poll at 20 Hz during motion).
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

    motor.trapezoid_move(MOVE_ROTATIONS, MOVE_SECONDS)
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # poll with sleeps -- never busy-poll the bus
    time.sleep(0.2)                     # brief mechanical settling before reading

    # One command returns all three positions: [commanded, hall, external encoder]
    commanded, measured, external = motor.get_comprehensive_position()
    print(f"Commanded position:     {commanded:+.4f} shaft rotations")
    print(f"Hall sensor position:   {measured:+.4f} shaft rotations")
    # The external encoder value comes from an OPTIONAL add-on quadrature encoder:
    # it reads 0 when none is fitted and is never zeroed by any command -- not
    # even 'Zero position'. The library wrongly applies the motor's position
    # conversion to this raw count (a known wart); multiply back by the
    # shaft_rotations factor to recover the encoder's own counts.
    print(f"External encoder value: {external * 3276800:.0f} raw counts")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
