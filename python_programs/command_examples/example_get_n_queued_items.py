#!/usr/bin/env python3
"""
Example: Get n queued items -- read how many moves are waiting in the motion queue.
Queues three short moves, then polls the count as it drains to 0. Polling this
command until it returns 0 is the canonical way to wait for motion to finish.
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
    time.sleep(0.3)                     # let the rotor settle onto its commutation step
    motor.zero_position()

    # Queue three short moves back-to-back; they execute in order. Each
    # trapezoid move may occupy up to 3 of the 32 queue slots.
    motor.trapezoid_move(0.5, 1.0)      # +0.5 rotation in 1 s
    motor.trapezoid_move(-0.5, 1.0)     # back again
    motor.trapezoid_move(0.5, 1.0)
    print("Queued 3 moves; watching the queue drain...")

    # Canonical wait-for-idle: poll until the queue reads 0, always sleeping
    # between polls (never busy-poll the bus).
    last_count = None
    while True:
        n = motor.get_n_queued_items()
        if n != last_count:
            print(f"  queued items: {n}")
            last_count = n
        if n == 0:
            break
        time.sleep(0.05)

    time.sleep(0.2)                     # brief mechanical settle before reading the position
    print(f"Motion complete. Final position: {motor.get_position():.3f} shaft rotations")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
