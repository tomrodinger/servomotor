#!/usr/bin/env python3
"""
Example: Set safety limits -- fence the motor into a safe position window.
Zeroes the position, sets limits of -2..+2 rotations, then performs a move that
stays inside the fence. Moves that would leave the fence are REJECTED with a
latched fatal error -- they are never clamped or trimmed.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
LOWER_LIMIT = -2.0                      # shaft rotations
UPPER_LIMIT = 2.0                       # shaft rotations
DISPLACEMENT = 1.0                      # shaft rotations -- comfortably inside the fence
DURATION = 2.0                          # seconds

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Safe order: enable, settle 0.3 s, zero, THEN tighten limits. Energizing
    # snaps the rotor to a commutation step; zeroing after that transient keeps
    # the fence centered on the true position.
    motor.enable_mosfets()
    time.sleep(0.3)
    motor.zero_position()

    # The limits must bracket the CURRENT position (0 after zeroing): limits
    # that exclude it fault the device essentially instantly (fatal error 25).
    # Limits are RAM-only; a reset restores the +/-infinity defaults.
    motor.set_safety_limits(LOWER_LIMIT, UPPER_LIMIT)
    print(f"Safety limits set: {LOWER_LIMIT:+.1f} to {UPPER_LIMIT:+.1f} shaft rotations.")

    motor.trapezoid_move(DISPLACEMENT, DURATION)    # compliant move: ends at +1.0, inside
    while motor.get_n_queued_items() > 0:           # wait for the queued move to finish
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading
    position = motor.get_position()
    print(f"Move inside the limits succeeded; now at {position:.3f} shaft rotations.")

    # Do NOT queue e.g. motor.trapezoid_move(5.0, 2.0) here: a move whose end
    # position falls outside the limits is rejected with latched fatal error 27
    # (a reversal whose turn-around point falls outside them is rejected with fatal error 26 or 27 -- in practice the predicted-position check (27) usually fires first). The motor then
    # disables itself and only system_reset() + 1.5 s wait recovers it.
finally:
    try:
        motor.system_reset()            # defensive: clear the tightened limits before exit
        time.sleep(1.5)                 # bus must stay silent after reset
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
