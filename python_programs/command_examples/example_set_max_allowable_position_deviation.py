#!/usr/bin/env python3
"""
Example: Set max allowable position deviation -- a collision/stall watchdog.
Tightens the allowed gap between commanded and measured position to 0.5
rotations, then performs a gentle move. If the shaft ever lags the command by
more than the limit (stall, jam, collision), fatal error 45 latches.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
MAX_DEVIATION = 0.5                     # shaft rotations (firmware default is 2)
DISPLACEMENT = 0.5                      # shaft rotations (gentle demo move)
DURATION = 2.0                          # seconds

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Order matters: enable + settle + zero FIRST, tighten the guard LAST.
    # The motor is ready immediately after enabling, but it may twitch as the
    # rotor snaps to a commutation step; we settle ONLY because we zero
    # precisely and arm a tight deviation limit right after -- doing that during
    # the twitch gives a corrupted zero or a spurious fatal error 45.
    motor.enable_mosfets()
    time.sleep(0.3)
    motor.zero_position()

    # Tighten the guard from the default 2 shaft rotations down to 0.5. From
    # now on, |commanded - measured| > 0.5 rotations latches fatal error 45
    # ASYNCHRONOUSLY -- the offending move itself still returns success, so
    # check get_status() after risky motion. The check stays armed even with
    # the MOSFETs disabled: back-driving the shaft past the limit trips it too.
    # The setting is RAM-only and reverts to 2 rotations on reset.
    motor.set_max_allowable_position_deviation(MAX_DEVIATION)
    print(f"Max allowable position deviation set to {MAX_DEVIATION} shaft rotations.")

    motor.trapezoid_move(DISPLACEMENT, DURATION)    # gentle move; tracks well within 0.5
    while motor.get_n_queued_items() > 0:           # wait for the queued move to finish
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading
    position = motor.get_position()
    print(f"Move completed with the deviation guard armed; now at "
          f"{position:.3f} shaft rotations.")
    print("If this motor stalls or collides mid-move, fatal error 45 latches; "
          "recover with system_reset() + 1.5 s of bus silence.")
finally:
    try:
        # Defensive: clear the tightened deviation guard before exit -- it stays
        # armed even with the MOSFETs disabled, so hand-turning the shaft past
        # 0.5 rotations after the demo would latch fatal error 45.
        motor.system_reset()
        time.sleep(1.5)                 # bus must stay silent after reset
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
