#!/usr/bin/env python3
"""
Example: Set maximum motor current -- cap the motor's drive strength.
Sets a working current limit of 200 before enabling the MOSFETs, then performs
a small move under that limit. The current limit doubles as a programmable
force limit: low values make the motor deliberately gentle and compliant.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
MOTOR_CURRENT = 200                     # internal units; good general working value
REGEN_CURRENT = 200                     # internal units; braking-side limit, same range rules
DISPLACEMENT = 1.0                      # shaft rotations (small, safe demo move)
DURATION = 2.0                          # seconds

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared",
                      current_unit="internal_current_units", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Set the current limit BEFORE enabling and moving -- the controller's
    # authority is derived from the value in effect at that moment. 200 is a
    # solid working value; ~390 is the practical maximum on the M17 (exceeding
    # the product ceiling latches fatal error 23). Going LOW (20-100) makes the
    # motor weak and compliant, so the current limit doubles as a programmable
    # force limit -- ideal for gentle homing or pressing softly against objects.
    motor.set_maximum_motor_current(MOTOR_CURRENT, REGEN_CURRENT)
    print(f"Maximum motor current set to {MOTOR_CURRENT} internal units "
          f"(regen: {REGEN_CURRENT}).")

    motor.enable_mosfets()
    time.sleep(0.3)                     # energizing snaps the rotor; let it settle
    motor.zero_position()               # establish the origin after the settle

    motor.trapezoid_move(DISPLACEMENT, DURATION)
    while motor.get_n_queued_items() > 0:   # wait for the queued move to finish
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading
    position = motor.get_position()
    print(f"Moved to {position:.3f} shaft rotations with the current limit at "
          f"{MOTOR_CURRENT}. Note: the limit is RAM-only and reverts on reset.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
