#!/usr/bin/env python3
"""
Example: Enable MOSFETs -- energize the motor driver outputs.
Enables the driver, waits out the mechanical settling transient, and verifies
the change via 'Get status' bit 1. You may feel the rotor snap and then hold.
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

MOSFETS_ENABLED_BIT = 1 << 1            # bit 1 of the 'Get status' flags

try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    flags, error_code = motor.get_status()
    print(f"Before enable: MOSFETs-enabled bit = {1 if flags & MOSFETS_ENABLED_BIT else 0}")

    motor.enable_mosfets()
    # The motor is ready to use immediately after enabling; it may twitch or rotate
    # slightly as the rotor snaps to a commutation step. The brief settle here is
    # only so the status read below and any precise zeroing happen after the twitch.
    time.sleep(0.3)

    flags, error_code = motor.get_status()
    print(f"After enable:  MOSFETs-enabled bit = {1 if flags & MOSFETS_ENABLED_BIT else 0} "
          f"(fatal error code {error_code})")
    print("The shaft should now resist being turned by hand.")

    motor.disable_mosfets()             # leave the motor de-energized when done
    print("MOSFETs disabled again; shaft is free.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
