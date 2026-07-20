#!/usr/bin/env python3
"""
Example: Disable MOSFETs -- de-energize the motor driver outputs.
Enables the driver, then disables it, and shows the change in the 'Get status'
MOSFETs-enabled bit (bit 1). Once disabled, the shaft turns freely by hand.
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

    motor.enable_mosfets()
    time.sleep(0.3)                     # let the rotor's commutation-snap transient settle
    flags, error_code = motor.get_status()
    print(f"After enable:  MOSFETs-enabled bit = {1 if flags & MOSFETS_ENABLED_BIT else 0} "
          f"(fatal error code {error_code})")

    # WARNING: 'Disable MOSFETs' does NOT stop or clear queued motion. Queued moves keep
    # advancing the commanded position with the outputs off, and the position-deviation
    # check then trips fatal error 45. Only disable when the queue is empty (as here) or
    # together with an emergency stop.
    motor.disable_mosfets()
    flags, error_code = motor.get_status()
    print(f"After disable: MOSFETs-enabled bit = {1 if flags & MOSFETS_ENABLED_BIT else 0} "
          f"(fatal error code {error_code})")
    print("The shaft should now turn freely by hand.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
