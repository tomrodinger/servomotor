#!/usr/bin/env python3
"""
Example: Get product description -- read the device's short description string.
Prints the fixed compile-time description (currently "Servomotor"). A quick,
side-effect-free way to confirm what kind of device is answering on the bus.
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

    # Single output -> the wrapper returns the bare string. The device transmits
    # the string INCLUDING its NUL terminator, so strip that before display.
    description = motor.get_product_description().rstrip('\x00')
    print(f"Product description: {description}")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
