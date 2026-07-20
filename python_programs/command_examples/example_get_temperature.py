#!/usr/bin/env python3
"""
Example: Get temperature -- read the temperature of the motor driver PCB.
Reads the dedicated analog temperature sensor and prints degrees Celsius.
At room temperature expect 0 (below-range sentinel); under load the value climbs.
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
                      acceleration_unit="rotations_per_second_squared",
                      temperature_unit="celsius", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    temperature = motor.get_temperature()
    # The sensor's conversion table only covers about 33-307 C. Any reading
    # outside that range is reported as exactly 0 -- so 0 means "below ~33 C"
    # (e.g. a motor at room temperature), NOT freezing.
    if temperature == 0:
        print("Temperature: 0 C -- driver PCB is below ~33 C (out-of-range sentinel).")
    else:
        print(f"Driver PCB temperature: {temperature} C (accuracy about +/- 3 C)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
