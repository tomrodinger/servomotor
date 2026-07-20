#!/usr/bin/env python3
"""
Example: Get supply voltage -- read the motor's power supply voltage.
Reads the bus voltage and prints it in volts. Expect your power supply's set
voltage within a few percent (these motors normally run from 12-24 V).
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
# voltage_unit="volts" matters: the library's default voltage unit is millivolts.
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared",
                      voltage_unit="volts", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    time.sleep(0.2)                     # ~0.2 s ADC settle -- already covered by the 1.5 s wait
                                        #  above; shown for when you read right after other resets

    # The device measures in tenths of a volt (averaging 4 ADC samples);
    # the library converts to the unit chosen above.
    voltage = motor.get_supply_voltage()
    print(f"Supply voltage: {voltage:.1f} V")
    # Sanity check: this should match your PSU setting within a few percent.
    # A wildly different reading points at a wiring or power problem.
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
