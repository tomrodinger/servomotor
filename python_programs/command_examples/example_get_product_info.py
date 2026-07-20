#!/usr/bin/env python3
"""
Example: Get product info -- read the device's identity: model, versions, serial, unique ID.
Prints all six fields. Before a firmware upgrade, the product code and firmware
compatibility code shown here must match the firmware file.
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

    # Six outputs -> a flat list. The hardware version arrives least-significant-first
    # as [patch, minor, major], so reverse it for the usual major.minor.patch display.
    product_code, fw_compat, hw_version, serial_number, unique_id, reserved = \
        motor.get_product_info()

    print(f"Product code:           {product_code.strip()}")   # 8 chars, space-padded
    print(f"Firmware compatibility: {fw_compat}")
    print(f"Hardware version:       {hw_version[2]}.{hw_version[1]}.{hw_version[0]}")
    print(f"Serial number:          {serial_number}")
    print(f"Unique ID:              {unique_id:016X}")
    print(f"Reserved (unused):      {reserved}")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
