#!/usr/bin/env python3
"""
Example: Vibrate -- turn the motor's vibration mode on for 2 seconds, then off.
IMPORTANT: only the M1 product physically vibrates. On M17, M23, and M2 this
command is a silent no-op that still returns success, so we check the product
code first and print which case applies.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
VIBRATE_SECONDS = 2.0                   # How long to leave vibration turned on

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # ONLY THE M1 PRODUCT VIBRATES. Other products (M17, M23, M2) accept this
    # command and reply with success, but nothing physically happens -- the
    # response alone cannot tell you, so identify the product first.
    info = motor.get_product_info()     # [productCode, fwCompat, hwVersion, serialNum, uniqueId, reserved]
    product = info[0].strip("\x00 ")    # productCode arrives as a padded 8-char string
    if product == "M1":
        print(f"Product {product}: the motor will physically vibrate now.")
    else:
        print(f"Product {product}: 'Vibrate' is a silent no-op on this product;")
        print("the commands below will still succeed, but the motor will not vibrate.")

    motor.vibrate(1)                    # nonzero = on; the amplitude is fixed in firmware,
                                        #  so the value does not scale the intensity
    print(f"Vibration ON for {VIBRATE_SECONDS:.0f} s ...")
    time.sleep(VIBRATE_SECONDS)
    motor.vibrate(0)                    # 0 = off (on M1 this leaves the MOSFETs enabled)
    print("Vibration OFF. Done.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
