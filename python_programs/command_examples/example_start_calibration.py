#!/usr/bin/env python3
"""
Example: Start calibration -- measure the hall sensors and commutation offset.
Motors are calibrated at the FACTORY and the results persist in flash, so a new
motor works in closed loop out of the box -- you rarely need this command.
Recalibrate only if closed-loop control misbehaves or after hardware service.
The shaft spins about 1.5 turns back and forth for roughly 20-60 s
(product-dependent; about 20 s measured on an M17), then the device saves the
results to flash and automatically REBOOTS itself. The bus must stay
COMPLETELY QUIET from the start command until well after that reboot.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
CALIBRATION_WAIT = 60.0                 # seconds of total bus silence; covers all products
                                        #  (about 20 s measured on an M17)

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    print("Note: motors are calibrated at the factory and the results persist in")
    print("flash, so running this is rarely needed.")

    # PRECONDITIONS: the shaft must be COMPLETELY FREE to rotate (no load, no
    # coupling). Calibration also requires MOSFETs disabled, an empty queue, no
    # test mode, and open-loop mode -- the fresh reset below guarantees all that.
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    motor.start_calibration()           # the success response only means it STARTED
    print(f"Calibration started; keeping the bus quiet for {CALIBRATION_WAIT:.0f} s ...")

    # Do NOT poll while calibration runs: polling during calibration can disturb
    # the measurement and reduce its accuracy, and when calibration finishes the
    # device saves to flash and automatically REBOOTS -- a poll landing after
    # that reboot pins the device in the bootloader. A generous fixed wait avoids
    # both hazards; 60 s covers all products (about 20 s measured on an M17).
    time.sleep(CALIBRATION_WAIT)

    # The auto-reboot erased all volatile state (zeroed position, limits, enabled
    # MOSFETs). Reset once more and verify a clean baseline before trusting it.
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    flags, fatal_error = motor.get_status()
    in_bootloader = bool(flags & (1 << 0))
    mosfets_on = bool(flags & (1 << 1))
    print(f"Post-calibration baseline: bootloader={in_bootloader}, "
          f"MOSFETs enabled={mosfets_on}, fatal error code={fatal_error}")
    if in_bootloader or mosfets_on or fatal_error != 0:
        raise RuntimeError("Device did not come back to a clean baseline")
    print("Calibration complete; results are saved in flash and persist across power cycles.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
