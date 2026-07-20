#!/usr/bin/env python3
"""
Example: Homing -- find a mechanical hard stop by moving until the motor stalls against it.
Enters closed loop, lowers the motor current so it only presses gently, homes up to
2 rotations in the negative direction, restores the current, and zeroes at the stop.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
HOMING_CURRENT = 50                     # internal current units: a gentle press into the hard stop
WORKING_CURRENT = 200                   # internal current units: normal working torque
MAX_DISPLACEMENT = -2.0                 # shaft rotations to search; the SIGN sets the direction
MAX_DURATION = 5.0                      # seconds allotted for the homing move
servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared",
                      current_unit="internal_current_units", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Homing requires closed-loop mode. go_to_closed_loop() enables the MOSFETs
    # by itself; skipping a separate 'Enable MOSFETs' may even give a gentler
    # engagement.
    motor.go_to_closed_loop()
    deadline = time.time() + 6.0
    while (motor.get_status()[0] >> 2) & 1 == 0:    # wait for the closed-loop status bit
        if time.time() > deadline:
            raise TimeoutError("Never entered closed loop -- is the motor calibrated?")
        time.sleep(0.1)
    time.sleep(0.3)                     # settle after the closed-loop transition

    # Soft-press homing: with a reduced current limit the motor can only push
    # gently, so hitting the hard stop is harmless. Args: (motor, regeneration) current.
    motor.set_maximum_motor_current(HOMING_CURRENT, HOMING_CURRENT)
    motor.homing(MAX_DISPLACEMENT, MAX_DURATION)
    deadline = time.time() + MAX_DURATION + 2.0
    while True:                         # poll status bit 4 (homing in progress) until it clears
        status_flags, fatal_error_code = motor.get_status()
        if fatal_error_code != 0:
            motor.system_reset()        # fatal errors latch; reset is the only way out
            time.sleep(1.5)             # bus silent after reset
            raise RuntimeError(f"Fatal error {fatal_error_code} during homing")
        if (status_flags >> 4) & 1 == 0:
            break
        if time.time() > deadline:
            raise TimeoutError("Homing did not finish in time")
        time.sleep(0.1)

    # If no obstacle was hit, the motor traveled the full MAX_DISPLACEMENT --
    # only the traveled distance tells the two outcomes apart.
    print(f"Homing done; traveled {motor.get_position():.3f} rotations from the start")

    motor.set_maximum_motor_current(WORKING_CURRENT, WORKING_CURRENT)
    motor.zero_position()               # homing does NOT zero -- establish the origin ourselves
    print(f"Origin set at the hard stop; position now {motor.get_position():.4f} rotations")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
