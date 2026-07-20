#!/usr/bin/env python3
"""
Example: Go to closed loop -- switch the motor into closed-loop position control.
Enters closed-loop mode (requires a motor that has been calibrated once with
'Start calibration'), then performs a small move. Watch the shaft turn 1 rotation.
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
    motor.go_to_closed_loop()           # enables the MOSFETs by itself -- no separate
                                        #  'Enable MOSFETs' needed; skipping it may even
                                        #  give a gentler engagement
    deadline = time.time() + 6.0
    while True:
        status_flags, fatal_error_code = motor.get_status()
        if status_flags & (1 << 2):     # bit 2 = closed-loop mode active
            break
        if time.time() > deadline:
            raise TimeoutError("Never entered closed loop -- is the motor calibrated?")
        time.sleep(0.1)
    print("Motor is now in closed-loop mode")
    time.sleep(0.3)                     # settle after the mode change so the zero is clean
    motor.zero_position()
    motor.trapezoid_move(1.0, 2.0)      # small safe demo move: 1 rotation in 2 seconds
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # poll the queue; never busy-poll without a sleep
    time.sleep(0.2)                     # brief mechanical settling before reading position
    print(f"Move complete; position = {motor.get_position():.3f} shaft rotations")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
