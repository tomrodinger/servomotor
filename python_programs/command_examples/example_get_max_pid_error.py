#!/usr/bin/env python3
"""
Example: Get max PID error -- measure closed-loop tracking quality.
Enters closed loop, clears the error window, performs a move, then reads the
[min, max] position error the PID controller saw during that move.
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
    motor.go_to_closed_loop()           # the error only accumulates while the PID loop runs,
                                        #  i.e. in closed loop. This enables the MOSFETs by
                                        #  itself; skipping a separate 'Enable MOSFETs' may
                                        #  even give a gentler engagement.
    deadline = time.time() + 6.0
    while True:
        status_flags, fatal_error_code = motor.get_status()
        if status_flags & (1 << 2):     # bit 2 = closed-loop mode active
            break
        if time.time() > deadline:
            raise TimeoutError("Never entered closed loop -- is the motor calibrated?")
        time.sleep(0.1)
    time.sleep(0.3)                     # settle after the mode change so the zero is clean
    motor.zero_position()
    # Every read RESETS the min/max window, so read once here and throw the result
    # away -- the next read then covers exactly the move below.
    motor.get_max_pid_error()
    motor.trapezoid_move(1.0, 2.0)      # small safe demo move: 1 rotation in 2 seconds
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # poll the queue; never busy-poll without a sleep
    time.sleep(0.2)                     # brief mechanical settling
    motor.set_position_unit("encoder_counts")   # the controller's native error unit
    min_err, max_err = motor.get_max_pid_error()
    # min > max means the reset sentinels came back (+2147483647 / -2147483648):
    # no PID samples since the last read -- "no data", not a huge real error.
    if min_err > max_err:
        print("No PID data accumulated since the last read")
    else:
        print(f"Tracking error during the move: min={min_err:.0f}, max={max_err:.0f} encoder counts")
        print("(1 shaft rotation = 3,276,800 counts; positive max = measured position lagged commanded)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
