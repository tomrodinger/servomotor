#!/usr/bin/env python3
"""
Example: Emergency stop -- halt motion instantly, clear the queue, disable the MOSFETs.
Starts a slow 10-second move, stops it after 1 second, then shows the queue is empty
and the MOSFETs are off. After the stop the shaft freewheels (no holding torque).
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

    motor.enable_mosfets()
    time.sleep(0.3)                     # let the rotor settle onto its commutation step
    motor.zero_position()

    motor.trapezoid_move(5.0, 10.0)     # slow, long move: 5 rotations over 10 s
    time.sleep(1.0)                     # let the motion get underway
    motor.emergency_stop()              # executes immediately: queue cleared, MOSFETs disabled

    queued = motor.get_n_queued_items()
    status_flags, fatal_error_code = motor.get_status()
    print(f"Queued items after stop:    {queued} (expect 0)")
    print(f"MOSFETs-enabled status bit: {(status_flags >> 1) & 1} (expect 0)")
    print(f"Fatal error code:           {fatal_error_code} (expect 0 -- an emergency stop is NOT a fatal error)")
    print(f"Stopped at position:        {motor.get_position():.3f} shaft rotations")

    # Recovery: since this is not a fatal error, enable_mosfets() (+0.3 s settle)
    # is enough to power the motor again. On current firmware (>= 0.15.4.0) the stop also resets the
    # planner's internal state, so enable_mosfets() (+0.3 s settle) is all that is
    # needed before queueing new moves. (Only firmware older than 0.15.4.0 left a
    # stale planner value and required a system reset first.)
    motor.system_reset()
    time.sleep(1.5)                     # bus silent again after reset
    print("Reset complete -- safe to queue new moves now.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
