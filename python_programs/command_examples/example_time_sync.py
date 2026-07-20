#!/usr/bin/env python3
"""
Example: Time sync -- discipline the motor's clock rate to the host's clock.
Zeros the device clock to establish a shared epoch, then sends the host's
elapsed microseconds at 10 Hz for 3 seconds, printing the motor's reported
clock error each time. Used before multi-motor coordinated motion.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
SYNC_DURATION = 3.0                     # seconds of syncing to demonstrate
SYNC_INTERVAL = 0.1                     # seconds between syncs (10 Hz is the recommended rate)
servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Establish a shared epoch: zero the device clock and record the host clock
    # at the same instant. (On a multi-motor bus, broadcast 'Reset time' to
    # alias 255 so all clocks zero together -- it also stops motion, so do it
    # before any motion starts.)
    motor.reset_time()
    epoch = time.monotonic()

    # 'Time sync' never sets the clock -- it trims the internal oscillator, i.e.
    # the clock's RATE, so the error converges gradually; budget ~5 s of regular
    # syncing before trusting tight synchronization. A broadcast time_sync is a
    # complete no-op: each motor on the bus must be addressed individually.
    print("Syncing at 10 Hz (positive error = motor clock behind the host)...")
    while time.monotonic() - epoch < SYNC_DURATION:
        elapsed_s = time.monotonic() - epoch          # master time in seconds
        time_error_us, rcc_icscr = motor.time_sync(elapsed_s)
        print(f"host clock {elapsed_s:>9.6f} s | motor clock error {time_error_us:>6} us (raw microseconds)")
        time.sleep(SYNC_INTERVAL)
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
