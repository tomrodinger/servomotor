#!/usr/bin/env python3
"""
Example: Capture hall sensor data -- record the three raw hall-sensor ADC channels during a move.
Starts a slow 1-rotation move, then captures 500 points from all three hall sensors while the
shaft turns. You should see the motor rotate slowly and the first captured points printed as
three ADC values per point.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
servomotor.set_serial_port(SERIAL_PORT)

MOVE_ROTATIONS = 1.0                    # shaft rotations (small, safe motion)
MOVE_DURATION = 5.0                     # seconds; slow enough that the capture happens mid-move
CAPTURE_TYPE = 1                        # 1 = raw hall-sensor ADC readings (3 channels)
N_POINTS = 500                          # points to read back
CHANNEL_BITMASK = 7                     # bits 0-2 set = capture all three hall sensors
TIME_STEPS_PER_SAMPLE = 8               # one sample every 8 time steps (1 time step = 32 us)
N_SAMPLES_TO_SUM = 16                   # samples summed into one transmitted point
DIVISION_FACTOR = 8                     # sample sum is divided by this before transmission

servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    motor.enable_mosfets()
    time.sleep(0.3)                     # energizing snaps the rotor to a step; let it settle
    motor.zero_position()
    # Generous deviation limit: the device ignores ALL bus traffic until the capture
    # finishes, so nothing must be able to trip a fatal error while we cannot intervene.
    motor.set_max_allowable_position_deviation(100)   # shaft rotations

    # Queue the motion BEFORE capturing: there is no way to send commands (or abort)
    # once the capture starts.
    motor.trapezoid_move(MOVE_ROTATIONS, MOVE_DURATION)
    time.sleep(0.2)                     # let the motor accelerate to a steady velocity

    # This call blocks while the response bytes stream in over the whole capture:
    # 500 points * 8 time steps * 16 sums * 32 us = about 2 s here.
    data = motor.capture_hall_sensor_data(CAPTURE_TYPE, N_POINTS, CHANNEL_BITMASK,
                                          TIME_STEPS_PER_SAMPLE, N_SAMPLES_TO_SUM,
                                          DIVISION_FACTOR)
    print(f"Received {len(data)} bytes = {len(data) // 6} points x 3 channels x 2 bytes")
    for i in range(0, 10 * 6, 6):       # decode the first 10 points (little-endian u16 triplets)
        h1 = int.from_bytes(data[i + 0:i + 2], "little")
        h2 = int.from_bytes(data[i + 2:i + 4], "little")
        h3 = int.from_bytes(data[i + 4:i + 6], "little")
        print(f"point {i // 6}: hall1={h1}  hall2={h2}  hall3={h3} (ADC units)")

    while motor.get_n_queued_items() > 0:   # let the queued move finish before cleanup
        time.sleep(0.05)
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
