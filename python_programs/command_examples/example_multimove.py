#!/usr/bin/env python3
"""
Example: Multimove -- queue several mixed velocity/acceleration moves in one packet.
Sends 3 moves in one command: jump to 1 rot/s for 1 s, decelerate at
-1 rot/s^2 for 1 s (back to standstill), then hold velocity 0 briefly.
The shaft turns ~1.5 rotations and comes smoothly to rest.
"""
import time
import servomotor
from servomotor import communication

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.

# Multimove takes RAW INTERNAL units, so we convert by hand using the factors
# published in servomotor/unit_conversions_M3.json:
TIMESTEPS_PER_SECOND = 31250            # internal time unit: 1 timestep = 32 us
VELOCITY_FACTOR = 109951162.7776        # internal velocity units per rotation/second
ACCELERATION_FACTOR = 56294.9953421312  # internal accel units per rotation/second^2

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    motor.enable_mosfets()
    time.sleep(0.3)                     # let the rotor settle on a commutation step
    motor.zero_position()

    vel = round(1.0 * VELOCITY_FACTOR)          # 1 rotation/second
    acc = round(-1.0 * ACCELERATION_FACTOR)     # -1 rotation/second^2 (deceleration)
    t_1s = round(1.0 * TIMESTEPS_PER_SECOND)    # 1 second
    t_01s = round(0.1 * TIMESTEPS_PER_SECOND)   # 0.1 seconds

    # moveTypes is a bitmask, LSB = first move: bit=1 -> velocity move,
    # bit=0 -> acceleration move. 0b101 = moves 0 and 2 are velocity moves,
    # move 1 is an acceleration move. The plan MUST end at zero velocity:
    # if the queue empties at nonzero velocity the firmware raises fatal
    # error 18 (ERROR_RUN_OUT_OF_QUEUE_ITEMS) and disables itself until reset.
    move_types = 0b101
    move_list = f"[[{vel}, {t_1s}], [{acc}, {t_1s}], [0, {t_01s}]]"

    # The high-level M3.multimove() wrapper cannot convert the mixed
    # velocity/acceleration/time list, so drive command 29 through the
    # library's low-level execute_command with internal-unit values.
    communication.execute_command(29, [3, move_types, move_list],
                                  alias_or_unique_id=motor.alias_or_unique_id,
                                  verbose=0)

    while motor.get_n_queued_items() > 0:   # wait for all 3 queued moves to execute
        time.sleep(0.05)
    time.sleep(0.2)                     # brief mechanical settling before reading

    position = motor.get_position()
    print(f"Final position: {position:.3f} shaft rotations (expected ~1.5)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
