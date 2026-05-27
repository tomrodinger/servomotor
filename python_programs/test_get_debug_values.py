#!/usr/bin/env python3
"""
Per-command test for "Get debug values" (cmd 45).

Cmd 45 has no input. It returns a fixed 30-field packed reply assembled
by the firmware at main.c:1022-1086. Fields, in order:

  0  i64 max_acceleration
  1  i64 max_velocity
  2  i64 current_velocity
  3  i32 measured_velocity
  4  u32 n_time_steps
  5  i64 debug_value1
  6  i64 debug_value2
  7  i64 debug_value3
  8  i64 debug_value4
  9  u16 all_motor_control_calculations_profiler_time
 10  u16 all_motor_control_calculations_profiler_max_time
 11  u16 get_sensor_position_profiler_time
 12  u16 get_sensor_position_profiler_max_time
 13  u16 compute_velocity_profiler_time
 14  u16 compute_velocity_profiler_max_time
 15  u16 motor_movement_calculations_profiler_time
 16  u16 motor_movement_calculations_profiler_max_time
 17  u16 motor_phase_calculations_profiler_time
 18  u16 motor_phase_calculations_profiler_max_time
 19  u16 motor_control_loop_period_profiler_time
 20  u16 motor_control_loop_period_profiler_max_time
 21  u16 hall_sensor1_voltage
 22  u16 hall_sensor2_voltage
 23  u16 hall_sensor3_voltage
 24  u32 commutation_position_offset
 25  u8  motor_phases_reversed
 26  i32 max_hall_position_delta
 27  i32 min_hall_position_delta
 28  i32 average_hall_position_delta
 29  u8  motor_pwm_voltage

What this test verifies:

  1. Shape: response is a 30-element list of ints (the framing of this
     fat reply round-trips).
  2. Per-field type ranges: each field fits its declared C type. A
     wrapper bug that confused signedness or width would show up here
     (e.g. a u32 surfacing as i32 would wrap negative for large values).
  3. Quiescent post-reset values: motor not enabled, queue empty.
       - current_velocity == 0 (this is the commanded velocity)
       - |measured_velocity| <= MEASURED_VELOCITY_NOISE_BAND (the hall-
         derived measurement carries a few counts of ADC noise even at
         rest; we allow that, but not a runaway value)
       - n_time_steps == 0
       - motor_phases_reversed in {0, 1}
     This catches a regression where the values become non-deterministic
     after reset.
  4. Hall sensor voltages are inside the 12-bit-x-4-sums envelope
     (0 .. 4*4095 = 16380). The firmware sums 4 ADC samples per channel
     (hall_sensor_calculations.c:65-70).
  5. Cross-read stability: max_acceleration and max_velocity do not
     change across consecutive reads on a quiet device (they are
     settings, not measurements). This proves we are not reading
     garbage.
     (NOTE: the *profiler* "max time" fields and the hall_position_delta
     min/max/avg fields are *read-with-reset* in firmware — see
     profiler.c:44-47 and motor_control.c:1572-1584 — so they are NOT
     monotonic across reads. Asserting monotonicity on them would be
     wrong. Verifying only that they fall within their declared type
     range is the correct check.)
  6. Liveness under motion: enable, enter closed loop, queue a small
     move; while a move is queued, current_velocity becomes
     non-zero at some sample. After the move drains and the motor
     settles, current_velocity returns to 0. This proves the field is
     wired to live state, not a constant.

This test needs a calibrated motor (closed loop requires it). The
bench M17 is calibrated per WORK_CHECKLIST.md.
"""

import argparse
import sys
import time
import servomotor

COUNTS_PER_ROTATION = 3276800
CLOSED_LOOP_BIT = 1 << 2
RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
CLOSED_LOOP_TIMEOUT_S = 6.0
HALL_ADC_SUM_MAX = 4 * 4095  # 4 ADC samples per channel, each 12-bit
MEASURED_VELOCITY_NOISE_BAND = 200  # plenty of headroom over the ~3-count idle jitter

I64_MIN, I64_MAX = -(1 << 63), (1 << 63) - 1
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1
U32_MAX = (1 << 32) - 1
U16_MAX = (1 << 16) - 1
U8_MAX = (1 << 8) - 1

# (name, lower, upper) per field index — mirrors the firmware struct order.
FIELD_RANGES = [
    ("max_acceleration",                                  I64_MIN, I64_MAX),
    ("max_velocity",                                      I64_MIN, I64_MAX),
    ("current_velocity",                                  I64_MIN, I64_MAX),
    ("measured_velocity",                                 I32_MIN, I32_MAX),
    ("n_time_steps",                                      0, U32_MAX),
    ("debug_value1",                                      I64_MIN, I64_MAX),
    ("debug_value2",                                      I64_MIN, I64_MAX),
    ("debug_value3",                                      I64_MIN, I64_MAX),
    ("debug_value4",                                      I64_MIN, I64_MAX),
    ("all_motor_control_calculations_profiler_time",      0, U16_MAX),
    ("all_motor_control_calculations_profiler_max_time",  0, U16_MAX),
    ("get_sensor_position_profiler_time",                 0, U16_MAX),
    ("get_sensor_position_profiler_max_time",             0, U16_MAX),
    ("compute_velocity_profiler_time",                    0, U16_MAX),
    ("compute_velocity_profiler_max_time",                0, U16_MAX),
    ("motor_movement_calculations_profiler_time",         0, U16_MAX),
    ("motor_movement_calculations_profiler_max_time",     0, U16_MAX),
    ("motor_phase_calculations_profiler_time",            0, U16_MAX),
    ("motor_phase_calculations_profiler_max_time",        0, U16_MAX),
    ("motor_control_loop_period_profiler_time",           0, U16_MAX),
    ("motor_control_loop_period_profiler_max_time",       0, U16_MAX),
    ("hall_sensor1_voltage",                              0, HALL_ADC_SUM_MAX),
    ("hall_sensor2_voltage",                              0, HALL_ADC_SUM_MAX),
    ("hall_sensor3_voltage",                              0, HALL_ADC_SUM_MAX),
    ("commutation_position_offset",                       0, U32_MAX),
    ("motor_phases_reversed",                             0, 1),
    ("max_hall_position_delta",                           I32_MIN, I32_MAX),
    ("min_hall_position_delta",                           I32_MIN, I32_MAX),
    ("average_hall_position_delta",                       I32_MIN, I32_MAX),
    ("motor_pwm_voltage",                                 0, U8_MAX),
]
N_FIELDS = len(FIELD_RANGES)

def named(values):
    return {name: v for (name, _, _), v in zip(FIELD_RANGES, values)}


def check_response(values, label):
    if not isinstance(values, list) or len(values) != N_FIELDS:
        raise AssertionError(f"{label}: expected {N_FIELDS}-element list, got: "
                             f"{type(values).__name__} len="
                             f"{len(values) if hasattr(values, '__len__') else 'n/a'}")
    for i, ((name, lo, hi), v) in enumerate(zip(FIELD_RANGES, values)):
        if not isinstance(v, int):
            raise AssertionError(f"{label}: field {i} '{name}' is not int: {v!r} "
                                 f"({type(v).__name__})")
        if not (lo <= v <= hi):
            raise AssertionError(f"{label}: field {i} '{name}'={v} out of "
                                 f"declared range [{lo}, {hi}]")


def wait_for_closed_loop(motor, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = motor.get_status()
        if status and len(status) >= 2 and (status[0] & CLOSED_LOOP_BIT):
            return status[0]
        time.sleep(0.1)
    raise AssertionError(f"closed-loop flag never set within {timeout_s} s; "
                         f"is the motor calibrated?")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get debug values' (cmd 45).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device and reading debug values at idle...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            r0 = motor.get_debug_values()
            check_response(r0, "post-reset")
            nv0 = named(r0)
            print(f"  current_velocity={nv0['current_velocity']}, "
                  f"measured_velocity={nv0['measured_velocity']}, "
                  f"n_time_steps={nv0['n_time_steps']}, "
                  f"hall=[{nv0['hall_sensor1_voltage']}, {nv0['hall_sensor2_voltage']}, "
                  f"{nv0['hall_sensor3_voltage']}], phases_reversed={nv0['motor_phases_reversed']}")
            if nv0['current_velocity'] != 0:
                raise AssertionError(f"post-reset current_velocity={nv0['current_velocity']}, expected 0")
            if abs(nv0['measured_velocity']) > MEASURED_VELOCITY_NOISE_BAND:
                raise AssertionError(f"post-reset measured_velocity={nv0['measured_velocity']} exceeds "
                                     f"|x|<={MEASURED_VELOCITY_NOISE_BAND} idle noise band")
            if nv0['n_time_steps'] != 0:
                raise AssertionError(f"post-reset n_time_steps={nv0['n_time_steps']}, expected 0 (queue empty)")

            print("Reading again on a quiet device; the settings fields must be stable...")
            r1 = motor.get_debug_values()
            check_response(r1, "second read")
            nv1 = named(r1)
            for setting in ("max_acceleration", "max_velocity"):
                if nv0[setting] != nv1[setting]:
                    raise AssertionError(f"{setting} drifted on a quiet device: "
                                         f"{nv0[setting]} -> {nv1[setting]}")

            print("Entering closed loop and queueing a small move; current_velocity must go non-zero...")
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.go_to_closed_loop()
            wait_for_closed_loop(motor, CLOSED_LOOP_TIMEOUT_S)
            motor.zero_position()
            target = COUNTS_PER_ROTATION // 8
            motor.trapezoid_move(target, 1.0)
            saw_motion = False
            saw_n_time_steps = False
            t_end = time.time() + 1.5
            while time.time() < t_end:
                rm = motor.get_debug_values()
                check_response(rm, "during move")
                nvm = named(rm)
                if nvm['current_velocity'] != 0:
                    saw_motion = True
                if nvm['n_time_steps'] > 0:
                    saw_n_time_steps = True
                if motor.get_n_queued_items() == 0:
                    break
                time.sleep(0.02)
            if not saw_motion:
                raise AssertionError("current_velocity never went non-zero during the move")
            if not saw_n_time_steps:
                raise AssertionError("n_time_steps never went above 0 while a move was queued")

            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.3)

            print("Move drained; current_velocity must be 0 again...")
            r_end = motor.get_debug_values()
            check_response(r_end, "after move")
            nv_end = named(r_end)
            if nv_end['current_velocity'] != 0:
                raise AssertionError(f"after move current_velocity={nv_end['current_velocity']}, expected 0")
            if nv_end['n_time_steps'] != 0:
                raise AssertionError(f"after move n_time_steps={nv_end['n_time_steps']}, expected 0")
            if abs(nv_end['measured_velocity']) > MEASURED_VELOCITY_NOISE_BAND:
                raise AssertionError(f"after move measured_velocity={nv_end['measured_velocity']} exceeds "
                                     f"|x|<={MEASURED_VELOCITY_NOISE_BAND} idle noise band")

            print("Returning to zero, dropping closed loop, resetting...")
            motor.trapezoid_move(0, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            motor.disable_mosfets()
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        if motor is not None:
            try:
                motor.disable_mosfets()
            except Exception:
                pass
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
