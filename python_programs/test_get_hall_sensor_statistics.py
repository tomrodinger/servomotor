#!/usr/bin/env python3
"""
Per-command test for "Get hall sensor statistics" (cmd 33).

Cmd 33 has no input and returns a fixed 10-field reply built from the
firmware's `hall_sensor_statistics_t` struct
(firmware/Src/hall_sensor_calculations.h:20-25). Field order (as the
Python wrapper exposes it, matching the JSON Output list):

  [0] u16 maxHall1   [1] u16 maxHall2   [2] u16 maxHall3
  [3] u16 minHall1   [4] u16 minHall2   [5] u16 minHall3
  [6] u64 sumHall1   [7] u64 sumHall2   [8] u64 sumHall3
  [9] u32 measurementCount

Each per-channel hall reading is the SUM of 4 consecutive 12-bit ADC
samples (hall_sensor_calculations.c:65-70), so any single sample is in
[0, 4*4095] = [0, 16380]. min/max therefore live in that interval.

Gathering is controlled by cmd 32. Gathering is OFF at boot.

What this test verifies:

  1. Shape and types: 10-element list of ints, each field within its
     declared C-type range.
  2. Post-reset state (gathering OFF, never enabled this session):
     min[]=0, max[]=0, sum[]=0, n=0 — the zero-initialized static
     storage state.
  3. After cmd 32 control=1 and a short sample window, the reading is
     a *real* observation:
       - 0 <= min[i] <= max[i] <= 16380 for each channel
       - sum[i] / n lies in [min[i], max[i]] (the average must fit
         inside the observed envelope)
       - n is non-trivial (kilohertz ISR over hundreds of ms)
  4. With gathering ON across two reads (no reset between them), the
     per-channel min is non-increasing, max is non-decreasing, sum is
     non-decreasing, and n strictly grows. This is the documented
     accumulator semantic — a regression would either freeze them
     (n stuck) or reset them on read (cmd 33 is read-only).
  5. After cmd 32 control=1 (reset) the *next* read of min/max
     reflects fresh accumulators: min[i] > the previous max[i] is
     not legal in steady state, but right after the reset firmware
     sets min[i]=65535 and max[i]=0, and the first samples in
     produce min[i] <= 16380 and max[i] >= 0 with min[i] <= max[i].
     We assert min[i] != 0 (proves the 65535 seed was applied, not
     the boot-time zero), and max[i] > 0 (proves a sample landed).

The hall sensors of an M17 at rest sit around the midline; max is
strictly larger than min because of ADC noise, but the spread is
typically small (a few hundred counts).
"""

import argparse
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
SAMPLE_WINDOW_S = 0.3
SECOND_SAMPLE_WINDOW_S = 0.2

N_FIELDS = 10
U16_MAX = (1 << 16) - 1
U32_MAX = (1 << 32) - 1
U64_MAX = (1 << 64) - 1
HALL_ADC_SUM_MAX = 4 * 4095  # 4 ADC samples per channel
HALL_SEED_MIN = 65535         # what the firmware writes on reset+enable

MIN_EXPECTED_SAMPLES = 100    # ~32 kHz ISR over 0.3 s yields ~9.6 k samples; this is a loose floor


def parse(stats, label):
    if not isinstance(stats, list) or len(stats) != N_FIELDS:
        raise AssertionError(f"{label}: expected {N_FIELDS}-element list, got: {stats!r}")
    for i, v in enumerate(stats):
        if not isinstance(v, int):
            raise AssertionError(f"{label}: field {i} is not int: {v!r} ({type(v).__name__})")
    max_v = stats[0:3]
    min_v = stats[3:6]
    sum_v = stats[6:9]
    n = stats[9]
    for i in range(3):
        if not (0 <= max_v[i] <= U16_MAX):
            raise AssertionError(f"{label}: max[{i}]={max_v[i]} out of u16 range")
        if not (0 <= min_v[i] <= U16_MAX):
            raise AssertionError(f"{label}: min[{i}]={min_v[i]} out of u16 range")
        if not (0 <= sum_v[i] <= U64_MAX):
            raise AssertionError(f"{label}: sum[{i}]={sum_v[i]} out of u64 range")
    if not (0 <= n <= U32_MAX):
        raise AssertionError(f"{label}: n={n} out of u32 range")
    return min_v, max_v, sum_v, n


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get hall sensor statistics' (cmd 33).")
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
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device; cmd 33 should return the boot-zero state...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            stats0 = motor.get_hall_sensor_statistics()
            min0, max0, sum0, n0 = parse(stats0, "post-reset")
            print(f"  min={min0}, max={max0}, sum={sum0}, n={n0}")
            if min0 != [0, 0, 0] or max0 != [0, 0, 0] or sum0 != [0, 0, 0] or n0 != 0:
                raise AssertionError(f"post-reset stats not boot-zero: min={min0} max={max0} "
                                     f"sum={sum0} n={n0}")

            print(f"control=1 then wait {SAMPLE_WINDOW_S} s; the reading must be a real observation...")
            motor.control_hall_sensor_statistics(1)
            time.sleep(SAMPLE_WINDOW_S)
            stats1 = motor.get_hall_sensor_statistics()
            min1, max1, sum1, n1 = parse(stats1, "first running read")
            print(f"  min={min1}, max={max1}, sum={sum1}, n={n1}")
            if n1 < MIN_EXPECTED_SAMPLES:
                raise AssertionError(f"n={n1} < {MIN_EXPECTED_SAMPLES}; ISR not running?")
            for i in range(3):
                if not (0 <= min1[i] <= max1[i] <= HALL_ADC_SUM_MAX):
                    raise AssertionError(f"channel {i+1}: min={min1[i]}, max={max1[i]} not in "
                                         f"[0, {HALL_ADC_SUM_MAX}] or min > max")
                avg = sum1[i] / n1
                if not (min1[i] - 0.5 <= avg <= max1[i] + 0.5):  # +/-0.5 for rounding
                    raise AssertionError(f"channel {i+1}: avg=sum/n={avg:.2f} outside "
                                         f"[{min1[i]}, {max1[i]}]")

            print(f"Wait {SECOND_SAMPLE_WINDOW_S} s more; n must grow, min monotonic ↓, max monotonic ↑, sum ↑...")
            time.sleep(SECOND_SAMPLE_WINDOW_S)
            stats2 = motor.get_hall_sensor_statistics()
            min2, max2, sum2, n2 = parse(stats2, "second running read")
            print(f"  min={min2}, max={max2}, sum={sum2}, n={n2}")
            if n2 <= n1:
                raise AssertionError(f"measurementCount did not grow: {n1} -> {n2}")
            for i in range(3):
                if min2[i] > min1[i]:
                    raise AssertionError(f"channel {i+1}: min went UP without a reset: "
                                         f"{min1[i]} -> {min2[i]}")
                if max2[i] < max1[i]:
                    raise AssertionError(f"channel {i+1}: max went DOWN without a reset: "
                                         f"{max1[i]} -> {max2[i]}")
                if sum2[i] < sum1[i]:
                    raise AssertionError(f"channel {i+1}: sum went DOWN without a reset: "
                                         f"{sum1[i]} -> {sum2[i]}")

            print("control=1 (reset & enable); the next read must reflect fresh accumulators...")
            motor.control_hall_sensor_statistics(1)
            time.sleep(SAMPLE_WINDOW_S)
            stats3 = motor.get_hall_sensor_statistics()
            min3, max3, sum3, n3 = parse(stats3, "post-reset running read")
            print(f"  min={min3}, max={max3}, sum={sum3}, n={n3}")
            if n3 >= n2:
                raise AssertionError(f"after reset, n={n3} >= previous n={n2}; reset did not happen")
            for i in range(3):
                # Proof the 65535-seed was applied (firmware
                # hall_sensor_calculations.c:248-259): min[i] != 0 means the channel
                # has been written-to at least once since reset, AND a real sample
                # has come in (otherwise min would still be 65535 and max would be 0).
                if min3[i] == 0:
                    raise AssertionError(f"channel {i+1}: min={min3[i]} after reset+samples — "
                                         f"either reset did not seed to 65535 or the channel "
                                         f"actually saw a 0 ADC sum (implausible for an M17)")
                if max3[i] == 0:
                    raise AssertionError(f"channel {i+1}: max=0 after {n3} samples — sensor inactive?")
                if min3[i] > max3[i]:
                    raise AssertionError(f"channel {i+1}: min={min3[i]} > max={max3[i]}")

            # Leave stats disabled so we don't burden a later test.
            motor.control_hall_sensor_statistics(0)
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
