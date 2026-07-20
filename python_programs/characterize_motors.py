#!/usr/bin/env python3
"""
Characterize the physical performance envelope of every servomotor on the bus:
the maximum velocity (rotations/second) and maximum acceleration
(rotations/second^2) each motor can actually achieve at its full current limit
and the present supply voltage.

Method (per motor, addressed by unique ID so duplicate aliases are harmless):
  1. Reset, enter closed loop at maximum current (390), zero the position.
  2. Calibrate the hall-capture scale: capture the position stream during a
     known 0.5-rotation trapezoid move (the capture's position unit is not
     encoder counts, so we derive raw-units-per-rotation empirically).
  3. For each direction (+/-), THREE trials: queue a short dwell followed by an
     instantaneous velocity step to an UNATTAINABLE speed (15 rot/s), then
     capture the fused hall position at 128 microseconds/sample while the
     motor spins up. The motor accelerates as hard as its torque allows and
     tops out at its physical maximum:
       - max velocity  = median of the speed plateau of the trace
       - max accel     = the 10-90% rise-time acceleration of the first
                         spin-up ramp, with sub-sample interpolation (the
                         unloaded rotor reaches full speed in under 1 ms).
                         NOTE: free-shaft values are torque/inertia-limited
                         and will be much lower under load.
  4. Per-motor result = median across trials, both directions.

Outputs:
  - motor_characterization_results.json  (all per-motor numbers + conditions)
  - motor_characterization_histogram.png (histograms in rot/s and rot/s^2)

Usage:
  python3 characterize_motors.py -p /dev/ttyUSB0          # discover all motors
  python3 characterize_motors.py -p /dev/ttyUSB0 -a X     # only alias X
  python3 characterize_motors.py -P                       # interactive port menu
"""

import argparse
import json
import statistics
import struct
import sys
import time

import servomotor
from servomotor import communication

C_PER_ROT = 3276800          # encoder counts per shaft rotation (M17/M23)
CAPTURE_DT = 2 * 2 * 32e-6   # timeStepsPerSample * nSamplesToSum * 32 us = 128 us
CAPTURE_POINTS = 2800        # 2800 * 128 us = 0.358 s window
# NOTE: the capture position stream is u16 and must be unwrapped; at 128 us/sample the
# largest per-sample delta at ~10 rot/s is ~17k raw units, safely below the 32768 wrap
# limit (at 1 ms sampling the stream aliases and velocities read ~6x too low).
STEP_VELOCITY = 15.0         # rot/s, deliberately above any attainable speed
STEP_DURATION = 0.4          # s
LEAD_DWELL = 0.05            # s of zero-velocity dwell so the capture catches the whole ramp
TRIALS_PER_DIRECTION = 3
CURRENT_LIMIT = 390          # internal units = full authority
RESET_DELAY_S = 1.5


def unwrap_u16(values):
    out = [values[0]]
    for v in values[1:]:
        d = (v - (out[-1] & 0xFFFF) + 32768) % 65536 - 32768
        out.append(out[-1] + d)
    return out


def smooth(xs, n=5):
    half = n // 2
    return [sum(xs[max(0, i - half):i + half + 1]) / len(xs[max(0, i - half):i + half + 1])
            for i in range(len(xs))]


def linear_slope(ts, vs):
    n = len(ts)
    st, sv = sum(ts), sum(vs)
    stt = sum(t * t for t in ts)
    stv = sum(t * v for t, v in zip(ts, vs))
    denom = n * stt - st * st
    return (n * stv - st * sv) / denom if denom else 0.0


def capture_position_trace(motor):
    """Run a capture and return the unwrapped raw position list."""
    data = bytes(motor.capture_hall_sensor_data(2, CAPTURE_POINTS, 1, 2, 2, 1))
    pts = list(struct.unpack(f'<{len(data) // 2}H', data))
    return unwrap_u16(pts)


def settle(motor):
    while motor.get_n_queued_items() > 0:
        time.sleep(0.03)
    prev = None
    for _ in range(100):
        time.sleep(0.12)
        p = motor.get_hall_sensor_position()
        if prev is not None and abs(p - prev) * C_PER_ROT < 30:
            return p
        prev = p
    return prev


def enter_closed_loop(motor):
    motor.set_maximum_motor_current(CURRENT_LIMIT, CURRENT_LIMIT)
    # The unattainable step (STEP_VELOCITY) exceeds the boot-default velocity limit
    # (9.333 rot/s, the datasheet 560 RPM, since firmware 0.15.8.0) -- raise the limit
    # first, the documented way to exceed the rating (clamp allows up to 18.67 rot/s).
    motor.set_maximum_velocity(18.0)
    motor.set_max_allowable_position_deviation(50.0)
    motor.go_to_closed_loop()
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        if motor.get_status()[0] & 0b100:
            break
        time.sleep(0.1)
    time.sleep(0.3)
    motor.zero_position()


def calibrate_capture_scale(motor):
    """Raw capture units per rotation, from a known 0.5-rot move."""
    motor.zero_position()
    motor.trapezoid_move(0.5, 0.3)
    trace = capture_position_trace(motor)   # 0.41 s window covers the whole move
    settle(motor)
    span = (sum(trace[-20:]) / 20) - (sum(trace[:5]) / 5)
    if abs(span) < 1000:
        raise RuntimeError("calibration capture saw no motion")
    return span / 0.5


def one_step_trial(motor, scale, direction):
    """Return (vmax rot/s, amax rot/s^2) from one velocity-step capture."""
    motor.zero_position()
    motor.move_with_velocity(0.0, LEAD_DWELL)                    # dwell: capture arms before the step
    motor.move_with_velocity(direction * STEP_VELOCITY, STEP_DURATION)
    motor.move_with_velocity(0.0, 0.05)
    trace = capture_position_trace(motor)
    settle(motor)

    rot = [p / scale for p in trace]
    v = [(rot[i + 1] - rot[i - 1]) / (2 * CAPTURE_DT) for i in range(1, len(rot) - 1)]
    v_plateau = smooth(v, 15)                       # heavy smoothing for the plateau statistic
    speeds_plateau = [abs(x) for x in v_plateau]
    top = sorted(speeds_plateau)[int(len(speeds_plateau) * 0.95)]
    plateau = [s for s in speeds_plateau if s > 0.9 * top]
    vmax = statistics.median(plateau)

    # accel = 10-90% rise-time acceleration on lightly smoothed velocity:
    # the unloaded spin-up takes only ~2 ms, so window-fit estimators are
    # unstable; the band-crossing time of the FIRST ramp is robust
    speeds = [abs(x) for x in smooth(v, 5)]
    lo, hi = 0.10 * vmax, 0.90 * vmax
    amax = 0.0
    i = next((k for k, sp in enumerate(speeds) if sp > lo), None)
    if i is not None:
        j = next((k for k in range(i, len(speeds)) if speeds[k] >= hi), None)
        if j is not None and j > i:
            # sub-sample interpolation of both crossings (the unloaded ramp is
            # only a handful of samples wide, so integer indices quantize badly)
            t_lo = (i - 1 + (lo - speeds[i - 1]) / (speeds[i] - speeds[i - 1])) * CAPTURE_DT \
                if i > 0 and speeds[i] != speeds[i - 1] else i * CAPTURE_DT
            t_hi = (j - 1 + (hi - speeds[j - 1]) / (speeds[j] - speeds[j - 1])) * CAPTURE_DT \
                if j > 0 and speeds[j] != speeds[j - 1] else j * CAPTURE_DT
            if t_hi > t_lo:
                amax = (hi - lo) / (t_hi - t_lo)
    if amax == 0.0:   # ramp mostly missed (very high accel): fall back to widest rise
        for i in range(len(speeds) - 3):
            if speeds[i] > 0.1 * vmax and speeds[i + 3] < 0.9 * vmax:
                amax = max(amax, (speeds[i + 3] - speeds[i]) / (3 * CAPTURE_DT))
    return vmax, amax


def characterize_motor(uid_hex):
    motor = servomotor.M3(uid_hex, time_unit="seconds", position_unit="shaft_rotations",
                          velocity_unit="rotations_per_second",
                          acceleration_unit="rotations_per_second_squared",
                          current_unit="internal_current_units", verbose=0)
    motor.system_reset()
    time.sleep(RESET_DELAY_S)
    supply_mv = motor.get_supply_voltage()
    fw = motor.get_firmware_version()[0]
    enter_closed_loop(motor)
    scale = calibrate_capture_scale(motor)

    vmaxes, amaxes = [], []
    for direction in (1, -1):
        for _ in range(TRIALS_PER_DIRECTION):
            vm, am = one_step_trial(motor, scale, direction)
            vmaxes.append(vm)
            amaxes.append(am)
    motor.system_reset()
    time.sleep(RESET_DELAY_S)
    return {
        "unique_id": uid_hex,
        "firmware": ".".join(str(x) for x in reversed(fw)),
        "supply_mv": supply_mv,
        "current_limit": CURRENT_LIMIT,
        "max_velocity_rot_per_s": round(statistics.median(vmaxes), 3),
        "max_acceleration_rot_per_s2": round(statistics.median(amaxes), 1),
        "velocity_trials": [round(x, 3) for x in vmaxes],
        "acceleration_trials": [round(x, 1) for x in amaxes],
        "capture_scale_raw_per_rot": round(scale, 1),
    }


def plot_histograms(results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vels = [r["max_velocity_rot_per_s"] for r in results]
    accs = [r["max_acceleration_rot_per_s2"] for r in results]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    ax1.hist(vels, bins=max(5, min(20, len(vels))), color="#3b7dd8", edgecolor="black")
    ax1.set_xlabel("Maximum velocity (rotations per second)")
    ax1.set_ylabel("Number of motors")
    ax1.set_title(f"Max velocity, n={len(vels)}\nmedian {statistics.median(vels):.2f} rot/s")
    ax2.hist(accs, bins=max(5, min(20, len(accs))), color="#d87a3b", edgecolor="black")
    ax2.set_xlabel("Maximum acceleration (rotations per second squared)")
    ax2.set_ylabel("Number of motors")
    ax2.set_title(f"Max acceleration, n={len(accs)}\nmedian {statistics.median(accs):.0f} rot/s²")
    supply = statistics.mean(r["supply_mv"] for r in results) / 1000.0
    fig.suptitle(f"Servomotor characterization — current limit {CURRENT_LIMIT}, supply ~{supply:.1f} V")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"Histogram written to {path}")


def main():
    parser = argparse.ArgumentParser(description="Characterize max velocity/acceleration of all motors on the bus.")
    parser.add_argument('-p', '--port', help='serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='interactive port selection')
    parser.add_argument('-a', '--alias', help='characterize only this alias/unique ID (skip discovery)')
    parser.add_argument('-o', '--output-prefix', default='motor_characterization',
                        help='prefix for the .json and .png outputs')
    parser.add_argument('-n', '--detections', type=int, default=3,
                        help='discovery rounds (use more on a crowded bus, e.g. 5 for a 39-motor rack)')
    args = parser.parse_args()

    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port()
    results = []
    try:
        if args.alias:
            targets = [args.alias]          # characterize just this one, addressed as given
            print(f"Characterizing single motor at address '{args.alias}'")
        else:
            devices = servomotor.detect_devices_iteratively(n_detections=args.detections)
            targets = [f"{d.unique_id:016X}" for d in devices]
            print(f"Discovered {len(targets)} motor(s): {targets}")

        for i, uid in enumerate(targets):
            print(f"\n=== Characterizing motor {i + 1}/{len(targets)}: {uid} ===")
            try:
                r = characterize_motor(uid)
                results.append(r)
                print(f"  max velocity     = {r['max_velocity_rot_per_s']} rot/s   (trials {r['velocity_trials']})")
                print(f"  max acceleration = {r['max_acceleration_rot_per_s2']} rot/s^2 (trials {r['acceleration_trials']})")
            except Exception as e:
                print(f"  FAILED: {type(e).__name__}: {e}")
                try:
                    m = servomotor.M3(uid, verbose=0)
                    m.system_reset()
                    time.sleep(RESET_DELAY_S)
                except Exception:
                    pass
    finally:
        servomotor.close_serial_port()

    if not results:
        print("No results collected.")
        return 1
    json_path = f"{args.output_prefix}_results.json"
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults written to {json_path}")
    plot_histograms(results, f"{args.output_prefix}_histogram.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
