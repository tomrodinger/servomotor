#!/usr/bin/env python3
"""
PID debug data capture & plot tool
==================================

Captures the firmware's PID controller internals (error, P term, I term,
D term, output) from a motor in closed-loop mode, using test mode 3
(READ_PID_DEBUG_DATA_TEST_MODE) and the "Read multipurpose buffer"
command (cmd 35). Saves a CSV, prints validation results, and renders a
plot. This is the baseline-measurement tool for the PID improvement work
documented in firmware/PID_IMPROVEMENT_PLAN.md.

How it works (firmware side, see firmware/Src/motor_control.c):

  * In closed loop with test_mode == 3, the PID controller writes a
    20-byte struct {error, p_term, i_term, d_term, output} (5 x int32)
    into the multipurpose buffer -- but only when the buffer is empty
    (multipurpose_data_type == 0).
  * Cmd 35 returns [data_type(1 byte) | payload] and clears the buffer,
    so the very next PID iteration (~32 us later) writes a fresh sample.
  * Polling cmd 35 in a tight loop therefore yields one PID snapshot per
    RS485 round trip (~2-4 ms at 230400 baud, i.e. a few hundred Hz).
    The 31.25 kHz loop is undersampled; that is fine for observing
    integrator windup, D-term noise floors, and settling behavior.
  * The 'error' field is captured AFTER the +/-max_error clamp, so
    p_term == error * kP holds exactly for every sample, and
    output == (p_term + i_term + d_term) >> PID_SHIFT_RIGHT holds
    exactly (arithmetic shift). The validation step checks both.

!! IMPORTANT: never send test_mode(0) to clear the test mode -- on
current firmware that routes into set_led_test_mode(0) which is
__disable_irq() + while(1) and locks the MCU until a power cycle
(see the led-test-mode-is-stuck-forever note / WORK_CHECKLIST TODO #8).
This tool always cleans up with system_reset() instead.

Usage examples:

  # 5 s standstill capture (D-term noise floor / holding behavior):
  python3 pid_debug_capture.py -p /dev/tty.usbserial-210 -a X --duration 5

  # Capture a move transient (2 rotations over 2 s, sent 1 s into capture):
  python3 pid_debug_capture.py -p /dev/tty.usbserial-210 --duration 6 --move 2

  # Same, with temporary PID constants (reverted by the final reset):
  python3 pid_debug_capture.py -p /dev/tty.usbserial-210 --duration 6 \
      --move 2 --set-pid 2000 50 175000

  # Re-plot a previously saved capture:
  python3 pid_debug_capture.py plot pid_capture_20260706_120000.csv

The final system_reset() also reverts any --set-pid constants to the
firmware defaults, so the motor is always left in its power-on state.
"""

import argparse
import os
import struct
import sys
import time

# The servomotor package lives in <repo>/python_programs/, next to the tests.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python_programs"))

import servomotor  # noqa: E402
from servomotor.communication import TimeoutError as ServoTimeoutError  # noqa: E402

RESET_DELAY_S = 2.0          # post-reset settle (bootloader window; see calibration-auto-reboots note)
ENABLE_SETTLE_S = 0.3        # MOSFET enable commutation-alignment transient
CLOSED_LOOP_TIMEOUT_S = 6.0
CONSECUTIVE_TIMEOUT_LIMIT = 5

CLOSED_LOOP_BIT = 1 << 2
BOOTLOADER_BIT = 1 << 0

TEST_MODE_PID_DEBUG = 3
MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA = 4
PID_DEBUG_PAYLOAD_BYTES = 5 * 4  # 5 x int32
FIELD_NAMES = ["error", "p_term", "i_term", "d_term", "output"]

# M17 reference values (defaults; used for reporting only, not hard asserts)
M17_PID_SHIFT_RIGHT = 11
M17_DEFAULT_KP = 2000
M17_DEFAULT_KI = 5
M17_DEFAULT_KD = 175000


class CaptureError(Exception):
    pass


def read_buffer_or_none(motor):
    """Cmd 35; returns bytes payload, or None on the (empty-buffer) timeout."""
    try:
        result = motor.read_multipurpose_buffer()
    except ServoTimeoutError:
        return None
    except Exception as e:
        if type(e).__name__ == "TimeoutError":  # bare-TimeoutError fallback
            return None
        raise
    return bytes(result)


def get_status_checked(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise CaptureError(f"{label}: get_status returned unusable value: {status!r}")
    flags, fatal_error_code = status[0], status[1]
    if fatal_error_code != 0:
        raise CaptureError(f"{label}: device reports fatal error code {fatal_error_code} (flags={flags:#06x})")
    if flags & BOOTLOADER_BIT:
        raise CaptureError(f"{label}: device is in the bootloader (flags={flags:#06x})")
    return flags


def wait_for_closed_loop(motor, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        flags = get_status_checked(motor, "waiting for closed loop")
        if flags & CLOSED_LOOP_BIT:
            return
        time.sleep(0.1)
    raise CaptureError(f"closed-loop flag never set within {timeout_s} s -- is the motor calibrated?")


def decode_sample(raw):
    """Validate framing of one cmd-35 payload and decode the 5 int32 fields."""
    if len(raw) != 1 + PID_DEBUG_PAYLOAD_BYTES:
        raise CaptureError(f"cmd 35 payload length {len(raw)} != {1 + PID_DEBUG_PAYLOAD_BYTES} "
                           f"(1 type byte + {PID_DEBUG_PAYLOAD_BYTES} PID bytes); raw={raw.hex()}")
    if raw[0] != MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA:
        raise CaptureError(f"cmd 35 data type byte = {raw[0]} (expected {MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA} "
                           f"for PID debug data); raw={raw.hex()}")
    return struct.unpack("<iiiii", raw[1:])


def do_capture(args):
    servomotor.set_serial_port_from_args(args)

    meta = {
        "port": args.port if args.port else "(interactive)",
        "alias": args.alias,
        "duration_s": args.duration,
        "set_pid": " ".join(str(v) for v in args.set_pid) if args.set_pid else "",
        "move_rotations": args.move if args.move is not None else "",
        "move_time_s": args.move_time if args.move is not None else "",
        "move_delay_s": args.move_delay if args.move is not None else "",
        "capture_started": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    samples = []        # (t_sample_s, error, p, i, d, output)
    timeouts = 0
    move_sent_at = None
    motor = None
    fatal_cleanup_message = None

    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias,
                              time_unit="seconds",
                              position_unit="shaft_rotations",
                              verbose=2 if args.verbose else 0)

        print("Resetting device to start from a known state...")
        motor.system_reset()
        time.sleep(RESET_DELAY_S)
        get_status_checked(motor, "after reset")

        if args.set_pid:
            kp, ki, kd = args.set_pid
            print(f"Setting PID constants: kP={kp} kI={ki} kD={kd} (reverted by the final reset)...")
            motor.set_pid_constants(kp, ki, kd)

        print("Enabling MOSFETs and entering closed loop...")
        motor.enable_mosfets()
        time.sleep(ENABLE_SETTLE_S)
        motor.go_to_closed_loop()
        wait_for_closed_loop(motor, CLOSED_LOOP_TIMEOUT_S)
        motor.zero_position()

        print("Asserting test mode 3 (PID debug data)...")
        motor.test_mode(TEST_MODE_PID_DEBUG)

        print(f"Capturing for {args.duration} s...")
        t0 = time.perf_counter()
        # Sample-time model: our previous read cleared the buffer, and the
        # PID repopulates it ~32 us later. So each sample was generated just
        # after the previous read completed; use that as its timestamp.
        t_prev_read_done = 0.0
        consecutive_timeouts = 0

        while True:
            t_now = time.perf_counter() - t0
            if t_now >= args.duration:
                break

            if (args.move is not None) and (move_sent_at is None) and (t_now >= args.move_delay):
                motor.trapezoid_move(args.move, args.move_time)
                move_sent_at = time.perf_counter() - t0
                print(f"  [{move_sent_at:.3f} s] trapezoid move sent: "
                      f"{args.move} rotations over {args.move_time} s")
                continue  # timestamp bookkeeping: skip the sample straddling the move command

            raw = read_buffer_or_none(motor)
            t_read_done = time.perf_counter() - t0
            if raw is None:
                timeouts += 1
                consecutive_timeouts += 1
                if consecutive_timeouts >= CONSECUTIVE_TIMEOUT_LIMIT:
                    # PID stopped feeding the buffer -- most likely a fatal error tripped.
                    status = motor.get_status()
                    raise CaptureError(f"{consecutive_timeouts} consecutive cmd-35 timeouts; "
                                       f"device status: {status!r}")
                t_prev_read_done = t_read_done
                continue
            consecutive_timeouts = 0
            fields = decode_sample(raw)
            samples.append((t_prev_read_done,) + fields)
            t_prev_read_done = t_read_done

    finally:
        # Always try to leave the motor reset (clears test mode, PID constants,
        # closed loop, and the buffer). NEVER test_mode(0) -- it locks the MCU.
        if motor is not None:
            try:
                print("Cleaning up: system_reset (clears test mode; reverts PID constants)...")
                motor.system_reset()
                time.sleep(RESET_DELAY_S)
                get_status_checked(motor, "after cleanup reset")
            except Exception as e:
                fatal_cleanup_message = f"cleanup failed: {e}"
        try:
            servomotor.close_serial_port()
        except Exception:
            pass  # port may never have opened; don't mask the original exception

    if fatal_cleanup_message:
        print(f"WARNING: {fatal_cleanup_message}")

    if not samples:
        raise CaptureError("no samples captured")

    meta["move_sent_at_s"] = f"{move_sent_at:.6f}" if move_sent_at is not None else ""
    meta["n_samples"] = len(samples)
    meta["timeouts"] = timeouts

    csv_path = args.output
    if not csv_path:
        csv_path = time.strftime("pid_capture_%Y%m%d_%H%M%S.csv")
    write_csv(csv_path, meta, samples)
    print(f"\nWrote {len(samples)} samples to {csv_path} ({timeouts} timeouts during capture)")

    ok = validate_and_summarize(samples, meta, args)

    if not args.no_plot:
        png_path = args.png if args.png else os.path.splitext(csv_path)[0] + ".png"
        make_plot(samples, meta, png_path, show=args.show)

    return ok


def write_csv(path, meta, samples):
    with open(path, "w") as f:
        f.write("# pid_debug_capture v1\n")
        for k, v in meta.items():
            f.write(f"# {k}: {v}\n")
        f.write("t_s," + ",".join(FIELD_NAMES) + "\n")
        for row in samples:
            f.write(f"{row[0]:.6f}," + ",".join(str(v) for v in row[1:]) + "\n")


def read_csv(path):
    meta = {}
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if ":" in line:
                    k, v = line[1:].split(":", 1)
                    meta[k.strip()] = v.strip()
                continue
            if line.startswith("t_s"):
                continue
            parts = line.split(",")
            try:
                row = (float(parts[0]),) + tuple(int(p) for p in parts[1:])
            except ValueError as e:
                raise CaptureError(f"corrupted capture file {path!r}: unparseable row {line!r} ({e})")
            if len(row) != 6:
                raise CaptureError(f"corrupted capture file {path!r}: row has {len(row)} fields, expected 6: {line!r}")
            samples.append(row)
    return meta, samples


# Full output authority for M17 at the default max PWM voltage (200 << 8). When the
# anti-windup saturates AND the back-calculated integral also hits its clamp, the
# reported (post-clamp) terms provably cannot satisfy the shift identity -- the output
# is pinned at +/-limit while the recomputed sum overshoots it in the same direction.
# Such samples are accepted as "saturation-consistent" instead of failing validation.
DEFAULT_OUTPUT_LIMIT = 200 << 8


def detect_shift(samples, output_limit=DEFAULT_OUTPUT_LIMIT):
    """Find (shift, rounding convention) such that every sample either satisfies the
    output identity or is saturation-consistent (see DEFAULT_OUTPUT_LIMIT note).

    Old firmware (<= 0.15.3.0): output = (p+i+d) >> s              -> "floor"
    New firmware (>= 0.15.3.1): output = (p+i+d + (1<<(s-1))) >> s -> "round-nearest"

    Returns [] for an empty sample list (no vacuous all-candidates result).
    """
    if not samples:
        return []

    def holds(shift, half):
        for (_, _, p, i, d, out) in samples:
            comp = (p + i + d + half) >> shift
            if comp == out:
                continue
            if abs(out) == output_limit and (comp > out if out > 0 else comp < out):
                continue  # saturated AW sample with clamped integral: consistent
            return False
        return True

    candidates = []
    for shift in range(0, 25):
        if holds(shift, 0):
            candidates.append((shift, "floor"))
        half = 1 << (shift - 1) if shift > 0 else 0
        if holds(shift, half):
            candidates.append((shift, "round-nearest"))
    return candidates


def detect_kp(samples):
    """p_term must be exactly error * kP; recover kP from samples with error != 0."""
    ratios = set()
    inexact = 0
    for (_, error, p, _, _, _) in samples:
        if error == 0:
            continue
        if p % error != 0:
            inexact += 1
            continue
        ratios.add(p // error)
    return ratios, inexact


def validate_and_summarize(samples, meta, args):
    print("\n=== Validation ===")
    ok = True
    n = len(samples)

    # 1. Sampling statistics
    dts = [samples[k + 1][0] - samples[k][0] for k in range(n - 1)]
    if dts:
        avg_dt = sum(dts) / len(dts)
        print(f"samples: {n}, span {samples[-1][0] - samples[0][0]:.3f} s, "
              f"avg spacing {avg_dt * 1000:.2f} ms (~{1 / avg_dt:.0f} Hz), max gap {max(dts) * 1000:.2f} ms")
        if any(dt <= 0 for dt in dts):
            print("FAIL: non-monotonic sample timestamps")
            ok = False

    # 2. Arithmetic identity: output == (p + i + d) >> PID_SHIFT_RIGHT
    shifts = detect_shift(samples, output_limit=getattr(args, "output_limit", DEFAULT_OUTPUT_LIMIT))
    if not shifts:
        mismatches = sum(1 for (_, _, p, i, d, out) in samples
                         if ((p + i + d) >> M17_PID_SHIFT_RIGHT) != out)
        print(f"FAIL: no shift in 0..24 satisfies output == (P+I+D) >> shift on all samples "
              f"({mismatches}/{n} mismatch at shift {M17_PID_SHIFT_RIGHT})")
        ok = False
    else:
        shift_values = [s for s, _ in shifts]
        note = "" if M17_PID_SHIFT_RIGHT in shift_values else f" (expected {M17_PID_SHIFT_RIGHT} for M17!)"
        print(f"PASS: output == (P+I+D) >> shift holds on all {n} samples; "
              f"consistent shift(s): {shifts}{note}")
        if M17_PID_SHIFT_RIGHT not in shift_values:
            ok = False

    # 3. P-term consistency: p_term == error * kP exactly
    ratios, inexact = detect_kp(samples)
    nonzero_error_samples = sum(1 for s in samples if s[1] != 0)
    if nonzero_error_samples == 0:
        print("SKIP: kP check (all samples have error == 0)")
    elif inexact or len(ratios) > 1:
        print(f"FAIL: p_term is not a single exact integer multiple of error "
              f"(ratios seen: {sorted(ratios)}, inexact divisions: {inexact})")
        ok = False
    else:
        kp_detected = ratios.pop() if ratios else None
        expected_kp = args.set_pid[0] if getattr(args, "set_pid", None) else None
        msg = f"PASS: p_term == error * {kp_detected} exactly on all {nonzero_error_samples} nonzero-error samples"
        if expected_kp is not None and kp_detected != expected_kp:
            print(msg + f" -- but --set-pid requested kP={expected_kp}: MISMATCH")
            ok = False
        else:
            print(msg)

    # 4. If kI == 0 was requested, the integral term must be exactly 0 throughout --
    #    EXCEPT on saturated samples: the anti-windup back-calculation (fw >= 0.15.3.1)
    #    writes its correction into integral_term even when the integral constant is 0
    #    (the value is zeroed again on the next tick), so a railed output legitimately
    #    reports a nonzero integral for that one sample.
    if getattr(args, "set_pid", None) and args.set_pid[1] == 0:
        limit = getattr(args, "output_limit", DEFAULT_OUTPUT_LIMIT)
        nonzero_i = sum(1 for s in samples if s[3] != 0 and abs(s[5]) != limit)
        n_sat = sum(1 for s in samples if abs(s[5]) == limit)
        if nonzero_i:
            print(f"FAIL: kI=0 was set but i_term != 0 on {nonzero_i}/{n} non-saturated samples")
            ok = False
        else:
            note = f" ({n_sat} saturated samples exempt)" if n_sat else ""
            print(f"PASS: kI=0 -> i_term == 0 on all {n} samples{note}")

    # 5. Field ranges / windup reporting (informational)
    print("\n=== Field statistics ===")
    for idx, name in enumerate(FIELD_NAMES, start=1):
        vals = [s[idx] for s in samples]
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / n
        print(f"  {name:>7}: min {min(vals):>12}  max {max(vals):>12}  mean {mean:>14.1f}  std {var ** 0.5:>12.1f}")

    i_vals = [s[3] for s in samples]
    i_peak = max(abs(v) for v in i_vals)
    if i_peak > 0:
        # Integral clamp on fw >= 0.15.3.1: (max_motor_pwm_voltage(200) << (11+8)) >> 2
        # = 26,214,400 (25% of full output authority; the anti-windup change).
        # Old firmware (<= 0.15.3.0) allowed the full authority, 4x this value.
        clamp_rail = (200 << (M17_PID_SHIFT_RIGHT + 8)) >> 2
        railed = sum(1 for v in i_vals if abs(v) >= clamp_rail)
        print(f"\nIntegral term peak |I| = {i_peak} "
              f"({100.0 * i_peak / clamp_rail:.1f}% of the integral clamp {clamp_rail}); "
              f"samples at/over the clamp: {railed}/{n} ({100.0 * railed / n:.1f}%)")

    print("\nValidation result:", "PASS" if ok else "FAIL")
    return ok


def make_plot(samples, meta, png_path, show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [s[0] for s in samples]
    error = [s[1] for s in samples]
    p_term = [s[2] for s in samples]
    i_term = [s[3] for s in samples]
    d_term = [s[4] for s in samples]
    output = [s[5] for s in samples]

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(f"PID debug capture -- {meta.get('capture_started', '')} "
                 f"(alias {meta.get('alias', '?')}, set_pid: {meta.get('set_pid') or 'defaults'})")

    axes[0].plot(t, error, linewidth=0.8)
    axes[0].set_ylabel("error (counts)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, p_term, linewidth=0.8, label="P term")
    axes[1].plot(t, i_term, linewidth=0.8, label="I term")
    axes[1].plot(t, d_term, linewidth=0.8, label="D term")
    axes[1].set_ylabel("terms (pre-shift)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, output, linewidth=0.8, color="tab:red")
    axes[2].set_ylabel("output (counts)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(True, alpha=0.3)

    move_sent_at = meta.get("move_sent_at_s")
    if move_sent_at and t[0] <= float(move_sent_at) <= t[-1]:
        for ax in axes:
            ax.axvline(float(move_sent_at), color="gray", linestyle="--", alpha=0.7)
        axes[0].text(float(move_sent_at), axes[0].get_ylim()[1], " move cmd",
                     va="top", ha="left", fontsize=8, color="gray")

    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    print(f"Wrote plot to {png_path}")
    if show:
        plt.show()


def do_plot_file(args):
    meta, samples = read_csv(args.csv_file)
    if args.start is not None or args.end is not None:
        start = args.start if args.start is not None else float("-inf")
        end = args.end if args.end is not None else float("inf")
        samples = [s for s in samples if start <= s[0] <= end]
    if not samples:
        raise CaptureError("no samples in the selected range")
    png_path = args.png if args.png else os.path.splitext(args.csv_file)[0] + ".png"
    make_plot(samples, meta, png_path, show=args.show)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Capture PID debug data (test mode 3) from a motor in closed loop.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="subcommand")

    plot_parser = subparsers.add_parser("plot", help="Re-plot a previously saved capture CSV")
    plot_parser.add_argument("csv_file")
    plot_parser.add_argument("--start", type=float, help="Plot from this time (s)")
    plot_parser.add_argument("--end", type=float, help="Plot until this time (s)")
    plot_parser.add_argument("--png", help="Output PNG path (default: alongside the CSV)")
    plot_parser.add_argument("--show", action="store_true", help="Open an interactive plot window")

    parser.add_argument("-p", "--port", help="Serial port device")
    parser.add_argument("-P", "--PORT", action="store_true", help="Interactive port selection")
    parser.add_argument("-a", "--alias", default="X", help="Alias of the device (default: X)")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--duration", type=float, default=5.0, help="Capture duration in seconds (default: 5)")
    parser.add_argument("--set-pid", nargs=3, type=int, metavar=("KP", "KI", "KD"),
                        help="Temporarily set PID constants for this capture (reverted by the final reset)")
    parser.add_argument("--move", type=float, default=None, metavar="ROTATIONS",
                        help="Send a trapezoid move of this many rotations during the capture")
    parser.add_argument("--move-time", type=float, default=2.0, help="Move duration in seconds (default: 2)")
    parser.add_argument("--move-delay", type=float, default=1.0,
                        help="Seconds into the capture at which to send the move (default: 1)")
    parser.add_argument("--output", help="Output CSV path (default: pid_capture_<timestamp>.csv)")
    parser.add_argument("--png", help="Output PNG path (default: alongside the CSV)")
    parser.add_argument("--no-plot", action="store_true", help="Skip plot generation")
    parser.add_argument("--output-limit", type=int, default=DEFAULT_OUTPUT_LIMIT,
                        help="Full output authority (max PWM voltage << 8) used to recognize "
                             "saturated anti-windup samples in validation; set this if the capture "
                             "was taken with a non-default 'Set maximum motor current' (default: 51200)")
    parser.add_argument("--show", action="store_true", help="Open an interactive plot window")

    args = parser.parse_args()

    try:
        if args.subcommand == "plot":
            ok = do_plot_file(args)
        else:
            if args.move is not None and args.duration < args.move_delay + args.move_time + 1.0:
                print(f"NOTE: duration {args.duration} s is short for a move that ends at "
                      f"{args.move_delay + args.move_time} s; consider a longer --duration "
                      f"to capture the settling tail.")
            ok = do_capture(args)
    except CaptureError as e:
        print(f"\nFAILED: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
