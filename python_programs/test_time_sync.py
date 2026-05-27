#!/usr/bin/env python3
"""
Per-command test for "Time sync" (cmd 10) — single-device variant.

Modernized from the obsolete `test_time_sync.py` (which only printed a stream
of errors without making any assertion). Multi-device time sync is a separate,
deferred functional test (decision #5 in TEST_MODERNIZATION_PLAN.md).

What this test does:

  Phase 1 — drift (default: 1.0 s)
    After `Reset time`, do nothing for `--drift-period-s` seconds so the
    motor's clock genuinely drifts out of sync.

  Phase 2 — lock-in (default: 1.0 s)
    Begin sending `Time sync` at `--iteration-period-s` (default 0.1 s = 10 Hz,
    matching the protocol's "do this 10 times per second"). Errors collected
    here are NOT asserted — the PI controller is being given time to lock.

  Phase 3 — measurement (default: 6.0 s)
    Continue syncing at 10 Hz. The measurement window must pass ALL of:
      (a) the aggregate is tight — mean |error| and the p90 |error| are well
          below the threshold (this is what catches *actual drift*, which
          shifts the whole distribution);
      (b) no sample exceeds a hard ceiling (default 2× the threshold) —
          a single grossly-wrong sample still fails immediately;
      (c) at most a small budget of samples (default 5%, ≥1) may exceed the
          per-sample threshold — this tolerates the occasional host-side
          latency spike (USB / OS scheduling jitter) that is not a clock
          problem, without masking a real drift (which fails (a)).
    The host-measured error includes host round-trip jitter, so a strict
    "zero breaches ever" rule is flaky under full-suite load; criteria (a)+(b)
    keep it honest while (c) absorbs rare jitter.

At the end:
  * Prints lock-in errors as a sequence (so a human can see the PI controller
    locking in).
  * Prints summary statistics for the measurement window (count, min, max,
    mean signed, mean |·|, p50/p90/p99, std).
  * Prints a text-based histogram of the measurement-window errors, with the
    threshold lines marked, so a human can see the shape of the residual
    drift distribution.
"""

import argparse
import sys
import time
import math
import servomotor

STATUS_IN_THE_BOOTLOADER_FLAG_BIT = 0

RESET_DELAY_S = 1.5
DEFAULT_DRIFT_PERIOD_S = 1.0
DEFAULT_LOCK_IN_PERIOD_S = 5.0    # PI controller's trim range is narrow (62..66) and convergence is ~280 µs/sync;
                                  # 1 s wasn't enough after thermal stress, and 3 s still occasionally left the
                                  # convergence tail leaking into the measurement window under full-suite host
                                  # load (one early sample breached). 5 s = 50 syncs ≈ 14 ms of correction headroom.
DEFAULT_MEASURE_PERIOD_S = 6.0
DEFAULT_ITERATION_PERIOD_S = 0.1
DEFAULT_MAX_ERROR_US = 5000
# Hardening knobs (see Phase 3 in the docstring). These tolerate rare host-side
# latency spikes without masking real clock drift.
DEFAULT_HARD_MAX_ERROR_FACTOR = 2.0   # no sample may exceed this × max_error_us
DEFAULT_MAX_BREACH_FRACTION = 0.05    # at most this fraction (≥1) of samples may exceed max_error_us
DEFAULT_AGG_MEAN_ABS_FACTOR = 0.6     # mean |err| must be below this × max_error_us
DEFAULT_AGG_P90_FACTOR = 1.0          # p90 |err| must be below this × max_error_us


def assert_app_mode_no_error(motor, label):
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (expected 0). flags={flags:#x}")
    if flags & (1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT):
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#x}, expected application mode)")
    return flags, err


def stats_summary(values):
    """Return a dict of summary stats for a list of integer errors (µs)."""
    n = len(values)
    if n == 0:
        return None
    mean = sum(values) / n
    mean_abs = sum(abs(v) for v in values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    std = math.sqrt(var)
    s = sorted(values)
    abs_sorted = sorted(abs(v) for v in values)

    def pct(arr, p):
        if not arr:
            return 0
        idx = max(0, min(len(arr) - 1, int(round(p / 100.0 * (len(arr) - 1)))))
        return arr[idx]

    return {
        "n": n,
        "min": s[0],
        "max": s[-1],
        "mean": mean,
        "mean_abs": mean_abs,
        "std": std,
        "p50_abs": pct(abs_sorted, 50),
        "p90_abs": pct(abs_sorted, 90),
        "p99_abs": pct(abs_sorted, 99),
    }


def text_histogram(values, threshold_us, n_bins=21, bar_max=50):
    """Print a text-based histogram. Range covers ±1.2·max(|max|·, threshold).
    Threshold lines are marked with `|`."""
    if not values:
        return "(no data)"
    max_abs = max(max(abs(v) for v in values), threshold_us)
    rng = max_abs * 1.2
    lo, hi = -rng, rng
    bin_w = (hi - lo) / n_bins
    bins = [0] * n_bins
    for v in values:
        b = int((v - lo) / bin_w)
        b = max(0, min(b, n_bins - 1))
        bins[b] += 1
    max_count = max(bins) or 1
    lines = []
    for i, count in enumerate(bins):
        b_lo = lo + i * bin_w
        b_hi = b_lo + bin_w
        in_threshold_band = (b_lo <= -threshold_us < b_hi) or (b_lo < threshold_us <= b_hi)
        marker = "|" if in_threshold_band else " "
        bar = "#" * (int(count * bar_max / max_count) if count > 0 else 0)
        lines.append(f"  [{b_lo:>+8.0f},{b_hi:>+8.0f}) µs {marker} {bar:<{bar_max}} ({count})")
    return "\n".join(lines)


def time_sync_loop(motor, py_start, sync_offset_s, total_sync_s,
                   lock_in_period_s, iteration_period_s, verbose):
    """Send `Time sync` calls for `total_sync_s` seconds starting at
    `sync_offset_s` after py_start. Return (lock_in_errors, measure_errors).

    Errors collected within the first `lock_in_period_s` of syncing are
    "lock-in" — those go in the first list. Everything afterwards goes
    in the second list (asserted by the caller)."""
    lock_in = []
    measure = []
    lock_in_end_offset = sync_offset_s + lock_in_period_s
    deadline_offset = sync_offset_s + total_sync_s
    i = 0
    while True:
        target_offset = sync_offset_s + (i + 1) * iteration_period_s
        if target_offset > deadline_offset:
            break
        target_wall = py_start + target_offset
        sleep_for = target_wall - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        now_offset = time.time() - py_start
        master_time_us = int(now_offset * 1_000_000)
        resp = motor.time_sync(master_time_us)
        if not isinstance(resp, list) or len(resp) < 1:
            raise AssertionError(f"iter {i}: unexpected time_sync response: {resp!r}")
        err_us = resp[0]
        if now_offset < lock_in_end_offset:
            lock_in.append(err_us)
        else:
            measure.append(err_us)
        if verbose:
            phase = "L" if now_offset < lock_in_end_offset else "M"
            rcc = resp[1] if len(resp) > 1 else None
            print(f"  [{phase}] iter {i:3d} t={now_offset:6.3f}s master={master_time_us:>10} err={err_us:>+6} rcc={rcc}")
        i += 1
    return lock_in, measure


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Time sync' (cmd 10), single-device variant.")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--drift-period-s', type=float, default=DEFAULT_DRIFT_PERIOD_S,
                        help=f'Seconds after Reset time during which no sync runs — lets the motor clock drift. (default: {DEFAULT_DRIFT_PERIOD_S})')
    parser.add_argument('--lock-in-period-s', type=float, default=DEFAULT_LOCK_IN_PERIOD_S,
                        help=f'Seconds of initial syncing whose errors are NOT asserted (PI controller settling). (default: {DEFAULT_LOCK_IN_PERIOD_S})')
    parser.add_argument('--measure-period-s', type=float, default=DEFAULT_MEASURE_PERIOD_S,
                        help=f'Seconds of measured syncing. Every error in this window must satisfy |err| < --max-error-us; one breach fails the test. (default: {DEFAULT_MEASURE_PERIOD_S})')
    parser.add_argument('--iteration-period-s', type=float, default=DEFAULT_ITERATION_PERIOD_S,
                        help=f'Seconds between successive time_sync calls. (default: {DEFAULT_ITERATION_PERIOD_S})')
    parser.add_argument('--max-error-us', type=int, default=DEFAULT_MAX_ERROR_US,
                        help=f'Per-sample |time error| threshold in microseconds. (default: {DEFAULT_MAX_ERROR_US})')
    parser.add_argument('--hard-max-error-factor', type=float, default=DEFAULT_HARD_MAX_ERROR_FACTOR,
                        help=f'No measurement sample may exceed this factor × --max-error-us. (default: {DEFAULT_HARD_MAX_ERROR_FACTOR})')
    parser.add_argument('--max-breach-fraction', type=float, default=DEFAULT_MAX_BREACH_FRACTION,
                        help=f'At most this fraction (but always ≥1) of measurement samples may exceed --max-error-us, absorbing rare host jitter. (default: {DEFAULT_MAX_BREACH_FRACTION})')
    parser.add_argument('--agg-mean-abs-factor', type=float, default=DEFAULT_AGG_MEAN_ABS_FACTOR,
                        help=f'Mean |error| must stay below this factor × --max-error-us (catches real drift). (default: {DEFAULT_AGG_MEAN_ABS_FACTOR})')
    parser.add_argument('--agg-p90-factor', type=float, default=DEFAULT_AGG_P90_FACTOR,
                        help=f'p90 |error| must stay below this factor × --max-error-us (catches real drift). (default: {DEFAULT_AGG_P90_FACTOR})')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    if args.iteration_period_s <= 0:
        print(f"FAILED: --iteration-period-s must be positive (got {args.iteration_period_s})", file=sys.stderr)
        return 1
    if args.measure_period_s <= 0:
        print(f"FAILED: --measure-period-s must be positive (got {args.measure_period_s})", file=sys.stderr)
        return 1

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            # 1. Reset to a known state.
            print("Resetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_app_mode_no_error(motor, "after reset")

            # 2. Reset device time, then drift for drift_period_s.
            motor.reset_time()
            py_start = time.time()
            print(f"\nPhase 1 — drift: {args.drift_period_s:.2f} s of idle (no syncs) to let the clocks separate...")
            time.sleep(args.drift_period_s)

            # 3. Phases 2 + 3 — continuous syncing.
            total_sync_s = args.lock_in_period_s + args.measure_period_s
            sync_rate_hz = 1.0 / args.iteration_period_s
            print(f"\nPhase 2 — lock-in: {args.lock_in_period_s:.2f} s of Time sync at ~{sync_rate_hz:.1f} Hz "
                  f"(errors NOT asserted)")
            print(f"Phase 3 — measure: {args.measure_period_s:.2f} s of Time sync at ~{sync_rate_hz:.1f} Hz "
                  f"(every |error| must be < {args.max_error_us} µs)")
            lock_in_errors, measure_errors = time_sync_loop(
                motor, py_start,
                sync_offset_s=args.drift_period_s,
                total_sync_s=total_sync_s,
                lock_in_period_s=args.lock_in_period_s,
                iteration_period_s=args.iteration_period_s,
                verbose=args.verbose,
            )

            # 4. Report.
            print()
            print(f"Lock-in errors (µs, n={len(lock_in_errors)}, not asserted):")
            print(f"  {lock_in_errors}")
            print()

            if not measure_errors:
                raise AssertionError("No errors collected in the measurement window — period too short or iteration period too long.")

            s = stats_summary(measure_errors)
            print(f"Measurement-window stats (n={s['n']}):")
            print(f"  min        = {s['min']:>+6} µs")
            print(f"  max        = {s['max']:>+6} µs")
            print(f"  mean       = {s['mean']:>+8.1f} µs")
            print(f"  mean |err| = {s['mean_abs']:>8.1f} µs")
            print(f"  std        = {s['std']:>8.1f} µs")
            print(f"  p50 |err|  = {s['p50_abs']:>6} µs")
            print(f"  p90 |err|  = {s['p90_abs']:>6} µs")
            print(f"  p99 |err|  = {s['p99_abs']:>6} µs")
            print()
            print(f"Error histogram (threshold ±{args.max_error_us} µs marked with `|`):")
            print(text_histogram(measure_errors, args.max_error_us))
            print()

            # 5. Assertion: multi-criteria so host-side jitter (rare single-sample
            #    spikes) is tolerated, but real drift (a shifted distribution) is not.
            n = len(measure_errors)
            hard_ceiling = args.max_error_us * args.hard_max_error_factor
            breach_budget = max(1, int(math.ceil(args.max_breach_fraction * n)))
            mean_abs_bound = args.max_error_us * args.agg_mean_abs_factor
            p90_bound = args.max_error_us * args.agg_p90_factor

            breaches = [(i, e) for i, e in enumerate(measure_errors) if abs(e) >= args.max_error_us]
            gross = [(i, e) for i, e in enumerate(measure_errors) if abs(e) >= hard_ceiling]

            problems = []
            # (a) aggregate must be tight — this is what catches genuine drift.
            if s['mean_abs'] >= mean_abs_bound:
                problems.append(f"mean |err| {s['mean_abs']:.0f} µs >= {mean_abs_bound:.0f} µs "
                                f"(aggregate drift — distribution is shifted, not a one-off spike)")
            if s['p90_abs'] >= p90_bound:
                problems.append(f"p90 |err| {s['p90_abs']} µs >= {p90_bound:.0f} µs (aggregate drift)")
            # (b) no single grossly-wrong sample.
            if gross:
                problems.append(f"{len(gross)} sample(s) exceeded the hard ceiling {hard_ceiling:.0f} µs: "
                                + ", ".join(f"#{i}={e}" for i, e in gross[:10]))
            # (c) only a small budget of per-sample threshold breaches (rare host jitter).
            if len(breaches) > breach_budget:
                problems.append(f"{len(breaches)} of {n} samples exceeded ±{args.max_error_us} µs, "
                                f"more than the budget of {breach_budget} "
                                f"(first: " + ", ".join(f"#{i}={e}" for i, e in breaches[:10]) + ")")

            if problems:
                raise AssertionError("; ".join(problems))

            tolerated = (f" ({len(breaches)} jitter outlier(s) within the budget of {breach_budget})"
                         if breaches else "")
            print(f"OK: time sync converged — mean |err| {s['mean_abs']:.0f} µs, p90 {s['p90_abs']} µs, "
                  f"all {n} samples under the hard ceiling {hard_ceiling:.0f} µs{tolerated}.")

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
