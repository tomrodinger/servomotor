#!/usr/bin/env python3
"""
FLEET CONSISTENCY: every motor on the bus must behave identically.

WHY THIS TEST EXISTS
    Every other test in this repository asks "did THIS motor do the right
    thing?". None of them can ask the question that a fleet makes possible:

        Given the same firmware and the same command, do all the motors
        produce the same answer?

    That is a different kind of check, and it catches a different kind of bug:

      * a unit that shipped with different firmware from the rest
      * behaviour that depends on a per-unit calibration constant when it
        should not (the planner is pure integer arithmetic -- it must not vary
        from motor to motor)
      * a timing-sensitive fault that only shows up on the one board whose
        crystal is at the far end of the tolerance spread, which a single-motor
        bench test would have to be very lucky to see
      * a limit check that fires with a different error code on some units

    A single motor passing tells you the firmware works somewhere. Thirty-five
    motors agreeing tells you it works because of the code and not because of
    that particular board.

WHAT IS CHECKED
    1. Identity      -- all report the same firmware version and product.
    2. Determinism   -- the same trapezoid move travels the same distance
                        on every unit.
    3. Planner shape -- the same plan costs the same number of queue slots
                        everywhere, including the 0.15.12.0 two-slot fast path.
    4. Refusals      -- the same illegal move is refused with the SAME error
                        code on every unit, not merely refused somehow.
    5. Hygiene       -- every motor starts and finishes clean, alias intact.

SAFETY
    MOSFETs are never enabled. Every measurement reads the COMMANDED position,
    which the planner advances whether or not the output stage is on, so
    nothing physically turns -- safe for the rack whether a shaft is free,
    loaded with a pulley, or bolted down.

    Motors are addressed INDIVIDUALLY by unique ID. There is no broadcast
    anywhere in this file and no alias is ever written, so it cannot disturb a
    shared rack the way `test_set_device_alias` does.
"""

import argparse
import sys
import time

import servomotor
from servomotor import M3
from servomotor.device_detection import detect_devices_iteratively

COUNTS_PER_ROTATION = 3276800
TIMESTEPS_PER_SECOND = 31250
RESET_WAIT = 1.4


class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.failures = []

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            print(f"  PASS  {name}")
        else:
            self.failed += 1
            self.failures.append(f"{name}: {detail}")
            print(f"  FAIL  {name}   {detail}")


def fresh(motor):
    motor.system_reset()
    time.sleep(RESET_WAIT)


def drain(motor, budget_s):
    """Wait for the queue to empty. Never guess a duration -- a fixed wait that
    is too short reads a partial move and looks exactly like a planner bug."""
    deadline = time.time() + budget_s
    while time.time() < deadline:
        try:
            if motor.get_n_queued_items() == 0:
                time.sleep(0.15)
                return True
        except Exception:
            return False
        time.sleep(0.02)
    return False


def probe_one(uid, res_detail):
    """Run the whole battery against one motor. Returns a dict of observations,
    or None if the motor could not be probed at all."""
    obs = {}
    try:
        m = M3(f"{uid:016X}", verbose=0)

        # --- identity -----------------------------------------------------
        fresh(m)
        fw = m.get_firmware_version()
        obs["firmware"] = repr(fw)
        obs["fatal_at_start"] = m.get_status()[1]

        # --- a plain move, timed and measured -----------------------------
        m.set_position_unit("encoder_counts")
        m.set_time_unit("timesteps")
        before = m.get_position()
        m.trapezoid_move(COUNTS_PER_ROTATION // 8, TIMESTEPS_PER_SECOND // 2)
        drained = drain(m, 8.0)
        obs["plain_move_moved"] = m.get_position() - before if drained else None
        obs["plain_move_fatal"] = m.get_status()[1]

        # --- slot cost, normal (3-slot) regime ----------------------------
        # A slow acceleration limit makes each segment about a second long, so
        # the queue cannot drain while the depth is being read.
        fresh(m)
        m.set_velocity_unit("rotations_per_second")
        m.set_acceleration_unit("rotations_per_second_squared")
        m.set_maximum_velocity(1.0)
        m.set_maximum_acceleration(1.0)
        m.set_position_unit("encoder_counts")
        m.set_time_unit("timesteps")
        m.trapezoid_move(COUNTS_PER_ROTATION // 4, 4 * TIMESTEPS_PER_SECOND)
        obs["slots_normal"] = m.get_n_queued_items()
        m.emergency_stop()
        time.sleep(0.3)

        # --- slot cost, one-tick-ramp (2-slot) fast path ------------------
        fresh(m)
        m.set_velocity_unit("rotations_per_second")
        m.set_maximum_velocity(0.3)          # with the 12000 rot/s^2 default
        m.set_position_unit("encoder_counts")
        m.set_time_unit("timesteps")
        m.trapezoid_move(COUNTS_PER_ROTATION // 100, 4 * TIMESTEPS_PER_SECOND)
        obs["slots_fastpath"] = m.get_n_queued_items()
        m.emergency_stop()
        time.sleep(0.3)

        # --- the same illegal move, and which error it earns --------------
        fresh(m)
        m.set_velocity_unit("rotations_per_second")
        m.set_maximum_velocity(0.2)
        m.set_position_unit("encoder_counts")
        m.set_time_unit("timesteps")
        try:
            m.trapezoid_move(COUNTS_PER_ROTATION, TIMESTEPS_PER_SECOND // 4)
        except Exception:
            pass
        time.sleep(0.3)
        obs["illegal_move_fatal"] = m.get_status()[1]

        # --- leave it clean ----------------------------------------------
        fresh(m)
        obs["fatal_at_end"] = m.get_status()[1]
        obs["ok"] = True
    except Exception as e:
        res_detail.append(f"{uid:016X}: {type(e).__name__}: {e}")
        obs["ok"] = False
    return obs


def agree(res, label, observations, key, tolerance=0):
    """Every motor must report the same value for `key`."""
    vals = {}
    for uid, obs in observations.items():
        if not obs.get("ok"):
            continue
        vals.setdefault(_bucket(obs.get(key), tolerance), []).append(uid)
    if len(vals) == 0:
        res.check(label, False, "no motor produced a reading")
        return
    if len(vals) == 1:
        only = next(iter(vals))
        res.check(f"{label} (all agree on {only})", True)
        return
    detail = "; ".join(
        f"{v} on {len(u)} motor(s) e.g. {u[0]:016X}" for v, u in sorted(vals.items(), key=lambda kv: -len(kv[1]))
    )
    res.check(label, False, f"DISAGREEMENT -> {detail}")


def _bucket(value, tolerance):
    """Reduce a reading to a comparable, hashable key.

    Some getters return lists (get_firmware_version is one), which cannot be
    dictionary keys, so anything that is not a plain scalar is compared by its
    repr. `tolerance` snaps near-equal numbers together so that a couple of
    encoder counts of jitter does not read as a fleet disagreement.
    """
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if tolerance:
            return round(value / tolerance) * tolerance
        return value
    if isinstance(value, str):
        return value
    return repr(value)


def main(args):
    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port()
    res = Results()
    detail = []
    try:
        print("Detecting motors on the bus (several rounds -- under-detection is transient)...")
        devices = detect_devices_iteratively(n_detections=4, verbose=False)
        uids = sorted({d.unique_id for d in devices})
        print(f"\nFound {len(uids)} motor(s)\n")
        res.check("at least two motors are present (a fleet test needs a fleet)",
                  len(uids) >= 2, f"found {len(uids)}")
        if len(uids) < 2:
            raise RuntimeError("not enough motors for a fleet comparison")

        if args.limit and len(uids) > args.limit:
            print(f"(limiting to the first {args.limit} motors as requested)\n")
            uids = uids[:args.limit]

        observations = {}
        for i, uid in enumerate(uids, 1):
            print(f"[{i}/{len(uids)}] probing {uid:016X} ...", flush=True)
            observations[uid] = probe_one(uid, detail)

        probed = [u for u, o in observations.items() if o.get("ok")]
        print(f"\nProbed {len(probed)}/{len(uids)} motors successfully\n")
        res.check("every motor completed the probe without a communication failure",
                  len(probed) == len(uids),
                  f"{len(uids) - len(probed)} failed: " + "; ".join(detail[:3]))

        print("Cross-motor comparisons:")
        # 1. Identity
        agree(res, "all motors run the same firmware version", observations, "firmware")

        # 2. Determinism of the planner across units
        agree(res, "the same trapezoid move travels the same distance on every motor",
              observations, "plain_move_moved", tolerance=4)
        agree(res, "no motor faults on the plain move", observations, "plain_move_fatal")

        # 3. Planner shape
        agree(res, "the normal trapezoid path costs the same slots on every motor",
              observations, "slots_normal")
        agree(res, "the one-tick-ramp fast path costs the same slots on every motor",
              observations, "slots_fastpath")

        # 4. Refusals agree, code and all
        agree(res, "the same illegal move earns the same error code on every motor",
              observations, "illegal_move_fatal")

        # 5. Hygiene
        agree(res, "every motor started clean", observations, "fatal_at_start")
        agree(res, "every motor finished clean", observations, "fatal_at_end")

        # Absolute checks on the agreed values, so a fleet that agrees on the
        # WRONG answer is still caught.
        good = [observations[u] for u in probed]
        if good:
            want = COUNTS_PER_ROTATION // 8
            res.check("the agreed distance is the one that was commanded",
                      all(g.get("plain_move_moved") is not None and
                          abs(g["plain_move_moved"] - want) <= 4 for g in good),
                      f"first={good[0].get('plain_move_moved')} want={want}")
            res.check("the agreed normal slot cost is 3",
                      all(g.get("slots_normal") == 3 for g in good),
                      f"first={good[0].get('slots_normal')}")
            res.check("the agreed fast-path slot cost is 2",
                      all(g.get("slots_fastpath") == 2 for g in good),
                      f"first={good[0].get('slots_fastpath')}")
            res.check("the agreed refusal is a limit error (15, 16 or 28)",
                      all(g.get("illegal_move_fatal") in (15, 16, 28) for g in good),
                      f"first={good[0].get('illegal_move_fatal')}")
            res.check("every motor is left with no fatal error",
                      all(g.get("fatal_at_end") == 0 for g in good))

    except Exception as e:
        print(f"\nTest aborted: {type(e).__name__}: {e}", file=sys.stderr)
        res.failed += 1
        res.failures.append(f"aborted: {e}")
    finally:
        servomotor.close_serial_port()
        print("Closed the serial port")

    print(f"\n{res.passed} passed, {res.failed} failed")
    for f in res.failures:
        print(f"  - {f}")
    if detail:
        print("\nPer-motor problems:")
        for d in detail:
            print(f"  - {d}")
    if res.failed == 0:
        print("\nPASSED")
        sys.exit(0)
    print("\nFAILED")
    sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Fleet consistency: every motor on the bus must answer the "
                    "same command the same way.")
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', action='store_true',
                        help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', help='ignored; motors are addressed by unique ID',
                        default=None)
    parser.add_argument('--limit', type=int, default=0,
                        help='probe at most this many motors (0 = all)')
    args = parser.parse_args()
    main(args)
