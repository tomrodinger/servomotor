"""Phase 11 — Thermal (run to overtemperature under load).

Run the motor at full power until it reaches its overtemperature cutoff (a
fatal error from the firmware) or until a maximum time limit, logging
temperature (cmd 42) every second.  Before the first rotation the max allowable
position deviation is set tight (cmd 44, default 0.01 rot): if the driver IC
overheats and momentarily cuts out (rotation stops) — e.g. from missing thermal
paste — the deviation trips a fatal error, which Stage B treats as a failure
(distinct from the expected overtemperature cutoff).

Collected (Stage A): the fresh baseline, the full per-second temperature series
(blob), the max temperature reached, the last temperature, and the fatal-error
code (if any) with whether the run reached the time limit.
"""

from __future__ import annotations

import time

from .base import Phase, CollectionContext
from .. import units, blobs
from ..transport import TimeoutError as RS485Timeout, FatalError

# Arm the tight deviation limit only after the motor has reached steady speed:
# the acceleration following-error at startup exceeds 0.01 rot on a healthy
# motor and would otherwise trip a false deviation fatal in the first second.
# A real driver cut-out happens later (when hot) and still trips it.
DEVIATION_ARM_DELAY_S = 2.0


class ThermalPhase(Phase):
    number = 11

    def collect(self, ctx: CollectionContext) -> None:
        max_time = float(ctx.params.get("max_time_s", 1200.0))
        max_current = int(ctx.params.get("max_current", 200))
        velocity = float(ctx.params.get("spin_velocity", 1.0))
        dev_tol_rot = float(ctx.params.get("deviation_tolerance_rot", 0.01))
        vel_internal = units.rot_per_s_to_internal(velocity)
        dev_counts = units.rotations_to_counts(dev_tol_rot)

        for batch in ctx.batched(ctx.calibrated_motors(), ctx.power_limit):
            ctx.check_cancel()
            ctx.wait_if_paused()
            baseline = {}
            series = {uid: [] for uid in batch}
            max_temp = {uid: None for uid in batch}
            last_temp = {uid: None for uid in batch}
            last_t = {uid: 0.0 for uid in batch}
            fatal = {uid: None for uid in batch}
            done = set()

            for uid in batch:
                ctx.mark_collecting(uid)
                try:
                    baseline[uid] = ctx.client(uid).get_temperature()
                    max_temp[uid] = baseline[uid]
                except (RS485Timeout, FatalError, Exception):
                    baseline[uid] = None
            # start spinning under load (the tight deviation limit is armed
            # below, after spin-up, to avoid a false startup trip)
            for uid in batch:
                try:
                    c = ctx.client(uid)
                    c.set_max_current(max_current)
                    c.enable_mosfets()
                    c.move_with_velocity(vel_internal,
                                         units.seconds_to_timesteps(max_time + 5))
                except (RS485Timeout, FatalError, Exception):
                    pass

            start = time.monotonic()
            next_log = start
            armed = False
            while time.monotonic() - start < max_time and len(done) < len(batch):
                ctx.check_cancel()
                now = time.monotonic()
                if not armed and now - start >= DEVIATION_ARM_DELAY_S:
                    # arm the cut-out detector once the motor is at steady speed
                    for uid in batch:
                        if uid in done:
                            continue
                        try:
                            ctx.client(uid).set_max_allowable_position_deviation(dev_counts)
                        except (RS485Timeout, FatalError, Exception):
                            pass
                    armed = True
                if now >= next_log:
                    t = now - start
                    for uid in batch:
                        if uid in done:
                            continue
                        try:
                            temp = float(ctx.client(uid).get_temperature())
                            series[uid].append((t, temp))
                            last_temp[uid] = temp
                            last_t[uid] = t
                            if max_temp[uid] is None or temp > max_temp[uid]:
                                max_temp[uid] = temp
                        except FatalError as exc:
                            # the motor faulted (overtemp cutoff, deviation trip,
                            # or other) — record the code and stop polling it
                            fatal[uid] = int(exc.args[0]) if exc.args else 1
                            done.add(uid)
                        except (RS485Timeout, Exception):
                            pass
                    next_log += 1.0
                ctx.sleep(min(0.2, max(0.0, next_log - time.monotonic())))

            ran_full = (time.monotonic() - start) >= max_time
            for uid in batch:
                try:
                    ctx.client(uid).disable_mosfets()
                except Exception:
                    pass
                obs = {
                    "baseline": baseline[uid],
                    "max_temp": max_temp[uid],
                    "last_temp": last_temp[uid],
                    "last_t": last_t[uid],
                    "fatal_code": fatal[uid],
                    "ran_full": ran_full and fatal[uid] is None,
                    "max_time_s": max_time,
                    "n_samples": len(series[uid]),
                }
                if series[uid]:
                    ctx.store(uid, raw_blob=blobs.pack_temperature_series(series[uid]),
                              observation=obs)
                else:
                    obs["missing"] = True
                    ctx.store(uid, observation=obs)


PHASE = ThermalPhase()
