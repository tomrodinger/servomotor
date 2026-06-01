"""Phase 11 — Thermal (thermistor / thermal behaviour under load).

Take a fresh baseline temperature, run the motor at full power (8 per section)
while logging temperature (cmd 42) every second for the configured duration.
The fresh baseline and the full per-second temperature series are stored as a
binary blob; rise/slope/start/R are derived in Stage B.
"""

from __future__ import annotations

import time

from .base import Phase, CollectionContext
from .. import units, blobs
from ..transport import TimeoutError as RS485Timeout, FatalError


class ThermalPhase(Phase):
    number = 11

    def collect(self, ctx: CollectionContext) -> None:
        duration = float(ctx.params.get("duration_s", 120.0))
        max_current = int(ctx.params.get("max_current", 200))
        velocity = float(ctx.params.get("spin_velocity", 1.0))
        vel_internal = units.rot_per_s_to_internal(velocity)

        for batch in ctx.batched(ctx.calibrated_motors(), ctx.power_limit):
            ctx.check_cancel()
            ctx.wait_if_paused()
            baselines = {}
            series = {uid: [] for uid in batch}
            for uid in batch:
                ctx.mark_collecting(uid)
                try:
                    baselines[uid] = ctx.client(uid).get_temperature()
                except (RS485Timeout, FatalError, Exception):
                    baselines[uid] = None
            # start all motors spinning under load
            for uid in batch:
                try:
                    c = ctx.client(uid)
                    c.set_max_current(max_current)
                    c.enable_mosfets()
                    c.move_with_velocity(vel_internal, units.seconds_to_timesteps(duration + 5))
                except (RS485Timeout, FatalError, Exception):
                    pass
            start = time.monotonic()
            next_log = start
            while time.monotonic() - start < duration:
                ctx.check_cancel()
                now = time.monotonic()
                if now >= next_log:
                    t = now - start
                    for uid in batch:
                        try:
                            temp = ctx.client(uid).get_temperature()
                            series[uid].append((t, float(temp)))
                        except (RS485Timeout, FatalError):
                            pass
                    next_log += 1.0
                ctx.sleep(min(0.2, max(0.0, next_log - time.monotonic())))
            for uid in batch:
                try:
                    ctx.client(uid).disable_mosfets()
                except (RS485Timeout, FatalError, Exception):
                    pass
                s = series[uid]
                if s:
                    ctx.store(uid, raw_blob=blobs.pack_temperature_series(s),
                              observation={"baseline": baselines[uid],
                                           "n_samples": len(s)})
                else:
                    ctx.store(uid, observation={"baseline": baselines[uid],
                                                "missing": True})


PHASE = ThermalPhase()
