"""Phase 12 — Open-loop burn-in (random-batch scheme).

Repeatedly pick 8 random motors and spin each a random distance in a random
direction for a fixed interval, for the full configured duration.  A tight
position-deviation limit (cmd 44) makes the firmware raise a fatal error on a
skipped step.  Any fatal error fails that motor's burn-in and excludes it from
further selection; the rest keep going.  Stage A records any fatal-error event.
"""

from __future__ import annotations

import random
import time

from .base import Phase, CollectionContext
from .. import units
from ..transport import TimeoutError as RS485Timeout, FatalError


class OpenLoopBurnInPhase(Phase):
    number = 12

    def collect(self, ctx: CollectionContext) -> None:
        duration = float(ctx.params.get("duration_s", 12600.0))
        interval = float(ctx.params.get("spin_interval_s", 5.0))
        dev_tol_rot = float(ctx.params.get("deviation_tolerance_rot", 0.01))
        max_current = int(ctx.params.get("max_current", 200))
        dev_counts = units.rotations_to_counts(dev_tol_rot)
        rng = random.Random()

        active = list(ctx.calibrated_motors())
        fatal = {uid: None for uid in active}
        set_up = set()
        end = time.monotonic() + duration

        while time.monotonic() < end and active:
            ctx.check_cancel()
            ctx.wait_if_paused()
            batch = rng.sample(active, min(ctx.power_limit, len(active)))
            for uid in batch:
                c = ctx.client(uid)
                try:
                    if uid not in set_up:
                        ctx.mark_collecting(uid)
                        c.set_max_current(max_current)
                        c.enable_mosfets()
                        ctx.sleep(0.3)          # settle before zeroing/limiting
                        c.zero_position()
                        c.set_max_allowable_position_deviation(dev_counts)
                        set_up.add(uid)
                    distance = rng.uniform(-3.0, 3.0)
                    c.trapezoid_move(units.rotations_to_counts(distance),
                                     units.seconds_to_timesteps(interval))
                except FatalError as exc:
                    fatal[uid] = int(exc.args[0]) if exc.args else 1
                except (RS485Timeout, Exception):
                    pass
            ctx.sleep(interval)
            # check the batch for fatal errors
            for uid in list(batch):
                try:
                    flags, code = ctx.client(uid).get_status()
                    if code:
                        fatal[uid] = code
                except (RS485Timeout, FatalError, Exception):
                    pass
                if fatal[uid] and uid in active:
                    active.remove(uid)

        for uid in ctx.calibrated_motors():
            code = fatal.get(uid)
            try:
                ctx.client(uid).disable_mosfets()
            except Exception:
                pass
            ctx.store(uid, observation={"fatal_error": code,
                                        "no_fatal_error": code is None})


PHASE = OpenLoopBurnInPhase()
