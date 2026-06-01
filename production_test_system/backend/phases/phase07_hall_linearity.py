"""Phase 7 — Hall linearity & hysteresis (static, stepped, 8 in parallel).

Open-loop, max current.  Step through N evenly-spaced absolute positions over
one rotation CW then CCW; at each step, settle, then read cmd 37 for the whole
group of 8 (fan-out/gather).  The full (commanded, hall) pair at every step for
both sweeps is stored as a binary blob; max deviation is derived in Stage B.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

from .base import Phase, CollectionContext
from .. import units, blobs
from ..transport import TimeoutError as RS485Timeout, FatalError


class HallLinearityPhase(Phase):
    number = 7

    def collect(self, ctx: CollectionContext) -> None:
        n_steps = int(ctx.params.get("n_steps", 2000))
        settle_s = int(ctx.params.get("settle_ms", 50)) / 1000.0
        max_current = int(ctx.params.get("max_current", 200))
        counts_per_rev = units.COUNTS_PER_ROTATION
        move_time_steps = units.seconds_to_timesteps(max(0.005, settle_s * 0.6))

        for batch in ctx.batched(ctx.calibrated_motors(), ctx.power_limit):
            ctx.check_cancel()
            ctx.wait_if_paused()
            samples: Dict[int, List[Tuple[int, int, int]]] = {uid: [] for uid in batch}
            try:
                for uid in batch:
                    ctx.mark_collecting(uid)
                    c = ctx.client(uid)
                    c.set_max_current(max_current)
                    c.enable_mosfets()
                    c.zero_position()
                ctx.sleep(0.05)
                for sweep, step_range in ((0, range(n_steps)), (1, range(n_steps - 1, -1, -1))):
                    for step in step_range:
                        ctx.check_cancel()
                        target = int(round(step * counts_per_rev / n_steps))
                        for uid in batch:
                            try:
                                ctx.client(uid).go_to_position(target, move_time_steps)
                            except (RS485Timeout, FatalError):
                                pass
                        ctx.sleep(settle_s)
                        for uid in batch:
                            try:
                                cmd, hall, _ = ctx.client(uid).get_comprehensive_position()
                                samples[uid].append((sweep, cmd, hall))
                            except (RS485Timeout, FatalError):
                                pass
                for uid in batch:
                    ctx.client(uid).disable_mosfets()
            except (RS485Timeout, FatalError, Exception):
                pass
            for uid in batch:
                s = samples[uid]
                if s:
                    ctx.store(uid, raw_blob=blobs.pack_position_stream(s),
                              observation={"n_samples": len(s)})
                else:
                    ctx.store(uid, observation={"n_samples": 0, "missing": True})


PHASE = HallLinearityPhase()
