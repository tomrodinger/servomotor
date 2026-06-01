"""Phase 9 — Current control (inverted pass/fail).

Set a very low max current (cmd 28), enable, zero, command a one-rotation move,
then read the hall position after it settles.  The motor should be too weak to
turn — Stage B passes if the hall position moved *less* than the no-rotation
threshold.  The between-phase system_reset restores the normal current limit.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from .. import units
from ..transport import TimeoutError as RS485Timeout, FatalError


class CurrentControlPhase(Phase):
    number = 9

    def collect(self, ctx: CollectionContext) -> None:
        low_current = int(ctx.params.get("low_current", 5))
        one_rotation = units.rotations_to_counts(1.0)
        move_time = units.seconds_to_timesteps(1.0)

        for batch in ctx.batched(ctx.calibrated_motors(), ctx.power_limit):
            ctx.check_cancel()
            ctx.wait_if_paused()
            for uid in batch:
                ctx.mark_collecting(uid)
                try:
                    c = ctx.client(uid)
                    c.set_max_current(low_current)
                    c.enable_mosfets()
                    c.zero_position()
                    c.trapezoid_move(one_rotation, move_time)
                except (RS485Timeout, FatalError, Exception):
                    pass
            ctx.sleep(1.2)   # let the (attempted) move settle
            for uid in batch:
                change_rot = None
                try:
                    hall = ctx.client(uid).get_hall_position()
                    change_rot = abs(units.counts_to_rotations(hall))
                    ctx.client(uid).disable_mosfets()
                except (RS485Timeout, FatalError, Exception):
                    change_rot = None
                if change_rot is None:
                    ctx.store(uid, observation={"missing": True})
                else:
                    ctx.store(uid, scalar=change_rot,
                              observation={"hall_position_change": change_rot})


PHASE = CurrentControlPhase()
