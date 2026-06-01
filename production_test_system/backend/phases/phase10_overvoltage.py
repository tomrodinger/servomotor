"""Phase 10 — Overvoltage protection (disabled until firmware support exists).

Requires two new firmware test modes (22 V and 26 V) via cmd 36 that do not yet
exist.  The phase is defined and disabled by default; if it is somehow run, it
records a missing observation rather than driving non-existent test modes.
"""

from __future__ import annotations

from .base import Phase, CollectionContext


class OvervoltagePhase(Phase):
    number = 10

    def collect(self, ctx: CollectionContext) -> None:
        ctx.log("Phase 10 requires firmware test modes (22 V / 26 V) that do "
                "not yet exist — recording missing observations.")
        for uid in ctx.motors:
            ctx.check_cancel()
            ctx.store(uid, observation={
                "tripped_low": None, "tripped_high": None,
                "missing": True, "reason": "firmware test modes not present"})


PHASE = OvervoltagePhase()
