"""Phase 5 — Calibration (full motion chain).

A batch of 8 motors is calibrated together.  Send ``start_calibration`` (cmd 6)
to all 8 in rapid succession, then keep the bus completely quiet for the hold
time (default 30 s).  The motor resets itself when calibration finishes — we
must **not** send ``system_reset`` here.  After the quiet hold, read status once
per motor; on a zero status, set ``calibration_done`` in the database.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from ..transport import TimeoutError as RS485Timeout, FatalError


class CalibrationPhase(Phase):
    number = 5

    def collect(self, ctx: CollectionContext) -> None:
        hold = float(ctx.params.get("quiet_hold_s", 30.0))
        stagger = float(ctx.params.get("start_stagger_s", 0.5))
        for batch in ctx.batched(ctx.motors, ctx.power_limit):
            ctx.check_cancel()
            ctx.wait_if_paused()
            # Pace the start-calibration commands (one per `stagger` seconds) so
            # the early motors are not hit by command traffic while they have
            # already begun calibrating (avoids ERROR_COMMAND_OVERFLOW).
            for uid in batch:
                ctx.mark_collecting(uid)
                try:
                    ctx.client(uid).start_calibration()
                except (RS485Timeout, FatalError, Exception):
                    pass
                ctx.sleep(stagger)
            # Then keep the bus completely quiet for the hold (no polling).
            ctx.sleep(hold)
            for uid in batch:
                flags, fatal = (None, None)
                status_zero = False
                try:
                    flags, fatal = ctx.client(uid).get_status()
                    status_zero = (flags == 0 and fatal == 0)
                except (RS485Timeout, FatalError, Exception):
                    status_zero = False
                if status_zero:
                    ctx.db.set_calibration_done(uid, True)
                ctx.store(uid, observation={
                    "status_flags": flags, "fatal_code": fatal,
                    "status_zero": status_zero})


PHASE = CalibrationPhase()
