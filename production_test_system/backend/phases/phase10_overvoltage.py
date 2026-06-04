"""Phase 10 — Overvoltage protection.

Requires firmware >= 0.15.1.0, which adds cmd 36 test modes that set the
overvoltage-protection threshold to a fixed voltage:

* **mode 74** sets the OV threshold to **22 V** — on a 24 V rack this must trip
  (the comparator fires and the firmware raises ``ERROR_OVERVOLTAGE``).
* **mode 75** sets it to **26 V** — on a 24 V rack this must NOT trip.

Procedure per motor (no motion, 8 in parallel per section): enter the 22 V mode,
wait, read status (cmd 16) for the trip; system_reset; enter the 26 V mode,
wait, read status; system_reset.  A trip shows up as fatal code
``ERROR_OVERVOLTAGE`` in the status word.  Stage B passes only if the 22 V mode
tripped AND the 26 V mode did not.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from .. import config
from ..transport import TimeoutError as RS485Timeout, FatalError

OV_MODE_22V = 74          # should trip on a 24 V rack
OV_MODE_26V = 75          # should not trip on a 24 V rack
ERROR_OVERVOLTAGE = 14    # firmware fatal code reported on a trip


class OvervoltagePhase(Phase):
    number = 10

    def collect(self, ctx: CollectionContext) -> None:
        settle = float(ctx.params.get("settle_s", 0.5))

        def enter_mode(uid, mode):
            try:
                ctx.client(uid).test_mode(mode)
            except (RS485Timeout, FatalError, Exception):
                # The trip may arrive as the command is processed; that's fine.
                pass

        def read_trip(uid):
            """Return True if the motor is in the OV fatal state, None if it
            could not be read."""
            try:
                _flags, code = ctx.client(uid).get_status()
                return code == ERROR_OVERVOLTAGE
            except (RS485Timeout, FatalError, Exception):
                return None

        def broadcast_reset():
            try:
                ctx.transport.transact(255, "System reset")
            except Exception:
                pass
            ctx.sleep(config.BOOTLOADER_EXIT_DELAY_S)
            ctx.transport.flush_input()

        for batch in ctx.batched(ctx.motors, ctx.power_limit):
            ctx.check_cancel()
            ctx.wait_if_paused()
            for uid in batch:
                ctx.mark_collecting(uid)

            # 22 V threshold — expect a trip
            for uid in batch:
                enter_mode(uid, OV_MODE_22V)
            ctx.sleep(settle)
            tripped_low = {uid: read_trip(uid) for uid in batch}
            broadcast_reset()      # clear the OV fatal before the next mode

            # 26 V threshold — expect no trip
            for uid in batch:
                enter_mode(uid, OV_MODE_26V)
            ctx.sleep(settle)
            tripped_high = {uid: read_trip(uid) for uid in batch}
            broadcast_reset()      # restore the normal OV threshold

            for uid in batch:
                low = tripped_low[uid]
                high = tripped_high[uid]
                if low is None and high is None:
                    ctx.store(uid, observation={"tripped_low": None,
                                                "tripped_high": None,
                                                "missing": True})
                else:
                    ctx.store(uid, observation={"tripped_low": low,
                                                "tripped_high": high})


PHASE = OvervoltagePhase()
