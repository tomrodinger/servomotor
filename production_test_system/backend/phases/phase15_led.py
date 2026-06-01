"""Phase 15 — LED test (must run last).

Drive both LEDs solid on via the firmware LED test mode (cmd 36, mode 10).
This LOCKS the motor until a power cycle, so nothing automated can run after it.
A human then confirms whether all LEDs lit; the per-device pass/fail
observation is recorded by the confirmation API (directly, or via the
removal-and-ping reconciliation), not during this collect step.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from ..state import GRID_ORANGE
from ..transport import TimeoutError as RS485Timeout, FatalError

# cmd 36 test modes 10..13 map to LED bitmasks 0..3 (green=bit0, red=bit1) in
# the firmware (main.c set_led_test_mode).  Mode 13 = bitmask 3 = BOTH LEDs on
# (mode 10 = bitmask 0 locks the motor with no LEDs lit).
LED_ON_TEST_MODE = 13


class LedTestPhase(Phase):
    number = 15

    def collect(self, ctx: CollectionContext) -> None:
        for uid in ctx.motors:
            ctx.check_cancel()
            try:
                ctx.client(uid).test_mode(LED_ON_TEST_MODE)
            except (RS485Timeout, FatalError, Exception):
                # Some firmwares stop responding the instant they lock; that is
                # expected, not an error.
                pass
            ctx.set_grid(uid, GRID_ORANGE)   # awaiting human confirmation

        # Raise the global human-confirmation prompt (idempotent across buses).
        def _set(state):
            state.led_prompt = {
                "pending": True,
                "question": "Did all motors pass the LED test (all LEDs lit)?",
            }
        ctx.state.update(_set)
        ctx.log("Phase 15: LED test mode sent; awaiting human confirmation. "
                "Motors are locked until a power cycle.")


PHASE = LedTestPhase()
