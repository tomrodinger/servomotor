"""Phase 13 — Closed-loop burn-in (all motors, staggered, many moves).

All motors run closed-loop for the full duration.  Each move is a trapezoid
move (cmd 2) to a random displacement within +/-N rotations over a fixed move
duration.  After (move duration + margin) the host reads the max PID error
(cmd 39, read-and-reset) and queues the next move — no status polling and no
deviation limit.  The full per-move PID series is stored as a binary blob; the
overall max PID deviation and fatal-error flag are derived in Stage B.
"""

from __future__ import annotations

import random
import time

from .base import Phase, CollectionContext
from .. import units, blobs
from ..transport import TimeoutError as RS485Timeout, FatalError


class ClosedLoopBurnInPhase(Phase):
    number = 13

    def collect(self, ctx: CollectionContext) -> None:
        duration = float(ctx.params.get("duration_s", 12600.0))
        move_duration = float(ctx.params.get("move_duration_s", 5.0))
        margin = float(ctx.params.get("quiet_margin_pct", 5.0)) / 100.0
        max_mag = float(ctx.params.get("max_move_magnitude_rot", 5.0))
        quiet = move_duration * (1.0 + margin)
        move_steps = units.seconds_to_timesteps(move_duration)
        rng = random.Random()

        motors = list(ctx.calibrated_motors())
        pid_series = {uid: [] for uid in motors}
        fatal = {uid: None for uid in motors}
        in_progress = {uid: False for uid in motors}
        next_due = {}

        for uid in motors:
            ctx.mark_collecting(uid)
            try:
                c = ctx.client(uid)
                c.enable_mosfets()
                c.go_to_closed_loop()
                c.zero_position()
            except (RS485Timeout, FatalError, Exception):
                pass

        # Let the servo stabilize after entering closed loop, then read-and-clear
        # the max PID error once (cmd 39 resets on read) so the activation
        # transient is discarded and not recorded as the first (spuriously high)
        # data point.
        ctx.sleep(0.3)
        for uid in motors:
            try:
                ctx.client(uid).get_max_pid_error()
            except (RS485Timeout, FatalError, Exception):
                pass

        now0 = time.monotonic()
        for uid in motors:
            next_due[uid] = now0 + rng.uniform(0.0, 1.0)   # staggered start
        end = now0 + duration
        active = set(motors)
        while time.monotonic() < end and active:
            ctx.check_cancel()
            ctx.wait_if_paused()
            now = time.monotonic()
            for uid in list(active):
                if now < next_due[uid]:
                    continue
                c = ctx.client(uid)
                if in_progress[uid]:
                    try:
                        mn, mx = c.get_max_pid_error()
                        pid_series[uid].append((mn, mx))
                    except FatalError as exc:
                        fatal[uid] = int(exc.args[0]) if exc.args else 1
                        active.discard(uid)
                        continue
                    except (RS485Timeout, Exception):
                        pass
                disp = rng.uniform(-max_mag, max_mag)
                try:
                    c.trapezoid_move(units.rotations_to_counts(disp), move_steps)
                    in_progress[uid] = True
                    next_due[uid] = now + quiet
                except FatalError as exc:
                    fatal[uid] = int(exc.args[0]) if exc.args else 1
                    active.discard(uid)
                except (RS485Timeout, Exception):
                    # A single motor's comms hiccup must not abort the whole
                    # phase; skip this move and retry it next round.
                    next_due[uid] = now + quiet
            ctx.sleep(0.05)

        for uid in motors:
            try:
                ctx.client(uid).disable_mosfets()
            except Exception:
                pass
            series = pid_series[uid]
            ctx.store(uid, raw_blob=blobs.pack_pid_series(series) if series else None,
                      observation={"fatal_error": fatal[uid],
                                   "no_fatal_error": fatal[uid] is None,
                                   "n_moves": len(series)})


PHASE = ClosedLoopBurnInPhase()
