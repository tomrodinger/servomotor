"""Phase 6 — Continuous-spin tracking (one motor at a time, dense sampling).

Open-loop, max current.  Spin forward 1.5 rotations then back, polling
``get_comprehensive_position`` (cmd 37) as fast as the bus allows.  The full
(commanded, hall) stream for both directions is stored as a binary blob; the
max deviation is derived later in Stage B.
"""

from __future__ import annotations

import time
from typing import List, Tuple

from .base import Phase, CollectionContext
from .. import units, blobs
from ..transport import TimeoutError as RS485Timeout, FatalError

TARGET_SAMPLES_PER_DIR = 4000   # aim for a few thousand points per direction


class SpinTrackingPhase(Phase):
    number = 6

    def collect(self, ctx: CollectionContext) -> None:
        rotations = float(ctx.params.get("spin_rotations", 1.5))
        velocity = float(ctx.params.get("spin_velocity", 0.5))
        max_current = int(ctx.params.get("max_current", 200))
        move_time_s = rotations / velocity if velocity else 1.0
        vel_internal = units.rot_per_s_to_internal(velocity)
        poll_interval = max(0.0005, move_time_s / TARGET_SAMPLES_PER_DIR)

        for uid in ctx.calibrated_motors():
            ctx.check_cancel()
            ctx.wait_if_paused()
            ctx.mark_collecting(uid)
            client = ctx.client(uid)
            samples: List[Tuple[int, int, int]] = []
            try:
                client.set_max_current(max_current)
                client.enable_mosfets()
                ctx.sleep(0.05)
                client.zero_position()
                for direction, sign in ((0, 1), (1, -1)):
                    client.move_with_velocity(sign * vel_internal,
                                               units.seconds_to_timesteps(move_time_s))
                    start = time.monotonic()
                    while time.monotonic() - start < move_time_s:
                        ctx.check_cancel()
                        try:
                            cmd, hall, _ = client.get_comprehensive_position()
                            samples.append((direction, cmd, hall))
                        except (RS485Timeout, FatalError):
                            break
                        time.sleep(poll_interval)
                client.disable_mosfets()
            except (RS485Timeout, FatalError, Exception):
                pass
            if samples:
                ctx.store(uid, raw_blob=blobs.pack_position_stream(samples),
                          observation={"n_samples": len(samples)})
            else:
                ctx.store(uid, observation={"n_samples": 0, "missing": True})


PHASE = SpinTrackingPhase()
