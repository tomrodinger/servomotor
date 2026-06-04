"""Phase 8 — Hall waveform & peak analysis (one motor at a time).

Capture the raw 3-channel hall waveform (same parameters as
``capture_hall_sensor_data.py``) while spinning ~1.4 rotations.  The full
capture is stored as a binary blob; the peak/valley analysis and all metrics
are done later in Stage B.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from .. import units
from ..transport import TimeoutError as RS485Timeout, FatalError


class HallWaveformPhase(Phase):
    number = 8

    def collect(self, ctx: CollectionContext) -> None:
        p = ctx.params
        capture_type = int(p.get("capture_type", 1))
        n_points = int(p.get("n_points", 4000))
        bitmask = int(p.get("channels_bitmask", 7))
        tsps = int(p.get("time_steps_per_sample", 1))
        n_sum = int(p.get("n_samples_to_sum", 64))
        div = int(p.get("division_factor", 16))
        rotations = float(p.get("spin_rotations", 1.4))
        # spin slowly enough that the capture spans ~1.4 rotations
        spin_time_s = 9.0

        for uid in ctx.calibrated_motors():
            ctx.check_cancel()
            ctx.wait_if_paused()
            ctx.mark_collecting(uid)
            client = ctx.client(uid)
            raw = b""
            try:
                client.set_max_allowable_position_deviation(2_000_000_000)
                client.enable_mosfets()
                ctx.sleep(0.05)
                client.trapezoid_move(units.rotations_to_counts(rotations),
                                      units.seconds_to_timesteps(spin_time_s))
                ctx.sleep(0.2)   # let it reach steady velocity before capturing
                raw = client.capture_hall_sensor_data(
                    capture_type, n_points, bitmask, tsps, n_sum, div, timeout=15.0)
                client.disable_mosfets()
            except (RS485Timeout, FatalError, Exception):
                raw = b""
            if raw:
                ctx.store(uid, raw_blob=raw,
                          observation={"n_bytes": len(raw),
                                       "n_samples_to_sum": n_sum,
                                       "division_factor": div})
            else:
                ctx.store(uid, observation={"missing": True})


PHASE = HallWaveformPhase()
