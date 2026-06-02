"""Phase 9 — Current control (closed-loop low-current PID-deviation band).

At a low max current the servo cannot achieve a deliberately-fast move, so it
lags and the closed-loop PID error grows large.  Procedure (low power, so most
commands are broadcast to the whole bus at once):

1. Broadcast a low current limit (cmd 28), enable, go to closed loop (cmd 17),
   zero.
2. Read max PID error (cmd 39) once per motor to **clear** it (cmd 39 is
   read-and-reset).
3. Broadcast a fast move (default 1.8 rotations in 0.5 s) that the low current
   cannot keep up with.
4. Wait (default 5 s) for the (much slower than commanded) move to finish.
5. Read max PID error again per motor — this captures the deviation during the
   move.

Stage B passes only if the max PID deviation is within a band: too small means
the current limit is not actually limiting (current-control path fault); too
large flags a different problem.  This replaced the old "must not rotate" check,
which failed because the motor can complete a slow move even at very low current.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from .. import units
from ..transport import TimeoutError as RS485Timeout, FatalError


class CurrentControlPhase(Phase):
    number = 9

    def collect(self, ctx: CollectionContext) -> None:
        low_current = int(ctx.params.get("low_current", 10))
        move_rot = float(ctx.params.get("move_rotations", 1.8))
        move_time_s = float(ctx.params.get("move_time_s", 0.5))
        wait_s = float(ctx.params.get("wait_s", 5.0))
        move_counts = units.rotations_to_counts(move_rot)
        move_steps = units.seconds_to_timesteps(move_time_s)

        motors = ctx.calibrated_motors()
        if not motors:
            return
        for uid in motors:
            ctx.mark_collecting(uid)

        def broadcast(command, inputs=None):
            try:
                ctx.transport.transact(255, command, inputs or [])
            except Exception:
                pass

        # 1. Low current, closed loop, zeroed — broadcast to the whole bus.
        broadcast("Set maximum motor current", [low_current, 0])
        broadcast("Enable MOSFETs")
        broadcast("Go to closed loop")
        ctx.sleep(0.3)
        broadcast("Zero position")
        ctx.sleep(0.2)

        # 2. Clear the PID-error register on each motor (cmd 39 read-and-reset).
        for uid in motors:
            try:
                ctx.client(uid).get_max_pid_error()
            except (RS485Timeout, FatalError, Exception):
                pass

        # 3-4. Broadcast the fast move and wait for it to (slowly) complete.
        broadcast("Trapezoid move", [move_counts, move_steps])
        ctx.sleep(wait_s)

        # 5. Read the max PID deviation accumulated during the move.
        for uid in motors:
            try:
                mn, mx = ctx.client(uid).get_max_pid_error()
                dev = max(abs(int(mn)), abs(int(mx)))
                ctx.store(uid, scalar=float(dev),
                          observation={"max_pid_deviation": dev,
                                       "min_pid": int(mn), "max_pid": int(mx)})
            except (RS485Timeout, FatalError, Exception):
                ctx.store(uid, observation={"missing": True})

        broadcast("Disable MOSFETs")


PHASE = CurrentControlPhase()
