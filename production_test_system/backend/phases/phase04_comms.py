"""Phase 4 — Comms quality (pings + communication statistics)."""

from __future__ import annotations

from .base import Phase, CollectionContext
from ..transport import TimeoutError as RS485Timeout, FatalError


class CommsPhase(Phase):
    number = 4

    def collect(self, ctx: CollectionContext) -> None:
        ping_count = int(ctx.params.get("ping_count", 1000))
        payload = bytes(range(10))   # 10-byte buf10 payload
        for uid in ctx.motors:
            ctx.check_cancel()
            ctx.mark_collecting(uid)
            client = ctx.client(uid)
            successes = 0
            for _ in range(ping_count):
                if ctx.cancelled:
                    break
                try:
                    echoed = client.ping(payload)
                    if bytes(echoed) == payload:
                        successes += 1
                except (RS485Timeout, FatalError, Exception):
                    pass
            stats = {}
            try:
                stats = client.get_communication_statistics(reset=0)
            except (RS485Timeout, FatalError, Exception):
                stats = {}
            rate = (100.0 * successes / ping_count) if ping_count else 0.0
            obs = {
                "ping_total": ping_count,
                "ping_success": successes,
                "success_rate": rate,
                "crc_errors": stats.get("crc_errors", 0),
                "comm_stats": stats,
            }
            ctx.store(uid, scalar=rate, observation=obs)


PHASE = CommsPhase()
