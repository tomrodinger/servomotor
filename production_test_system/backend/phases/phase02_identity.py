"""Phase 2 — Identity (read product info, cmd 22)."""

from __future__ import annotations

from .base import Phase, CollectionContext
from ..transport import TimeoutError as RS485Timeout, FatalError


class IdentityPhase(Phase):
    number = 2

    def collect(self, ctx: CollectionContext) -> None:
        for uid in ctx.motors:
            ctx.check_cancel()
            ctx.mark_collecting(uid)
            info = None
            try:
                info = ctx.client(uid).get_product_info()
            except (RS485Timeout, FatalError, Exception):
                info = None
            if info:
                ctx.db.update_identity(uid, info["product_type"],
                                       info["hw_version"], info["scc"])
                ctx.store(uid, observation={
                    "product_type": info["product_type"],
                    "hw_version": info["hw_version"],
                    "scc": info["scc"],
                    "serial_number": info["serial_number"],
                    "responded": True,
                })
            else:
                ctx.store(uid, observation={"responded": False})


PHASE = IdentityPhase()
