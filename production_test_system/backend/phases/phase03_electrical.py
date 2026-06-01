"""Phase 3 — Electrical baseline (supply voltage, temperature, status)."""

from __future__ import annotations

from .base import Phase, CollectionContext
from .. import units
from ..transport import TimeoutError as RS485Timeout, FatalError


class ElectricalPhase(Phase):
    number = 3

    def collect(self, ctx: CollectionContext) -> None:
        for uid in ctx.motors:
            ctx.check_cancel()
            ctx.mark_collecting(uid)
            obs = {"responded": False}
            scalar = None
            try:
                client = ctx.client(uid)
                voltage = units.raw_voltage_to_volts(client.get_supply_voltage_raw())
                temperature = client.get_temperature()
                flags, fatal = client.get_status()
                scalar = voltage
                obs = {
                    "supply_voltage": voltage,
                    "temperature": temperature,
                    "status_flags": flags,
                    "fatal_code": fatal,
                    "responded": True,
                }
            except (RS485Timeout, FatalError, Exception):
                pass
            ctx.store(uid, scalar=scalar, observation=obs)


PHASE = ElectricalPhase()
