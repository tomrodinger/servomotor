"""Phase 1 — Firmware (MCU flash + RS485).

Broadcast-flash all motors to the target release, then read the firmware
version back from each device individually.  Stage A stores the version read
back and whether the device responded; pass/fail (version == target and
responded) is decided in Stage B.

The broadcast flash itself requires the RS485 bootloader protocol against real
hardware; in simulation it is skipped.  The testable part — reading the version
back from every device — runs in both cases.
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from ..transport import TimeoutError as RS485Timeout, FatalError


class FirmwarePhase(Phase):
    number = 1

    def collect(self, ctx: CollectionContext) -> None:
        target = ctx.params.get("target_firmware_version", "0.15.0.0")
        flash_enabled = ctx.params.get("flash_enabled", True)
        if ctx.transport.is_simulated or not flash_enabled:
            ctx.log("Phase 1: %s — reading versions only (no flash)." %
                    ("simulated bus" if ctx.transport.is_simulated else "flash disabled"))
        else:
            ctx.log("Phase 1: broadcast-flashing firmware %s to all motors." % target)
            try:
                from .. import firmware_flash
                firmware_flash.flash_all_broadcast(ctx.transport, target, log=ctx.log)
            except Exception as exc:   # never abort collection on a flash hiccup
                ctx.log("Phase 1: broadcast flash error (continuing): %s" % exc)

        for uid in ctx.motors:
            ctx.check_cancel()
            ctx.mark_collecting(uid)
            version = None
            responded = False
            try:
                version = ctx.client(uid).get_firmware_version()
                responded = True
                ctx.set_firmware_version(uid, version)
            except (RS485Timeout, FatalError, Exception):
                responded = False
            ctx.store(uid, observation={"firmware_version": version,
                                        "responded": responded})


PHASE = FirmwarePhase()
