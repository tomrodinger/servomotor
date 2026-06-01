"""Phase 14 — Set factory-default alias.

Broadcast set-alias to 'X' (cmd 21 to the broadcast address) so every device is
set in one shot, wait for the auto-reboot, then read the alias back from each
device.  Once every device is 'X' the alias is no longer a unique address, so
the read-back is matched by **unique ID**: a detection pass (cmd 20) reports
each device's unique ID together with its current alias, which we key by unique
ID.  (Get-product-info, cmd 22, does not return the alias, so detection is used
for the read-back.)
"""

from __future__ import annotations

from .base import Phase, CollectionContext
from .. import detection


class SetAliasPhase(Phase):
    number = 14

    def collect(self, ctx: CollectionContext) -> None:
        factory_alias = str(ctx.params.get("factory_alias", "X"))
        reboot_delay = float(ctx.params.get("reboot_delay_s", 1.0))
        alias_value = ord(factory_alias[0]) if factory_alias else 255

        for uid in ctx.motors:
            ctx.mark_collecting(uid)

        # Broadcast the set-alias; the non-volatile write makes every device
        # auto-reboot, so wait before talking to the bus again.
        try:
            ctx.transport.transact(255, "Set device alias", [alias_value])
        except Exception as exc:
            ctx.log("Phase 14: broadcast set-alias error: %s" % exc)
        ctx.sleep(reboot_delay)
        ctx.transport.flush_input()

        # Read the alias back per unique ID via a robust (collision-tolerant)
        # detection pass — once every device is 'X' the alias is no longer a
        # unique address, so detection (which reports unique ID + alias) is the
        # reliable way to verify it.
        alias_by_uid = {}
        try:
            for entry in detection.run_detect_pass(ctx.transport, reset_before=True):
                alias_by_uid[int(entry[0])] = int(entry[1])
        except Exception as exc:
            ctx.log("Phase 14: alias read-back error: %s" % exc)

        for uid in ctx.motors:
            raw = alias_by_uid.get(uid)
            if raw is None:
                ctx.store(uid, observation={"alias": None, "missing": True})
            else:
                char = chr(raw) if 33 <= raw <= 126 else None
                ctx.store(uid, observation={"alias": char, "alias_raw": raw})


PHASE = SetAliasPhase()
