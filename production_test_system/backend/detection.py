"""Device detection and test-set building (Tab 1).

Detection (``detect_devices``, cmd 20) is used *only* to build the per-bus test
set.  Each detect pass quiets and resets the bus (mirroring the library's
``detect_devices_iteratively``), then issues one detect and unions the results
into the bus's test set.  The new-vs-known colour is decided from two facts:
whether the ID is already in *this bus's* test set, and whether it is already in
the database — the main purpose being to surface a non-unique "unique" ID.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Tuple

from . import config
from .database import Database
from .state import BusState, COLOR_GREEN, COLOR_ORANGE
from .transport import Transport, CommunicationError

# Detection responses arrive after a random 0-1 s delay, so give the read a
# generous window to collect them all.
DETECT_TIMEOUT = 1.5
# Detection is probabilistic: with ~48 devices answering at once a single read
# usually collides (so it returns only a partial subset) and rarely captures
# everyone.  So a pass keeps reading and unioning partial results until the set
# stops growing for several reads in a row (converged), bounded by a max — this
# mirrors the library's detect_devices_iteratively.  The operator can still
# queue more passes for stubborn buses via the per-bus pending counter.
DETECT_STABLE_READS = 3
DETECT_MAX_ATTEMPTS = 15


def run_detect_pass(transport: Transport, reset_before: bool = True) -> List[List[Any]]:
    """Run one detection pass, returning a list of ``[unique_id, alias]``.

    Unions partial reads across attempts until the discovered set stops growing
    for ``DETECT_STABLE_READS`` consecutive reads (or ``DETECT_MAX_ATTEMPTS`` is
    reached).
    """
    union = {}
    stable = 0
    for _ in range(DETECT_MAX_ATTEMPTS):
        if reset_before:
            transport.transact(255, "System reset")
            time.sleep(config.BOOTLOADER_EXIT_DELAY_S)
        transport.flush_input()
        before = len(union)
        try:
            results = transport.transact(255, "Detect devices", timeout=DETECT_TIMEOUT)
        except CommunicationError:
            results = []
        transport.flush_input()           # drain any late/colliding responses
        for entry in results:
            union[int(entry[0])] = int(entry[1]) if len(entry) > 1 else 255
        stable = stable + 1 if len(union) == before else 0
        if union and stable >= DETECT_STABLE_READS:
            break
    return [[u, a] for u, a in union.items()]


def detect_aliases(transport: Transport, expected_ids, max_passes: int = 6):
    """Read back (unique_id -> alias) for a known set of motors.

    Unions detection passes until every ``expected_ids`` motor has been seen (or
    ``max_passes`` is reached).  Used by Phase 14 to verify the factory alias:
    detection is probabilistic, so a single pass can miss a straggler and
    falsely report it as absent — looping until the known set is complete avoids
    that.  Genuinely-absent motors are simply not in the returned dict.
    """
    expected = set(int(u) for u in expected_ids)
    found = {}
    for _ in range(max_passes):
        for entry in run_detect_pass(transport, reset_before=True):
            found[int(entry[0])] = int(entry[1]) if len(entry) > 1 else 255
        if expected <= set(found):
            break
    return found


def classify_and_add(db: Database, bus: BusState,
                     results: List[List[Any]]) -> List[Tuple[int, str]]:
    """Union ``results`` into ``bus``'s test set, colouring each motor.

    Mutates ``bus`` (the caller should hold the AppState lock).  Returns the
    list of ``(unique_id, color)`` decided this pass.
    """
    decided: List[Tuple[int, str]] = []
    for entry in results:
        unique_id = int(entry[0])
        alias = int(entry[1]) if len(entry) > 1 else 255
        in_set_before = unique_id in bus.test_set
        is_new = db.record_detection(unique_id)   # writes first_detected once
        in_db_before = not is_new
        if in_set_before:
            color = COLOR_GREEN
        elif not in_db_before:
            color = COLOR_GREEN
        else:
            color = COLOR_ORANGE
        bus.add_to_set(unique_id, alias, color)
        decided.append((unique_id, color))
    return decided
