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
# Detection is probabilistic: a single read may miss devices (random response
# delays) or collide (CRC/framing error on the half-duplex bus).  So one
# operator "pass" unions several collision-free reads (mirroring the library's
# detect_devices_iteratively(n_detections=3)); collided reads are retried and
# don't count toward the target.
DETECT_CLEAN_READS = 3
DETECT_MAX_ATTEMPTS = 10


def run_detect_pass(transport: Transport, reset_before: bool = True) -> List[List[Any]]:
    """Run one detection pass, returning a list of ``[unique_id, alias]``.

    Unions ``DETECT_CLEAN_READS`` collision-free reads (retrying collisions up
    to ``DETECT_MAX_ATTEMPTS``).  The operator can queue more passes for very
    crowded buses via the per-bus pending counter.
    """
    union = {}
    clean = 0
    for _ in range(DETECT_MAX_ATTEMPTS):
        if clean >= DETECT_CLEAN_READS:
            break
        if reset_before:
            transport.transact(255, "System reset")
            time.sleep(config.BOOTLOADER_EXIT_DELAY_S)
            transport.flush_input()
        try:
            results = transport.transact(255, "Detect devices", timeout=DETECT_TIMEOUT)
        except CommunicationError:
            transport.flush_input()
            continue                       # collision: retry, doesn't count
        clean += 1
        for entry in results:
            union[int(entry[0])] = int(entry[1]) if len(entry) > 1 else 255
    return [[u, a] for u, a in union.items()]


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
