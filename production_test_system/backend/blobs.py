"""Binary (de)serialization for the large raw arrays stored in ``phase_data``.

Big datasets — Phase 6/7 (commanded, hall) sample streams, Phase 11 temperature
series, Phase 13 per-move PID series — are stored as compact binary blobs
rather than text/JSON to keep the database small.  Each blob is self-describing:
a 4-byte magic, a 1-byte format tag and a 4-byte record count, then fixed-size
records.
"""

from __future__ import annotations

import struct
from typing import List, Tuple

_MAGIC = b"PTS1"

FMT_POSITION = 1     # (tag:i8, commanded:i64, hall:i64)
FMT_TEMP = 2         # (t:f32, temp:f32)
FMT_PID = 3          # (min:i32, max:i32)

_RECORD = {
    FMT_POSITION: "<bqq",
    FMT_TEMP: "<ff",
    FMT_PID: "<ii",
}


def _pack(fmt_tag: int, records: List[tuple]) -> bytes:
    rec_fmt = _RECORD[fmt_tag]
    out = bytearray()
    out += _MAGIC
    out += struct.pack("<BI", fmt_tag, len(records))
    size = struct.calcsize(rec_fmt)
    for rec in records:
        out += struct.pack(rec_fmt, *rec)
    assert len(out) == len(_MAGIC) + 5 + size * len(records)
    return bytes(out)


def _unpack(blob: bytes) -> Tuple[int, List[tuple]]:
    if blob[:4] != _MAGIC:
        raise ValueError("bad blob magic")
    fmt_tag, count = struct.unpack_from("<BI", blob, 4)
    rec_fmt = _RECORD[fmt_tag]
    size = struct.calcsize(rec_fmt)
    base = 9
    records = [struct.unpack_from(rec_fmt, blob, base + i * size) for i in range(count)]
    return fmt_tag, records


# -- position streams (Phase 6 / 7) -------------------------------------------
def pack_position_stream(samples: List[Tuple[int, int, int]]) -> bytes:
    """samples: list of (tag, commanded_counts, hall_counts)."""
    return _pack(FMT_POSITION, samples)


def unpack_position_stream(blob: bytes) -> List[Tuple[int, int, int]]:
    _, recs = _unpack(blob)
    return recs


# -- temperature series (Phase 11) --------------------------------------------
def pack_temperature_series(samples: List[Tuple[float, float]]) -> bytes:
    """samples: list of (t_seconds, temperature_celsius)."""
    return _pack(FMT_TEMP, samples)


def unpack_temperature_series(blob: bytes) -> List[Tuple[float, float]]:
    _, recs = _unpack(blob)
    return recs


# -- PID series (Phase 13) ----------------------------------------------------
def pack_pid_series(samples: List[Tuple[int, int]]) -> bytes:
    """samples: list of (min_pid_error, max_pid_error) per move."""
    return _pack(FMT_PID, samples)


def unpack_pid_series(blob: bytes) -> List[Tuple[int, int]]:
    _, recs = _unpack(blob)
    return recs
