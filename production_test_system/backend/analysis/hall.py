"""Hall-waveform peak/valley analysis (Phase 8 post-processing).

The peak/valley finder mirrors the firmware's ``handle_calibration_logic``
(``firmware/Src/motor_control.c``): a running local max/min that confirms an
extremum once the signal reverses by more than a hysteresis threshold.  The
firmware uses ``HALL_PEAK_FIND_THRESHOLD = 2000`` on raw readings; because the
captured data is scaled by ``n_samples_to_sum / division_factor`` (default
64/16 = 4x), the equivalent threshold here defaults to 8000.

All of this runs in Stage B from the stored raw blob — never during collection.
"""

from __future__ import annotations

import struct
from typing import Any, Dict, List, Tuple

N_POLES = 50          # one extremum-producing magnet per pole; firmware N_POLES
SELECT_COUNT = 50     # take the middle 50 extrema (25 peaks + 25 valleys)


def parse_capture(raw: bytes, n_channels: int = 3) -> List[List[int]]:
    """Decode a raw capture blob into per-channel sample lists.

    The capture is interleaved u16 little-endian samples: ch0, ch1, ch2, ch0...
    """
    stride = 2 * n_channels
    n = len(raw) // stride
    channels: List[List[int]] = [[] for _ in range(n_channels)]
    for i in range(n):
        base = i * stride
        for ch in range(n_channels):
            channels[ch].append(
                struct.unpack_from("<H", raw, base + 2 * ch)[0])
    return channels


def find_extrema(samples: List[int], threshold: int) -> List[Tuple[str, int, int]]:
    """Return confirmed extrema as ``(kind, value, index)``.

    ``kind`` is ``"peak"`` or ``"valley"``; ``value`` is the extremum amplitude;
    ``index`` is the sample index at which the running extreme occurred (matching
    the firmware, which records the position of the local max/min, not where it
    is later confirmed).
    """
    extrema: List[Tuple[str, int, int]] = []
    rising = True
    local_max = 0
    local_max_pos = 0
    local_min = 65535
    local_min_pos = 0
    for i, reading in enumerate(samples):
        if rising:
            if reading > local_max:
                local_max = reading
                local_max_pos = i
            if local_max - reading > threshold:
                extrema.append(("peak", local_max, local_max_pos))
                local_min = reading
                local_min_pos = i
                rising = False
        else:
            if reading < local_min:
                local_min = reading
                local_min_pos = i
            if reading - local_min > threshold:
                extrema.append(("valley", local_min, local_min_pos))
                local_max = reading
                local_max_pos = i
                rising = True
    return extrema


def _select_middle(extrema: List[Tuple[str, int, int]],
                   count: int = SELECT_COUNT) -> List[Tuple[str, int, int]]:
    """Skip the first extremum, then take the middle ``count`` of the rest."""
    rest = extrema[1:]
    if len(rest) <= count:
        return rest
    start = (len(rest) - count) // 2
    return rest[start:start + count]


def analyze_channel(samples: List[int], threshold: int) -> Dict[str, Any]:
    """Derive Phase 8 metrics for one channel."""
    extrema = find_extrema(samples, threshold)
    selected = _select_middle(extrema)
    peaks = [v for (k, v, _) in selected if k == "peak"]
    valleys = [v for (k, v, _) in selected if k == "valley"]
    positions = [p for (_, _, p) in selected]

    metrics: Dict[str, Any] = {
        "extrema_count": len(extrema),
        "selected_count": len(selected),
    }
    if selected:
        all_vals = [v for (_, v, _) in selected]
        metrics["hall_min"] = min(all_vals)
        metrics["hall_max"] = max(all_vals)
    if peaks:
        avg_peak = sum(peaks) / len(peaks)
        metrics["avg_peak"] = avg_peak
        metrics["max_peak_from_avg"] = max(abs(p - avg_peak) for p in peaks)
        metrics["max_adjacent_peak_dev"] = (
            max(abs(peaks[i] - peaks[i - 1]) for i in range(1, len(peaks)))
            if len(peaks) > 1 else 0)
    if valleys:
        avg_valley = sum(valleys) / len(valleys)
        metrics["avg_valley"] = avg_valley
        metrics["max_valley_from_avg"] = max(abs(v - avg_valley) for v in valleys)
        metrics["max_adjacent_valley_dev"] = (
            max(abs(valleys[i] - valleys[i - 1]) for i in range(1, len(valleys)))
            if len(valleys) > 1 else 0)
    if peaks and valleys:
        metrics["span"] = metrics["avg_peak"] - metrics["avg_valley"]
    if len(positions) > 1:
        spacings = [positions[i] - positions[i - 1] for i in range(1, len(positions))]
        metrics["peak_spacing_min"] = min(spacings)
        metrics["peak_spacing_max"] = max(spacings)
    return metrics


def analyze_capture(raw: bytes, threshold: int,
                    n_channels: int = 3) -> Dict[str, Any]:
    """Full Phase 8 analysis: per-channel metrics + extrema for plotting."""
    channels = parse_capture(raw, n_channels)
    result: Dict[str, Any] = {"channels": {}, "extrema": {}}
    for ch, samples in enumerate(channels):
        result["channels"][ch] = analyze_channel(samples, threshold)
        result["extrema"][ch] = find_extrema(samples, threshold)
    return result
