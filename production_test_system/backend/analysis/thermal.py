"""Thermal best-fit (Phase 11 post-processing).

Given a per-second temperature series, derive the linear best fit:
``temperature = slope * t + start_temp`` plus the correlation coefficient R,
and the overall temperature rise (final minus the fresh baseline).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence


def linear_fit(times: Sequence[float], temps: Sequence[float]) -> Dict[str, float]:
    """Least-squares fit; returns slope, intercept (start temp at t=0) and R."""
    n = len(times)
    if n < 2:
        return {"slope": 0.0, "start_temp": temps[0] if temps else 0.0, "r": 0.0}
    mean_t = sum(times) / n
    mean_y = sum(temps) / n
    sxx = sum((t - mean_t) ** 2 for t in times)
    syy = sum((y - mean_y) ** 2 for y in temps)
    sxy = sum((times[i] - mean_t) * (temps[i] - mean_y) for i in range(n))
    slope = sxy / sxx if sxx else 0.0
    start_temp = mean_y - slope * mean_t
    if sxx == 0 or syy == 0:
        r = 0.0
    else:
        r = sxy / math.sqrt(sxx * syy)
    return {"slope": slope, "start_temp": start_temp, "r": r}


def analyze(baseline: Optional[float], series: List[List[float]]) -> Dict[str, Any]:
    """Analyse a thermal run.

    ``series`` is a list of ``[t_seconds, temperature]`` samples; ``baseline`` is
    the fresh baseline temperature read before the motor ran.
    """
    if not series:
        return {"temp_rise": None, "fit_slope": None,
                "fit_start_temp": None, "fit_r": None}
    times = [s[0] for s in series]
    temps = [s[1] for s in series]
    fit = linear_fit(times, temps)
    base = baseline if baseline is not None else temps[0]
    return {
        "temp_rise": temps[-1] - base,
        "fit_slope": fit["slope"],
        "fit_start_temp": fit["start_temp"],
        "fit_r": fit["r"],
    }
