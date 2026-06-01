"""Stage B — PNG generation (disposable plots regenerable from the raw data).

`Generate PNGs` renders the plots from the stored ``phase_data`` blobs into the
plots folder.  PNGs are NOT stored in the database — they are pure derived
artifacts; deleting the folder loses nothing.  Each filename embeds the device
unique ID and the plot type, so the Tab 2 viewer can group them by type:
``<plottype>_<UNIQUEID>.png``.

Matplotlib runs headless (Agg backend) so this works on a server with no display.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from . import config, blobs, units
from .analysis import hall, thermal
from .database import Database, uid_hex
from .settings import Settings

# Plot-type prefixes, in the display order Tab 2 groups them.
PLOT_TYPES = ["spin_tracking", "hall_linearity", "hall_waveform",
              "temperature", "pid_deviation"]


def _path(plot_type: str, unique_id: int) -> str:
    return os.path.join(config.PLOTS_DIR, "%s_%s.png" % (plot_type, uid_hex(unique_id)))


def _position_plot(db: Database, uid: int, phase: int, plot_type: str,
                   title: str, labels) -> Optional[str]:
    row = db.latest_phase_data(uid, phase)
    if not row or not row.get("raw_blob"):
        return None
    samples = blobs.unpack_position_stream(row["raw_blob"])
    if not samples:
        return None
    series: Dict[int, list] = {0: [], 1: []}
    for tag, commanded, hall_pos in samples:
        series.setdefault(tag, []).append(
            (units.counts_to_rotations(commanded), units.counts_to_rotations(hall_pos)))
    fig, ax = plt.subplots(figsize=(7, 5))
    for tag, pts in series.items():
        if not pts:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, ".", markersize=2, label=labels.get(tag, "trace %d" % tag))
    ax.set_xlabel("commanded position (rotations)")
    ax.set_ylabel("hall position (rotations)")
    ax.set_title("%s — %s" % (title, uid_hex(uid)))
    ax.legend()
    ax.grid(True, alpha=0.3)
    path = _path(plot_type, uid)
    fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)
    return path


def plot_spin_tracking(db, settings, uid) -> Optional[str]:
    return _position_plot(db, uid, 6, "spin_tracking", "Phase 6 spin tracking",
                          {0: "forward", 1: "reverse"})


def plot_hall_linearity(db, settings, uid) -> Optional[str]:
    return _position_plot(db, uid, 7, "hall_linearity", "Phase 7 hall linearity",
                          {0: "CW", 1: "CCW"})


def plot_hall_waveform(db, settings, uid) -> Optional[str]:
    row = db.latest_phase_data(uid, 8)
    if not row or not row.get("raw_blob"):
        return None
    threshold = int(settings.phase_criteria(8).get("peak_find_hysteresis", 8000))
    channels = hall.parse_capture(row["raw_blob"])
    extrema = {ch: hall.find_extrema(channels[ch], threshold) for ch in range(3)}
    fig, axes = plt.subplots(4, 1, figsize=(9, 11), sharex=True)
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for ch in range(3):
        ax = axes[ch]
        ax.plot(channels[ch], color=colors[ch], linewidth=0.6)
        for kind, value, idx in extrema[ch]:
            ax.plot(idx, value, "x", color=("red" if kind == "peak" else "black"),
                    markersize=5)
        ax.set_ylabel("ch %d" % (ch + 1))
        ax.grid(True, alpha=0.3)
    for ch in range(3):
        axes[3].plot(channels[ch], color=colors[ch], linewidth=0.6,
                     label="ch %d" % (ch + 1))
    axes[3].set_ylabel("all")
    axes[3].set_xlabel("sample")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)
    axes[0].set_title("Phase 8 hall waveform — %s (x = detected peaks/valleys)"
                      % uid_hex(uid))
    path = _path("hall_waveform", uid)
    fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)
    return path


def plot_temperature(db, settings, uid) -> Optional[str]:
    row = db.latest_phase_data(uid, 11)
    if not row or not row.get("raw_blob"):
        return None
    series = [list(s) for s in blobs.unpack_temperature_series(row["raw_blob"])]
    if not series:
        return None
    baseline = (row.get("observation") or {}).get("baseline")
    times = [s[0] for s in series]
    temps = [s[1] for s in series]
    fit = thermal.linear_fit(times, temps)
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(times, temps, ".", markersize=3, label="measured")
    if times:
        xs = [times[0], times[-1]]
        ys = [fit["slope"] * x + fit["start_temp"] for x in xs]
        ax.plot(xs, ys, "-", color="red",
                label="fit: %.3f C/s, R=%.3f" % (fit["slope"], fit["r"]))
    if baseline is not None:
        ax.axhline(baseline, color="gray", linestyle="--", alpha=0.6,
                   label="baseline %.0f C" % baseline)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("temperature (C)")
    ax.set_title("Phase 11 thermal — %s" % uid_hex(uid))
    ax.legend(); ax.grid(True, alpha=0.3)
    path = _path("temperature", uid)
    fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)
    return path


def plot_pid_deviation(db, settings, uid) -> Optional[str]:
    row = db.latest_phase_data(uid, 13)
    if not row or not row.get("raw_blob"):
        return None
    series = blobs.unpack_pid_series(row["raw_blob"])
    if not series:
        return None
    per_move = [max(abs(mn), abs(mx)) for mn, mx in series]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(per_move, ".", markersize=3)
    ax.set_xlabel("move number")
    ax.set_ylabel("max PID deviation")
    ax.set_title("Phase 13 closed-loop PID deviation — %s" % uid_hex(uid))
    ax.grid(True, alpha=0.3)
    path = _path("pid_deviation", uid)
    fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)
    return path


_GENERATORS = [plot_spin_tracking, plot_hall_linearity, plot_hall_waveform,
               plot_temperature, plot_pid_deviation]


def generate_for_motor(db: Database, settings: Settings, uid: int) -> List[str]:
    config.ensure_data_dirs()
    out = []
    for gen in _GENERATORS:
        try:
            p = gen(db, settings, uid)
            if p:
                out.append(p)
        except Exception:
            pass
    return out


def generate_all(db: Database, settings: Settings,
                 motor_ids: Optional[List[int]] = None) -> Dict[str, int]:
    if motor_ids is None:
        motor_ids = db.all_motor_ids()
    total = 0
    for uid in motor_ids:
        total += len(generate_for_motor(db, settings, uid))
    return {"motors": len(motor_ids), "pngs": total}


def list_pngs_by_type(motor_ids: List[int]) -> Dict[str, List[Dict[str, str]]]:
    """Group existing PNGs by plot type for the Tab 2 viewer (filtered set)."""
    grouped: Dict[str, List[Dict[str, str]]] = {pt: [] for pt in PLOT_TYPES}
    wanted = {uid_hex(u) for u in motor_ids}
    for pt in PLOT_TYPES:
        for uid in motor_ids:
            fname = "%s_%s.png" % (pt, uid_hex(uid))
            if os.path.exists(os.path.join(config.PLOTS_DIR, fname)):
                grouped[pt].append({"unique_id": uid_hex(uid), "filename": fname})
    return grouped
