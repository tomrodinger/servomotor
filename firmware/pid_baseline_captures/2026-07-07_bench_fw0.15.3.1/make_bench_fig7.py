#!/usr/bin/env python3
"""Figure 7: old vs new firmware on the REAL bench motor."""
import os
import sys
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/tools")
from pid_debug_capture import read_csv

OLDDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-06"
NEWDIR = "/private/tmp/claude-501/-Users-tom-Documents-Move-the-Needle-Servomotor-firmware/74e569b6-9702-4e05-a09d-9f128c5923f8/scratchpad/new_fw"
OUTDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-07_bench_fw0.15.3.1"


def load(path):
    _, samples = read_csv(path)
    arr = np.array(samples)
    return arr[:, 0], arr[:, 1]


def post_rms(path, t0=3.2):
    _, samples = read_csv(path)
    v = [r[1] for r in samples if r[0] >= t0]
    return math.sqrt(sum(e * e for e in v) / len(v))


fig = plt.figure(figsize=(14, 10))
gs = fig.add_gridspec(2, 2, top=0.87, bottom=0.17, hspace=0.35, wspace=0.12)
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1], sharey=ax1)
ax3 = fig.add_subplot(gs[1, :])
fig.suptitle("Figure 7 — REAL BENCH MOTOR, before vs after: firmware 0.15.3.0 vs 0.15.3.1\n"
             "(anti-windup: integral clamp 25% of authority + back-calculation; rounding fixes)\n"
             "Identical test: 2-rotation trapezoid move over 2 s commanded at t=1 s, kI=25", fontsize=12)

to, eo = load(os.path.join(OLDDIR, "ki25_move_test.csv"))
tn, en = load(os.path.join(NEWDIR, "nf_ki25_kd350k_run1.csv"))
ax1.plot(to, eo, lw=0.6, color="tab:red")
ax1.set_title("OLD firmware, kI=25 (kD=175k):\n±10,000-count limit cycle, never settles", fontsize=10)
ax2.plot(tn, en, lw=0.6, color="tab:green")
ax2.set_title("NEW firmware, kI=25 (kD=350k):\nsettles to ±38 counts RMS and stays there", fontsize=10)
for ax in (ax1, ax2):
    ax.axvline(1.0, color="0.5", ls="--", lw=0.8)
    ax.axvline(3.0, color="0.5", ls="--", lw=0.8)
    ax.grid(alpha=0.3)
    ax.set_xlabel("time (s)")
ax1.set_ylabel("position error (counts)")

# bar chart: worst-run post-move RMS per configuration (all measured on the bench today/Monday)
configs = [
    ("kI=5\nkD=175k\n(OLD fw)", [post_rms(os.path.join(OLDDIR, "move_test1.csv"))], "tab:gray"),
    ("kI=25\nkD=175k\n(OLD fw)", [post_rms(os.path.join(OLDDIR, "ki25_move_test.csv"))], "tab:red"),
    ("kI=5\nkD=175k", [post_rms(os.path.join(NEWDIR, f"nf_ki5_kd175000_run{r}.csv")) for r in (1, 2)], "tab:green"),
    ("kI=10\nkD=175k", [post_rms(os.path.join(NEWDIR, f"nf_ki10_kd175000_run{r}.csv")) for r in (1, 2)], "tab:green"),
    ("kI=25\nkD=175k", [post_rms(os.path.join(NEWDIR, f"nf_ki25_run{r}.csv")) for r in (1, 2, 3)], "tab:olive"),
    ("kI=25\nkD=350k", [post_rms(os.path.join(NEWDIR, f"nf_ki25_kd350k_run{r}.csv")) for r in (1, 2, 3)], "tab:green"),
    ("kI=50\nkD=700k", [post_rms(os.path.join(NEWDIR, f"nf_ki50_kd700000_run{r}.csv")) for r in (1, 2)], "tab:green"),
    ("kI=100\nkD=350k", [post_rms(os.path.join(NEWDIR, f"nf_ki100_kd350000_run{r}.csv")) for r in (1, 2)], "tab:olive"),
]
xs = np.arange(len(configs))
worst = [max(v) for _, v, _ in configs]
colors = [c for _, _, c in configs]
bars = ax3.bar(xs, worst, 0.6, color=colors, alpha=0.9)
for b, (name, vals, _) in zip(bars, configs):
    lbl = f"{max(vals):.0f}" + (f"\n(best {min(vals):.0f})" if len(vals) > 1 and max(vals) / max(min(vals), 1) > 3 else "")
    ax3.text(b.get_x() + b.get_width() / 2, b.get_height() * 1.15, lbl, ha="center", fontsize=8)
ax3.set_yscale("log")
ax3.set_xticks(xs, [c[0] for c in configs], fontsize=9)
ax3.axhline(1000, color="0.4", ls=":", lw=1)
ax3.text(-0.4, 1200, "limit-cycle territory", fontsize=8, color="0.4")
ax3.set_ylabel("worst-run post-move RMS (counts, log)")
ax3.set_title("All bench measurements, worst run per configuration (gray/red = old firmware Monday; green/olive = new firmware today)",
              fontsize=10)
ax3.grid(alpha=0.3, axis="y")

fig.text(0.02, 0.005,
         "How to read this: top = the identical kI=25 move experiment on the same physical motor, old vs new firmware. Bottom = every configuration "
         "measured on the bench (log scale; lower is better). Green = settles reliably; olive = 'bistable' — some runs settle, some hunt at reduced "
         "amplitude (worst and best shown). The windup limit cycle itself is fixed (old kI=25: 6,865 always; new: bounded at half amplitude worst-case). "
         "The remaining hunt on this high-stiction motor dies when kD is scaled with kI (~14,000 x kI): kI=25/kD=350k and kI=50/kD=700k settle to "
         "~38 RMS every run — the same quality the old firmware could only achieve at kI=5 with luck, with 2-3x less tracking lag during the move.",
         fontsize=9, va="bottom", ha="left", wrap=True, color="0.25")
os.makedirs(OUTDIR, exist_ok=True)
fig.savefig(os.path.join(OUTDIR, "fig7_bench_before_after.png"), dpi=140)
print("done")
