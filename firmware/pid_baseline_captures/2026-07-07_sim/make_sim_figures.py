#!/usr/bin/env python3
"""Figures 5-6: simulator validation against the bench + effect of the anti-windup fix."""
import os
import sys
import subprocess
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/tools")
from pid_debug_capture import read_csv

BENCHDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-06"
OUTDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-07_sim"
HERE = os.path.dirname(os.path.abspath(__file__))


def load(path):
    _, samples = read_csv(path)
    arr = np.array(samples)
    return arr[:, 0], arr[:, 1]   # t, error


def caption(fig, text):
    fig.text(0.02, 0.005, text, fontsize=9, va="bottom", ha="left", wrap=True, color="0.25")


# ------------------------------------------------------------------ figure 5: sim credibility
def fig5():
    tb, eb = load(os.path.join(BENCHDIR, "ki25_move_test.csv"))
    ts, es = load(os.path.join(HERE, "sim_ki25_old_final.csv"))
    tb5, eb5 = load(os.path.join(BENCHDIR, "move_test1.csv"))
    ts5, es5 = load(os.path.join(HERE, "sim_ki5_old_final.csv"))

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharey="row")
    fig.subplots_adjust(top=0.86, bottom=0.15, hspace=0.35, wspace=0.08)
    fig.suptitle("Figure 5 — Is the simulator believable? Same experiments: real motor (left) vs simulation (right)\n"
                 "Plant: stepper torque curve + stick-slip friction + sensor noise, calibrated to the bench motor.\n"
                 "Controller: the firmware's exact integer math (verified bit-identical over 4.5M samples).", fontsize=12)

    for ax, (t, e, title) in zip(axes.flat, [
            (tb5, eb5, "BENCH: kI=5 (default) — settles after move"),
            (ts5, es5, "SIM: kI=5 (default) — settles after move"),
            (tb, eb, "BENCH: kI=25 — limit cycle, never settles"),
            (ts, es, "SIM: kI=25 — limit cycle, never settles")]):
        ax.plot(t, e, lw=0.6, color="tab:blue" if "kI=5" in title else "tab:red")
        ax.set_title(title, fontsize=10)
        ax.axvline(1.0, color="0.5", ls="--", lw=0.8)
        ax.axvline(3.0, color="0.5", ls="--", lw=0.8)
        ax.grid(alpha=0.3)
        ax.set_xlabel("time (s)")
    axes[0][0].set_ylabel("position error (counts)")
    axes[1][0].set_ylabel("position error (counts)")

    caption(fig, "How to read this: top row = the stable default-gain move; bottom row = the kI=25 windup limit cycle. Dashed lines = move start/end. "
                 "What matches: breakaway spike height (~10k), standstill noise floor (~33 counts), D-term bias (-85k) and std (~180k), "
                 "kI=5 settling, kI=25 sustained ±9-10k oscillation that never decays, cycle frequency within 2x (22 vs 35 Hz). "
                 "Honest differences: the sim's cycle starts a moment after the move (noise-triggered) and comes in bursts, and the sim tracks "
                 "more quietly DURING the move — the binary stick-slip model is coarser than real presliding friction. The bench remains the "
                 "final arbiter; the sim's job is comparing algorithms on equal terms.")
    fig.savefig(os.path.join(OUTDIR, "fig5_sim_vs_bench_validation.png"), dpi=140)
    plt.close(fig)


# ------------------------------------------------------------------ figure 6: the fix
def fig6():
    to, eo = load(os.path.join(HERE, "sim_ki25_old_final.csv"))
    tn, en = load(os.path.join(HERE, "sim_ki25_step2_final.csv"))

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, top=0.86, bottom=0.16, hspace=0.35, wspace=0.25)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1], sharey=ax1)
    ax3 = fig.add_subplot(gs[1, :])
    fig.suptitle("Figure 6 — The fix, in simulation: anti-windup (integral clamp 25% of authority + back-calculation)\n"
                 "+ rounding fixes. Identical plant, identical move (2 rot / 2 s), kI=25, seed 4.", fontsize=12)

    ax1.plot(to, eo, lw=0.6, color="tab:red")
    ax1.set_title("CURRENT firmware (0.15.3.0): limit cycle forever", fontsize=10)
    ax2.plot(tn, en, lw=0.6, color="tab:green")
    ax2.set_title("PROPOSED (0.15.3.1): settles and stays settled", fontsize=10)
    for ax in (ax1, ax2):
        ax.axvline(1.0, color="0.5", ls="--", lw=0.8)
        ax.axvline(3.0, color="0.5", ls="--", lw=0.8)
        ax.grid(alpha=0.3)
        ax.set_xlabel("time (s)")
    ax1.set_ylabel("position error (counts)")

    # kI sweep summary: WORST-CASE post-move RMS over 10 seeds (the system is bistable at
    # low kI, so the max is the decision-relevant statistic: "can it get stuck oscillating?")
    kis = [5, 10, 25, 50, 100, 200]
    med = {"old": [], "step2": []}
    for alg in ("old", "step2"):
        for ki in kis:
            vals = []
            for seed in range(1, 11):
                out = subprocess.run([os.path.join(HERE, "pid_sim"), "--alg", alg, "--move", "2",
                                      "--duration", "8", "--ki", str(ki), "--seed", str(seed), "--quiet"],
                                     capture_output=True, text=True).stdout
                vals.append(float([l for l in out.splitlines() if l.startswith("post_move_rms")][0].split("=")[1]))
            med[alg].append(max(vals))
    x = np.arange(len(kis))
    w = 0.35
    b1 = ax3.bar(x - w/2, med["old"], w, color="tab:red", alpha=0.85, label="current firmware")
    b2 = ax3.bar(x + w/2, med["step2"], w, color="tab:green", alpha=0.85, label="proposed fix")
    ax3.set_yscale("log")
    ax3.set_xticks(x, [str(k) for k in kis])
    ax3.set_xlabel("integral constant kI")
    ax3.set_ylabel("post-move error RMS (counts, log)")
    ax3.set_title("WORST-CASE post-move residual error vs kI (10 random-noise seeds) — >1000 counts means a sustained limit cycle", fontsize=10)
    ax3.axhline(1000, color="0.4", ls=":", lw=1)
    ax3.text(0.05, 1150, "limit-cycle threshold", fontsize=8, color="0.4")
    for bars in (b1, b2):
        for b in bars:
            ax3.text(b.get_x() + b.get_width()/2, b.get_height()*1.1, f"{b.get_height():.0f}",
                     ha="center", fontsize=8)
    ax3.legend()
    ax3.grid(alpha=0.3, axis="y")

    caption(fig, "How to read this: top = the identical kI=25 experiment on the simulated bench motor with current vs proposed firmware. "
                 "Bottom = worst case over 10 noise seeds: with current firmware, EVERY kI (even the shipped default 5) can get stuck in a "
                 "limit cycle on this high-stiction motor — it is luck of the disturbance; with the fix, no seed limit-cycles for kI 5..100 "
                 "(worst case a few hundred counts of residual wander), and even kI=200 improves >10x. kI around 25-50 becomes the practical "
                 "optimum and settles TIGHTER than today's default.")
    fig.savefig(os.path.join(OUTDIR, "fig6_fix_effect.png"), dpi=140)
    plt.close(fig)


fig5()
fig6()
print("done")
