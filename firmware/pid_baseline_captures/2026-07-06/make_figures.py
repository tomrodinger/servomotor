#!/usr/bin/env python3
"""Generate annotated sanity-check figures from the 2026-07-06 PID baseline captures."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CAPDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-06"
OUTDIR = CAPDIR

SHIFT = 11                    # PID_SHIFT_RIGHT on M17
RAIL = 200 << (11 + 8)        # max_integral_term at default max PWM voltage 200 = 104,857,600
FULL_AUTHORITY = RAIL >> SHIFT  # 51,200 output counts = full PWM voltage


import sys
sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/tools")
from pid_debug_capture import read_csv


def load(name):
    _, samples = read_csv(os.path.join(CAPDIR, name))
    arr = np.array(samples)
    return {"t_s": arr[:, 0], "error": arr[:, 1], "p_term": arr[:, 2],
            "i_term": arr[:, 3], "d_term": arr[:, 4], "output": arr[:, 5]}


def caption(fig, text):
    fig.text(0.02, 0.005, text, fontsize=9, va="bottom", ha="left", wrap=True,
             family="sans-serif", color="0.25")


# ---------------------------------------------------------------- figure 1
def fig1_standstill():
    d = load("standstill_test2.csv")
    t, err, p, i, dd = d["t_s"], d["error"], d["p_term"], d["i_term"], d["d_term"]

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.subplots_adjust(top=0.90, bottom=0.16, hspace=0.25)
    fig.suptitle("Figure 1 — Standstill, default gains (kP=2000, kI=5, kD=175000)\n"
                 "Motor enters closed loop at t=0 and just holds position for 5 s. No move is commanded.",
                 fontsize=13)

    axes[0].plot(t, err, lw=0.7, color="tab:blue")
    axes[0].set_ylabel("position error (counts)")
    axes[0].annotate("closed-loop entry transient\n(rotor snaps to commutation angle)",
                     xy=(0.03, 2000), xytext=(0.55, 1500),
                     arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
    ss = t > 1.0
    axes[0].annotate(f"steady state: error std = {err[ss].std():.0f} counts "
                     f"(1 count = 1/3,276,800 rev)",
                     xy=(3.0, 300), fontsize=9, color="0.3")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, i / 1e6, lw=0.9, color="tab:orange", label="I term")
    axes[1].plot(t, p / 1e6, lw=0.7, color="tab:blue", alpha=0.7, label="P term")
    axes[1].set_ylabel("I and P terms (millions, pre-shift)")
    i_hold = i[ss].mean()
    axes[1].axhline(i_hold / 1e6, color="tab:orange", ls=":", alpha=0.6)
    axes[1].annotate(f"I ramps up and holds ≈ {i_hold/1e6:.1f} M\n"
                     f"= {i_hold/RAIL*100:.0f}% of the integral clamp (104.9 M)\n"
                     f"= the static holding effort — this is I doing its job",
                     xy=(1.5, i_hold / 1e6), xytext=(2.2, 5.0),
                     arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
    axes[1].legend(loc="center right")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, dd / 1e3, lw=0.7, color="tab:green")
    axes[2].set_ylabel("D term (thousands, pre-shift)")
    axes[2].set_xlabel("time (s)")
    d_mean = dd[ss].mean()
    axes[2].axhline(0, color="0.4", lw=0.8)
    axes[2].axhline(d_mean / 1e3, color="tab:red", ls="--", lw=1.2)
    axes[2].annotate(f"the motor is NOT moving, yet the D term shows a ±{dd[ss].std()/1e3:.0f}k noise band\n"
                     f"AND a DC offset of {d_mean/1e3:.0f}k (red dashed) — a real derivative of a\n"
                     f"stationary position must average to zero. Both effects are defects (see Figure 4).",
                     xy=(2.5, d_mean / 1e3), xytext=(0.8, -2900),
                     arrowprops=dict(arrowstyle="->", color="tab:red"), fontsize=9,
                     bbox=dict(facecolor="white", edgecolor="0.7", alpha=0.85))
    axes[2].grid(alpha=0.3)

    caption(fig,
            "How to read this: the PID runs every 32 µs; we sample its internals over RS485 at ~240 Hz. "
            "'Pre-shift' terms are summed and divided by 2^11 = 2048 to form the output, so 2048 pre-shift units = 1 output count. "
            "Healthy: error settles near zero, I term settles at the constant holding effort. "
            "Suspicious: the D term's noise band (±100 counts on the output after the shift) and its negative DC bias at standstill.")
    fig.savefig(os.path.join(OUTDIR, "fig1_standstill_annotated.png"), dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------- figure 2
def fig2_move_defaults():
    d = load("move_test1.csv")
    t, err, i = d["t_s"], d["error"], d["i_term"]
    T_MOVE, T_END = 1.006, 3.006

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.subplots_adjust(top=0.88, bottom=0.18, hspace=0.2)
    fig.suptitle("Figure 2 — Move with DEFAULT gains (kI=5): stable, this is the healthy reference\n"
                 "Trapezoid move: 2 rotations over 2 s, commanded at t≈1.0 s",
                 fontsize=13)

    for ax in axes:
        ax.axvspan(T_MOVE, T_END, color="tab:gray", alpha=0.12)
        ax.axvline(T_MOVE, color="0.4", ls="--", lw=1)
        ax.axvline(T_END, color="0.4", ls="--", lw=1)
        ax.grid(alpha=0.3)

    axes[0].plot(t, err, lw=0.7, color="tab:blue")
    axes[0].set_ylabel("position error (counts)")
    axes[0].annotate("acceleration kick:\npeak lag ≈ 10,600 counts\n(0.3% of a rotation)",
                     xy=(1.05, 10300), xytext=(1.9, 8500),
                     arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
    axes[0].annotate("deceleration/stop kick", xy=(2.98, -4600), xytext=(3.6, -3800),
                     arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
    axes[0].annotate("settles cleanly — no oscillation\nafter the move. THIS is the\nbehavior we must preserve.",
                     xy=(4.5, 500), fontsize=10, color="tab:green")
    axes[0].text(1.95, -4200, "shaded = motor moving", fontsize=9, color="0.4")

    axes[1].plot(t, i / 1e6, lw=0.9, color="tab:orange")
    axes[1].axhline(RAIL / 1e6, color="tab:red", ls="--", lw=1.2)
    axes[1].axhline(-RAIL / 1e6, color="tab:red", ls="--", lw=1.2)
    axes[1].text(0.15, RAIL / 1e6 - 12, "integral clamp (±104.9 M) — the I term is ALLOWED to command\n"
                 "100% of full motor output on its own (this is the windup headroom)",
                 fontsize=9, color="tab:red")
    axes[1].set_ylim(-RAIL / 1e6 * 1.15, RAIL / 1e6 * 1.15)
    i_peak = i.max()
    axes[1].annotate(f"peak {i_peak/1e6:.0f} M = {i_peak/RAIL*100:.0f}% of clamp\n"
                     f"with kI=5 — plenty of margin,\nso defaults do not limit-cycle",
                     xy=(t[i.argmax()], i_peak / 1e6), xytext=(4.2, 55),
                     arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
    axes[1].set_ylabel("I term (millions, pre-shift)")
    axes[1].set_xlabel("time (s)")

    caption(fig,
            "How to read this: with the shipped default kI=5, a gentle 1 rev/s move produces a big transient error while accelerating "
            "(the loop has no feedforward, so P and I must generate all of the tracking effort), the I term climbs to only ~17% of its clamp, "
            "and everything settles. Compare with Figure 3, where the ONLY change is kI=25.")
    fig.savefig(os.path.join(OUTDIR, "fig2_move_defaults_annotated.png"), dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------- figure 3
def fig3_windup():
    d5 = load("move_test1.csv")
    d25 = load("ki25_move_test.csv")
    T_MOVE, T_END = 1.006, 3.006

    fig = plt.figure(figsize=(13, 11))
    gs = fig.add_gridspec(3, 1, top=0.90, bottom=0.14, hspace=0.33)
    ax0 = fig.add_subplot(gs[0])
    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    ax2 = fig.add_subplot(gs[2])
    fig.suptitle("Figure 3 — Same move, kI raised 5 → 25: integrator windup limit cycle\n"
                 "(this is the reported \"I above ~10 just oscillates\" bug, reproduced on the bench)",
                 fontsize=13)

    ax0.plot(d25["t_s"], d25["error"], lw=0.6, color="tab:red", label="kI = 25")
    ax0.plot(d5["t_s"], d5["error"], lw=0.8, color="tab:blue", label="kI = 5 (default)")
    ax0.axvline(T_MOVE, color="0.4", ls="--", lw=1)
    ax0.axvline(T_END, color="0.4", ls="--", lw=1)
    ax0.legend(loc="lower right")
    ax0.set_ylabel("position error (counts)")
    ax0.annotate("move ends at t≈3.0 — but with kI=25 the ±10,000-count\noscillation continues at full amplitude, forever",
                 xy=(3.05, 9000), xytext=(4.0, 13000),
                 arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=10)
    ax0.set_ylim(-16000, 18000)
    ax0.grid(alpha=0.3)

    ax1.plot(d25["t_s"], d25["i_term"] / 1e6, lw=0.6, color="tab:orange")
    ax1.axhline(RAIL / 1e6, color="tab:red", ls="--", lw=1.2)
    ax1.axhline(-RAIL / 1e6, color="tab:red", ls="--", lw=1.2)
    ax1.text(0.1, RAIL / 1e6 + 6, "integral clamp ±104.9 M", fontsize=9, color="tab:red")
    ax1.set_ylabel("I term, kI=25 (millions)")
    ax1.set_ylim(-RAIL / 1e6 * 1.25, RAIL / 1e6 * 1.25)
    ax1.annotate("I term swings ±40 M (±40% of clamp) every cycle.\n"
                 "At kI=25 the accumulator charges to FULL motor authority in 2.6 ms of clamped error,\n"
                 "then must discharge through an equally large error of opposite sign → self-sustaining cycle.\n"
                 "There is no anti-windup in the firmware — this is the root cause.",
                 xy=(4.6, -38), xytext=(1.6, -100), fontsize=10,
                 arrowprops=dict(arrowstyle="->", color="0.3"),
                 bbox=dict(facecolor="white", edgecolor="0.7", alpha=0.85))
    ax1.grid(alpha=0.3)

    m = (d25["t_s"] >= 3.0) & (d25["t_s"] <= 3.5)
    tz, ez, iz = d25["t_s"][m], d25["error"][m], d25["i_term"][m]
    # measure the cycle frequency from mean-crossings of the error
    sign_change = np.where(np.diff(np.signbit(ez - ez.mean())))[0]
    freq = len(sign_change) / 2.0 / (tz[-1] - tz[0])
    ax2.plot(tz, ez, lw=1.2, color="tab:red", label="error (left axis)")
    ax2.set_ylabel("error (counts)", color="tab:red")
    ax2b = ax2.twinx()
    ax2b.plot(tz, iz / 1e6, lw=1.2, color="tab:orange", label="I term (right axis)")
    ax2b.set_ylabel("I term (millions)", color="tab:orange")
    ax2.set_xlabel("time (s)  —  zoom on 3.0–3.5 s, AFTER the move has ended")
    ax2.set_title(f"zoom: measured limit-cycle frequency ≈ {freq:.0f} Hz; "
                  "the I term lags the error (integrator phase lag) — the signature of an integrator-driven cycle",
                  fontsize=10)
    ax2.grid(alpha=0.3)

    caption(fig,
            "How to read this: top = position error for the identical 2-rotation move with the only difference being kI (blue 5, red 25). "
            "Middle = the integral accumulator at kI=25 against its clamp. Bottom = post-move zoom; the error and the I term oscillate at "
            f"≈{freq:.0f} Hz with I lagging, i.e. the integrator stores a huge 'debt' each half-cycle and forces the overshoot that sustains the cycle. "
            "Fix planned: back-calculation anti-windup + a much smaller integral clamp (I only needs to cover static holding effort, "
            "which Figure 1 measured at ~8% of the current clamp).")
    fig.savefig(os.path.join(OUTDIR, "fig3_windup_kI25_annotated.png"), dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------- figure 4
def fig4_dterm():
    dk = load("ki0_test.csv")     # kI=0, kD=175000 -> D visible, no I drift
    d0 = load("kd0_test.csv")     # kI=5, kD=0      -> control: no D at all
    ss_k = dk["t_s"] > 0.5
    ss_0 = d0["t_s"] > 0.5

    fig = plt.figure(figsize=(13, 10))
    gs = fig.add_gridspec(2, 2, top=0.88, bottom=0.17, hspace=0.3, wspace=0.25)
    axA = fig.add_subplot(gs[0, :])
    axB = fig.add_subplot(gs[1, 0])
    axC = fig.add_subplot(gs[1, 1])
    fig.suptitle("Figure 4 — Why the D term \"seems to not do much\": it is mostly noise, plus a rounding bug\n"
                 "(capture with kI=0, kD=175000: motor at STANDSTILL, so the true derivative is ≈ 0)",
                 fontsize=13)

    t, dd = dk["t_s"][ss_k], dk["d_term"][ss_k]
    d_mean = dd.mean()
    axA.plot(t, dd / 1e3, lw=0.7, color="tab:green")
    axA.axhline(0, color="0.3", lw=1)
    axA.axhline(d_mean / 1e3, color="tab:red", ls="--", lw=1.4)
    axA.axhspan(-31 * 5468 / 1e3, 0, color="tab:red", alpha=0.10)
    axA.set_ylabel("D term (thousands, pre-shift)")
    axA.set_xlabel("time (s)")
    axA.annotate(f"mean = {d_mean/1e3:.0f}k (red dashed): a stationary motor has a persistent NEGATIVE D output.\n"
                 "Cause (verified in code): the filter decay (lpf*31)>>5 rounds toward −∞, so any negative\n"
                 "filter value from −1 to −31 NEVER decays to zero, while positive values do. The shaded band\n"
                 "is where that sticky bias can sit (−31×5468 ≈ −170k … 0); the measured mean falls mid-band.",
                 xy=(2.0, d_mean / 1e3), xytext=(0.55, -640),
                 arrowprops=dict(arrowstyle="->", color="tab:red"), fontsize=9,
                 bbox=dict(facecolor="white", edgecolor="0.7", alpha=0.85))
    axA.set_title("(A) D term vs time at standstill — should be white noise around ZERO, is biased negative", fontsize=10)
    axA.grid(alpha=0.3)

    axB.hist(dd / 1e3, bins=60, color="tab:green", alpha=0.8)
    axB.axvline(0, color="0.3", lw=1.2)
    axB.axvline(d_mean / 1e3, color="tab:red", ls="--", lw=1.4)
    axB.set_xlabel("D term (thousands, pre-shift)")
    axB.set_ylabel("samples")
    axB.set_title("(B) Same data as histogram —\nthe whole distribution is shifted left of 0", fontsize=10)
    axB.grid(alpha=0.3)

    # noise contribution of each term at the output, and the payoff comparison
    contrib = {
        "P term\n(kD=175000 run)": dk["p_term"][ss_k].std() / 2048,
        "D term\n(kD=175000 run)": dk["d_term"][ss_k].std() / 2048,
    }
    errs = {
        "error std\nWITH D\n(kD=175000)": dk["error"][ss_k].std(),
        "error std\nWITHOUT D\n(kD=0)": d0["error"][ss_0].std(),
    }
    xs = np.arange(4)
    vals = list(contrib.values()) + list(errs.values())
    colors = ["tab:blue", "tab:green", "tab:purple", "tab:gray"]
    bars = axC.bar(xs, vals, color=colors, alpha=0.85)
    axC.set_xticks(xs, list(contrib.keys()) + list(errs.keys()), fontsize=8)
    for b, v in zip(bars, vals):
        axC.text(b.get_x() + b.get_width() / 2, v + 1, f"{v:.0f}", ha="center", fontsize=10)
    axC.set_ylabel("counts (std dev)")
    axC.set_title("(C) The deal we get from D today:\nleft pair = noise injected into the output;  right pair = position-error payoff",
                  fontsize=10)
    axC.grid(alpha=0.3, axis="y")

    caption(fig,
            "How to read this: at standstill the true velocity is zero, so an ideal D term would be zero-mean noise. "
            "(A)+(B): the measured D term instead sits at a negative DC value — a pure integer-rounding artifact that adds a constant "
            "torque bias (~40 output counts). (C): D injects ~3x more noise into the motor output than the P term does at the same "
            "operating point (left two bars), yet the position error is IDENTICAL with D fully on vs completely off (right two bars) — "
            "i.e. the D term currently buys nothing at standstill. Cause: it differentiates the noisy hall-sensor position over a 32 µs "
            "baseline. Fix planned: derive damping from the already-computed 640 µs velocity estimate minus the trajectory's known "
            "desired velocity, which has ~20x better signal-to-noise and no added filter lag.")
    fig.savefig(os.path.join(OUTDIR, "fig4_dterm_problems_annotated.png"), dpi=140)
    plt.close(fig)


fig1_standstill()
fig2_move_defaults()
fig3_windup()
fig4_dterm()
print("done")
