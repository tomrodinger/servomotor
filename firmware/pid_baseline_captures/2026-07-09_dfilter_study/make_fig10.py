#!/usr/bin/env python3
"""Figure 10: the D-term filter — what it is, the measured noise, and the shift sweep."""
import os
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-09_dfilter_study"
os.makedirs(OUTDIR, exist_ok=True)

sim = json.load(open(os.path.join(HERE, "dfilter_sim_sweep.json")))
bench = {}
for sh in (3, 4, 5, 6, 7):
    bench[sh] = json.load(open(os.path.join(HERE, "dfilter_bench", f"bench_shift{sh}.json")))

fig = plt.figure(figsize=(14, 12.5))
gs = fig.add_gridspec(2, 2, top=0.89, bottom=0.16, hspace=0.38, wspace=0.25)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
axC = fig.add_subplot(gs[1, 0])
axD = fig.add_subplot(gs[1, 1])
fig.suptitle("Figure 10 — The D-term noise filter: measured sensor noise, and the filter-strength sweep\n"
             "The filter is a leaky integrator (EMA) over 2^SHIFT ticks; current firmware uses SHIFT=5 (τ≈1.0 ms, cutoff ≈155 Hz).\n"
             "kD is auto-rescaled by 2^-SHIFT so the effective derivative gain stays constant across shifts.", fontsize=12)

# ---- A: the measured noise (whiteness)
v1 = np.load(os.path.join(HERE, "hall_noise_v1.npy"))
fs = 31250 / 16
x = v1 - v1.mean()
f = np.fft.rfftfreq(len(x), 1 / fs)
psd = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
# bin-average for a readable curve
nb = 40
fb, pb = [], []
edges = np.logspace(np.log10(2), np.log10(fs / 2), nb)
for lo, hi in zip(edges[:-1], edges[1:]):
    m_ = (f >= lo) & (f < hi)
    if m_.sum():
        fb.append(np.sqrt(lo * hi)); pb.append(psd[m_].mean())
axA.loglog(fb, pb, "o-", ms=3, color="tab:blue")
axA.axhline(np.mean(pb), color="tab:red", ls="--", lw=1)
axA.set_xlabel("frequency (Hz)")
axA.set_ylabel("noise power (rel.)")
axA.set_title("(A) Measured hall-position noise spectrum at standstill\n(rotor magnetically locked, per-tick readings)", fontsize=10)
axA.text(3, np.mean(pb) * 1.5, "flat = WHITE noise, σ = 38.5 counts/tick\n(sum-of-16 and sum-of-64 scaling confirm:\nratios 1.00 and 0.99 vs white prediction)",
         fontsize=9, color="tab:red")
axA.grid(alpha=0.3, which="both")

# ---- B: analytic/sim noise gain + phase lag vs shift
shifts = [0, 2, 3, 4, 5, 6, 7]
d_noise = [sim[str(s)]["ss_d"] for s in shifts]
axB.semilogy(shifts, d_noise, "o-", color="tab:green", label="D-term noise std (sim, pre-shift)")
bshifts = [3, 4, 5, 6, 7]
axB.semilogy(bshifts, [bench[s]["ss_d_std"] for s in bshifts], "s--", color="tab:olive", label="D-term noise std (BENCH)")
ax2 = axB.twinx()
lag_ms = [0.032 * (1 << s) for s in shifts]
ax2.plot(shifts, lag_ms, "^-", color="tab:purple", label="filter time constant (ms)")
ax2.set_ylabel("time constant (ms)", color="tab:purple")
axB.set_xlabel("filter SHIFT (EMA over 2^SHIFT ticks)")
axB.set_ylabel("D-term noise std (pre-shift counts)")
axB.set_title("(B) The tradeoff: each +1 shift halves D noise (sim & bench agree)\nbut doubles the phase lag", fontsize=10)
axB.legend(loc="upper right", fontsize=8)
axB.grid(alpha=0.3)
axB.annotate("current\n(SHIFT=5)", xy=(5, sim["5"]["ss_d"]), xytext=(5.4, sim["5"]["ss_d"] * 4),
             arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)

# ---- C: what happens to CONTROL quality (bench, medians)
w = 0.35
x_ = np.arange(len(bshifts))
mv = [bench[s]["move_rms"] for s in bshifts]
sn = [bench[s]["snap_ov"] for s in bshifts]
b1 = axC.bar(x_ - w / 2, mv, w, color="tab:blue", alpha=0.85, label="move settle RMS")
b2 = axC.bar(x_ + w / 2, [v / 100 for v in sn], w, color="tab:red", alpha=0.85, label="snap overshoot / 100")
axC.set_yscale("log")
for b, v in zip(b1, mv):
    axC.text(b.get_x() + w / 2, max(v, 30) * 1.15, f"{v:.0f}", ha="center", fontsize=8)
for b, v in zip(b2, sn):
    axC.text(b.get_x() + w / 2, max(v / 100, 30) * 1.15, f"{v:.0f}", ha="center", fontsize=8, color="tab:red")
axC.set_xticks(x_, [f"SHIFT={s}\nτ={0.032*(1<<s):.2f} ms" for s in bshifts], fontsize=8)
axC.set_title("(C) BENCH control quality vs shift (kP=2000, kI=25, kD=350k)\nmove settle is flat 3..6, then the τ≈4 ms lag destabilizes SHIFT=7", fontsize=10)
axC.legend(fontsize=8)
axC.grid(alpha=0.3, axis="y")
axC.annotate("HUNTS:\n27,911", xy=(4 - w / 2, 20000), fontsize=9, color="tab:blue", ha="center")

# ---- D: the no-filter case + summary verdict
labels = ["SHIFT=0\n(no filter)\nSIM", "SHIFT=3", "SHIFT=4", "SHIFT=5\n(current)", "SHIFT=6", "SHIFT=7"]
move_vals = [sim["0"]["move"], bench[3]["move_rms"], bench[4]["move_rms"], bench[5]["move_rms"], bench[6]["move_rms"], bench[7]["move_rms"]]
colors = ["0.5", "tab:orange", "tab:green", "tab:green", "tab:olive", "tab:red"]
bars = axD.bar(range(6), move_vals, 0.6, color=colors, alpha=0.85)
axD.set_yscale("log")
for b, v in zip(bars, move_vals):
    axD.text(b.get_x() + 0.3, max(v, 30) * 1.2, f"{v:.0f}", ha="center", fontsize=9)
axD.set_xticks(range(6), labels, fontsize=8)
axD.set_ylabel("move settle RMS (counts, log)")
axD.set_title("(D) Verdict: the filter is REQUIRED (no-filter can't settle),\nSHIFT 4-6 all work, 5 sits at the optimum plateau, 7 is over the cliff", fontsize=10)
axD.grid(alpha=0.3, axis="y")

fig.text(0.02, 0.005,
         "Background: this filter is not unusual — a low-pass on the derivative term is standard industrial-PID practice (the 'derivative "
         "filter', typically τD/8..τD/20); the 'arbitrary' choice turned out to be the textbook structure. What was there before: "
         "lpf = (lpf*31)>>5 + Δerror, whose arithmetic shift never let negative values decay (a permanent ≈−85k D bias, fixed in 0.15.3.1 by "
         "rounding the decay toward zero — the topology was NOT changed). (A): the measured sensor noise is white, so no filter SHAPE (notch, "
         "higher-order) can beat a plain low-pass — only the time constant matters; a 2nd-order filter would buy steeper rolloff at the cost "
         "of MORE lag, and (C) shows lag is the binding constraint. (B)+(C): each +1 shift halves the D noise (bench matches theory within "
         "measurement error) but doubles the lag; control quality is flat from SHIFT 3-6 because the loop absorbs the remaining noise, then "
         "collapses at SHIFT=7 (τ≈4.1 ms ≈ the loop's own dynamics — the D term brakes 4 ms late and sustains the stick-slip hunt: "
         "settle RMS 27,911). Snap-overshoot medians (red) vary within the motor's stiction spread; no shift beats 5 outside that spread. "
         "RECOMMENDATION: keep SHIFT=5. It sits mid-plateau with 2 steps of margin to the cliff, and the remaining D noise (±200 output "
         "counts) measurably does not degrade error, settle, or landing.",
         fontsize=9, va="bottom", ha="left", wrap=True, color="0.25")
fig.savefig(os.path.join(OUTDIR, "fig10_dfilter_study.png"), dpi=140)
print("saved fig10")
