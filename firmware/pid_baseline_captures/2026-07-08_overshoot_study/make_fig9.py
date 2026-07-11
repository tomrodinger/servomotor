#!/usr/bin/env python3
"""Figure 9: why 'no overshoot' was unachievable — the approach-charged integral."""
import os
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-08_overshoot_study"
J = json.load(open(os.path.join(HERE, "overshoot_analysis.json")))

fig = plt.figure(figsize=(14, 12.5))
gs = fig.add_gridspec(2, 2, top=0.90, bottom=0.15, hspace=0.35, wspace=0.25)
axA = fig.add_subplot(gs[0, :])
axB = fig.add_subplot(gs[1, 0])
axC = fig.add_subplot(gs[1, 1])
fig.suptitle("Figure 9 — Why the snap-back cannot land without overshoot: the integral charges during the approach\n"
             "and keeps PUSHING through the landing. (Simulation, firmware-exact PID; plant calibrated to the bench motor.)",
             fontsize=12)

# ---- A: the smoking gun (tick-level dissection, frictionless so nothing hides the mechanism)
rows = []
for line in open(os.path.join(HERE, "e5c_dissect.csv")):
    if line.startswith("#") or line.startswith("t_s"):
        continue
    p = line.strip().split(",")
    rows.append([float(p[0])] + [int(x) for x in p[1:]])
rows = np.array(rows)
i0 = np.argmax(rows[:, 1] > 5000)          # step application
t = (rows[:, 0] - rows[i0, 0]) * 1000.0    # ms
m = (t >= -0.5) & (t <= 12)
err = rows[:, 1]
p_c = rows[:, 2] / 2048.0
i_c = rows[:, 3] / 2048.0
d_c = rows[:, 4] / 2048.0
out = rows[:, 5]

axA.plot(t[m], err[m], color="tab:blue", lw=1.6, label="position error (counts)")
axA.plot(t[m], p_c[m], color="tab:cyan", lw=1.1, label="P contribution (output counts)")
axA.plot(t[m], i_c[m], color="tab:orange", lw=1.6, label="I contribution")
axA.plot(t[m], d_c[m], color="tab:green", lw=1.1, label="D contribution")
axA.plot(t[m], out[m], color="tab:red", lw=1.1, ls="--", label="total output")
axA.axhline(0, color="0.4", lw=0.7)
axA.set_xlabel("time after the setpoint step (ms)")
axA.set_ylabel("counts / output counts")
axA.set_title("(A) One landing, dissected (10k-count step, friction & noise removed so only the controller acts)", fontsize=10)
axA.legend(fontsize=8, loc="upper right")
axA.grid(alpha=0.3)
axA.annotate("while the rotor approaches (error > 0 the whole way),\nthe integral charges up to ~+9,000 output counts",
             xy=(3.0, 8600), xytext=(4.6, 12000),
             arrowprops=dict(arrowstyle="->", color="tab:orange"), fontsize=9, color="tab:orange")
axA.annotate("D brakes hard (negative)...", xy=(2.3, -13800), xytext=(0.3, -16000),
             arrowprops=dict(arrowstyle="->", color="tab:green"), fontsize=9, color="tab:green")
axA.annotate("...but I cancels it: the TOTAL output turns positive\nagain while the rotor is still short of the target —\nthe controller pushes when it should brake",
             xy=(4.6, 1200), xytext=(6.2, 4600),
             arrowprops=dict(arrowstyle="->", color="tab:red"), fontsize=9, color="tab:red")
axA.annotate("error crosses zero at speed\nand sails past = the overshoot floor",
             xy=(5.6, -500), xytext=(8.6, -6000),
             arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)

# ---- B: the floor is flat vs step size and kD cannot remove it
steps = [5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000]
for kd, c in ((175000, "tab:orange"), (350000, "tab:green"), (700000, "tab:red")):
    ov = [J["e1"][f"{kd}_{s}"]["overshoot"] for s in steps]
    axB.plot(steps, ov, "o-", color=c, ms=4, label=f"kD={kd:,}")
axB.set_xscale("log")
axB.set_ylim(0, 9000)
axB.set_xlabel("setpoint step size (counts, log scale)")
axB.set_ylabel("overshoot (counts, median of 3)")
axB.set_title("(B) The floor is ~constant from 5k to 2M counts —\nclassical damping theory would predict overshoot ∝ size", fontsize=10)
axB.legend(fontsize=8)
axB.grid(alpha=0.3)
axB.annotate("same ~2-3k floor everywhere:\nthe integral rails at its clamp on any\napproach longer than a few ms, and the\nlanding must 'pay off' that stored push",
             xy=(100000, 2600), xytext=(9000, 5600),
             arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=8)

# ---- C: cures
labels = ["snap 0.61 rot\n(baseline,\nkD=350k)", "same distance as\nTRAPEZOID move\n(0.15 s)",
          "snap + integral\nseparation T=5k\n(bench plant)", "snap + int. sep.\n(healthy plant,\n6/6 seeds)"]
vals = [J["e2"]["full plant|350000"]["overshoot"], J["e3"]["trap_0.15"], 0, 0]
colors = ["tab:red", "tab:blue", "tab:green", "tab:green"]
bars = axC.bar(range(4), vals, 0.6, color=colors, alpha=0.85)
for b, v in zip(bars, vals):
    axC.text(b.get_x() + 0.3, max(v, 40) + 60, f"{v:.0f}", ha="center", fontsize=10)
axC.set_xticks(range(4), labels, fontsize=8)
axC.set_ylabel("overshoot (counts)")
axC.set_title("(C) What eliminates it: don't let I charge during the transient\n(profile the setpoint, or freeze integration while |error| is large)", fontsize=10)
axC.grid(alpha=0.3, axis="y")

fig.text(0.02, 0.005,
         "The answer to 'why can't we reach the no-overshoot (overdamped) regime': the loop IS heavily damped in the classical sense "
         "(at kD=350k the P-D pair alone is equivalent to damping ratio ~2.4). The overshoot floor comes from the INTEGRAL term: every "
         "approach charges it (error keeps one sign the whole way), and at the landing it acts as a stored forward push that cancels the "
         "D term's braking — panel A shows the total output turning positive again while the rotor is still short of the target. Raising kD "
         "makes the approach slower, giving I MORE time to charge, which is why more damping paradoxically did not help (and why kI=5 "
         "overshot more than kI=100 on the bench: slower unwind). The anti-windup clamp (25% authority) bounds the stored push — hence the "
         "flat ~2-3k floor — but does not remove it. Panel C: the floor vanishes if the integral cannot charge during the transient: a "
         "profiled (trapezoid) move keeps the error small the whole way (4x better), and integral separation (freeze integration while "
         "|error| > ~5,000 counts) achieves ZERO overshoot on both the bench-calibrated and healthy plants with tracking unaffected. "
         "CAVEAT before adopting integral separation in firmware: on a heavily loaded axis it can starve the integrator during breakaway "
         "(3 of 6 loaded-case seeds got worse) — it needs a load-aware design pass + the full test pipeline first.",
         fontsize=9, va="bottom", ha="left", wrap=True, color="0.25")

fig.savefig(os.path.join(OUTDIR, "fig9_overshoot_root_cause.png"), dpi=140)
print("saved fig9")
