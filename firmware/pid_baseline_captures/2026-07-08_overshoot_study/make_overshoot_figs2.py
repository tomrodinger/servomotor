#!/usr/bin/env python3
"""Figure 8 (v2): snap-back-from-rest overshoot study, 3 reps per config."""
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ov_step2")
OUTDIR = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-08_overshoot_study"
os.makedirs(OUTDIR, exist_ok=True)
NOISE = 150
BIG = 300

def load(tag):
    t, e = [], []
    t_stop = None
    for line in open(os.path.join(HERE, f"ov_{tag}.csv")):
        if line.startswith("# burst:"):
            t_stop = float(re.search(r"stop at t=([0-9.]+)", line).group(1))
        elif not line.startswith("#") and not line.startswith("t_s"):
            p = line.split(",")
            t.append(float(p[0])); e.append(int(p[1]))
    return np.array(t) - t_stop, np.array(e)

def metrics(t, e):
    step = e.max()                                   # the commanded step as seen by the loop
    post_m = (t > 0) & (t < 2.0)
    tp, ep = t[post_m], e[post_m]
    # landing overshoot: most-negative error within 0.6 s after the first zero crossing
    idx = np.where(ep <= 0)[0]
    overshoot = 0.0
    t_cross = None
    if len(idx):
        t_cross = tp[idx[0]]
        w = (tp >= t_cross) & (tp <= t_cross + 0.6)
        overshoot = max(0.0, -ep[w].min())
    # oscillation half-cycles above +/-BIG within 0.6 s of the crossing
    hc = 0
    if t_cross is not None:
        w = (tp >= t_cross) & (tp <= t_cross + 0.6)
        seg = ep[w]
        sign = np.sign(seg)
        prev = 0
        for c in list(np.where(np.diff(sign) != 0)[0]) + [len(seg) - 1]:
            if len(seg[prev:c + 1]) and np.abs(seg[prev:c + 1]).max() > BIG:
                hc += 1
            prev = c + 1
        hc = max(0, hc - 1)
    # settle: LAST time |e| exceeds BIG in the whole 2 s window (robust to delayed hunts)
    exc = np.where(np.abs(ep) > BIG)[0]
    settle = tp[exc[-1]] if len(exc) else 0.0
    # late-hunt RMS in [1.0, 2.0] s: sustained oscillation flag
    late = ep[(tp >= 1.0)]
    late_rms = float(np.sqrt(np.mean(late.astype(float) ** 2))) if len(late) else 0.0
    return step, overshoot, hc, settle, late_rms

CONFIGS = [
    ("kd87k",   "kD=87.5k"), ("kd175k", "kD=175k\n(old default)"), ("kd350k", "kD=350k"),
    ("kd700k",  "kD=700k"), ("kd1400k", "kD=1.4M"),
    ("kp1000",  "kP=1000\n(kD=350k)"), ("kp4000", "kP=4000\n(kD=350k)"),
    ("ki5",     "kI=5\n(kD=350k)"), ("ki100", "kI=100\n(kD=350k)"),
]
REPS = ["r1", "r2", "r3", "r4", "r5"]

data = {}    # tag -> list of (t, e, metrics)
for tag, _ in CONFIGS:
    runs = []
    for rep in REPS:
        t, e = load(f"{tag}_{rep}")
        runs.append((t, e, metrics(t, e)))
    data[tag] = runs

def median_rep(tag):   # rep whose overshoot is the median
    runs = sorted(data[tag], key=lambda r: r[2][1])
    return runs[len(runs) // 2]

fig = plt.figure(figsize=(14, 13))
gs = fig.add_gridspec(3, 2, top=0.90, bottom=0.14, hspace=0.45, wspace=0.22)
axA = fig.add_subplot(gs[0, :])
axB = fig.add_subplot(gs[1, 0])
axC = fig.add_subplot(gs[1, 1])
axD = fig.add_subplot(gs[2, 0])
axE = fig.add_subplot(gs[2, 1])
fig.suptitle("Figure 8 — Snap-back-from-rest overshoot study (firmware 0.15.3.3, 20 V, 5 repeats per configuration)\n"
             "Stimulus: the setpoint jumps 0.6 of a rotation in 40 ms — far faster than the rotor can follow — then holds still.\n"
             "The rotor snaps from rest toward a stationary target, approaching at full authority: the commanded twin of\n"
             "twisting the shaft ~216 degrees by hand and letting go. kP=2000, kI=25 unless stated.", fontsize=11)

# ---- A: anatomy (median kd350k rep)
t, e, (step, ov, hc, st, lr) = median_rep("kd350k")
axA.plot(t, e, lw=1.0, color="tab:blue")
axA.axvline(0, color="tab:red", ls="--", lw=1.0)
axA.set_xlim(-0.35, 1.2)
axA.set_ylabel("position error (counts)")
axA.set_xlabel("time relative to end of setpoint step (s)")
axA.set_title(f"(A) Anatomy of one snap-back (kD=350k): landing after a 1.97M-count snap, overshoot {ov:.0f} counts", fontsize=10)
axA.annotate("error rails at the 52,428-count clamp while the rotor\ncloses the 0.6-rotation gap at full drive authority\n(the debug value is the clamped error)",
             xy=(-0.01, step * 0.97), xytext=(-0.29, step * 0.75),
             arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
axA.annotate("the final approach enters the\nlinear region: P/D take over braking",
             xy=(t[np.argmin(np.abs(e - step / 2))], step / 2), xytext=(0.25, step * 0.62),
             arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
i_min = np.argmin(e[(t > 0)])
axA.annotate(f"the landing: overshoot {ov:.0f} counts\nthen settles into the noise band",
             xy=(t[t > 0][i_min], -ov), xytext=(0.45, step * 0.25),
             arrowprops=dict(arrowstyle="->", color="0.3"), fontsize=9)
axA.grid(alpha=0.3)

# ---- B: kD sweep (median reps)
for tag, lbl in CONFIGS[:5]:
    t, e, _ = median_rep(tag)
    axB.plot(t, e, lw=1.0, label=lbl.replace("\n", " "))
axB.set_xlim(-0.02, 0.8); axB.set_ylim(-3200, 3200)
axB.axhline(0, color="0.4", lw=0.7)
axB.axhspan(-NOISE, NOISE, color="tab:green", alpha=0.08)
axB.set_title("(B) The landing vs kD (zoom): more D = softer landing, until too much", fontsize=10)
axB.set_xlabel("time after step (s)"); axB.set_ylabel("error (counts)")
axB.legend(fontsize=7, loc="lower right"); axB.grid(alpha=0.3)

# ---- C: metrics, median with rep scatter
names, med_ov, med_st, med_lr = [], [], [], []
for tag, lbl in CONFIGS:
    ovs = sorted(r[2][1] for r in data[tag])
    sts = sorted(r[2][3] for r in data[tag])
    lrs = sorted(r[2][4] for r in data[tag])
    mid = len(ovs) // 2
    names.append(lbl); med_ov.append(ovs[mid]); med_st.append(sts[mid]); med_lr.append(lrs[mid])
x = np.arange(len(names))
colors = ["tab:red"] * 5 + ["tab:purple"] * 2 + ["tab:orange"] * 2
bars = axC.bar(x, med_ov, 0.62, color=colors, alpha=0.85)
for xi, (tag, _) in zip(x, CONFIGS):
    for r in data[tag]:
        axC.plot(xi, r[2][1], "k.", ms=4, alpha=0.6)
for b, ov, st, lr in zip(bars, med_ov, med_st, med_lr):
    note = f"{ov:.0f}\nsettle {st*1000:.0f} ms"
    if lr > 500: note += f"\nHUNTS ({lr:.0f} rms)"
    axC.text(b.get_x() + b.get_width() / 2, max(ov, 60) + 60, note, ha="center", fontsize=7)
axC.set_xticks(x, [n for n in names], fontsize=7)
axC.set_ylabel("overshoot (counts), median of 5")
axC.set_ylim(0, max(med_ov) * 1.5 + 150)
axC.set_title("(C) Overshoot (bars, median; dots = repeats) + settle time; HUNTS = still oscillating at 1-2 s", fontsize=10)
axC.grid(alpha=0.3, axis="y")

# ---- D: kP effect
for tag, lbl in [("kp1000", "kP=1000"), ("kd350k", "kP=2000"), ("kp4000", "kP=4000")]:
    t, e, _ = median_rep(tag)
    axD.plot(t, e, lw=1.0, label=lbl)
axD.set_xlim(-0.02, 0.8); axD.set_ylim(-3200, 3200)
axD.axhline(0, color="0.4", lw=0.7); axD.axhspan(-NOISE, NOISE, color="tab:green", alpha=0.08)
axD.set_title("(D) kP at fixed kD=350k: stiffness vs landing", fontsize=10)
axD.set_xlabel("time after step (s)"); axD.set_ylabel("error (counts)")
axD.legend(fontsize=8); axD.grid(alpha=0.3)

# ---- E: kI effect
for tag, lbl in [("ki5", "kI=5"), ("kd350k", "kI=25"), ("ki100", "kI=100")]:
    t, e, _ = median_rep(tag)
    axE.plot(t, e, lw=1.0, label=lbl)
axE.set_xlim(-0.02, 0.8); axE.set_ylim(-3200, 3200)
axE.axhline(0, color="0.4", lw=0.7); axE.axhspan(-NOISE, NOISE, color="tab:green", alpha=0.08)
axE.set_title("(E) kI at fixed kD=350k: minor effect on the landing (anti-windup at work)", fontsize=10)
axE.set_xlabel("time after step (s)"); axE.set_ylabel("error (counts)")
axE.legend(fontsize=8); axE.grid(alpha=0.3)

fig.text(0.02, 0.005,
         "How to read this: error = setpoint - rotor. Positive = rotor behind; a NEGATIVE excursion after the catch-up = the rotor sailed "
         "past the target (overshoot). Green band = +/-150 counts noise floor. Metrics: overshoot = deepest negative excursion within "
         "0.6 s of the first zero crossing; settle = last time |error| exceeded 300 counts in the 2 s window; HUNTS = RMS at 1-2 s after "
         "the step is still large (sustained oscillation, not a landing problem). Dots on the bars show all 5 repeats - run-to-run spread "
         "is the motor's stiction variability, so judge medians. This experiment is the commanded equivalent of twisting the shaft ~3.2 "
         "degrees by hand and releasing it.",
         fontsize=9, va="bottom", ha="left", wrap=True, color="0.25")
fig.savefig(os.path.join(OUTDIR, "fig8_overshoot_study.png"), dpi=140)
print("saved fig8")
print(f"{'config':>22} {'step':>7} {'overshoot(3 reps)':>22} {'settle ms':>10} {'late RMS':>9}")
for tag, lbl in CONFIGS:
    ms = [r[2] for r in data[tag]]
    ovs = "/".join(f"{m[1]:.0f}" for m in ms)
    print(f"{lbl.replace(chr(10),' '):>22} {ms[0][0]:>7.0f} {ovs:>28} {sorted(m[3] for m in ms)[2]*1000:>10.0f} {sorted(m[4] for m in ms)[2]:>9.0f}")
