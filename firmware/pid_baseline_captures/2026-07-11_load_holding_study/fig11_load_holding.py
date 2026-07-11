#!/usr/bin/env python3
"""Fig 11: M17 load-holding / torque-at-small-error study (2026-07-11).
Data sources: 4-agent sim analysis (workflow_results_load_holding.json) using the
firmware-exact pid_algorithm_sim plant (tau_max 0.40 N*m @ V=200, static friction 0.13,
kp=2000 ki=25 kd=350000 unless noted). Never overwrites earlier figures."""
import csv, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = "/Users/tom/Documents/Move_the_Needle/Servomotor/firmware/pid_baseline_captures/2026-07-11_load_holding_study/fig11_load_holding_annotated.png"
SCR = "/private/tmp/claude-501/-Users-tom-Documents-Move-the-Needle-Servomotor-firmware/74e569b6-9702-4e05-a09d-9f128c5923f8/scratchpad"
CPR = 3276800.0  # counts per revolution
TAU_MAX = 0.40   # N*m at V=200

def sustained_torque(e_counts, i_rail_pre):
    """Sustained (steady-state) torque at settled error e with integral railed."""
    out = (2000 * e_counts + i_rail_pre + 1024) // 2048
    lead = min(out, 16384)
    v = min(int(out) >> 8, 200)
    return TAU_MAX * (v / 200.0) * math.sin(2 * math.pi * lead / 65536.0)

fig = plt.figure(figsize=(16, 13))
gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 0.85], hspace=0.34, wspace=0.22,
                      left=0.06, right=0.97, top=0.90, bottom=0.14)
axA = fig.add_subplot(gs[0, 0]); axB = fig.add_subplot(gs[0, 1]); axC = fig.add_subplot(gs[1, :])
fig.suptitle("Fig 11 — M17 load holding: how much torque can the loop apply when the error is SMALL?\n"
             "(simulator, firmware-exact integer PID + bench-calibrated stick-slip plant; kp=2000 ki=25 kd=350000, V=200)",
             fontsize=15, fontweight="bold")

# ---------------- Panel A: sustained torque vs settled error ----------------
errs = list(range(0, 55000, 250))
pct = [e / CPR * 100 for e in errs]
axA.plot(pct, [sustained_torque(e, 26214400) / TAU_MAX * 100 for e in errs],
         color="tab:blue", lw=2.5, label="NEW firmware (25% integral clamp)")
axA.plot(pct, [sustained_torque(e, 52428800) / TAU_MAX * 100 for e in errs],
         color="tab:green", lw=1.8, ls="--", label="hypothetical 50% clamp (hunts at no-load!)")
axA.axhline(100, color="tab:red", lw=2.5, label="OLD firmware (100% windup) — but hunts, see B")
# sim-validated settled points (currentstate agent, task 5)
simE = [6331, 9335, 15696, 23461, 32100, 42681]
simT = [0.147, 0.171, 0.219, 0.278, 0.344, 0.398]
axA.plot([e / CPR * 100 for e in simE], [t / TAU_MAX * 100 for t in simT], "o",
         color="tab:blue", ms=7, mfc="white", label="sim-measured settled points (within ~1%)")
axA.axvline(0.1, color="black", lw=1.2, ls=":")
axA.annotate("Tom's target:\nerror = 0.1% of a rev\n(3,277 counts)\n\nNEW fw sustains only\n0.124 N·m = 31% of stall\n(P: 3,200 + I-rail: 12,800\n= 16,000 of 51,200 counts)",
             xy=(0.1, 31), xytext=(0.32, 12), fontsize=10,
             arrowprops=dict(arrowstyle="->", color="black"),
             bbox=dict(boxstyle="round", fc="lightyellow", ec="gray"))
axA.annotate("full torque only once the\nerror reaches 1.20% of a rev\n(39,322 counts = 4.3°)",
             xy=(1.20, 99), xytext=(0.62, 72), fontsize=10,
             arrowprops=dict(arrowstyle="->", color="tab:blue"),
             bbox=dict(boxstyle="round", fc="white", ec="tab:blue"))
axA.set_xlabel("settled position error (% of one revolution)")
axA.set_ylabel("sustained motor torque (% of stall)")
axA.set_title("A — Sustained torque available vs position error\n(integral fully wound; windup takes only 3–13 ms — the CLAMP is the limit, not the delay)", fontsize=11)
axA.set_xlim(0, 1.6); axA.set_ylim(0, 110)
axA.grid(alpha=0.3); axA.legend(fontsize=8.5, loc="lower right")

# ---------------- Panel B: standing error vs load ----------------
loads = [0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38]
s2err = [6331, 7215, 8344, 9335, 13255, 15696, 18168, 20963, 23461, 20913, 27735, 32100, 42681]
axB.plot(loads, [e / CPR * 100 for e in s2err], "o-", color="tab:blue", lw=2.5,
         label="NEW firmware: quiet, but PERMANENT standing error")
old_pts = {0.28: [14261, 868, 4165], 0.30: [16050, 4118, 4059, 3989, 5005], 0.32: [4435, 18277, 18198],
           0.34: [4842], 0.36: [4906], 0.38: [42636]}
for L, es in old_pts.items():
    axB.plot([L] * len(es), [e / CPR * 100 for e in es], "s", color="tab:red", ms=6, alpha=0.75)
axB.plot([], [], "s", color="tab:red", label="OLD firmware: stuck error, seed-dependent (bimodal),\nwhile driving FULL power continuously (heat!)")
axB.axvspan(0.13, 0.27, color="tab:red", alpha=0.12)
axB.text(0.205, 0.78, "OLD firmware HUNTS here\n17–22 Hz, ±3,000 counts,\nevery seed, every ki (5–100)\n(mean error ~0, but never rests)",
         ha="center", fontsize=9.5, color="darkred",
         bbox=dict(boxstyle="round", fc="mistyrose", ec="tab:red"))
axB.plot(0.20, 620 / CPR * 100, "*", color="tab:green", ms=20, mec="k",
         label="zero-cross-gated integral PROTOTYPE (sim only):\n~600 counts, quiet, settles in 0.1–0.6 s")
axB.axhline(0.1, color="black", lw=1.2, ls=":")
axB.text(0.335, 0.115, "0.1%-of-rev target", fontsize=9)
axB.annotate("loads below static friction (0.13 N·m)\nare held by friction alone (zero error)",
             xy=(0.14, 0.05), xytext=(0.15, 0.55), fontsize=9,
             arrowprops=dict(arrowstyle="->", color="gray"))
axB.set_xlabel("constant external load torque (N·m)   [stall = 0.40]")
axB.set_ylabel("settled position error (% of one revolution)")
axB.set_title("B — Standing error while holding a constant load\n(NEW fw: error set by the 25% clamp; ki only changes speed, never the ceiling)", fontsize=11)
axB.set_xlim(0.12, 0.40); axB.set_ylim(0, 1.55)
axB.grid(alpha=0.3); axB.legend(fontsize=8.5, loc="upper left")

# ---------------- Panel C: Tom's exact scenario traces ----------------
def read_trace(path):
    ts, es = [], []
    for row in csv.reader(open(path)):
        if not row or not row[0][0].isdigit():
            continue
        t = float(row[0])
        if t >= 0.85:
            ts.append(t); es.append(float(row[1]))
    return ts, es

for fname, color, lbl in ((f"{SCR}/tomstep_old.csv", "tab:red", "OLD firmware: mean ~0 but oscillates ±3,000 counts at ~20 Hz FOREVER (audible hum, heat)"),
                          (f"{SCR}/tomstep_step2.csv", "tab:blue", "NEW firmware: railed I + P = 0.124 N·m < 0.13 N·m breakaway — stuck at FULL error 3.7 s, one noise-\ntriggered hop, then parks 2,020 counts short FOREVER"),
                          (f"{SCR}/tomstep_gate.csv", "tab:green", "zero-cross-gated integral prototype: winds past the clamp, settles at −915 counts, quiet")):
    ts, es = read_trace(fname)
    axC.plot(ts, es, color=color, lw=1.0 if "OLD" in lbl else 1.8, alpha=0.8 if "OLD" in lbl else 1.0, label=lbl)
axC.axhline(0, color="gray", lw=0.8)
axC.axhspan(-1000, 1000, color="tab:green", alpha=0.07)
axC.text(7.9, 820, "gate deadband ±1,000 counts (±0.031% rev)", fontsize=8.5, color="darkgreen", ha="right")
axC.set_xlabel("time (s)   —   at t=1 s the setpoint steps by 0.1% of a revolution (3,277 counts), Tom's exact scenario")
axC.set_ylabel("position error (counts)")
axC.set_title("C — The 0.1%-of-a-rev scenario, head to head (seed 1; seeds 2–5 agree)", fontsize=11)
axC.set_xlim(0.85, 8); axC.set_ylim(-4600, 4600)
axC.grid(alpha=0.3); axC.legend(fontsize=9.5, loc="lower right")

fig.text(0.06, 0.015,
         "READING GUIDE: The anti-windup 25% integral clamp that fixed the kI oscillation also caps sustained torque at small error (A). Raising the clamp is NOT an option:\n"
         "the no-load limit cycle already returns one notch up at 31.25% (physics: I-rail torque must stay below breakaway friction, 0.094 < 0.13 N·m — so ANY fixed clamp that helps\n"
         "under load hunts at no-load). The OLD firmware only appeared to solve this: it hunts for every load 0.14–0.26 N·m and holds 0.28+ at permanent full power (B, red). A state-\n"
         "dependent mechanism (integral gated by error zero-crossings) achieves both goals in simulation (B star, C green) but is a PROTOTYPE: not in firmware, needs bench validation.\n"
         "All numbers: V=200, one bench-calibrated plant (static friction 0.13 N·m from a stiff QC-reject unit); fleet stiction spread unmeasured. 1,157 sim runs, overflow violations: 0.",
         fontsize=9.5, va="bottom", family="sans-serif",
         bbox=dict(boxstyle="round", fc="whitesmoke", ec="gray"))

fig.savefig(OUT, dpi=130)
print("wrote", OUT)
