#!/usr/bin/env python3
"""Deep-analysis driver: why can't a big snap land without overshoot?
E1: step-size sweep x kD (overshoot floor appears when the approach saturates)
E2: nonlinearity ablation at a big step (rank the culprits)
E3: profiled-move cure demo
E4: small-step kD sweep (critical damping exists in the linear region)
Results -> overshoot_analysis.json
"""
import json
import subprocess
import statistics

SIM = "./pid_sim"
SEEDS = [1, 3, 6]
CPR = 3276800.0

def run(**kw):
    cmd = [SIM, "--quiet", "--alg", "step2", "--kp", "2000", "--ki", "25"]
    for k, v in kw.items():
        if k in ("no_slew", "linear_torque"):
            if v: cmd.append("--" + k.replace("_", "-"))
        else:
            cmd += ["--" + k.replace("_", "-"), str(v)]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    r = {}
    for line in out.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            try: r[k] = float(v)
            except ValueError: pass
    return r

def med(vals): return statistics.median(vals)

results = {}

print("[E1] step-size sweep x kD")
e1 = {}
for kd in (87500, 175000, 350000, 700000):
    for counts in (5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000):
        runs = [run(kd=kd, step=counts / CPR, duration=4, seed=s) for s in SEEDS]
        e1[f"{kd}_{counts}"] = {
            "overshoot": med([r["overshoot_counts"] for r in runs]),
            "vel_entry_rev_s": med([r["vel_at_linear_entry_cps"] for r in runs]) / CPR,
            "peak_vel": med([r["peak_vel_rev_s"] for r in runs]),
        }
        print(f"  kd={kd} step={counts}: ov={e1[f'{kd}_{counts}']['overshoot']:.0f} "
              f"v_entry={e1[f'{kd}_{counts}']['vel_entry_rev_s']:.2f} rev/s", flush=True)
results["e1"] = e1

print("[E2] ablation at 0.61-rot step")
e2 = {}
ABL = {
    "full plant": {},
    "no stiction": {"static_friction": 0.001, "coulomb": 0.001},
    "no STEP-slew limit": {"no_slew": 1},
    "linear torque": {"linear_torque": 1},
    "all removed": {"static_friction": 0.001, "coulomb": 0.001, "no_slew": 1, "linear_torque": 1},
}
for abl, extra in ABL.items():
    for kd in (175000, 350000, 700000, 1400000):
        runs = [run(kd=kd, step=0.61, duration=4, seed=s, **extra) for s in SEEDS]
        e2[f"{abl}|{kd}"] = {
            "overshoot": med([r["overshoot_counts"] for r in runs]),
            "vel_entry_rev_s": med([r["vel_at_linear_entry_cps"] for r in runs]) / CPR,
        }
        print(f"  {abl} kd={kd}: ov={e2[f'{abl}|{kd}']['overshoot']:.0f} "
              f"v_entry={e2[f'{abl}|{kd}']['vel_entry_rev_s']:.2f}", flush=True)
results["e2"] = e2

print("[E3] profiled-move cure: same 0.61 rot as trapezoid")
e3 = {}
for mt in (0.15, 0.3, 0.6):
    runs = [run(kd=350000, move=0.61, move_time=mt, duration=4, seed=s) for s in SEEDS]
    e3[f"trap_{mt}"] = med([r["overshoot_counts"] for r in runs])
    print(f"  trapezoid {mt}s: ov={e3[f'trap_{mt}']:.0f}", flush=True)
runs = [run(kd=350000, step=0.61, duration=4, seed=s) for s in SEEDS]
e3["step"] = med([r["overshoot_counts"] for r in runs])
results["e3"] = e3

print("[E4] small-step (10k counts) fine kD sweep: does a no-overshoot tune exist?")
e4 = {}
for kd in (87500, 175000, 262500, 350000, 500000, 700000, 1000000, 1400000):
    runs = [run(kd=kd, step=10000 / CPR, duration=3, seed=s) for s in SEEDS]
    e4[str(kd)] = med([r["overshoot_counts"] for r in runs])
    print(f"  kd={kd}: ov={e4[str(kd)]:.0f}", flush=True)
results["e4"] = e4

json.dump(results, open("overshoot_analysis.json", "w"), indent=1)
print("saved overshoot_analysis.json")
