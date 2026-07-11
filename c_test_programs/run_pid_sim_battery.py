#!/usr/bin/env python3
"""
PID algorithm simulation battery
================================

Runs pid_algorithm_sim across a matrix of scenarios and algorithm variants and
tabulates the results, checking the acceptance criteria from
firmware/PID_IMPROVEMENT_PLAN.md. Exit code 0 = all criteria pass.

Usage:
    python3 run_pid_sim_battery.py [--sim ./pid_algorithm_sim] [--outdir DIR] [--csv-dir DIR]
"""
import argparse
import itertools
import json
import os
import subprocess
import sys

ALGS = ["old", "step1", "step2", "step3"]


def run_sim(simbin, outdir, tag, **kw):
    cmd = [simbin, "--quiet"]
    for k, v in kw.items():
        cmd += ["--" + k.replace("_", "-"), str(v)]
    if outdir:
        cmd += ["--csv", os.path.join(outdir, f"sim_{tag}.csv")]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode not in (0,):
        raise RuntimeError(f"sim failed ({tag}): rc={proc.returncode}\n{proc.stdout}\n{proc.stderr}")
    result = {}
    for line in proc.stdout.splitlines():
        if "=" in line:
            for part in line.split():
                if "=" in part:
                    k, v = part.split("=", 1)
                    try:
                        result[k] = float(v)
                    except ValueError:
                        result[k] = v
    return result


def fmt(v):
    if isinstance(v, float):
        return f"{v:,.1f}" if abs(v) < 1e6 else f"{v/1e6:,.1f}M"
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default="./pid_algorithm_sim")
    ap.add_argument("--outdir", default=None, help="write per-run CSVs here (default: none)")
    ap.add_argument("--report", default="pid_sim_battery_report.md")
    args = ap.parse_args()

    checks = []          # (name, ok, detail)
    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))
        print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f"  [{detail}]" if detail else ""))

    report = ["# PID sim battery report",
              "",
              "Final proposed design = **step2** (rounding fixes + back-calculation anti-windup",
              "+ integral clamp at 25% authority, existing LPF derivative topology).",
              "`old` = shipped 0.15.3.0 behavior (regression reference).",
              "`step3` = experimental FIR-32 derivative (kept for reference; rejected).",
              "",
              "NOTE on acceptance thresholds: the stick-slip plant is bistable at low kI —",
              "the old algorithm limit-cycles on ~40% of seeds even at the default kI=5.",
              "Post-move RMS < 500 counts (0.015% of a rev) counts as 'settled, no limit cycle';",
              "a limit cycle measures in the thousands.",
              ""]

    # ---------------------------------------------------------------- 1. standstill
    print("\n[1] Standstill, default gains, all algorithms")
    report += ["## 1. Standstill (5 s, defaults kP=2000 kI=5 kD=175000)", "",
               "| alg | err std | D mean | D std | overflow |", "|---|---|---|---|---|"]
    ss = {}
    for alg in ALGS:
        r = run_sim(args.sim, args.outdir, f"ss_{alg}", alg=alg, duration=5)
        ss[alg] = r
        report.append(f"| {alg} | {fmt(r['ss_err_std'])} | {fmt(r['ss_d_mean'])} | "
                      f"{fmt(r['ss_d_std'])} | {int(r['overflow_violations'])} |")
    report.append("")
    check("old alg reproduces sticky D bias (mean < -40k)", ss["old"]["ss_d_mean"] < -40000,
          f"{ss['old']['ss_d_mean']:.0f}")
    for alg in ["step1", "step2", "step3"]:
        check(f"{alg}: D bias eliminated (|mean| < 15k)", abs(ss[alg]["ss_d_mean"]) < 15000,
              f"{ss[alg]['ss_d_mean']:.0f}")
    # Note: the FIR-32 difference (step3) has sqrt(2)x the noise of the leaky filter, not less
    # (measured and derived analytically); this is one reason step3 was rejected.
    check("step3: D noise ~= sqrt(2) x old (sanity of the analysis; ratio 1.2..1.7)",
          1.2 < ss["step3"]["ss_d_std"] / ss["old"]["ss_d_std"] < 1.7,
          f"ratio {ss['step3']['ss_d_std'] / ss['old']['ss_d_std']:.2f}")
    for alg in ALGS:
        check(f"{alg}: standstill error std not degraded (< 45 counts)", ss[alg]["ss_err_std"] < 45,
              f"{ss[alg]['ss_err_std']:.1f}")

    # ---------------------------------------------------------------- 2. default move
    print("\n[2] Default-gain move (2 rot / 2 s), all algorithms")
    report += ["## 2. Move 2 rot / 2 s at default gains", "",
               "| alg | peak err | post RMS | settle ms | osc Hz | sat ticks |", "|---|---|---|---|---|---|"]
    mv = {}
    for alg in ALGS:
        r = run_sim(args.sim, args.outdir, f"mv_{alg}", alg=alg, duration=8, move=2)
        mv[alg] = r
        report.append(f"| {alg} | {fmt(r['move_peak_err'])} | {fmt(r['post_move_rms'])} | "
                      f"{fmt(r['settle_ms'])} | {fmt(r['osc_freq_hz'])} | {int(r['saturated_ticks'])} |")
    report.append("")
    for alg in ["step2", "step3"]:
        check(f"{alg}: default move settles (post RMS < 500)", mv[alg]["post_move_rms"] < 500,
              f"{mv[alg]['post_move_rms']:.0f}")
    check("step1 behaves like old at defaults (peak err within 25%)",
          abs(mv["step1"]["move_peak_err"] - mv["old"]["move_peak_err"]) < 0.25 * mv["old"]["move_peak_err"],
          f"{mv['step1']['move_peak_err']:.0f} vs {mv['old']['move_peak_err']:.0f}")

    # ---------------------------------------------------------------- 3. kI sweep (the windup test)
    print("\n[3] kI sweep with move: windup / limit-cycle test")
    report += ["## 3. kI sweep, move 2 rot / 2 s (limit-cycle = post RMS > 1000)", "",
               "| kI | " + " | ".join(f"{a} post-RMS" for a in ALGS) + " |",
               "|---|" + "---|" * len(ALGS)]
    sweep_rows = {}
    for ki in [5, 10, 25, 50, 100, 200]:
        row = {}
        for alg in ALGS:
            r = run_sim(args.sim, args.outdir, f"ki{ki}_{alg}", alg=alg, duration=8, move=2, ki=ki)
            row[alg] = r
        sweep_rows[ki] = row
        report.append(f"| {ki} | " + " | ".join(fmt(row[a]["post_move_rms"]) for a in ALGS) + " |")
    report.append("")
    check("old alg limit-cycles at kI=25 (validates the sim reproduces the bug)",
          sweep_rows[25]["old"]["post_move_rms"] > 1000,
          f"{sweep_rows[25]['old']['post_move_rms']:.0f}")
    for ki in [5, 10, 25, 50, 100]:
        check(f"step2 (final design): no limit cycle at kI={ki} (post RMS < 500)",
              sweep_rows[ki]["step2"]["post_move_rms"] < 500,
              f"{sweep_rows[ki]['step2']['post_move_rms']:.0f}")
    check("step2: kI=200 vastly better than old (RMS < 10% of old)",
          sweep_rows[200]["step2"]["post_move_rms"] < 0.1 * sweep_rows[200]["old"]["post_move_rms"],
          f"{sweep_rows[200]['step2']['post_move_rms']:.0f} vs {sweep_rows[200]['old']['post_move_rms']:.0f}")

    # ---------------------------------------------------------------- 4. kD sweep (damping usefulness)
    print("\n[4] kD sweep at kI=25 with move (final design): usable damping range")
    report += ["## 4. kD sweep, kI=25, move 2 rot / 2 s (median of 3 seeds: move peak err / post RMS)", "",
               "| kD | step2 (final) | step3 (reference) |", "|---|---|---|"]
    kd_rows = {}
    for kd in [0, 87500, 175000, 350000, 700000, 1000000]:
        row = {}
        for alg in ["step2", "step3"]:
            runs = [run_sim(args.sim, args.outdir, f"kd{kd}_{alg}_s{s}", alg=alg,
                            duration=8, move=2, ki=25, kd=kd, seed=s) for s in (1, 6, 10)]
            med = sorted(runs, key=lambda r: r["post_move_rms"])[1]
            row[alg] = med
        kd_rows[kd] = row
        report.append(f"| {kd} | {fmt(row['step2']['move_peak_err'])} / {fmt(row['step2']['post_move_rms'])} "
                      f"| {fmt(row['step3']['move_peak_err'])} / {fmt(row['step3']['post_move_rms'])} |")
    report.append("")
    check("step2: default kD=175k damps well (median post RMS < 500)",
          kd_rows[175000]["step2"]["post_move_rms"] < 500,
          f"{kd_rows[175000]['step2']['post_move_rms']:.0f}")
    for kd in [350000, 700000, 1000000]:
        check(f"step2: graceful at kD={kd} (median post RMS < 500)",
              kd_rows[kd]["step2"]["post_move_rms"] < 500,
              f"{kd_rows[kd]['step2']['post_move_rms']:.0f}")
    check("kD=0 documented as unusable (median post RMS > 2000 even with AW)",
          kd_rows[0]["step2"]["post_move_rms"] > 2000,
          f"{kd_rows[0]['step2']['post_move_rms']:.0f}")

    # ---------------------------------------------------------------- 5. stress & edge cases
    print("\n[5] Stress & edge cases")
    report += ["## 5. Stress & edge cases", "", "| case | alg | key metrics | overflow |", "|---|---|---|---|"]
    stress = [
        ("fast move (5 rot in 0.7 s, saturates)", dict(move=5, move_time=0.7, duration=6)),
        ("reversal (-3 rot)", dict(move=-3, duration=8)),
        ("external load 0.05 Nm", dict(move=2, duration=8, load=0.05)),
        ("external load 0.05 Nm, kI=25", dict(move=2, duration=8, load=0.05, ki=25)),
        ("high noise (sigma 100)", dict(move=2, duration=8, noise=100)),
        ("tiny move (0.01 rot)", dict(move=0.01, move_time=0.5, duration=4)),
        ("slow long move (10 rot / 20 s)", dict(move=10, move_time=20, duration=24)),
        ("kP=0 kD=0 (I only)", dict(move=1, duration=8, kp=0, kd=0, ki=50)),
        ("huge gains", dict(move=2, duration=8, kp=20000, ki=500, kd=2000000)),
        ("long standstill 60 s", dict(duration=60)),
    ]
    for name, kw in stress:
        for alg in ["old", "step2"]:
            tag = f"stress_{name.split()[0].strip('(')}_{alg}"
            r = run_sim(args.sim, args.outdir, tag, alg=alg, **kw)
            key = (f"peak_err={r.get('move_peak_err', r.get('max_abs_error', 0)):.0f} "
                   f"postRMS={r.get('post_move_rms', 0):.0f} sat={int(r['saturated_ticks'])}")
            report.append(f"| {name} | {alg} | {key} | {int(r['overflow_violations'])} |")
            check(f"{name} [{alg}]: no int32 overflow", r["overflow_violations"] == 0)
            if alg == "step2" and "huge" not in name and "kP=0" not in name:
                check(f"{name} [step2]: bounded post-move error (RMS < 2000)",
                      r.get("post_move_rms", 0) < 2000, f"{r.get('post_move_rms', 0):.0f}")
    report.append("")

    # ---------------------------------------------------------------- 6. seed robustness
    print("\n[6] Seed robustness (step2/step3, kI=25 move, 5 seeds)")
    report += ["## 6. Seed robustness (kI=25 move, post RMS per seed)", "",
               "| seed | step2 | step3 |", "|---|---|---|"]
    for seed in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
        vals = {}
        for alg in ["step2", "step3"]:
            r = run_sim(args.sim, args.outdir, f"seed{seed}_{alg}", alg=alg, duration=8, move=2, ki=25, seed=seed)
            vals[alg] = r["post_move_rms"]
        check(f"seed {seed} [step2]: no limit cycle at kI=25 (post RMS < 500)", vals["step2"] < 500,
              f"{vals['step2']:.0f}")
        report.append(f"| {seed} | {fmt(vals['step2'])} | {fmt(vals['step3'])} |")
    report.append("")

    # ---------------------------------------------------------------- summary
    n_fail = sum(1 for _, ok, _ in checks if not ok)
    report += ["## Result", "",
               f"**{len(checks) - n_fail}/{len(checks)} checks passed**", ""]
    if n_fail:
        report += ["Failed checks:", ""]
        report += [f"- {name} ({detail})" for name, ok, detail in checks if not ok]

    with open(args.report, "w") as f:
        f.write("\n".join(report) + "\n")
    print(f"\n{'=' * 60}\n{len(checks) - n_fail}/{len(checks)} checks passed; report: {args.report}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
