#!/usr/bin/env python3
"""
Which motor commands does the on-device test suite actually exercise?

The suite has grown to 80-odd modules organised by BEHAVIOUR (the planner, the
queue, the fatal-error contract, and so on). That is the right way to organise
tests, but it makes one question hard to answer: is there a COMMAND that no test
ever sends? A command with no coverage is invisible in a behaviour-organised
suite precisely because no behaviour chapter claims it.

This script answers that by mapping every command in motor_commands.json to its
Arduino library method and counting call sites across the tm_*.cpp modules.

    ./coverage_report.py              # full table, thinnest coverage first
    ./coverage_report.py --gaps 8     # only commands with fewer than 8 uses

WHAT THE NUMBERS DO AND DO NOT MEAN
    A high count means a command is used often, NOT that it is well tested --
    `getPositionRaw` leads the table because it is how almost every other
    assertion is measured, not because anything tests it thoroughly.

    A LOW count is the useful signal: it means few modules touch that command at
    all, so any defect in it has few chances to be noticed.

    Counts are approximate. Commands with both a converted and a `...Raw` form
    are counted under whichever name the mapping resolves to first, so a command
    may be exercised somewhat more than shown. Treat the table as a ranking, not
    a measurement.
"""

import argparse
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
COMMANDS = os.path.join(REPO, "python_programs", "servomotor", "motor_commands.json")
HEADER = os.path.join(REPO, "Arduino_library", "Servomotor.h")


def camel(name):
    parts = [p for p in re.split(r"[^A-Za-z0-9]+", name.strip()) if p]
    if not parts:
        return None
    out = parts[0].lower()
    for p in parts[1:]:
        out += p[0].upper() + p[1:].lower()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gaps", type=int, default=None,
                    help="only show commands used fewer than this many times")
    args = ap.parse_args()

    with open(COMMANDS) as f:
        cmds = json.load(f)
    with open(HEADER) as f:
        hdr = f.read()
    methods = set(re.findall(r"\b([a-z][A-Za-z0-9]*)\s*\(", hdr))

    blob = ""
    files = sorted(glob.glob(os.path.join(HERE, "tm_*.cpp")))
    for path in files:
        with open(path, encoding="utf-8", errors="replace") as f:
            blob += f.read()

    rows = []
    for c in cmds:
        name, cid = c["CommandString"], c["CommandEnum"]
        cand = camel(name) or ""
        real = None
        for m in methods:
            if m.lower() in (cand.lower(), cand.lower() + "raw"):
                real = m
                break
        if real is None:
            for m in sorted(methods, key=len, reverse=True):
                if len(m) > 5 and cand.lower().startswith(m.lower()):
                    real = m
                    break
        uses = 0
        if real:
            uses += len(re.findall(r"\b" + re.escape(real) + r"\s*\(", blob))
            uses += len(re.findall(r"\b" + re.escape(real) + r"Raw\s*\(", blob))
        rows.append((uses, cid, name, real or "(unmapped)"))

    rows.sort()
    if args.gaps is not None:
        rows = [r for r in rows if r[0] < args.gaps]

    print(f"{len(files)} test modules scanned\n")
    print(f"{'uses':>5}  {'id':>3}  {'command':34}  arduino method")
    print("-" * 84)
    for uses, cid, name, real in rows:
        print(f"{uses:5}  {cid:3}  {name[:34]:34}  {real}")

    zero = [r for r in rows if r[0] == 0]
    print(f"\n{len(zero)} command(s) with no direct use in the modules scanned")
    for _, cid, name, _real in zero:
        print(f"  - {cid}: {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
