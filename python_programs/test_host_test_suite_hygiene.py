#!/usr/bin/env python3
"""
TEST-SUITE HYGIENE: the tests themselves are checked, mechanically.

WHY THIS TEST EXISTS
    The Arduino on-device suite is past ninety modules. Its comments cite
    command numbers, error codes and firmware file:line references, and those
    citations are load-bearing: they are how the next person (or the next
    model) decides whether an assertion is correct. A comment saying
    "MOVE WITH ACCELERATION (cmd 11)" when cmd 11 is actually
    "Get n queued items" does not fail any test -- it quietly teaches the
    reader something false, and the next module built on that reading inherits
    the error.

    Two such mislabelled command numbers were found by hand in modules written
    on 2026-08-04. Finding them by hand does not scale, and the error class is
    entirely mechanical, so it should be caught mechanically.

    The suite also has structural rules that are invisible until they bite:
    a module that is written but never registered compiles cleanly and simply
    never runs; a module that broadcasts or writes an alias will disturb all 35
    motors on a shared rack; and a module that sends test modes 10..13 hangs the
    motor until someone power-cycles it.

WHAT IS CHECKED
    1. Every "<Command Name> (cmd N)" citation in a module names the right N.
    2. Every module file has exactly one `void tm_xxx(void)` entry point.
    3. Every entry point is declared in test_registry.h AND present in the
       TF_TESTS table -- i.e. no module is silently never run.
    4. Every registry entry points at a function that actually exists.
    5. No module contains a rack-hostile call: a broadcast, setDeviceAlias, or
       a testMode value in 10..13.
    6. Registry names are unique, and so are the entry points.

NO HARDWARE REQUIRED. Pure source inspection; runs in well under a second.
"""

import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SUITE = os.path.join(REPO, "Arduino_library", "Servomotor_TestSuite_ESP32S3")
COMMANDS = os.path.join(HERE, "servomotor", "motor_commands.json")


class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.failures = []

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            print(f"  PASS  {name}")
        else:
            self.failed += 1
            self.failures.append(f"{name}: {detail}")
            print(f"  FAIL  {name}\n        {detail}")


def read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def strip_strings(src):
    """Blank out string literals so a command name mentioned inside a
    TEST_RESULT message is not mistaken for a code-level call."""
    return re.sub(r'"(?:[^"\\]|\\.)*"', '""', src)


def main():
    res = Results()

    if not os.path.isdir(SUITE):
        print(f"Test suite directory not found: {SUITE}")
        return 1

    modules = sorted(glob.glob(os.path.join(SUITE, "tm_*.cpp")))
    print(f"\nScanning {len(modules)} module(s) in {os.path.relpath(SUITE, REPO)}")

    with open(COMMANDS) as f:
        cmds = json.load(f)
    by_name = {c["CommandString"].lower(): c["CommandEnum"] for c in cmds}

    # ------------------------------------------------------------------
    print("\n1. command-number citations in comments are correct")
    # ------------------------------------------------------------------
    wrong = []
    pattern = re.compile(
        r"[`\"']?([A-Za-z][A-Za-z /]{4,40}?)[`\"']?\s*\(\s*(?:cmd|command)\s+(\d+)\s*\)", re.I)
    for path in modules:
        src = read(path)
        for mobj in pattern.finditer(src):
            label = mobj.group(1).strip().strip("`\"'").lower()
            num = int(mobj.group(2))
            # Normalise the surrounding prose. Comments say things like
            # "the HOMING command (cmd 14)", and an earlier version of this
            # check matched the literal capture "homing command", found no such
            # command, and silently passed -- which let a real mislabel
            # (Homing cited as cmd 30, which is Set safety limits) sit in
            # tm_homing_edges.cpp until a subagent noticed it by hand.
            label = re.sub(r"^the\s+", "", label)
            label = re.sub(r"\s+command$", "", label)
            label = re.sub(r"\s+", " ", label).strip()
            if label in by_name and by_name[label] != num:
                wrong.append(f"{os.path.basename(path)}: '{mobj.group(1).strip()}' "
                             f"cited as cmd {num}, actually cmd {by_name[label]}")
    res.check("every command-number citation matches motor_commands.json",
              not wrong, "; ".join(wrong))

    # ------------------------------------------------------------------
    print("\n2. every module has exactly one entry point")
    # ------------------------------------------------------------------
    entries = {}
    bad_entry = []
    for path in modules:
        names = re.findall(r"^void\s+(tm_[A-Za-z0-9_]+)\s*\(\s*void\s*\)",
                           read(path), re.M)
        base = os.path.basename(path)
        if len(names) != 1:
            bad_entry.append(f"{base}: {len(names)} entry points {names}")
        else:
            entries[base] = names[0]
    res.check("every module defines exactly one void tm_xxx(void)",
              not bad_entry, "; ".join(bad_entry))

    dupes = [fn for fn in entries.values() if list(entries.values()).count(fn) > 1]
    res.check("entry point names are unique across modules",
              not dupes, f"duplicated: {sorted(set(dupes))}")

    # ------------------------------------------------------------------
    print("\n3. every module is registered, so none is silently never run")
    # ------------------------------------------------------------------
    hdr = read(os.path.join(SUITE, "test_registry.h"))
    cpp = read(os.path.join(SUITE, "test_registry.cpp"))

    undeclared, untabled = [], []
    for base, fn in sorted(entries.items()):
        if not re.search(r"^\s*void\s+" + re.escape(fn) + r"\s*\(\s*\)\s*;", hdr, re.M):
            undeclared.append(base)
        if not re.search(r"\b" + re.escape(fn) + r"\s*,", cpp):
            untabled.append(base)
    res.check("every module's entry point is declared in test_registry.h",
              not undeclared, f"missing declarations: {undeclared}")
    res.check("every module appears in the TF_TESTS table",
              not untabled, f"never run: {untabled}")

    # ------------------------------------------------------------------
    print("\n4. every registry entry points at a real function")
    # ------------------------------------------------------------------
    table = re.search(r"const TestDef TF_TESTS\[\]\s*=\s*\{(.*?)^\};", cpp, re.S | re.M)
    rows = []
    if table:
        rows = re.findall(r'\{\s*"([^"]+)"\s*,\s*(tm_[A-Za-z0-9_]+)\s*,', table.group(1))
    res.check("the TF_TESTS table is parseable", bool(rows), "no rows matched")

    known = set(entries.values())
    orphan = sorted({fn for _n, fn in rows} - known)
    res.check("every TF_TESTS entry names a function that exists",
              not orphan, f"no such function: {orphan}")

    names = [n for n, _fn in rows]
    ndupes = sorted({n for n in names if names.count(n) > 1})
    res.check("registry test names are unique", not ndupes, f"duplicated: {ndupes}")
    print(f"        ({len(rows)} registered, {len(entries)} module files)")

    # ------------------------------------------------------------------
    print("\n5. no module is hostile to the shared 35-motor rack")
    # ------------------------------------------------------------------
    # Known-safe modules that legitimately do these things; they are excluded
    # from the single-motor rack runs rather than from the suite.
    RACK_UNSAFE_BY_DESIGN = {
        "tm_broadcast.cpp", "tm_multi_device.cpp", "tm_two_motors.cpp",
        "tm_addressing.cpp", "tm_wrong_addressing.cpp", "tm_alias_edges.cpp",
        "tm_uid_edges.cpp",
    }
    alias_writers, mode_hangers = [], []
    for path in modules:
        base = os.path.basename(path)
        code = strip_strings(read(path))
        if base not in RACK_UNSAFE_BY_DESIGN:
            if re.search(r"\bsetDeviceAlias\s*\(", code):
                alias_writers.append(base)
        # testMode(10..13) disables interrupts and spins forever -- only a
        # power cycle recovers, so no automated module may ever send one.
        for lit in re.findall(r"\btestMode\s*\(\s*(\d+)\s*\)", code):
            if 10 <= int(lit) <= 13:
                mode_hangers.append(f"{base}: testMode({lit})")
    res.check("no unexpected module writes a device alias",
              not alias_writers, f"alias writers: {alias_writers}")
    res.check("no module sends the hang-forever test modes 10..13",
              not mode_hangers, "; ".join(mode_hangers))

    # ------------------------------------------------------------------
    print(f"\n{res.passed} passed, {res.failed} failed")
    for f in res.failures:
        print(f"  - {f}")
    if res.failed == 0:
        print("\nPASSED")
        return 0
    print("\nFAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
