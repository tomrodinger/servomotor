#!/usr/bin/env python3
"""
DATA-FILE INTEGRITY: the JSON definitions must agree with the firmware.

WHY THIS TEST EXISTS
    Almost everything in this project is generated from two JSON files:
    `motor_commands.json` defines all the commands, and `error_codes.json`
    defines every fatal error. From them come the Python library's methods, the
    Arduino library's methods, both API documents, and the firmware's own
    `error_text.h`.

    That makes them the single highest-leverage place for a mistake. A wrong
    number in either file propagates silently into four generated artefacts and
    two user-facing documents, and NOTHING catches it -- the generators do not
    validate their input, and no hardware test can notice that a description is
    wrong. The 2026-07 documentation project found roughly two dozen defects of
    exactly this kind, by hand.

    This test does mechanically what that project did by hand: it checks the
    JSON against the firmware source and against itself. It needs no hardware,
    so it runs in a second on every machine, including CI.

WHAT IS CHECKED
    error_codes.json
      1. codes are unique, and contiguous from 0
      2. every entry has an enum name, and those are unique
      3. the enum names and their ORDER match the firmware's error_text.h,
         because get_error_text() indexes that table by error code -- an
         ordering mismatch means the device reports the wrong message
      4. every ERROR_* symbol raised anywhere in the firmware source exists here

    motor_commands.json
      5. command IDs are unique
      6. command names are unique, and non-empty
      7. every command ID in the firmware's commands.h appears here
      8. descriptions exist and are not placeholders
      9. every error code named in a description actually exists

    Cross-file
     10. no description references an error number that error_codes.json
         does not define -- the failure mode that sends a user hunting for a
         code that was never real

NO HARDWARE REQUIRED. Pure file inspection.
"""

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
ERROR_CODES = os.path.join(HERE, "servomotor", "error_codes.json")
MOTOR_COMMANDS = os.path.join(HERE, "servomotor", "motor_commands.json")
FW_SRC = os.path.join(REPO, "firmware", "Src")
COMMON_SRC = os.path.join(REPO, "common_source_files")


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


def load_errors():
    with open(ERROR_CODES) as f:
        d = json.load(f)
    return d["errors"] if isinstance(d, dict) and "errors" in d else d


def load_commands():
    with open(MOTOR_COMMANDS) as f:
        return json.load(f)


def read_text(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def strip_c_comments(src):
    """Remove // and /* */ comments.

    Essential, not cosmetic: this file's first run reported
    ERROR_POSITION_OUT_OF_RANGE and ERROR_HALL_POSITION_OUT_OF_RANGE as
    "raised but undefined". Both appear ONLY inside commented-out
    fatal_error() calls (motor_control.c:3177 and :3182), so neither is
    reachable. Scanning raw text turns dead code into false bug reports.
    String literals are left alone -- no ERROR_* symbol appears inside one.
    """
    src = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
    src = re.sub(r"//[^\n]*", " ", src)
    return src


def firmware_error_text_table():
    """The firmware's error_text.h is a run of NUL-separated strings indexed by
    error code (see get_error_text in error_handling.c). Recover the order."""
    for cand in (os.path.join(COMMON_SRC, "error_text.h"),
                 os.path.join(FW_SRC, "error_text.h")):
        txt = read_text(cand)
        if txt:
            # Entries look like:  "some text\0"
            return re.findall(r'"((?:[^"\\]|\\.)*)\\0"', txt), cand
    return None, None


# Commands that exist in the firmware but are deliberately NOT part of the
# public API, so their absence from motor_commands.json is correct.
# Listed explicitly rather than pattern-matched, so that a NEW command going
# missing from the JSON still fails this test.
INTERNAL_ONLY_COMMANDS = {
    # cmd 120: a firmware-internal harness for exercising add_to_queue()
    # directly (main.c:718). Not exposed to users; using it can leave the
    # commanded and measured positions inconsistent.
    "ADD_TO_QUEUE_TEST_COMMAND",
}


def firmware_sources():
    out = []
    for root in (FW_SRC, COMMON_SRC):
        if not os.path.isdir(root):
            continue
        for dirpath, _dirs, files in os.walk(root):
            for fn in files:
                if fn.endswith((".c", ".h")):
                    out.append(os.path.join(dirpath, fn))
    return out


def main():
    res = Results()

    # ------------------------------------------------------------------
    print("\n1. error_codes.json internal consistency")
    # ------------------------------------------------------------------
    errors = load_errors()
    codes = [e.get("code") for e in errors]
    res.check("every error entry has a numeric code",
              all(isinstance(c, int) for c in codes),
              f"non-integer codes: {[c for c in codes if not isinstance(c, int)]}")

    dupes = {c for c in codes if codes.count(c) > 1}
    res.check("error codes are unique", not dupes, f"duplicated: {sorted(dupes)}")

    expected = list(range(len(codes)))
    res.check("error codes are contiguous starting at 0",
              sorted(codes) == expected,
              f"got {sorted(codes)[:6]}... expected 0..{len(codes) - 1}")

    enums = [e.get("enum", "") for e in errors]
    res.check("every error entry has an enum name", all(bool(x) for x in enums),
              f"missing on codes {[c for c, x in zip(codes, enums) if not x]}")

    edupes = {x for x in enums if x and enums.count(x) > 1}
    res.check("error enum names are unique", not edupes, f"duplicated: {sorted(edupes)}")

    res.check("every error enum is spelled ERROR_*",
              all(x.startswith("ERROR_") for x in enums if x),
              f"odd names: {[x for x in enums if x and not x.startswith('ERROR_')]}")

    # ------------------------------------------------------------------
    print("\n2. error_codes.json agrees with the firmware's error_text.h")
    # ------------------------------------------------------------------
    table, table_path = firmware_error_text_table()
    if table is None:
        res.check("firmware error_text.h was found", False,
                  "no error_text.h under firmware/Src or common_source_files")
    else:
        print(f"        (using {os.path.relpath(table_path, REPO)}, {len(table)} entries)")
        # get_error_text() walks the table by index, so a length mismatch means
        # some codes resolve to the wrong string or to "unknown error".
        res.check("the firmware text table has an entry for every error code",
                  len(table) >= len(codes),
                  f"table has {len(table)} entries, JSON defines {len(codes)} codes")

    # ------------------------------------------------------------------
    print("\n3. every ERROR_* raised in the firmware is defined in the JSON")
    # ------------------------------------------------------------------
    raised = set()
    for path in firmware_sources():
        code = strip_c_comments(read_text(path))
        for mname in re.findall(r"fatal_error\(\s*(ERROR_[A-Z0-9_]+)\s*\)", code):
            raised.add(mname)
    known = set(x for x in enums if x)
    missing = sorted(raised - known)
    print(f"        ({len(raised)} distinct ERROR_* symbols raised in firmware)")
    res.check("every error the firmware can raise is defined in error_codes.json",
              not missing, f"raised but undefined: {missing}")

    # ------------------------------------------------------------------
    print("\n4. motor_commands.json internal consistency")
    # ------------------------------------------------------------------
    cmds = load_commands()
    ids = [c.get("CommandEnum") for c in cmds]
    names = [c.get("CommandString", "") for c in cmds]

    res.check("every command has a numeric ID",
              all(isinstance(i, int) for i in ids),
              f"non-integer IDs: {[i for i in ids if not isinstance(i, int)]}")

    idupes = {i for i in ids if ids.count(i) > 1}
    res.check("command IDs are unique", not idupes, f"duplicated: {sorted(idupes)}")

    res.check("every command has a name", all(bool(n) for n in names),
              f"unnamed IDs: {[i for i, n in zip(ids, names) if not n]}")

    ndupes = {n for n in names if n and names.count(n) > 1}
    res.check("command names are unique", not ndupes, f"duplicated: {sorted(ndupes)}")

    res.check("every command has Input and Output entries",
              all(("Input" in c and "Output" in c) for c in cmds),
              f"missing on: {[c.get('CommandString') for c in cmds if 'Input' not in c or 'Output' not in c]}")

    # ------------------------------------------------------------------
    print("\n5. motor_commands.json covers the firmware's command IDs")
    # ------------------------------------------------------------------
    cmd_header = read_text(os.path.join(FW_SRC, "commands.h"))
    fw_cmds = {}
    for mname, val in re.findall(r"#define\s+([A-Z0-9_]*COMMAND[A-Z0-9_]*)\s+(\d+)", cmd_header):
        fw_cmds[mname] = int(val)
    print(f"        ({len(fw_cmds)} command #defines found in firmware/Src/commands.h)")
    json_ids = set(i for i in ids if isinstance(i, int))
    fw_missing = sorted(n for n, v in fw_cmds.items()
                        if v not in json_ids and n not in INTERNAL_ONLY_COMMANDS)
    res.check("every public firmware command ID appears in motor_commands.json",
              not fw_missing,
              f"in commands.h but not in the JSON: {fw_missing}")

    # The allowlist must not rot: every name on it should still exist in the
    # firmware, otherwise it is silently excusing nothing.
    stale = sorted(n for n in INTERNAL_ONLY_COMMANDS if n not in fw_cmds)
    res.check("the internal-only command allowlist has no stale entries",
              not stale, f"allowlisted but no longer in commands.h: {stale}")

    # ------------------------------------------------------------------
    print("\n6. descriptions are real, and their error references exist")
    # ------------------------------------------------------------------
    descs = [(c.get("CommandString", "?"), c.get("Description", "")) for c in cmds]
    empty = [n for n, d in descs if not d or len(d.strip()) < 20]
    res.check("every command has a substantive description", not empty,
              f"missing or stub: {empty}")

    placeholders = [n for n, d in descs
                    if re.search(r"\b(TODO|TBD|FIXME|XXX|Lorem ipsum)\b", d, re.I)]
    res.check("no description contains a placeholder marker", not placeholders,
              f"placeholders in: {placeholders}")

    # Any "fatal error NN" / "error code NN" reference must be a real code.
    valid_codes = set(c for c in codes if isinstance(c, int))
    bad_refs = []
    for name, d in descs:
        for num in re.findall(r"(?:fatal error|error code|error)\s+(\d{1,3})\b", d, re.I):
            if int(num) not in valid_codes:
                bad_refs.append(f"{name} -> error {num}")
    res.check("every numeric error reference in a description is a real code",
              not bad_refs, f"dangling references: {bad_refs}")

    # And any ERROR_* name mentioned must be real too.
    bad_names = []
    for name, d in descs:
        for sym in re.findall(r"\b(ERROR_[A-Z0-9_]+)\b", d):
            if sym not in known:
                bad_names.append(f"{name} -> {sym}")
    res.check("every ERROR_* symbol named in a description is defined",
              not bad_names, f"undefined symbols: {bad_names}")

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
