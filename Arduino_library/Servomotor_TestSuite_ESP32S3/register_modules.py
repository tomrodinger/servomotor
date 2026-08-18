#!/usr/bin/env python3
"""
Register any tm_*.cpp module that is not yet in the test registry.

Adding a module by hand means editing two files consistently -- a declaration in
test_registry.h and a matching row in the TF_TESTS table in test_registry.cpp.
Getting one of them wrong produces either a link error or, worse, a module that
compiles and is silently never run. With the suite past eighty modules that is a
real hazard, and it is entirely mechanical, so it should not be done by hand.

    ./register_modules.py            # show what would be added
    ./register_modules.py --apply    # add it

The entry point is discovered from the file itself (`void tm_something(void)`),
and the registered NAME is that function with the leading `tm_` removed, which
is the convention every existing entry follows.

TIMEOUTS. The per-test watchdog has to exceed the module's real runtime or the
harness force-reboots the device and records a CRASH. New modules get a
deliberately generous default; tighten it by hand afterwards if you care. A
timeout that is too long only delays a hang being noticed, whereas one that is
too short fails a healthy test -- so the default errs long.
"""

import argparse
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REG_H = os.path.join(HERE, "test_registry.h")
REG_CPP = os.path.join(HERE, "test_registry.cpp")
DEFAULT_TIMEOUT = 1800


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def entry_points():
    """Map module file -> entry point function name."""
    found = {}
    for path in sorted(glob.glob(os.path.join(HERE, "tm_*.cpp"))):
        src = read(path)
        names = re.findall(r"^void\s+(tm_[A-Za-z0-9_]+)\s*\(\s*void\s*\)", src, re.M)
        if len(names) == 1:
            found[os.path.basename(path)] = names[0]
        elif not names:
            print(f"  WARNING: {os.path.basename(path)} has no 'void tm_xxx(void)' entry point")
        else:
            print(f"  WARNING: {os.path.basename(path)} has {len(names)} entry points: {names}")
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true", help="write the changes")
    ap.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT,
                    help=f"per-test watchdog for new entries (default {DEFAULT_TIMEOUT}s)")
    args = ap.parse_args()

    hdr, cpp = read(REG_H), read(REG_CPP)
    eps = entry_points()

    missing = []
    for fname, fn in sorted(eps.items()):
        declared = re.search(r"^\s*void\s+" + re.escape(fn) + r"\s*\(\s*\)\s*;", hdr, re.M)
        tabled = re.search(r"\b" + re.escape(fn) + r"\s*,", cpp)
        if not declared or not tabled:
            missing.append((fname, fn, bool(declared), bool(tabled)))

    if not missing:
        print(f"All {len(eps)} modules are registered. Nothing to do.")
        return 0

    print(f"{len(missing)} module(s) not fully registered:\n")
    for fname, fn, declared, tabled in missing:
        state = []
        if not declared:
            state.append("no declaration")
        if not tabled:
            state.append("not in TF_TESTS")
        print(f"  {fname:42} {fn:34} ({', '.join(state)})")

    if not args.apply:
        print("\nRe-run with --apply to add them.")
        return 0

    # --- add declarations, appended after the last existing one -------------
    decls = list(re.finditer(r"^void\s+tm_[A-Za-z0-9_]+\s*\(\s*\)\s*;\s*$", hdr, re.M))
    if not decls:
        print("ERROR: could not find any existing declaration to anchor to.")
        return 1
    insert_at = decls[-1].end()
    added_decls = "".join(f"\nvoid {fn}();" for _f, fn, d, _t in missing if not d)
    hdr = hdr[:insert_at] + added_decls + hdr[insert_at:]

    # --- add table rows just before the closing brace of TF_TESTS -----------
    mobj = re.search(r"(const TestDef TF_TESTS\[\]\s*=\s*\{)(.*?)(^\};)", cpp, re.S | re.M)
    if not mobj:
        print("ERROR: could not locate the TF_TESTS table.")
        return 1
    rows = ""
    for _fname, fn, _d, tabled in missing:
        if tabled:
            continue
        name = fn[3:] if fn.startswith("tm_") else fn
        rows += f'    {{ "{name}",{" " * max(1, 24 - len(name))}{fn},{" " * max(1, 26 - len(fn))}"",{" " * 17}{args.timeout}, true  }},\n'
    cpp = cpp[:mobj.end(2)] + rows + cpp[mobj.end(2):]

    with open(REG_H, "w", encoding="utf-8") as f:
        f.write(hdr)
    with open(REG_CPP, "w", encoding="utf-8") as f:
        f.write(cpp)

    print(f"\nRegistered {len(missing)} module(s) with a {args.timeout}s timeout.")
    print("Now rebuild:  ./build_host_suite.sh   (and ./flash_suite.sh for the device)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
