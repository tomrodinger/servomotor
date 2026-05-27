#!/usr/bin/env python3
"""
Run the host-only test suite -- no motor and no serial port required.

These tests (test_host_*.py) exercise the pure-software parts of the
servomotor library: command encoding, response decoding, CRC32, and the
import / code-generation path. Use this as a fast pre-commit / CI check,
or before the MicroPython refactor to prove the PC path is not broken.

    python3 run_host_tests.py

Exit code is 0 when every host test passes, 1 otherwise.
"""

import glob
import os
import subprocess
import sys

GREEN = "\033[0;32m"
RED = "\033[0;31m"
NC = "\033[0m"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    tests = sorted(glob.glob(os.path.join(here, "test_host_*.py")))

    if not tests:
        print(RED + "Error: no test_host_*.py files found." + NC)
        return 1

    print("Running %d host-only test(s) (no hardware needed)" % len(tests))
    print("=" * 60)

    passed = 0
    failed = 0
    results = []

    for test_path in tests:
        name = os.path.basename(test_path)
        print("\n--- %s ---" % name)
        result = subprocess.run([sys.executable, test_path],
                                capture_output=True, text=True, cwd=here)
        print(result.stdout, end="")
        if result.stderr:
            print(result.stderr, end="")

        last_line = ""
        if result.stdout.strip():
            last_line = result.stdout.strip().splitlines()[-1]
        ok = result.returncode == 0 and "PASSED" in last_line

        if ok:
            passed += 1
            results.append((name, True))
            print(GREEN + name + " PASSED" + NC)
        else:
            failed += 1
            results.append((name, False))
            print(RED + "%s FAILED (exit %d)" % (name, result.returncode) + NC)

    print("\n" + "=" * 60)
    print("Host test summary:")
    for name, ok in results:
        mark = (GREEN + "PASS" + NC) if ok else (RED + "FAIL" + NC)
        print("  %s  %s" % (mark, name))
    print("%d passed, %d failed, %d total" % (passed, failed, len(tests)))

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
