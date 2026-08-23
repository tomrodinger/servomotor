#!/usr/bin/env python3
"""Reap ORPHANED headless Chrome processes.

Only ever touches processes whose argv contains "--headless", and only those older than AGE
seconds. Tom's real browser never carries that flag, so it cannot be matched — which is the whole
point. NEVER widen this to match "Google Chrome": that substring is in the real browser's argv and
killing it destroys his open tabs (it has happened ten times on this project).

macOS `ps` has no `etimes`, so elapsed time is parsed from `etime` ([[dd-]hh:]mm:ss).
"""
import os
import re
import signal
import subprocess
import sys
import time

AGE = int(sys.argv[1]) if len(sys.argv) > 1 else 300
MAIN = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def etime_to_secs(s):
    s = s.strip()
    if not s:
        return None
    days = 0
    if "-" in s:
        d, s = s.split("-", 1)
        days = int(d)
    parts = [int(p) for p in s.split(":")]
    while len(parts) < 3:
        parts.insert(0, 0)
    h, m, sec = parts[-3], parts[-2], parts[-1]
    return days * 86400 + h * 3600 + m * 60 + sec


def snapshot():
    out = subprocess.run(["ps", "-eo", "pid=,etime=,command="],
                         capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        m = re.match(r"\s*(\d+)\s+(\S+)\s+(.*)$", line)
        if not m:
            continue
        pid, et, cmd = int(m.group(1)), m.group(2), m.group(3)
        rows.append((pid, etime_to_secs(et) or 0, cmd))
    return rows


def main():
    rows = snapshot()
    headless = [(p, a, c) for (p, a, c) in rows if "--headless" in c]
    stale = [(p, a) for (p, a, c) in headless if a > AGE]

    killed = 0
    for pid, age in stale:
        try:
            os.kill(pid, signal.SIGTERM)
            killed += 1
        except (ProcessLookupError, PermissionError):
            pass

    if killed:
        time.sleep(1.5)
        rows = snapshot()
        for (p, a, c) in rows:
            if "--headless" in c and a > AGE:
                try:
                    os.kill(p, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass

    rows = snapshot()
    remaining = [(p, a) for (p, a, c) in rows if "--headless" in c]
    real = [p for (p, a, c) in rows if c.startswith(MAIN) and "--headless" not in c]

    print("reaped %d stale headless proc(s) older than %ds" % (killed, AGE))
    print("headless remaining: %d%s" % (
        len(remaining),
        "  (ages: %s)" % ", ".join("%ds" % a for _, a in sorted(remaining, key=lambda x: -x[1])[:6])
        if remaining else ""))
    print("real browser: %s" % ("OK pid " + str(real[0]) if real else "NOT FOUND — investigate!"))
    return 0 if real else 1


if __name__ == "__main__":
    sys.exit(main())
