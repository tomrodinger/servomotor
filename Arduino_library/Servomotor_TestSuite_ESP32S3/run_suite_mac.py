#!/usr/bin/env python3
"""Mac-side orchestrator/collector for the on-device ESP32-S3 test suite.

Opens the ESP32's USB CDC port, resets the board, sends `run` (or `run N`),
then logs everything the suite streams while it executes one test per boot
(the board restarts itself between tests; this script rides through the
reconnects). Finishes when the firmware prints SUITE COMPLETE.

Usage:
  python3 run_suite_mac.py [--port /dev/cu.usbmodem2101] [--only N]
                           [--attach] [--timeout 2700] [--log suite_run.log]

Exit code 0 iff the firmware reports unexpected=0 (known-bug failures are
expected and do not fail the run).
"""
import argparse
import re
import sys
import time

import serial


def open_port(port, deadline):
    while time.time() < deadline:
        try:
            s = serial.Serial(port, 230400, timeout=0.5)
            s.dtr = False
            return s
        except Exception:
            time.sleep(0.7)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2101")
    ap.add_argument("--only", type=int, default=None, help="run a single test index")
    ap.add_argument("--attach", action="store_true",
                    help="don't start a run; just log an already-running suite")
    ap.add_argument("--timeout", type=int, default=2700)
    ap.add_argument("--log", default="suite_run.log")
    args = ap.parse_args()

    deadline = time.time() + args.timeout
    log = open(args.log, "w")

    def out(line):
        stamp = time.strftime("%H:%M:%S")
        print(f"[{stamp}] {line}")
        log.write(f"[{stamp}] {line}\n")
        log.flush()

    ser = open_port(args.port, time.time() + 30)
    if not ser:
        out(f"ERROR: cannot open {args.port}")
        return 2

    # Reset the board so we start from a clean boot (RTS pulse, DTR low).
    try:
        ser.rts = True
        time.sleep(0.15)
        ser.rts = False
        ser.reset_input_buffer()
    except Exception:
        pass
    out(f"connected to {args.port}; board reset")

    sent_cmd = args.attach
    unexpected = None
    summary_lines = []
    in_summary = False
    buf = b""

    while time.time() < deadline:
        try:
            data = ser.read(256)
        except Exception:
            out("(port dropped — reconnecting)")
            try:
                ser.close()
            except Exception:
                pass
            ser = open_port(args.port, deadline)
            if not ser:
                out("ERROR: lost port and could not reconnect")
                return 2
            continue

        if not data:
            continue
        buf += data
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").rstrip("\r")
            if not line:
                continue
            out(line)

            if not sent_cmd and line.startswith("IDLE"):
                cmd = "run\n" if args.only is None else f"run {args.only}\n"
                time.sleep(0.2)
                ser.write(cmd.encode())
                out(f">>> sent: {cmd.strip()}")
                sent_cmd = True

            if line.startswith("==== SUITE RESULTS"):
                in_summary = True
            if in_summary:
                summary_lines.append(line)
            m = re.search(r"SUITE_SUMMARY .*unexpected=(\d+)", line)
            if m:
                unexpected = int(m.group(1))
            if line.startswith("SUITE COMPLETE"):
                out("")
                out("========== FINAL SUMMARY (echo) ==========")
                for s in summary_lines:
                    out(s)
                ok = (unexpected == 0)
                out(f"RESULT: {'OK — no unexpected failures' if ok else 'UNEXPECTED FAILURES PRESENT'}")
                return 0 if ok else 1

    out("ERROR: timeout waiting for SUITE COMPLETE")
    return 2


if __name__ == "__main__":
    sys.exit(main())
