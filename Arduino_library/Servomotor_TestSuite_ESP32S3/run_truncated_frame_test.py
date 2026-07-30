#!/usr/bin/env python3
"""
Runner for truncated_frame_test — the transport-layer regression test for
Communication::getResponse().

Creates a pseudo-terminal, launches the test binary against the slave side, and
plays a scripted fake device on the master side. No motor, no ESP32, no
hardware of any kind: the whole point is to produce a truncated reply on
demand, which a real motor cannot be asked to do.

    ./build_truncated_frame_test.sh      # build first
    python3 run_truncated_frame_test.py  # run all cases

Exit code 0 only if every case behaves correctly. A hang in the library shows
up as the binary being killed by its own SIGALRM watchdog (exit 42), which this
runner reports as HUNG.
"""

import os
import pty
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BINARY = os.path.join(HERE, "truncated_frame_test")

# Protocol constants, mirroring Communication.h.
RESPONSE_CHARACTER_CRC32_DISABLED = 252


def encode_size_byte(total_packet_size: int) -> int:
    """Size byte encoding: value << 1, LSB set to 1 to mark a valid first byte."""
    return (total_packet_size << 1) | 1


# A reply that starts and never finishes: one valid size byte claiming a
# 10-byte packet, then silence. This is what a collision, a motor rebooting
# mid-reply, or noise on an unbiased RS485 pair looks like to the receiver.
TRUNCATED_FIRST_BYTE = bytes([encode_size_byte(10)])

# A reply that gets FURTHER than one byte but still never finishes: it declares
# a 10-byte packet and delivers only 4. This is the case that decides whether
# fixing the hang is enough on its own, or whether the leftover bytes poison the
# NEXT command - i.e. whether a freeze was merely traded for a stuck error loop.
PARTIAL_FRAME = bytes([
    encode_size_byte(10),
    RESPONSE_CHARACTER_CRC32_DISABLED,
    0x00,
    0xAA,
])

# A complete, well-formed 'Get status' reply with CRC32 disabled:
#   size(1) + responseChar(1) + errorCode(1) + payload(3) = 6 bytes total.
# The 3-byte payload is uint16 statusFlags + uint8 fatalErrorCode.
GOOD_REPLY = bytes([
    encode_size_byte(6),
    RESPONSE_CHARACTER_CRC32_DISABLED,
    0x00,               # no error
    0x02, 0x00,         # statusFlags = 0x0002 (MOSFETs enabled)
    0x00,               # fatalErrorCode = 0
])


def drain(fd, seconds=0.25):
    """Read and discard whatever is pending, without blocking forever."""
    got = b""
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            chunk = os.read(fd, 1024)
        except BlockingIOError:
            time.sleep(0.01)
            continue
        except OSError:
            break
        if chunk:
            got += chunk
            deadline = time.time() + 0.05   # keep reading while data flows
        else:
            time.sleep(0.01)
    return got


def wait_for_command(fd, timeout=5.0):
    """Block until the binary sends us a command frame. Proves it is up and
    has finished configuring termios, so our reply cannot race its setup."""
    deadline = time.time() + timeout
    got = b""
    while time.time() < deadline:
        try:
            chunk = os.read(fd, 1024)
        except BlockingIOError:
            time.sleep(0.005)
            continue
        except OSError:
            break
        if chunk:
            got += chunk
            # A command frame is short; once bytes stop arriving we have it all.
            time.sleep(0.05)
            got += drain(fd, 0.05)
            return got
        time.sleep(0.005)
    return got


def run_case(case: str) -> bool:
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    os.set_blocking(master_fd, False)

    # "partial" exercises the same two-round shape as "resync"; only the peer
    # script differs, so the binary runs the same case.
    binary_case = "two_rounds" if case == "partial" else case
    proc = subprocess.Popen(
        [BINARY, slave_name, binary_case],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    # The child has its own descriptor now; holding ours open would keep the
    # master from ever seeing EOF.
    os.close(slave_fd)

    try:
        cmd = wait_for_command(master_fd)
        if not cmd:
            print(f"  [{case}] the binary never sent a command - setup problem, not a library result")
            proc.kill()
            return False

        if case == "truncated":
            os.write(master_fd, TRUNCATED_FIRST_BYTE)
            # ...and then deliberately nothing more, ever.

        elif case == "silent":
            pass  # send nothing at all

        elif case in ("resync", "partial"):
            os.write(master_fd, TRUNCATED_FIRST_BYTE if case == "resync" else PARTIAL_FRAME)
            # Round 1 must time out. Then the caller is expected to drain stale
            # bytes before retrying - the documented recovery - so we mimic a
            # clean bus before answering round 2 properly.
            cmd2 = wait_for_command(master_fd, timeout=10.0)
            if not cmd2:
                print(f"  [{case}] the binary never sent its second command")
                proc.kill()
                return False
            os.write(master_fd, GOOD_REPLY)

        try:
            out, _ = proc.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
            print(f"  [{case}] HUNG: the runner's own timeout fired")
            return False

        rc = proc.returncode
        for line in (out or "").strip().splitlines():
            print(f"    {line}")

        if rc == 42:
            print(f"  [{case}] HUNG - the receive path never returned (the defect)")
            return False
        if rc != 0:
            print(f"  [{case}] FAILED (exit {rc})")
            return False
        print(f"  [{case}] OK")
        return True

    finally:
        try:
            os.close(master_fd)
        except OSError:
            pass
        if proc.poll() is None:
            proc.kill()


def main():
    if not os.path.exists(BINARY):
        print(f"Binary not found: {BINARY}")
        print("Build it first:  ./build_truncated_frame_test.sh")
        return 2

    cases = sys.argv[1:] or ["silent", "truncated", "resync", "partial"]
    print("Transport-layer receive-path tests (no hardware required)")
    results = {}
    for case in cases:
        print(f"\n-- case: {case}")
        results[case] = run_case(case)

    print("\n==== SUMMARY ====")
    for case, ok in results.items():
        print(f"  {case:<12} {'PASS' if ok else 'FAIL'}")
    failed = [c for c, ok in results.items() if not ok]
    if failed:
        print(f"\n{len(failed)} case(s) failed: {', '.join(failed)}")
        return 1
    print("\nAll cases passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
