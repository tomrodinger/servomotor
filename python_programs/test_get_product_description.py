#!/usr/bin/env python3
"""
Per-command test for "Get product description" (cmd 24).

Cmd 24 has no input and returns a null-terminated string baked into the
firmware at build time (the PRODUCT_DESCRIPTION compile-time string).

What this test verifies:
  1. The response is a string.
  2. After stripping the trailing NUL the description is non-empty.
  3. Every character is printable ASCII (no stray control bytes) — a real
     sanity check on the response framing and the device side string
     buffer.
  4. The value is static: a second read returns the exact same string.

This test deliberately does *not* assert a specific product description
text — that string is set per-build and asserting a particular value
would couple the test to one product/firmware combo. The shape checks
above catch the realistic failure modes (mis-framed response, corrupted
buffer, returning garbage).
"""

import argparse
import string
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
PRINTABLE = set(string.printable) - set("\x0b\x0c")  # plain printable, no vertical tab / form feed


def normalize_description(raw):
    """Coerce the response into a plain Python str and strip the trailing NUL."""
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="strict")
    if not isinstance(raw, str):
        raise AssertionError(f"expected str/bytes, got {type(raw).__name__}: {raw!r}")
    return raw.rstrip("\x00")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get product description' (cmd 24).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Reading product description...")
            raw = motor.get_product_description()
            desc = normalize_description(raw)
            print(f"  raw={raw!r}")
            print(f"  description (NUL stripped) = {desc!r}")

            if len(desc) == 0:
                raise AssertionError("product description is empty after stripping NUL")
            bad = [(i, c) for i, c in enumerate(desc) if c not in PRINTABLE]
            if bad:
                raise AssertionError(f"product description contains non-printable bytes at "
                                     f"positions {bad!r}")

            print("Re-reading; the description must be identical (static)...")
            raw2 = motor.get_product_description()
            desc2 = normalize_description(raw2)
            if desc2 != desc:
                raise AssertionError(f"product description changed between reads: "
                                     f"{desc!r} -> {desc2!r}")

            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
