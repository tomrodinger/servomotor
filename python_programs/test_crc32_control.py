#!/usr/bin/env python3
"""
Per-command test for "CRC32 control" (cmd 46).

Cmd 46 input: (u8 enableCrc32) — 1 enables CRC32 checking, 0 disables it.
Returns a success response. The firmware boots with CRC32 enabled
(RS485.c: crc32_enabled = 1) and a system reset restores that default.

The firmware sets the new CRC32 state *before* transmitting the reply, so the
reply to a disable command comes back WITHOUT a CRC and the reply to an enable
command comes back WITH a CRC. The library's response parser auto-detects CRC
presence from the response character (253 = with CRC, 252 = without), so the
receive side handles either; only the *outgoing* framing has to match the
firmware's current state.

The high-level M3 methods always append a CRC, so this test drives the bus
through servomotor.communication.execute_command with an explicit
`crc32_enabled` flag per call, matching the firmware's current state.

What this test verifies:
  1. With CRC enabled (default), a normal command sent WITH a CRC succeeds.
  2. Enforcement: with CRC enabled, a command sent WITHOUT a CRC is rejected
     (the firmware drops the malformed packet → the client times out). This
     proves CRC checking is actually on, not ignored.
  3. crc32_control(0) disables checking; afterwards a command sent WITHOUT a
     CRC succeeds.
  4. crc32_control(1) re-enables checking; afterwards a command sent WITH a
     CRC succeeds again.

The device is always left with CRC enabled (the default the rest of the suite
relies on), via a reset that works regardless of the current CRC state.
"""

import argparse
import sys
import time
import servomotor
from servomotor import communication
from servomotor.communication import TimeoutError, CommunicationError, FatalError

GET_STATUS_COMMAND_ID = 16
SYSTEM_RESET_COMMAND_ID = 27
CRC32_CONTROL_COMMAND_ID = 46
RESET_DELAY_S = 1.5


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'CRC32 control' (cmd 46).")
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
    alias = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)
        alias = motor.alias_or_unique_id

        def send(cmd, inputs, crc):
            return communication.execute_command(cmd, inputs, alias_or_unique_id=alias,
                                                 crc32_enabled=crc, verbose=verbose_level)

        def get_status(crc):
            resp = send(GET_STATUS_COMMAND_ID, [], crc)
            if not resp or not resp[0] or len(resp[0]) < 2:
                raise AssertionError(f"get_status returned unusable response: {resp!r}")
            return resp[0]

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting (CRC enabled by default)...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("[1] get_status WITH a CRC (CRC enabled) — must succeed...")
            st = get_status(crc=True)
            print(f"    ok: status={st}")

            print("[2] get_status WITHOUT a CRC (CRC still enabled) — must be rejected...")
            rejected = False
            try:
                st = get_status(crc=False)
            except (TimeoutError, CommunicationError, FatalError) as e:
                print(f"    correctly rejected ({type(e).__name__}).")
                rejected = True
            if not rejected:
                raise AssertionError("a no-CRC command was accepted while CRC checking is enabled — "
                                     "CRC enforcement is not working")

            print("[3] crc32_control(0) to DISABLE (sent WITH a CRC), then get_status WITHOUT a CRC...")
            send(CRC32_CONTROL_COMMAND_ID, [0], crc=True)
            st = get_status(crc=False)
            print(f"    disabled mode ok: status={st}")

            print("[4] crc32_control(1) to RE-ENABLE (sent WITHOUT a CRC), then get_status WITH a CRC...")
            send(CRC32_CONTROL_COMMAND_ID, [1], crc=False)
            st = get_status(crc=True)
            print(f"    re-enabled ok: status={st}")

            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        # Always leave the device with CRC enabled. We may have failed while CRC
        # was disabled, so try a no-CRC reset first (works if disabled), then a
        # with-CRC reset (works if enabled). A reset restores the CRC-enabled default.
        if alias is not None:
            for crc in (False, True):
                try:
                    communication.execute_command(SYSTEM_RESET_COMMAND_ID, [],
                                                  alias_or_unique_id=alias, crc32_enabled=crc, verbose=0)
                    break
                except Exception:
                    continue
            time.sleep(RESET_DELAY_S)
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
