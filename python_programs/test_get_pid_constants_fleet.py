#!/usr/bin/env python3
"""Run PID readback hardware tests on each unique ID in a saved inventory.

Inventory JSON must contain a 'devices' array with 'unique_id' hex strings,
as recorded in firmware/pid_readback_validation/2026-10-29/inventory_before.json.
Only listed devices are reset/retuned; --motion additionally moves each one.
The broadcast check sends only the read-only Get PID constants command.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import servomotor
from servomotor import communication
from test_get_pid_constants import DEFAULTS, add_test_arguments, require, run_test


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-p', '--port', required=True)
    parser.add_argument('--inventory', type=Path, required=True)
    parser.add_argument('--expected-count', type=int, required=True)
    add_test_arguments(parser)
    args = parser.parse_args()
    if args.samples < 0 or args.repeat < 1 or args.expected_count < 1:
        parser.error('invalid sample/repeat/count argument')
    inventory = json.loads(args.inventory.read_text())
    uids = [int(d['unique_id'], 16) for d in inventory['devices']]
    require(len(uids) == len(set(uids)) == args.expected_count, 'inventory count/uniqueness mismatch')
    require(all(255 < uid <= 0xffffffffffffffff for uid in uids), 'inventory must use unique IDs, not aliases')
    report = {"port": args.port, "expected_count": args.expected_count, "devices": [], "passed": False}
    started = time.monotonic()

    def save():
        if args.report:
            args.report.write_text(json.dumps(report, indent=2) + '\n')

    servomotor.set_serial_port(args.port)
    try:
        servomotor.open_serial_port()
        for index, uid in enumerate(uids, 1):
            print(f"MOTOR {index}/{len(uids)} {uid:016X}", flush=True)
            motor = servomotor.M3(uid, time_unit='seconds', position_unit='encoder_counts', verbose=0)
            require(motor.get_product_info()[4] == uid, 'addressed motor identity mismatch')
            result = run_test(motor, args.samples, args.repeat, args.motion, args.protocol_edge_cases)
            report['devices'].append(result)
            save()
            print(f"RESULT {uid:016X}: {'PASS' if result['passed'] else 'FAIL'} ({result['checks']} checks)", flush=True)
            require(result['passed'], f"motor {uid:016X} failed: {result}")
        servomotor.flush_receive_buffer()
        communication.send_command(255, 48, b'', crc32_enabled=True, verbose=0)
        # Read with the normal 1.2 s timeout; any reply to this broadcast is a failure.
        require(communication.ser.read(1) == b'', 'broadcast getter unexpectedly replied')
        report['broadcast_silent'] = True
        # The broadcast must not disturb later addressed requests.
        for uid, result in zip(uids, report['devices']):
            motor = servomotor.M3(uid, verbose=0)
            require(motor.get_pid_constants() == DEFAULTS[result['product']], 'broadcast changed gains')
            require(motor.get_status() == [0, 0], 'broadcast changed final status')
        report['passed'] = True
    except Exception as error:
        report['error'] = f"{type(error).__name__}: {error}"
    finally:
        servomotor.close_serial_port()
        report['elapsed_seconds'] = round(time.monotonic() - started, 3)
        save()
    print(f"{sum(d['passed'] for d in report['devices'])}/{len(uids)} motors passed", flush=True)
    if not report['passed']:
        print(report.get('error', 'incomplete run'))
    print('PASSED' if report['passed'] else 'FAILED')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
