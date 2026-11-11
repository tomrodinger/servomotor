#!/usr/bin/env python3
"""M17 bench acceptance for compact commands 49/50, stable 0.17.0.0 or development 0.17.0.3+ (moves one motor).

Use a unique ID on shared-alias racks. Requires explicit --motion. Resets the
selected motor, uses modest current limits and +/-1 degree steps, then restores
startup defaults with outputs disabled. --protocol-edge-cases adds malformed
requests, safety/deviation limits and test-mode rejection on the selected motor.
"""
import argparse
import json
from pathlib import Path
import random
import struct
import time
import zlib

import servomotor
from servomotor import communication
from test_get_pid_constants import CaptureSerial, require, reset


def compact_position_reply_supported(firmware):
    version, in_bootloader = firmware
    numeric = tuple(reversed(version))
    return not in_bootloader and (numeric == (0, 17, 0, 0) or numeric >= (0, 17, 0, 3))


def fatal(motor, operation, code, report):
    try:
        operation()
    except servomotor.FatalError as error:
        require(error.args == (code,), f'expected fatal {code}, got {error}')
    else:
        raise AssertionError(f'operation did not raise fatal {code}')
    status = motor.get_status()
    require(status == [0, code], f'fatal error did not disable outputs: {status}')
    report.setdefault('fatal_errors', []).append(code)


def wire(motor, target, combined):
    original = communication.ser
    capture = CaptureSerial(original)
    communication.ser = capture
    try:
        reply = (motor.set_immediate_position_target_and_get_position(target) if combined else
                 motor.set_immediate_position_target(target))
    finally:
        communication.ser = original
    crc = motor.crc32_enabled
    body = b'\xfe' + struct.pack('<Q', motor.alias_or_unique_id) + bytes([50 if combined else 49]) + struct.pack('<q', target)
    expected_tx = bytes([((1 + len(body) + (4 if crc else 0)) << 1) | 1]) + body
    if crc:
        expected_tx += struct.pack('<I', zlib.crc32(expected_tx))
    require(bytes(capture.tx) == expected_tx, 'wrong target request bytes')
    if combined:
        require(isinstance(reply, int), f'wrong measured position: {reply}')
        payload = struct.pack('<q', reply)
        expected_rx = bytes([31 if crc else 23, 253 if crc else 252, 0]) + payload
    else:
        expected_rx = bytes([13 if crc else 5, 253 if crc else 252])
    if crc:
        expected_rx += struct.pack('<I', zlib.crc32(expected_rx))
    require(bytes(capture.rx) == expected_rx, 'wrong target response bytes/order/CRC')
    return {'command': 50 if combined else 49, 'crc32': crc, 'tx': capture.tx.hex(), 'rx': capture.rx.hex()}


def exercise_extra(motor, step, count, protocol_edges, report):
    motor.set_immediate_position_target(0)
    time.sleep(0.15)
    report['wire_samples'] = []
    for crc in (True, False):
        if not crc:
            motor.crc32_control(0)
            motor.set_crc32_enabled(False)
        for combined in (False, True):
            report['wire_samples'].append(wire(motor, 0, combined))
        motor.crc32_control(1)
        motor.set_crc32_enabled(True)
    start_stats = motor.get_communication_statistics(0)
    report['profiler_before'] = motor.get_debug_values()
    rng = random.Random(motor.alias_or_unique_id)
    before_gains = motor.get_pid_constants()
    started = time.monotonic()
    samples = []
    crc_transactions = {'enabled': 0, 'disabled': 0}
    for index in range(count):
        if count > 1 and index == count // 2:
            motor.crc32_control(0)
            motor.set_crc32_enabled(False)
        target = rng.randint(-step, step)
        measured = motor.set_immediate_position_target_and_get_position(target)
        crc_transactions['enabled' if motor.crc32_enabled else 'disabled'] += 1
        require(isinstance(measured, int), f'invalid stress measurement: {measured}')
        if index < 3 or index >= count - 3:
            samples.append({'target': target, 'measured': measured})
    report['stress'] = {'transactions': count, 'elapsed_seconds': round(time.monotonic() - started, 4),
                        'crc_transactions': crc_transactions, 'position_samples': samples}
    require(motor.get_position() == target, 'last stress reference was not applied exactly')
    require(motor.get_n_queued_items() == 0 and motor.get_status() == [6, 0], 'stress queue/output state changed')
    report['profiler_after'] = motor.get_debug_values()
    if tuple(reversed(report['firmware'][0])) >= (0, 17, 0, 2):
        require(report['profiler_after'][10] < 32, 'control ISR exceeded its 32-us period')
    report['communication_statistics_after_stress'] = motor.get_communication_statistics(0)
    require(report['communication_statistics_after_stress'] == start_stats, 'communication errors during retargeting')
    require(motor.get_pid_constants() == before_gains, 'retargeting changed PID gains')
    motor.crc32_control(1)
    motor.set_crc32_enabled(True)
    motor.set_immediate_position_target(0)
    time.sleep(0.2)
    motor.emergency_stop()
    # Emergency stop disables outputs and clears the queue, retaining the
    # closed-loop mode bit. Reset, separately, returns to open-loop mode.
    require(motor.get_status() == [4, 0] and motor.get_n_queued_items() == 0, 'emergency stop failed')
    fatal(motor, lambda: motor.set_immediate_position_target_and_get_position(0), 55, report)
    reset(motor)
    motor.set_maximum_motor_current(50, 50)
    motor.go_to_closed_loop()
    motor.zero_position()
    if not protocol_edges:
        return
    for command in (49, 50):
        for crc in (True, False):
            if not crc:
                motor.crc32_control(0)
                motor.set_crc32_enabled(False)
            fatal(motor, lambda: communication.send_command(motor.alias_or_unique_id, command, b'\0' * 7,
                                                           crc32_enabled=crc, verbose=0), 51, report)
            reset(motor)
            motor.set_maximum_motor_current(50, 50)
            motor.go_to_closed_loop()
            motor.zero_position()
        method = motor.set_immediate_position_target if command == 49 else motor.set_immediate_position_target_and_get_position
        motor.set_safety_limits(-step, step)
        fatal(motor, lambda: method(3 * step), 27, report)
        reset(motor)
        motor.set_maximum_motor_current(50, 50)
        motor.go_to_closed_loop()
        motor.zero_position()
        motor.set_max_allowable_position_deviation(step)
        fatal(motor, lambda: method(3 * step), 45, report)
        reset(motor)
        motor.set_maximum_motor_current(50, 50)
        motor.go_to_closed_loop()
        motor.zero_position()
        motor.test_mode(3)
        fatal(motor, lambda: method(0), 19, report)
        reset(motor)
        motor.set_maximum_motor_current(50, 50)
        motor.go_to_closed_loop()
        motor.zero_position()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-p', '--port', required=True)
    parser.add_argument('-a', '--alias', required=True, help='16-digit unique ID')
    parser.add_argument('--motion', action='store_true', required=True, help='Authorize the small bench moves')
    parser.add_argument('--samples', type=int, default=250, help='Repeated random target/position transactions within +/-1 degree')
    parser.add_argument('--protocol-edge-cases', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error('--samples must be positive')
    if len(args.alias) != 16 or any(c not in '0123456789abcdefABCDEF' for c in args.alias):
        parser.error('pass an exact 16-digit motor unique ID')
    uid = int(args.alias, 16)
    servomotor.set_serial_port(args.port)
    motor = servomotor.M3(uid, position_unit='encoder_counts', time_unit='seconds', verbose=0)
    report = {'unique_id': args.alias.upper(), 'passed': False, 'states': [],
              'requested_current_limits_internal_units': [50, 50]}
    started = time.monotonic()
    opened = False
    compatible = False
    try:
        servomotor.open_serial_port(); opened = True
        info = motor.get_product_info()
        version = motor.get_firmware_version()
        require(info[0].strip() == 'M17' and info[4] == uid, 'M17 identity mismatch')
        require(compact_position_reply_supported(version), 'compact command 50 requires stable 0.17.0.0 or development 0.17.0.3+')
        compatible = True
        report.update(product_info=info, firmware=version)
        reset(motor)
        fatal(motor, lambda: motor.set_immediate_position_target(0), 13, report)
        reset(motor)
        motor.set_maximum_motor_current(50, 50)
        motor.go_to_closed_loop()
        time.sleep(0.2)
        require(motor.get_status() == [6, 0], 'enabled closed-loop entry failed')
        motor.zero_position()
        counts = motor.get_product_specs()[1]
        step = round(counts / 360)
        motor.set_immediate_position_target(step)
        require(motor.get_position() == step and motor.get_n_queued_items() == 0, 'standalone target/queue mismatch')
        gains = motor.get_pid_constants()
        for target in (-step, step, 0):
            measured = motor.set_immediate_position_target_and_get_position(target)
            require(isinstance(measured, int), f'invalid measured position: {measured}')
            report['states'].append({'target': target, 'measured': measured})
            require(motor.get_position() == target, 'reference was not applied exactly')
            require(motor.get_n_queued_items() == 0 and motor.get_status() == [6, 0], 'queue/output state changed')
            time.sleep(0.15)
            measured = motor.get_hall_sensor_position()
            report.setdefault('settled_positions', []).append({'target': target, 'measured': measured, 'error_counts': measured - target})
            require(abs(measured - target) < counts / 720, 'tracking error exceeds 0.5 degrees')
        require(motor.get_pid_constants() == gains, 'target commands changed gains')
        exercise_extra(motor, step, args.samples, args.protocol_edge_cases, report)
        # Subsequent legacy trajectories must use the new planner baseline.
        motor.go_to_position(step, 0.5)
        deadline = time.monotonic() + 2
        while motor.get_n_queued_items():
            require(time.monotonic() < deadline, 'legacy move did not complete')
            time.sleep(0.02)
        require(abs(motor.get_position() - step) <= 16, 'legacy move did not start from immediate target')
        # Keep a stationary item queued while testing the queue guard.
        motor.move_with_velocity(0, 1.0)
        require(motor.get_n_queued_items() > 0, 'guard test queue empty')
        fatal(motor, lambda: motor.set_immediate_position_target_and_get_position(0), 8, report)
        reset(motor)
        motor.set_maximum_motor_current(50, 50)
        motor.go_to_closed_loop(); motor.disable_mosfets()
        fatal(motor, lambda: motor.set_immediate_position_target(0), 55, report)
        report['passed'] = True
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if opened and compatible:
            try:
                reset(motor)
                require(motor.get_status() == [0, 0] and motor.get_n_queued_items() == 0, 'cleanup failed')
                report['cleanup_passed'] = True
            except Exception as error:
                report.update(passed=False, cleanup_passed=False, cleanup_error=str(error))
        if opened:
            servomotor.close_serial_port()
        report['elapsed_seconds'] = round(time.monotonic() - started, 3)
        if args.report:
            args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    print('PASSED' if report['passed'] else 'FAILED')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
