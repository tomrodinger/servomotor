#!/usr/bin/env python3
"""Hardware test for Get PID constants (48), released in firmware 0.16.0.0
(also supports the validated 0.15.12.1 development build).

Resets the addressed motor and writes raw gain edge cases with outputs disabled.
Restores startup defaults, CRC32 enabled and outputs disabled on exit. --motion
additionally turns an unloaded M17 by 45 degrees and back while reading/retuning
near its normal gains. --protocol-edge-cases deliberately tests fatal size errors
and resets afterward. On a shared-alias rack, pass a 16-digit unique ID with -a.
"""

import argparse
import json
from pathlib import Path
import random
import struct
import sys
import time
import zlib

import servomotor
from servomotor import communication

DEFAULTS = {"M1": [5000, 1, 5000], "M2": [20000, 1, 100000],
            "M17": [2000, 5, 175000], "M23": [500, 1, 1000]}
RESET_DELAY = 1.5


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def reset(motor):
    motor.system_reset()
    motor.set_crc32_enabled(True)
    time.sleep(RESET_DELAY)  # Keep the bus quiet through the bootloader window.
    servomotor.flush_receive_buffer()


class CaptureSerial:
    """Capture actual UART bytes while delegating I/O to the real serial port."""
    def __init__(self, link):
        self.link, self.tx, self.rx = link, bytearray(), bytearray()

    def write(self, data):
        self.tx.extend(data)
        return self.link.write(data)

    def read(self, size):
        data = self.link.read(size)
        self.rx.extend(data)
        return data

    def __getattr__(self, name):
        return getattr(self.link, name)


def check_wire(motor, gains):
    original = communication.ser
    capture = CaptureSerial(original)
    communication.ser = capture
    try:
        reply = communication.send_command(motor.alias_or_unique_id, 48, b"",
                                           crc32_enabled=motor.crc32_enabled, verbose=0)
    finally:
        communication.ser = original
    payload = struct.pack("<III", *gains)
    require(reply == [payload], f"wrong raw payload: {reply!r}")
    expected = bytes([39 if motor.crc32_enabled else 31,
                      253 if motor.crc32_enabled else 252, 0]) + payload
    if motor.crc32_enabled:
        expected += struct.pack("<I", zlib.crc32(expected))
    require(bytes(capture.rx) == expected, "wrong response length/order/CRC/status")
    address = motor.alias_or_unique_id
    body = (bytes([address]) if address <= 255 else b"\xfe" + struct.pack("<Q", address)) + b"\x30"
    request = bytes([((1 + len(body) + (4 if motor.crc32_enabled else 0)) << 1) | 1]) + body
    if motor.crc32_enabled:
        request += struct.pack("<I", zlib.crc32(request))
    require(bytes(capture.tx) == request, "wrong request address/command/payload/CRC")
    return {"crc32": motor.crc32_enabled, "request": capture.tx.hex(), "response": capture.rx.hex()}


def exercise_motion(motor, defaults, result):
    require(defaults == DEFAULTS["M17"], "--motion currently targets M17 only")
    motor.set_pid_constants(*defaults)
    motor.set_maximum_motor_current(100, 100)
    motor.go_to_closed_loop()
    time.sleep(0.4)
    require(motor.get_status()[0] & 4, "closed loop did not start")
    motor.zero_position()
    counts_per_rotation = motor.get_product_specs()[1]
    target = counts_per_rotation // 8
    safe_gains = [defaults, [1999, 5, 175032], [2001, 5, 175000]]
    initial_reads = result.setdefault("motion_reads", 0)
    result.setdefault("motion_endpoints", [])
    for endpoint in (target, 0):
        motor.trapezoid_move(endpoint - motor.get_position(), 1.5)
        require(motor.get_n_queued_items() > 0, "move never entered the queue")
        deadline = time.monotonic() + 6
        index = 0
        while motor.get_n_queued_items() > 0:
            require(time.monotonic() < deadline, "move did not finish")
            gains = safe_gains[index % len(safe_gains)]
            motor.set_pid_constants(*gains)
            # Require whole triplets after each setting change during motion.
            before = motor.get_n_queued_items()
            require(motor.get_pid_constants() == gains, "torn/stale gains while moving")
            after = motor.get_n_queued_items()
            require(after <= before, "reading PID constants added a queue entry")
            status = motor.get_status()
            require(status[1] == 0 and status[0] & 6 == 6, "read changed enabled/closed-loop state")
            result["motion_reads"] += 1
            index += 1
            time.sleep(0.015)
        motor.set_pid_constants(*defaults)
        time.sleep(0.25)
        commanded = motor.get_position()
        measured = motor.get_hall_sensor_position()
        result["motion_endpoints"].append({"commanded": commanded, "measured": measured})
        # Trapezoid planning uses fixed-point arithmetic; the existing motion
        # regression harness allows 16 counts of endpoint quantization.
        require(abs(commanded - endpoint) <= 16,
                f"commanded endpoint {commanded}, expected {endpoint} within 16 counts")
        require(abs(measured - endpoint) < 8000, f"tracking error {measured - endpoint} counts")
    require(result["motion_reads"] - initial_reads >= 10, "too few in-flight reads")
    motor.disable_mosfets()


def run_test(motor, samples=32, repeat=1, motion=False, protocol_edges=False):
    result = {"address": f"{motor.alias_or_unique_id:016X}", "passed": False,
              "checks": 0, "round_trips": 0, "wire_samples": []}

    def check(condition, message):
        require(condition, message)
        result["checks"] += 1

    defaults = None
    started = time.monotonic()
    # Check compatibility before resetting/changing any settings.
    info = motor.get_product_info()
    version = motor.get_firmware_version()
    result.update(product=info[0].strip(), unique_id=f"{info[4]:016X}", firmware=version)
    require(tuple(reversed(version[0])) >= (0, 15, 12, 1), "firmware has no PID getter")
    defaults = DEFAULTS[result["product"]]
    try:
        for iteration in range(repeat):
            print(f"  {result['unique_id']} pass {iteration + 1}/{repeat}: defaults and raw round trips", flush=True)
            reset(motor)
            check(motor.get_pid_constants() == defaults, "startup defaults differ")
            status = motor.get_status()
            check(status == [0, 0], "expected idle motor with outputs disabled")
            position = motor.get_position()
            rng = random.Random(info[4] + iteration)
            cases = [defaults, [0, 0, 0], [1, 256, 65536], [0xffffffff] * 3,
                     [0x7fffffff, 0x80000000, 0xffffffff], [0xffffffff, 0, 31],
                     [0, 0xffffffff, 32]] + [[rng.getrandbits(32) for _ in range(3)] for _ in range(samples)]
            for gains in cases:
                motor.set_pid_constants(*gains)
                for _ in range(2):
                    check(motor.get_pid_constants() == gains, f"readback differs from {gains}")
                result["round_trips"] += 1
            check(motor.get_status() == status, "idle getter changed output/mode/status")
            check(motor.get_position() == position, "idle getter changed commanded position")
            check(motor.get_n_queued_items() == 0, "idle getter changed queue")

            print(f"  {result['unique_id']}: CRC modes and exact wire bytes", flush=True)
            for crc in (True, False):
                if not crc:
                    motor.crc32_control(0)
                    motor.set_crc32_enabled(False)
                for gains in ([0x01234567, 0x89abcdef, 0xfedcba98], defaults):
                    motor.set_pid_constants(*gains)
                    check(motor.get_pid_constants() == gains, "CRC-mode readback failed")
                    result["wire_samples"].append(check_wire(motor, gains))
                    result["checks"] += 1
                motor.crc32_control(1)
                motor.set_crc32_enabled(True)

            motor.set_pid_constants(1234, 7, 111111)
            reset(motor)
            check(motor.get_pid_constants() == defaults, "nondefault gains persisted across reset")

            if protocol_edges:
                print(f"  {result['unique_id']}: malformed requests and reset recovery", flush=True)
                request = bytes.fromhex(result['wire_samples'][-4]['request'])
                counters = motor.get_communication_statistics(0)
                corrupt = bytearray(request)
                corrupt[-1] ^= 1
                communication.ser.write(corrupt)
                try:
                    communication.get_response(verbose=0)
                except communication.TimeoutError:
                    result['checks'] += 1
                else:
                    raise AssertionError('bad-CRC read request was answered')
                check(motor.get_communication_statistics(0)[0] == counters[0] + 1,
                      'bad CRC did not increment its counter')
                check(motor.get_pid_constants() == defaults, 'bad CRC disturbed the getter')

                # Fragment a valid request, then abandon a partial request and
                # verify recovery after the existing 100 ms receive timeout.
                communication.ser.write(request[:7])
                time.sleep(0.005)
                communication.ser.write(request[7:])
                check(communication.get_response(verbose=0) == struct.pack('<III', *defaults),
                      'fragmented request failed')
                communication.ser.write(request[:7])
                time.sleep(0.2)
                servomotor.flush_receive_buffer()
                check(motor.get_pid_constants() == defaults, 'partial request did not recover')

                for crc, payload in ((True, b"\0"), (False, b"\0" * 12)):
                    if not crc:
                        motor.crc32_control(0)
                        motor.set_crc32_enabled(False)
                    try:
                        communication.send_command(motor.alias_or_unique_id, 48, payload,
                                                   crc32_enabled=crc, verbose=0)
                    except communication.FatalError as error:
                        check(error.args == (51,), f"wrong size error: {error!r}")
                    else:
                        raise AssertionError("non-empty getter payload was accepted")
                    reset(motor)
                    check(motor.get_pid_constants() == defaults, "failed reset recovery")

            if motion:
                print(f"  {result['unique_id']}: 45-degree move and return with live readback", flush=True)
                exercise_motion(motor, defaults, result)
                result["checks"] += 1
        result["passed"] = True
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
    finally:
        try:
            reset(motor)
            check(motor.get_pid_constants() == defaults, "final reset failed to restore defaults")
            check(motor.get_status() == [0, 0], "final state is not idle/disabled/healthy")
            check(motor.get_n_queued_items() == 0, "final queue not empty")
            result["cleanup_passed"] = True
        except Exception as error:
            result["passed"] = False
            result["cleanup_passed"] = False
            result["cleanup_error"] = str(error)
    result["elapsed_seconds"] = round(time.monotonic() - started, 3)
    return result


def add_test_arguments(parser):
    parser.add_argument('--samples', type=int, default=32, help='Random full-range triplets per pass (outputs disabled)')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--motion', action='store_true', help='Move an unloaded M17 by 45 degrees and back')
    parser.add_argument('--protocol-edge-cases', action='store_true', help='Test fatal malformed-payload errors and recovery')
    parser.add_argument('--report', type=Path, help='Write JSON evidence')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-p', '--port')
    parser.add_argument('-P', '--PORT', action='store_true')
    parser.add_argument('-a', '--alias', required=True, help='Alias or 16-digit unique ID; use UID on shared-alias racks')
    parser.add_argument('-v', '--verbose', action='store_true')
    add_test_arguments(parser)
    args = parser.parse_args()
    if args.samples < 0 or args.repeat < 1:
        parser.error('samples must be nonnegative and repeat must be positive')
    servomotor.set_serial_port_from_args(args)
    result = {"passed": False}
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, time_unit='seconds', position_unit='encoder_counts', verbose=2 if args.verbose else 0)
        result = run_test(motor, args.samples, args.repeat, args.motion, args.protocol_edge_cases)
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
    finally:
        servomotor.close_serial_port()
    if args.report:
        args.report.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    print('PASSED' if result['passed'] else 'FAILED')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
