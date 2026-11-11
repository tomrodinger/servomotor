"""Timed moves directly after nonzero immediate targets, without a reset."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parent
os.environ['XDG_CONFIG_HOME'] = str(OUT / '.serial_config')
sys.path.insert(0, str(ROOT / 'python_programs'))
import servomotor
from test_get_pid_constants import reset

PORT = '/dev/cu.usbserial-113210'
UID = int('04245C407D26CCD0', 16)
report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(), 'port': PORT,
          'unique_id': f'{UID:016X}', 'passed': False, 'moves': [], 'cleanup_passed': False,
          'requested_current_limits_internal_units': [50, 50]}
servomotor.set_serial_port(PORT)
servomotor.open_serial_port()
motor = servomotor.M3(UID, position_unit='encoder_counts', time_unit='seconds', verbose=0)
try:
    assert motor.get_product_info()[4] == UID and motor.get_firmware_version() == [[0, 0, 17, 0], 0]
    reset(motor)
    motor.set_maximum_motor_current(50, 50)
    motor.go_to_closed_loop()
    time.sleep(0.2)
    motor.zero_position()
    counts = motor.get_product_specs()[1]
    step = round(counts / 360)
    gains = motor.get_pid_constants()
    before = motor.get_communication_statistics(0)
    for crc in (True, False):
        motor.crc32_control(int(crc))
        motor.set_crc32_enabled(crc)
        for combined in (False, True):
            for index in range(4):
                start = step if index & 1 else -step
                target = -start
                if combined:
                    motor.set_immediate_position_target_and_get_position(start)
                else:
                    motor.set_immediate_position_target(start)
                assert motor.get_position() == start and motor.get_n_queued_items() == 0
                time.sleep(0.15)
                # No reset/zero operation between the immediate command and
                # the timed move: its planner must start at the new reference.
                motor.go_to_position(target, 0.35)
                deadline = time.monotonic() + 2
                while motor.get_n_queued_items():
                    assert time.monotonic() < deadline
                    time.sleep(0.02)
                actual_reference = motor.get_position()
                assert abs(actual_reference - target) <= 16
                time.sleep(0.1)
                measured = motor.get_hall_sensor_position()
                assert abs(measured - target) < counts / 720
                assert motor.get_status() == [6, 0]
                item = {'command': 50 if combined else 49, 'crc32': crc,
                        'immediate_reference': start, 'timed_target': target,
                        'final_reference': actual_reference, 'final_measured': measured}
                report['moves'].append(item)
                print('PASS', item, flush=True)
    assert motor.get_pid_constants() == gains
    after = motor.get_communication_statistics(0)
    assert after == before
    report.update(communication_statistics_before=before, communication_statistics_after=after, passed=True)
except Exception as error:
    report['error'] = f'{type(error).__name__}: {error}'
    raise
finally:
    try:
        reset(motor)
        assert motor.get_status() == [0, 0] and motor.get_n_queued_items() == 0
        report['cleanup_passed'] = True
    finally:
        servomotor.close_serial_port()
        (OUT / 'planner_followup.json').write_text(json.dumps(report, indent=2) + '\n')
print('PASSED: 16 timed moves directly after nonzero immediate references, both commands/CRC modes')
