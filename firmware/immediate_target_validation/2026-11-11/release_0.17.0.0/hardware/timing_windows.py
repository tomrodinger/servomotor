"""Separate transition/steady-state timing using existing canary telemetry."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import statistics
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
          'unique_id': f'{UID:016X}', 'passed': False, 'cleanup_passed': False,
          'disabled_idle': [], 'steady_retargeting': [],
          'metric_note': 'Existing period profiler samples near ISR end, after motor phase calculation. End-to-end intervals include changes in calculation duration; they are not ISR-entry timestamp intervals. Sample distributions below are sampled telemetry/window maxima, not a distribution of every control tick.'}

def sample(motor):
    debug = motor.get_debug_values()
    return {'calculation_us': debug[9], 'calculation_max_us': debug[10],
            'end_to_end_interval_us': debug[19], 'end_to_end_interval_max_us': debug[20]}

def stats(values):
    return {'min': min(values), 'mean': statistics.mean(values), 'max': max(values)}

servomotor.set_serial_port(PORT)
servomotor.open_serial_port()
motor = servomotor.M3(UID, position_unit='encoder_counts', verbose=0)
try:
    assert motor.get_product_info()[4] == UID and motor.get_firmware_version() == [[0, 0, 17, 0], 0]
    reset(motor)
    time.sleep(0.2)
    sample(motor)  # Clear startup maxima.
    for index in range(20):
        time.sleep(0.02)
        report['disabled_idle'].append(sample(motor))
    motor.set_maximum_motor_current(50, 50)
    sample(motor)  # Isolate the enable/closed-loop transition.
    motor.go_to_closed_loop()
    time.sleep(0.2)
    report['enable_transition'] = sample(motor)
    motor.zero_position()
    step = round(motor.get_product_specs()[1] / 360)
    for index in range(100):
        motor.set_immediate_position_target_and_get_position(0)
    sample(motor)  # Exclude enable/zero/warmup from steady windows.
    before = motor.get_communication_statistics(0)
    for window in range(30):
        for index in range(100):
            motor.set_immediate_position_target_and_get_position(step if index & 1 else -step)
        item = sample(motor)
        assert item['calculation_max_us'] < 32
        report['steady_retargeting'].append(item)
        print('Window', window + 1, item, flush=True)
    after = motor.get_communication_statistics(0)
    assert after == before
    report['communication_statistics_before'] = before
    report['communication_statistics_after'] = after
    for phase in ('disabled_idle', 'steady_retargeting'):
        report[phase + '_summary'] = {key: stats([row[key] for row in report[phase]])
                                     for key in report[phase][0]}
    report['steady_target_updates'] = 3000
    report['warmup_target_updates'] = 100
    report['passed'] = True
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
        (OUT / 'timing_windows.json').write_text(json.dumps(report, indent=2) + '\n')
print('PASSED: separate disabled, enable-transition and 30 steady-retargeting timing windows')
