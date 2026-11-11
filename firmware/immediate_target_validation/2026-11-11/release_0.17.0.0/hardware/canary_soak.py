"""Mixed-command soak on the already upgraded canary only; no flashing."""
from datetime import datetime, timezone
import json
import math
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
          'unique_id': f'{UID:016X}', 'passed': False, 'phases': [], 'cleanup_passed': False,
          'requested_current_limits_internal_units': [50, 50]}
servomotor.set_serial_port(PORT)
servomotor.open_serial_port()
motor = servomotor.M3(UID, position_unit='encoder_counts', verbose=0)
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
    for crc in (True, False):
        motor.crc32_control(int(crc))
        motor.set_crc32_enabled(crc)
        motor.get_debug_values()  # Reset interval profiler maxima.
        before = motor.get_communication_statistics(0)
        latencies = []
        phase = {'crc32': crc, 'command_49': 0, 'command_50': 0, 'command_37': 0}
        started = time.monotonic()
        for i in range(10000):
            # Bounded sine and alternating references exercise changes of
            # direction and repeated identical targets without queued motion.
            target = round(step * math.sin(i * 0.047)) if i % 3 else (step if i & 1 else -step)
            began = time.perf_counter_ns()
            if i % 10 == 0:
                motor.set_immediate_position_target(target)
                phase['command_49'] += 1
            else:
                measured = motor.set_immediate_position_target_and_get_position(target)
                assert isinstance(measured, int) and abs(measured) < 4 * step
                phase['command_50'] += 1
            latencies.append((time.perf_counter_ns() - began) / 1e6)
            if i % 100 == 0:
                state = motor.get_comprehensive_position()
                phase['command_37'] += 1
                assert state[0] == target and abs(state[1]) < 4 * step
                assert motor.get_n_queued_items() == 0 and motor.get_status() == [6, 0]
            if i % 1000 == 999:
                print('CRC', crc, 'completed', i + 1, '/ 10000', flush=True)
        # Holding the last reference must continue after host updates stop.
        time.sleep(0.4)
        assert motor.get_position() == target
        settled = motor.get_hall_sensor_position()
        assert abs(settled - target) < counts / 720
        assert motor.get_pid_constants() == gains
        after = motor.get_communication_statistics(0)
        assert after == before
        debug = motor.get_debug_values()
        assert debug[10] < 32
        latencies.sort()
        phase.update(elapsed_seconds=time.monotonic() - started,
                     latency_ms={'min': latencies[0], 'median': latencies[len(latencies)//2],
                                 'p95': latencies[int(len(latencies)*0.95)], 'max': latencies[-1]},
                     communication_statistics_before=before, communication_statistics_after=after,
                     peak_control_isr_us=debug[10], settled_error_counts=settled-target)
        report['phases'].append(phase)
        (OUT / 'canary_soak.json').write_text(json.dumps(report, indent=2) + '\n')
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
        (OUT / 'canary_soak.json').write_text(json.dumps(report, indent=2) + '\n')
print('PASSED: mixed-command canary soak, both CRC modes, command-37 interleaving, hold and cleanup')
