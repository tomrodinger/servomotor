"""Run the established canary tests sequentially on the identity-checked rack."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from rack_canary import OUT, ROOT, PORT, UID, read_motor, verify_adapter
import servomotor
from test_get_pid_constants import reset

os.environ['XDG_CONFIG_HOME'] = str(OUT / '.serial_config')
report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(), 'port': PORT,
          'unique_id': UID, 'firmware': '0.17.0.0', 'passed': False, 'stages': []}
stages = [
    ('acceptance', [sys.executable, str(ROOT / 'python_programs/test_immediate_position_target.py'),
                    '-p', PORT, '-a', UID, '--motion', '--samples', '10000',
                    '--protocol-edge-cases', '--report', str(OUT / 'canary_extended.json')]),
    ('planner', [sys.executable, str(OUT / 'planner_followup.py')]),
    ('cpp', ['/tmp/servomotor-release017-cpp', PORT, UID]),
    ('soak', [sys.executable, str(OUT / 'canary_soak.py')]),
    ('timing', [sys.executable, str(OUT / 'timing_windows.py')]),
]
try:
    assert json.loads((OUT / 'upgrade_canary.json').read_text())['passed']
    for name, command in stages:
        verify_adapter()
        print('START', name, flush=True)
        started = time.monotonic()
        with (OUT / (name + '.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=240)
        item = {'stage': name, 'returncode': result.returncode, 'elapsed_seconds': time.monotonic()-started}
        report['stages'].append(item)
        (OUT / 'physical_run.json').write_text(json.dumps(report, indent=2)+'\n')
        assert result.returncode == 0, name + ' failed; see ' + str(OUT / (name + '.log'))
        print('PASS', name, round(item['elapsed_seconds'], 2), 'seconds', flush=True)
    report['passed'] = True
except Exception as error:
    report['error'] = f'{type(error).__name__}: {error}'
    # Each stage has its own cleanup. Also recover safely if a child was killed
    # by its timeout before executing that cleanup.
    try:
        verify_adapter()
        servomotor.set_serial_port(PORT)
        servomotor.open_serial_port()
        try:
            current = read_motor(UID)
            assert current['firmware'] == [[0,0,17,0],0]
            motor = servomotor.M3(int(UID,16), verbose=0)
            if current['status'][1] == 0:
                motor.emergency_stop()
            reset(motor)
            report['failure_cleanup'] = read_motor(UID)
            assert report['failure_cleanup']['status'] == [0,0]
            assert report['failure_cleanup']['queue_depth'] == 0
        finally:
            servomotor.close_serial_port()
    except Exception as cleanup_error:
        report['cleanup_error'] = str(cleanup_error)
    raise
finally:
    (OUT / 'physical_run.json').write_text(json.dumps(report, indent=2)+'\n')
print('PASSED: all five physical canary stages', flush=True)
