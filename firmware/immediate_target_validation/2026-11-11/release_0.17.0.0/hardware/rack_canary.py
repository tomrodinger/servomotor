"""Read-only rack snapshots and a single identity-checked canary update."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parent
os.environ['XDG_CONFIG_HOME'] = str(OUT / '.serial_config')
sys.path.insert(0, str(ROOT / 'python_programs'))
import servomotor
from servomotor.vendor.serial.tools import list_ports

PORT = '/dev/cu.usbserial-113210'
UID = '04245C407D26CCD0'
VERSION = [[0, 0, 17, 0], 0]

def verify_adapter():
    ports = {p.device: p for p in list_ports.comports()}
    port = ports[PORT]
    assert port.location == '0-1.1.3.2.1'
    assert port.serial_number == '569921bdd691ef11ad9242c1cd3f4b7e'
    owners = subprocess.run(['lsof', '-t', PORT, PORT.replace('/cu.', '/tty.')], capture_output=True, text=True)
    assert not owners.stdout.strip(), 'Rack port already in use: ' + owners.stdout
    return {'port': PORT, 'location': port.location, 'adapter_serial_number': port.serial_number}

def read_motor(uid):
    m = servomotor.M3(int(uid, 16), verbose=0)
    version = m.get_firmware_version()
    info = m.get_product_info()
    assert info[0] == 'M17     ' and info[1] == 3 and info[2] == [0, 5, 1] and info[4] == int(uid, 16)
    return {'unique_id': uid, 'product_info': info, 'firmware': version,
            'status': m.get_status(), 'queue_depth': m.get_n_queued_items(),
            'pid_constants': m.get_pid_constants() if tuple(reversed(version[0])) >= (0, 16, 0, 0) else None,
            'communication_statistics': m.get_communication_statistics(0)}

def snapshot(final=False):
    scope = verify_adapter()
    prior_path = OUT / 'inventory_before.json' if final else ROOT / 'firmware/immediate_target_validation/2026-11-10/copy_only_get_position/hardware/final_verification.json'
    prior = json.loads(prior_path.read_text())
    report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(), **scope,
              'passed': False, 'devices': [], 'nonresponding_historical_ids': []}
    destination = OUT / ('final_verification.json' if final else 'inventory_before.json')
    servomotor.set_serial_port(PORT)
    servomotor.open_serial_port()
    try:
        ordered = sorted(prior['devices'], key=lambda d: (d['unique_id'] != UID, d['unique_id']))
        for before in ordered:
            try:
                item = read_motor(before['unique_id'])
            except servomotor.TimeoutError:
                if final or before['unique_id'] == UID:
                    raise
                report['nonresponding_historical_ids'].append(before['unique_id'])
                print(before['unique_id'], 'historical ID did not respond; no mutation', flush=True)
                servomotor.flush_receive_buffer()
                continue
            report['devices'].append(item)
            destination.write_text(json.dumps(report, indent=2) + '\n')
            assert item['product_info'] == before['product_info']
            if item['unique_id'] == UID:
                assert item['status'][1] == 0 and item['queue_depth'] == 0
            if final:
                assert item['firmware'] == (VERSION if item['unique_id'] == UID else before['firmware'])
                assert item['pid_constants'] == ([2000, 5, 175000] if item['unique_id'] == UID else before['pid_constants'])
                if item['unique_id'] == UID:
                    assert item['status'] == [0, 0]
                    assert item['communication_statistics'] == [0] * 6
                else:
                    assert item['status'] == before['status'] and item['queue_depth'] == before['queue_depth']
            print(item['unique_id'], item['firmware'], item['status'], flush=True)
        report['count'] = len(report['devices'])
        report['passed'] = True
    finally:
        servomotor.close_serial_port()
        destination.write_text(json.dumps(report, indent=2) + '\n')
    return report

def flash():
    verify_adapter()
    manifest = json.loads((OUT.parent / 'summary.json').read_text())
    image = next(i for i in manifest['images'] if i['path'].endswith('_M17_fw0.17.0.0_scc3_hw1.5.firmware'))
    image_path = ROOT / image['path']
    assert hashlib.sha256(image_path.read_bytes()).hexdigest() == image['sha256']
    assert image_path.read_bytes()[:9] == b'M17     \x03'
    for name in ('main.c', 'motor_control.c', 'motor_control.h'):
        rel = 'firmware/Src/' + name
        assert hashlib.sha256((ROOT / rel).read_bytes()).hexdigest() == manifest['source_sha256'][rel]
    prior = json.loads((OUT / 'inventory_before.json').read_text())
    assert prior['passed'] and prior['port'] == PORT
    original = next(d for d in prior['devices'] if d['unique_id'] == UID)
    servomotor.set_serial_port(PORT)
    servomotor.open_serial_port()
    try:
        before = read_motor(UID)
        assert before['firmware'] == original['firmware']
        assert before['product_info'] == original['product_info']
        assert before['status'][1] == 0 and before['queue_depth'] == 0
        # The canary may have been left holding a reference by manual use.
        # Stop and reset only this UID before entering its bootloader.
        motor = servomotor.M3(int(UID, 16), verbose=0)
        motor.emergency_stop()
        motor.system_reset()
        time.sleep(1.5)
        assert motor.get_status() == [0, 0] and motor.get_n_queued_items() == 0
    finally:
        servomotor.close_serial_port()
    report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(), 'port': PORT,
              'unique_id': UID, 'before': before, 'version': '0.17.0.0',
              'image_sha256': image['sha256'], 'passed': False}
    try:
        print('Flashing only', UID, 'to 0.17.0.0', flush=True)
        with (OUT / 'upgrade_canary.log').open('w') as log:
            result = subprocess.run([sys.executable, str(ROOT / 'python_programs/upgrade_firmware.py'),
                                     '-p', PORT, '-a', UID, str(image_path)], cwd=ROOT,
                                    stdout=log, stderr=subprocess.STDOUT, timeout=120)
        assert result.returncode == 0, 'Firmware updater failed'
        time.sleep(1.5)
        servomotor.open_serial_port()
        try:
            report['after'] = read_motor(UID)
            assert report['after']['product_info'] == original['product_info']
            assert report['after']['firmware'] == VERSION
            assert report['after']['status'] == [0, 0] and report['after']['queue_depth'] == 0
            assert report['after']['pid_constants'] == [2000, 5, 175000]
            report['passed'] = True
        finally:
            servomotor.close_serial_port()
    finally:
        (OUT / 'upgrade_canary.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASSED: canary 0.17.0.0, identity verified, disabled/healthy', flush=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('preflight', 'flash', 'postflight'))
    args = parser.parse_args()
    if args.phase == 'flash':
        flash()
    else:
        print('PASSED: snapshot', snapshot(args.phase == 'postflight')['count'], 'motors')
