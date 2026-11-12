#!/usr/bin/env python3
"""Validate a local wheel offline in a fresh environment; never contacts motors.

Usage: python3 validate_built_library.py /path/to/servomotor-*.whl
No network access or package upload is performed. The public command contract
suite is copied outside the source tree so it must use the installed wheel.
"""
import argparse
import email
import json
from pathlib import Path
import shutil
import os
import subprocess
import tempfile
import venv
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('wheel', type=Path)
    args = parser.parse_args()
    wheel = args.wheel.resolve()
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        metadata = email.message_from_bytes(archive.read(next(n for n in names if n.endswith('.dist-info/METADATA'))))
        version = metadata['Version']
        assert metadata['Name'] == 'servomotor'
        assert metadata['Requires-Python'] == '>=3.8'
        assert not metadata.get_all('Requires-Dist'), 'package must remain dependency-free'
        commands = json.loads(archive.read('servomotor/motor_commands.json'))
        commands = {item['CommandEnum']: item for item in commands}
        assert commands[48]['Input'] is None and len(commands[48]['Output']) == 3
        assert commands[49]['Output'] == 'success_response'
        assert commands[50]['CommandString'] == 'Set immediate position target and get position'
        assert len(commands[50]['Output']) == 1
        assert commands[50]['Output'][0]['Description'].startswith('i64:')
        # The candidate must contain the reviewed sources and canonical data.
        source_package = Path(__file__).resolve().parent / 'servomotor'
        for path in source_package.rglob('*'):
            if path.is_file() and path.suffix in ('.py', '.json'):
                member = 'servomotor/' + path.relative_to(source_package).as_posix()
                assert archive.read(member) == path.read_bytes(), 'stale packaged source: ' + member
        for name in names:
            assert not any(part in name for part in ['__pycache__', '.bak', '.DS_Store', 'serial_device.txt'])
            assert not name.endswith(('.pyc', '.pyo', '~'))
        assert any(name.endswith('/licenses/LICENCE') or name.endswith('.dist-info/LICENCE') for name in names)
    here = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix='servomotor-wheel-') as folder:
        folder = Path(folder)
        environment = folder / 'venv'
        # Match the venv CLI: copying Apple's framework executable can break
        # its loader paths; Windows keeps copies to avoid symlink privileges.
        venv.EnvBuilder(with_pip=True, symlinks=os.name != 'nt').create(environment)
        scripts = environment / ('Scripts' if os.name == 'nt' else 'bin')
        python = scripts / ('python.exe' if os.name == 'nt' else 'python')
        subprocess.run([str(python), '-m', 'pip', 'install', '--disable-pip-version-check', '--no-index', '--no-deps', str(wheel)], cwd=folder, check=True, timeout=120)
        for name in ['host_test_framework.py', 'test_host_library_commands.py']:
            shutil.copy2(here / name, folder / name)
        script = '''import servomotor, importlib.metadata, pathlib
assert servomotor.__version__ == EXPECTED
assert importlib.metadata.version('servomotor') == EXPECTED
assert 'site-packages' in pathlib.Path(servomotor.__file__).parts
for name in ['serial_functions','serial_abstraction','communication','device_detection',
             'M3','command_loader','platform_detect','platform_utils','terminal_format_wrapper']:
    __import__('servomotor.' + name)
from servomotor.vendor import serial
assert serial.Serial.__module__.startswith('servomotor.vendor.serial.')
print('Installed wheel imports and metadata PASSED')
'''.replace('EXPECTED', repr(version))
        subprocess.run([str(python), '-c', script], cwd=folder, check=True, timeout=30)
        subprocess.run([str(python), str(folder / 'test_host_library_commands.py')], cwd=folder, check=True, timeout=120)
        for command in ['servomotor_command', 'detect_and_set_alias_all_devices', 'show_device_information_for_all_devices', 'upgrade_firmware']:
            executable = scripts / (command + ('.exe' if os.name == 'nt' else ''))
            subprocess.run([str(executable), '--help'], cwd=folder, check=True, timeout=30, stdout=subprocess.PIPE)
    print('Installed wheel and command contract PASSED: ' + version)


if __name__ == '__main__':
    main()
