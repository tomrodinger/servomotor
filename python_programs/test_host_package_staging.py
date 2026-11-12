#!/usr/bin/env python3
"""Offline package staging checks, including dirty developer-tree fixtures."""
import ast
import importlib.util
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
STAGER = ROOT / 'Arduino_library/stage_library.py'
spec = importlib.util.spec_from_file_location('stage_servomotor', STAGER)
arduino = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arduino)


class PackageStaging(unittest.TestCase):
    def test_python_staging_retains_sources_and_strips_machine_files(self):
        if not shutil.which('bash'):
            self.skipTest('bash required for release staging script')
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            dist = base / 'PyPi_distribution'
            package = base / 'servomotor'
            dist.mkdir(); package.mkdir()
            script = dist / 'prepare_src.sh'
            shutil.copy2(ROOT / 'python_programs/PyPi_distribution/prepare_src.sh', script)
            real = ['__init__.py', 'M3.py', 'motor_commands.json', 'data_types.json', 'unit_conversions_M3.json',
                    'vendor/serial/__init__.py']
            junk = ['M3.py.bak', 'M3.py.bak3', 'M3.py.old', 'M3.py.working', 'motor_commands.json.backup',
                    'M3.py~', '.DS_Store', 'serial_device.txt', '__pycache__/M3.pyc',
                    'vendor/serial/__pycache__/__init__.pyc', 'vendor/serial/serial.pyc', 'vendor/serial/serial.pyo']
            for name in real + junk:
                path = package / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(name)
            for name in ['terminal_formatting.py', 'servomotor_command.py', 'detect_and_set_alias_all_devices.py',
                         'show_device_information_for_all_devices.py', 'upgrade_firmware.py']:
                (base / name).write_text(name)
            for _ in range(2):
                subprocess.run(['bash', str(script)], cwd=folder, check=True, stdout=subprocess.PIPE)
                contents = {str(p.relative_to(dist / 'src/servomotor')) for p in (dist / 'src/servomotor').rglob('*') if p.is_file()}
                self.assertEqual(contents, set(real))
            self.assertTrue(all((package / name).is_file() for name in junk), 'staging must leave developer sources intact')

    def test_arduino_package_layout_and_deployer_file_parity(self):
        deployment = (ROOT / 'Arduino_library/deploy_to_arduino_library.sh').read_text()
        manifest = re.search(r'CORE_FILES=\((.*?)\)', deployment, re.S).group(1)
        self.assertEqual(set(re.findall(r'"([^"]+)"', manifest)), set(arduino.CORE_FILES))
        with tempfile.TemporaryDirectory() as folder:
            destination = arduino.stage(Path(folder) / 'Servomotor-0.11.0')
            self.assertTrue((destination / 'src/Servomotor.cpp').is_file())
            self.assertFalse((destination / 'Servomotor.cpp').exists())
            self.assertEqual((destination / 'Servomotor.h').read_bytes(), (destination / 'src/Servomotor.h').read_bytes())
            self.assertFalse(list(destination.rglob('*test*')))
            self.assertTrue((destination / 'CHANGELOG.md').is_file())
            self.assertGreater(len(list((destination / 'examples').glob('*/*.ino'))), 0)
            path = arduino.archive(destination)
            first = path.read_bytes(); path.unlink()
            self.assertEqual(first, arduino.archive(destination).read_bytes())
            with zipfile.ZipFile(path) as contents:
                self.assertEqual(path.name, 'Servomotor-0.11.0.zip')
                self.assertTrue(all(n.startswith('Servomotor-0.11.0/') and '..' not in n for n in contents.namelist()))
            with self.assertRaises(FileExistsError):
                arduino.stage(destination)

    def test_missing_arduino_source_fails_before_creating_output(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / 'output'
            with self.assertRaises(FileNotFoundError):
                arduino.stage(destination, folder)
            self.assertFalse(destination.exists())

    def test_hardware_suite_registry_fits_persisted_run_state(self):
        suite = ROOT / 'Arduino_library/Servomotor_TestSuite_ESP32S3'
        header = (suite / 'test_registry.h').read_text()
        registry = (suite / 'test_registry.cpp').read_text()
        harness = (suite / 'Servomotor_TestSuite_ESP32S3.ino').read_text()
        capacity = int(re.search(r'TF_TEST_CAPACITY = (\d+)', header).group(1))
        entries = re.findall(r'\{\s*"[^"]+",\s*tm_\w+', registry)
        self.assertLessEqual(len(entries), capacity)
        self.assertLess(capacity, 255)
        self.assertIn('#define MAX_TESTS TF_TEST_CAPACITY', harness)
        self.assertIn('<= TF_TEST_CAPACITY', registry)
        self.assertIn('tm_immediate_position', registry)

    def test_generated_bindings_match_catalog_and_templates(self):
        # Regenerate in a scratch tree so checking codegen never edits sources.
        with tempfile.TemporaryDirectory(prefix='servomotor-codegen-') as folder:
            scratch = Path(folder)
            shutil.copytree(ROOT / 'autogeneration', scratch / 'autogeneration',
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            package = scratch / 'python_programs/servomotor'
            package.mkdir(parents=True)
            for path in (ROOT / 'python_programs/servomotor').glob('*.json'):
                shutil.copy2(path, package / path.name)
            output = scratch / 'Arduino_library'; output.mkdir()
            subprocess.run([sys.executable, str(scratch / 'autogeneration/generate_command_code.py'),
                            '-y', '--no-backup'], cwd=scratch, check=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
            def normalize(text):
                return re.sub(r'^// This file was autogenerated.*$', '// generated timestamp', text, flags=re.M)
            for name in ('Commands.h', 'Servomotor.h', 'Servomotor.cpp'):
                with self.subTest(name=name):
                    self.assertEqual(normalize((output / name).read_text()),
                                     normalize((ROOT / 'Arduino_library' / name).read_text()))

    def test_python_source_uses_declared_minimum_grammar(self):
        # feature_version is supported by the CPython versions in CI; this
        # checks syntax only, not a substitute for actually running 3.8 in CI.
        for path in (ROOT / 'python_programs/servomotor').glob('*.py'):
            with self.subTest(path=path.name):
                ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))


if __name__ == '__main__':
    result = unittest.main(exit=False).result
    print('Package staging tests ' + ('PASSED' if result.wasSuccessful() else 'FAILED'))
    raise SystemExit(not result.wasSuccessful())
