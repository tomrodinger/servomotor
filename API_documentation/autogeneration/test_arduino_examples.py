"""Documentation examples must compile against the generated Arduino bindings."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

from M17_generate_api_documentation import APIDocumentationGenerator

ROOT = Path(__file__).resolve().parents[2]


class ArduinoExamplesTest(unittest.TestCase):
    def test_target_and_scalar_examples_compile(self):
        commands = json.loads((ROOT / 'python_programs/servomotor/motor_commands.json').read_text())
        generator = APIDocumentationGenerator()
        examples = [generator.generate_arduino_example(c) for c in commands if c['CommandEnum'] in (38, 49, 50)]
        self.assertEqual(len(examples), 3)
        self.assertTrue(all(len(line) <= 90 for example in examples for line in example.splitlines()))
        source = '#include "Servomotor.h"\nvoid examples(Servomotor &motor) {\n'
        source += '\n'.join('{\n' + example + '\n}' for example in examples) + '\n}\n'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'examples.cpp'
            path.write_text(source)
            subprocess.run(shlex.split(os.environ.get('CXX', 'c++')) + [
                '-std=c++17', '-fsyntax-only', '-I' + str(ROOT / 'Arduino_library'), str(path)], check=True)


if __name__ == '__main__':
    unittest.main()
