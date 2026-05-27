#!/usr/bin/env python3
"""
Host-only test: servomotor.get_command_id() returns the right enum for every
command string in motor_commands.json.

Modernized from the obsolete test_servomotor_module_get_command_id.py, which
hardcoded a stale list of 44 commands. This version reads the canonical
motor_commands.json and verifies all 48 entries.

No hardware. Uses the shared host_test_framework.
"""

import json
import os
import sys
import servomotor
from host_test_framework import TestRunner


def main():
    t = TestRunner("test_host_get_command_id")

    # Load motor_commands.json from the package directory.
    json_path = os.path.join(
        os.path.dirname(os.path.abspath(servomotor.__file__)),
        "motor_commands.json",
    )
    with open(json_path) as f:
        commands = json.load(f)

    t.check("motor_commands.json loaded", isinstance(commands, list) and len(commands) > 0)

    seen_enums = set()
    for entry in commands:
        name = entry["CommandString"]
        expected_id = entry["CommandEnum"]
        actual_id = servomotor.get_command_id(name)
        t.check_eq(f"get_command_id({name!r}) == {expected_id}", actual_id, expected_id)
        seen_enums.add(expected_id)

    # Sanity: the JSON should have a contiguous enum block starting at 0.
    if seen_enums:
        t.check_eq("CommandEnum values are 0..N-1 contiguous",
                   seen_enums, set(range(len(seen_enums))))

    # An unknown command should return None.
    t.check("get_command_id('this command does not exist') is None",
            servomotor.get_command_id("this command does not exist") is None)

    t.finish()


if __name__ == "__main__":
    main()
