#!/usr/bin/env python3
"""
Host-only test: import / smoke test.

No motor / serial port required. Catches import-graph breakage and missing
generated command methods -- the kind of failure that would otherwise only
surface the first time hardware is connected. Validates:
  * `import servomotor` and `from servomotor import M3` succeed,
  * an M3 instance can be constructed (integer alias, single-char alias,
    default units),
  * the dynamically generated command methods exist and are callable,
  * the JSON command / data-type tables loaded and command lookup works,
  * platform detection and the core protocol constants are present.

Accepts (and ignores) the -p / -P / -a arguments so run_all_tests.py can
invoke it alongside the hardware tests.
"""

from host_test_framework import TestRunner


# Generated command method names that must exist on the M3 class.
EXPECTED_METHODS = [
    "disable_mosfets", "enable_mosfets", "go_to_position", "move_with_velocity",
    "move_with_acceleration", "homing", "ping", "get_temperature",
    "get_status", "get_comprehensive_position", "system_reset",
    "set_device_alias", "multimove", "get_firmware_version",
]

# (command name, expected command ID) pairs for the lookup check.
EXPECTED_COMMAND_IDS = [
    ("Disable MOSFETs", 0), ("Go to position", 4), ("Ping", 31),
    ("Get temperature", 42), ("System reset", 27),
]


def main():
    t = TestRunner("test_host_import_smoke")

    # Importing the package runs the whole JSON-load + code-generation path.
    import servomotor
    from servomotor import M3, communication
    from servomotor.platform_detect import is_standard_python, is_micropython
    t.check("package 'servomotor' imported", servomotor is not None)
    t.check("M3 class imported", M3 is not None)

    # Construct M3 instances the way the test programs do.
    t.check_eq("M3(integer alias) stores the alias",
               M3(88, verbose=0).alias_or_unique_id, 88)
    t.check_eq("M3(single-char alias 'X') resolves to ASCII 88",
               M3("X", verbose=0).alias_or_unique_id, 88)
    # M3() with no alias is a supported "deferred alias" state -- the alias
    # is supplied later via use_this_alias_or_unique_id() or the -a global.
    motor = M3(verbose=0)
    t.check("M3() no-alias construction works", motor is not None)
    t.check_eq("M3() leaves the alias unset (None)",
               motor.alias_or_unique_id, None)
    for unit_attr in ("_time_unit", "_position_unit", "_velocity_unit",
                      "_current_unit"):
        t.check("M3 instance has " + unit_attr, hasattr(motor, unit_attr))

    # The command methods are generated from motor_commands.json onto M3.
    for name in EXPECTED_METHODS:
        t.check("M3 has callable method '%s'" % name,
                callable(getattr(M3, name, None)))

    # Command-name lookup.
    for name, expected_id in EXPECTED_COMMAND_IDS:
        t.check_eq("get_command_id('%s')" % name,
                   servomotor.get_command_id(name), expected_id)
    t.check_eq("get_command_id of an unknown command is None",
               servomotor.get_command_id("No Such Command"), None)

    # The JSON tables must have loaded into the communication module.
    t.check("registered_commands table is populated",
            bool(communication.registered_commands))
    t.check("registered_data_types table is populated",
            bool(communication.registered_data_types))

    # Platform detection: exactly one of standard / MicroPython is true.
    t.check("platform detection is consistent",
            is_standard_python() != is_micropython())

    # Core protocol constants.
    t.check_eq("ALL_ALIAS constant", communication.ALL_ALIAS, 255)
    t.check_eq("RESPONSE_CHARACTER_CRC32_ENABLED constant",
               communication.RESPONSE_CHARACTER_CRC32_ENABLED, 253)
    t.check_eq("RESPONSE_CHARACTER_CRC32_DISABLED constant",
               communication.RESPONSE_CHARACTER_CRC32_DISABLED, 252)
    t.check("PROTOCOL_VERSION is defined",
            isinstance(communication.PROTOCOL_VERSION, int))
    t.check("package exposes calculate_crc32",
            callable(servomotor.calculate_crc32))

    t.finish()


if __name__ == "__main__":
    main()
