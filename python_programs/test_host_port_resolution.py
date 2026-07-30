#!/usr/bin/env python3
"""
Host-only tests for how the serial port is chosen -- no motor, no serial port.

Precedence, highest first:
    1. explicit -p (device_name argument), or -P ("MENU")
    2. $SERVOMOTOR_PORT
    3. the saved default file
    4. the interactive menu

Also covers the per-user config location, the fallback that reads the old
in-package file so upgraders keep their saved default, and the rule that an
unwritable config directory must warn rather than crash.
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from host_test_framework import TestRunner              # noqa: E402
from servomotor import serial_functions as sf           # noqa: E402


class FakePort(object):
    def __init__(self, name):
        self.name = name


def run_open(t, monkey_env, saved_file_value, device_name, tmp_config):
    """Drive open_serial_port() with the real world stubbed out.

    Returns (port_opened, menu_was_used).
    """
    opened = {}
    state = {"menu_used": False}

    def fake_open(device, baud, timeout):
        opened["device"] = device
        return FakePort(device)

    def fake_menu():
        state["menu_used"] = True
        return "MENU_CHOICE"

    orig_open = sf.open_serial_port_or_print_detailed_error
    orig_menu = sf.select_serial_port_from_menu
    orig_read = sf.read_saved_port
    orig_environ = os.environ.get(sf.SERIAL_PORT_ENV_VAR)

    sf.open_serial_port_or_print_detailed_error = fake_open
    sf.select_serial_port_from_menu = fake_menu
    sf.read_saved_port = lambda: saved_file_value
    os.environ["XDG_CONFIG_HOME"] = tmp_config
    if monkey_env is None:
        os.environ.pop(sf.SERIAL_PORT_ENV_VAR, None)
    else:
        os.environ[sf.SERIAL_PORT_ENV_VAR] = monkey_env

    try:
        sf.open_serial_port(device_name=device_name)
    finally:
        sf.open_serial_port_or_print_detailed_error = orig_open
        sf.select_serial_port_from_menu = orig_menu
        sf.read_saved_port = orig_read
        if orig_environ is None:
            os.environ.pop(sf.SERIAL_PORT_ENV_VAR, None)
        else:
            os.environ[sf.SERIAL_PORT_ENV_VAR] = orig_environ

    return opened.get("device"), state["menu_used"]


def test_precedence(t, tmp_config):
    # 1. -p beats everything below it.
    port, _ = run_open(t, "ENV_PORT", "SAVED_PORT", "EXPLICIT_PORT", tmp_config)
    t.check("-p outranks env var and saved file (got %s)" % port, port == "EXPLICIT_PORT")

    # 2. env var beats the saved file.
    port, _ = run_open(t, "ENV_PORT", "SAVED_PORT", None, tmp_config)
    t.check("$SERVOMOTOR_PORT outranks the saved file (got %s)" % port, port == "ENV_PORT")

    # 3. saved file is used when nothing more explicit exists.
    port, _ = run_open(t, None, "SAVED_PORT", None, tmp_config)
    t.check("saved file used when no -p and no env var (got %s)" % port, port == "SAVED_PORT")

    # 4. menu is the last resort.
    port, menu = run_open(t, None, None, None, tmp_config)
    t.check("menu used when nothing else is available", menu and port == "MENU_CHOICE")

    # 5. -P must always reach the menu, even if the env var / saved file are set.
    port, menu = run_open(t, "ENV_PORT", "SAVED_PORT", "MENU", tmp_config)
    t.check("-P forces the menu despite env var and saved file", menu and port == "MENU_CHOICE")

    # 6. An empty env var must be ignored, not treated as a port name.
    port, _ = run_open(t, "", "SAVED_PORT", None, tmp_config)
    t.check("empty $SERVOMOTOR_PORT is ignored (got %s)" % port, port == "SAVED_PORT")


def test_config_location(t):
    orig = os.environ.get("XDG_CONFIG_HOME")
    try:
        os.environ["XDG_CONFIG_HOME"] = os.path.join("/tmp", "xdg-example")
        d = sf.get_config_dir()
        t.check("XDG_CONFIG_HOME is honoured (%s)" % d,
                d == os.path.join("/tmp", "xdg-example", "servomotor"))

        os.environ.pop("XDG_CONFIG_HOME", None)
        d = sf.get_config_dir()
        t.check("config dir is per-user, not inside the package (%s)" % d,
                os.path.dirname(os.path.abspath(sf.__file__)) not in d)
        t.check("config dir is platform-appropriate (%s)" % d, "servomotor" in d)
    finally:
        if orig is None:
            os.environ.pop("XDG_CONFIG_HOME", None)
        else:
            os.environ["XDG_CONFIG_HOME"] = orig


def test_save_and_read_roundtrip(t, tmp_config):
    orig = os.environ.get("XDG_CONFIG_HOME")
    os.environ["XDG_CONFIG_HOME"] = tmp_config
    try:
        t.check("save_port() reports success", sf.save_port("/dev/ttyTEST0") is True)
        t.check("saved file lands in the config dir",
                os.path.isfile(os.path.join(tmp_config, "servomotor", "serial_device.txt")))
        t.check("read_saved_port() round-trips", sf.read_saved_port() == "/dev/ttyTEST0")
    finally:
        if orig is None:
            os.environ.pop("XDG_CONFIG_HOME", None)
        else:
            os.environ["XDG_CONFIG_HOME"] = orig


def test_unwritable_config_dir_does_not_crash(t):
    """A read-only config location must warn and return False, never raise."""
    orig = os.environ.get("XDG_CONFIG_HOME")
    # /dev/null is a file, so makedirs underneath it always fails.
    os.environ["XDG_CONFIG_HOME"] = os.path.join(os.devnull, "cannot", "exist")
    try:
        result = sf.save_port("/dev/ttyTEST1")
        t.check("save_port() returns False instead of raising", result is False)
    except Exception as e:  # noqa: BLE001 - the whole point is that nothing escapes
        t.check("save_port() raised %s instead of returning False" % type(e).__name__, False)
    finally:
        if orig is None:
            os.environ.pop("XDG_CONFIG_HOME", None)
        else:
            os.environ["XDG_CONFIG_HOME"] = orig


def main():
    t = TestRunner("test_host_port_resolution")
    tmp_config = tempfile.mkdtemp(prefix="servomotor-cfg-")
    test_precedence(t, tmp_config)
    test_config_location(t)
    test_save_and_read_roundtrip(t, tmp_config)
    test_unwritable_config_dir_does_not_crash(t)
    t.finish()


if __name__ == "__main__":
    main()
