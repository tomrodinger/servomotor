#!/usr/bin/env python3
"""
Host-only test: product-scoping of command descriptions in motor_commands.json.

No motor / serial port required.

WHY THIS EXISTS
    Command descriptions in motor_commands.json are not just prose. They are
    embedded verbatim into the generated Arduino library (as comments in
    Servomotor.cpp) and into BOTH generated API documents, so a description
    that is wrong for a product ships that error to every customer of that
    product.

    A real instance: the 'Go to closed loop' description used to state
    unconditionally that the command "runs a current-sensor baseline check
    during that step, which can raise fatal error 22,
    ERROR_CURRENT_SENSOR_FAILED". That is false on M17. In
    firmware/Src/motor_control.c the function is compiled three ways:

        #if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)   -> checks
        #if defined(PRODUCT_NAME_M17)                              -> does NOT
        #if defined(PRODUCT_NAME_M23)                              -> checks

    The M17 build only sets TIM1->CCR1 = 0 and calls enable_mosfets(); there is
    no baseline sample and no path to error 22. 'Enable MOSFETs' had always
    scoped this correctly ("...; M17 performs no current-sensor check") while
    'Go to closed loop' did not.

WHAT IT GUARDS
    That any description mentioning the current-sensor check names the products
    it applies to, rather than stating it unconditionally. This is a cheap
    guard against the claim being reintroduced by a future edit.

Accepts (and ignores) the -p / -P / -a arguments so run_all_tests.py can invoke
it alongside the hardware tests.
"""

import json
import os

from host_test_framework import TestRunner


# Commands whose descriptions touch the current-sensor baseline check. Both
# reach that code through check_current_sensor_and_enable_mosfets().
CURRENT_SENSOR_COMMANDS = ["Enable MOSFETs", "Go to closed loop"]

# If a description mentions any of these, it is talking about the check.
CURRENT_SENSOR_MARKERS = ["current-sensor", "current sensor", "ERROR_CURRENT_SENSOR_FAILED"]

# ...and if it does, it must also say which products it applies to. M17 is the
# product that does NOT perform the check, so naming M17 is the load-bearing
# part: a reader on an M17 must not be told to expect error 22.
PRODUCT_SCOPING_MARKERS = ["M17 performs no current-sensor check",
                           "M17 performs no current sensor check"]


def load_commands():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "servomotor", "motor_commands.json")
    with open(path, "r") as f:
        return json.load(f)


def main():
    t = TestRunner("test_host_command_description_accuracy")

    commands = load_commands()
    by_name = {c["CommandString"]: c for c in commands}

    for name in CURRENT_SENSOR_COMMANDS:
        t.check_eq("'%s' exists in motor_commands.json" % name,
                   name in by_name, True)
        if name not in by_name:
            continue

        description = by_name[name]["Description"]
        mentions_check = any(m in description for m in CURRENT_SENSOR_MARKERS)

        t.check_eq("'%s' description mentions the current-sensor check" % name,
                   mentions_check, True)

        if mentions_check:
            scoped = any(m in description for m in PRODUCT_SCOPING_MARKERS)
            t.check_eq(
                "'%s' says explicitly that M17 performs no current-sensor check"
                % name, scoped, True)

    # The specific unconditional wording that shipped the false claim must not
    # come back in ANY command description.
    bad_wording = "runs a current-sensor baseline check during that step"
    offenders = [c["CommandString"] for c in commands
                 if bad_wording in c.get("Description", "")]
    t.check_eq("no description claims the current-sensor check unconditionally "
               "(offenders: %s)" % (", ".join(offenders) or "none"),
               offenders, [])

    t.finish()


if __name__ == "__main__":
    main()
