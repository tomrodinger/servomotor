#! /usr/bin/env python3

import servomotor
import time

alias = ord("X")
VERBOSE = True

servomotor.open_serial_port()

servomotor.execute_command(alias, "ENABLE_MOSFETS_COMMAND", [])

time.sleep(1)

servomotor.execute_command(alias, "SYSTEM_RESET_COMMAND", [], verbose=VERBOSE)

time.sleep(0.2)

servomotor.close_serial_port()
