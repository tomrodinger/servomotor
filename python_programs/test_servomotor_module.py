#! /usr/bin/env python3

import servomotor
import time

alias = ord("X")
VERBOSE = True

servomotor.open_serial_port()

servomotor.execute_command(alias, "Enable MOSFETs", [])

time.sleep(1)

servomotor.execute_command(alias, "System reset", [], verbose=VERBOSE)

time.sleep(0.2)

servomotor.close_serial_port()
