#! /usr/bin/env python3

import servomotor
import time

motorX = servomotor.M3("X", motor_type="M3", time_unit="seconds", position_unit="degrees", velocity_unit="degrees/s", acceleration_unit="degree/s^2", current_unit="mA", voltage_unit="V", temperature_unit="C", verbose=True)

servomotor.open_serial_port()
motorX.system_reset()
time.sleep(1.2)
motorX.enable_mosfets()
motorX.go_to_position(position=2000000, time=32000)
servomotor.close_serial_port()
del motorX
