#!/usr/bin/env python3

import servomotor

# The commands supported at this time are:
all_commands = \
'''
  0: Disable MOSFETs
  1: Enable MOSFETs
  2: Trapezoid move
  3: Set maximum velocity
  4: Go to position
  5: Set maximum acceleration
  6: Start calibration
  7: Capture hall sensor data
  8: Reset time
  9: Get current time
 10: Time sync
 11: Get queue size
 12: Emergency stop
 13: Zero position
 14: Homing
 15: Get hall sensor position
 16: Get status
 17: Go to closed loop
 18: Get update frequency
 19: Move with acceleration
 20: Detect devices
 21: Set device alias
 22: Get product info
 23: Firmware upgrade
 24: Get product description
 25: Get firmware version
 26: Move with velocity
 27: System reset
 28: Set maximum motor current
 29: Multi-move
 30: Set safety limits
 31: Ping
 32: Control hall sensor statistics
 33: Get hall sesor statistics
 34: Get position
 35: Read multipurpose buffer
 36: Test mode
 37: Get comprehensive position
 38: Get supply voltage
 39: Get max PID error
 40: Vibrate
 41: Identify
 42: Get temperature
 43: Set PID constants
'''
# You can generate this list of supported commands by running:
#        ./servomotor_command.py -c | grep ": " | grep -v "Input" | grep -v "Response"

print("We are going to test that the get_command_id function works correctly by seeing if it return the right values for all of the following commands:")
print(all_commands)

all_passed = True

for line in all_commands.split("\n"):
    if len(line) == 0:
        continue
    command_name = line.split(":")[1].strip()
    command_id = servomotor.get_command_id(command_name)
    print("Command ID for", command_name + ":", command_id)
    if command_id != int(line.split(":")[0]):
        print("Test failed")
        all_passed = False

if all_passed:
    print("PASSED")
else:
    print("FAILED")
