#!/usr/bin/env python3
"""Measure the motor-control ISR duration via the firmware's always-on TIM14 profiler.

Fields from 'Get debug values': [9]=last-ISR duration us, [10]=max since last read (resets on read).
Procedure: closed loop with default gains (stable on old and new firmware), sample
N windows at standstill and N windows during a slow move.
"""
import sys
import time
sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs")
import servomotor

LABEL = sys.argv[1] if len(sys.argv) > 1 else "unknown"

class A: port = '/dev/tty.usbserial-210'; PORT = False
servomotor.set_serial_port_from_args(A)
servomotor.open_serial_port()
m = servomotor.M3('X', time_unit='seconds', position_unit='shaft_rotations', verbose=0)

m.system_reset(); time.sleep(2.0)
v = m.get_firmware_version()
fw = f"{v[0][3]}.{v[0][2]}.{v[0][1]}.{v[0][0]}"
m.set_pid_constants(2000, 5, 175000)          # defaults: stable on both firmwares
m.set_max_allowable_position_deviation(50)
m.enable_mosfets(); time.sleep(0.3)
m.go_to_closed_loop()
deadline = time.time() + 6
while time.time() < deadline:
    s = m.get_status()
    if s and (s[0] & 4):
        break
    time.sleep(0.1)
m.zero_position(); time.sleep(1.0)

def sample_window(seconds):
    m.get_debug_values()          # resets the max register
    time.sleep(seconds)           # ~31250 ISRs per second accumulate a fresh max
    d = m.get_debug_values()
    return int(d[9]), int(d[10])  # (last-ISR us, window-max us)

print(f"firmware {fw} [{LABEL}]")
ss = [sample_window(2.0) for _ in range(6)]
print("standstill: last-ISR us =", [a for a, _ in ss], " window-max us =", [b for _, b in ss])

m.trapezoid_move(4, 10.0)         # slow 4-rotation move over 10 s
time.sleep(0.5)
mv = [sample_window(1.5) for _ in range(5)]
print("during move: last-ISR us =", [a for a, _ in mv], " window-max us =", [b for _, b in mv])
time.sleep(3.0)                   # let the move finish

s = m.get_status()
print(f"status: flags={s[0]:#06x} fatal={s[1]}")
m.system_reset(); time.sleep(1.5)
servomotor.close_serial_port()
