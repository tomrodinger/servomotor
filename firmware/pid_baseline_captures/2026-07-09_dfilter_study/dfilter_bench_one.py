#!/usr/bin/env python3
"""Bench battery for one D-filter variant (already flashed): standstill + 2 moves + 3 snaps.
Usage: dfilter_bench_one.py SHIFT_TAG OUTDIR"""
import sys
import time
import struct
import os
import math
import statistics

sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs")
import servomotor
from servomotor.communication import TimeoutError as ServoTimeoutError

tag, outdir = sys.argv[1], sys.argv[2]
os.makedirs(outdir, exist_ok=True)

class A: port = '/dev/tty.usbserial-210'; PORT = False
servomotor.set_serial_port_from_args(A)
servomotor.open_serial_port()
m = servomotor.M3('X', time_unit='seconds', position_unit='shaft_rotations',
                  velocity_unit='rotations_per_second', verbose=0)

def enter_cl(kp=2000, ki=25, kd=350000):
    m.system_reset(); time.sleep(2.0)
    m.set_max_allowable_position_deviation(50)
    m.set_pid_constants(kp, ki, kd)
    m.enable_mosfets(); time.sleep(0.3)
    m.go_to_closed_loop()
    deadline = time.time() + 8
    while time.time() < deadline:
        s = m.get_status()
        if s and (s[0] & 4): break
        time.sleep(0.1)
    m.zero_position(); time.sleep(1.0)
    m.test_mode(3)

def drain(seconds):
    rows = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            raw = bytes(m.read_multipurpose_buffer())
        except (ServoTimeoutError, Exception):
            continue
        if len(raw) == 21 and raw[0] == 4:
            rows.append((time.time() - t0,) + struct.unpack("<iiiii", raw[1:]))
    return rows

results = {"tag": tag}

# --- standstill ---
enter_cl()
rows = drain(5.0)
ss = [r for r in rows if r[0] > 1.0]
results["ss_err_std"] = statistics.pstdev([r[1] for r in ss])
results["ss_d_std"] = statistics.pstdev([r[4] for r in ss])
results["ss_d_mean"] = statistics.mean([r[4] for r in ss])
results["ss_out_std"] = statistics.pstdev([r[5] for r in ss])

# --- 2 moves ---
mv = []
for k in range(2):
    enter_cl()
    m.trapezoid_move(2, 2.0)
    rows = drain(5.5)
    post = [r[1] for r in rows if r[0] >= 3.2]
    mv.append(math.sqrt(sum(e * e for e in post) / len(post)))
results["move_rms"] = statistics.median(mv)
results["move_rms_all"] = mv

# --- 3 snaps (0.61 rot burst like the overshoot study) ---
ovs = []
for k in range(3):
    enter_cl()
    m.move_with_velocity(0, 0.3)
    m.move_with_velocity(15.0, 0.040)
    m.move_with_velocity(0, 0.1)
    rows = drain(2.5)
    stop_t = None
    # find landing: first sample where error <= 0 after the big positive excursion
    big = [i for i, r in enumerate(rows) if r[1] > 20000]
    ov = 0
    if big:
        after = rows[big[-1]:]
        crossed = [i for i, r in enumerate(after) if r[1] <= 0]
        if crossed:
            t0 = after[crossed[0]][0]
            seg = [r[1] for r in after if t0 <= r[0] <= t0 + 0.6]
            ov = max(0, -min(seg))
    ovs.append(ov)
s = m.get_status()
results["snap_ov"] = statistics.median(ovs)
results["snap_ov_all"] = ovs
results["fatal"] = s[1]

m.system_reset(); time.sleep(1.5)
servomotor.close_serial_port()

import json
json.dump(results, open(os.path.join(outdir, f"bench_{tag}.json"), "w"), indent=1)
print(f"{tag}: err_std={results['ss_err_std']:.1f} d_std={results['ss_d_std']:.0f} "
      f"move_rms={results['move_rms']:.0f} snap_ov={results['snap_ov']:.0f} "
      f"(all {results['snap_ov_all']}) fatal={results['fatal']}")
