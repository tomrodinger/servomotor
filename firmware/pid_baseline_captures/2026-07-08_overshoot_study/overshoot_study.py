#!/usr/bin/env python3
"""
Overshoot characterization: command a velocity burst (instant velocity step, no
accel ramp) so the setpoint sprints ahead of the rotor, then stops. The rotor
chases, catches up, and potentially overshoots -- a commanded stand-in for
hand-displace-and-release. Captures PID internals via test mode 3 during the
whole event, for a sweep of PID configurations.

Usage: python3 overshoot_study.py OUTDIR "KP KI KD TAG" ["KP KI KD TAG" ...]
"""
import sys
import time
import struct
import os

sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs")
import servomotor
from servomotor.communication import TimeoutError as ServoTimeoutError

VEL_REV_S = 15.0     # burst velocity (i32 command ceiling is ~19.5 rev/s)
BURST_S = 0.040      # 15 rev/s x 40 ms = 0.6-rotation setpoint step -> like a hand twist, released
PRE_S = 0.7          # capture before the burst
POST_S = 2.2         # capture after the stop

outdir = sys.argv[1]
configs = [tuple(a.split()) for a in sys.argv[2:]]
os.makedirs(outdir, exist_ok=True)

class A: port = '/dev/tty.usbserial-210'; PORT = False
servomotor.set_serial_port_from_args(A)
servomotor.open_serial_port()
m = servomotor.M3('X', time_unit='seconds', position_unit='shaft_rotations',
                  velocity_unit='rotations_per_second', verbose=0)

for kp, ki, kd, tag in configs:
    m.system_reset(); time.sleep(2.0)
    m.set_max_allowable_position_deviation(50)
    m.set_pid_constants(int(kp), int(ki), int(kd))
    m.enable_mosfets(); time.sleep(0.3)
    m.go_to_closed_loop()
    deadline = time.time() + 8
    while time.time() < deadline:
        s = m.get_status()
        if s and (s[0] & 4):
            break
        time.sleep(0.1)
    m.zero_position(); time.sleep(1.0)   # let entry transient die
    m.test_mode(3)

    rows = []
    burst_done = False
    t0 = time.time()
    t_burst = None
    while True:
        t = time.time() - t0
        if t > PRE_S + BURST_S + POST_S:
            break
        if not burst_done and t >= PRE_S:
            # three queued items: a 0.3 s zero-velocity lead-in (gives RS485 time to deliver
            # the rest of the queue before the sub-ms burst runs out), the instant velocity
            # step, then the instant stop. A queue ending at nonzero velocity trips error 18.
            m.move_with_velocity(0, 0.3)
            m.move_with_velocity(VEL_REV_S, BURST_S)
            m.move_with_velocity(0, 0.1)
            t_burst = (time.time() - t0) + 0.3   # the burst executes after the lead-in
            burst_done = True
            continue
        try:
            raw = bytes(m.read_multipurpose_buffer())
        except (ServoTimeoutError, Exception):
            continue
        if len(raw) == 21 and raw[0] == 4:
            rows.append((t,) + struct.unpack("<iiiii", raw[1:]))

    s = m.get_status()
    path = os.path.join(outdir, f"ov_{tag}.csv")
    with open(path, "w") as f:
        f.write("# overshoot study v1\n")
        f.write(f"# set_pid: {kp} {ki} {kd}\n")
        f.write(f"# burst: {VEL_REV_S} rev/s for {BURST_S} s, sent at t={t_burst:.4f}, stop at t={t_burst + BURST_S:.4f}\n")
        f.write(f"# fatal_error: {s[1]}\n")
        f.write("t_s,error,p_term,i_term,d_term,output\n")
        for r in rows:
            f.write(f"{r[0]:.6f}," + ",".join(str(v) for v in r[1:]) + "\n")
    print(f"{tag}: {len(rows)} samples, fatal={s[1]}, saved {path}", flush=True)

m.system_reset(); time.sleep(1.5)
servomotor.close_serial_port()
print("all runs complete")
