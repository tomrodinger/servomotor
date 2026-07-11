#!/usr/bin/env python3
"""Ultracode hardware campaign: adversarial scenarios never exercised on the bench.
All sections must PASS. Leaves the motor reset. ~25 min including the 500-move soak."""
import sys
import time
import struct
import math
import random

sys.path.insert(0, "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs")
import servomotor
from servomotor.communication import TimeoutError as ServoTimeoutError, FatalError as ServoFatalError

class A: port = '/dev/tty.usbserial-210'; PORT = False
servomotor.set_serial_port_from_args(A)
servomotor.open_serial_port()
m = servomotor.M3('X', time_unit='seconds', position_unit='shaft_rotations',
                  velocity_unit='rotations_per_second', verbose=0)

random.seed(20260710)
FAILURES = []

def section(name):
    print(f"\n[{name}]", flush=True)

def check(cond, what):
    print(("  PASS: " if cond else "  FAIL: ") + what, flush=True)
    if not cond:
        FAILURES.append(what)

def reset():
    m.system_reset(); time.sleep(2.0)

def enter_cl(kp=2000, ki=25, kd=350000, dev=50):
    reset()
    m.set_max_allowable_position_deviation(dev)
    m.set_pid_constants(kp, ki, kd)
    m.enable_mosfets(); time.sleep(0.3)
    m.go_to_closed_loop()
    deadline = time.time() + 8
    while time.time() < deadline:
        s = m.get_status()
        if s and (s[0] & 4):
            break
        time.sleep(0.1)
    m.zero_position(); time.sleep(0.5)

def pid_samples(seconds):
    rows = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            raw = bytes(m.read_multipurpose_buffer())
        except Exception:
            continue
        if len(raw) == 21 and raw[0] == 4:
            rows.append(struct.unpack("<iiiii", raw[1:]))
    return rows

def status_err():
    s = m.get_status()
    return s[1]

# ---------------- H1: version ----------------
section("H1 firmware version on the bench")
reset()
v = m.get_firmware_version()
check(v[0] == [4, 3, 15, 0] and v[1] == 0, f"running 0.15.3.4, not bootloader (got {v})")

# ---------------- H2: default PID constants after reset ----------------
section("H2 default PID constants (kP=2000 recovered from live data, no set_pid sent)")
reset()
m.set_max_allowable_position_deviation(50)
m.enable_mosfets(); time.sleep(0.3)
m.go_to_closed_loop()
deadline = time.time() + 8
while time.time() < deadline:
    s = m.get_status()
    if s and (s[0] & 4):
        break
    time.sleep(0.1)
m.zero_position(); time.sleep(0.5)
m.test_mode(3)
rows = pid_samples(2.0)
ratios = set()
for (e, p, i, d, out) in rows:
    if e != 0 and p % e == 0:
        ratios.add(p // e)
check(ratios == {2000}, f"default kP detected as 2000 (got {ratios}), {len(rows)} samples")
check(status_err() == 0, "no fatal error")

# ---------------- H3: gain-change fuzz during motion ----------------
section("H3 gain-change fuzz: 150 random set_pid_constants while moving")
enter_cl()
POOL_P = [0, 1, 500, 2000, 4000, 20000, 65535, 2147483648, 4294967295]
POOL_I = [0, 1, 5, 25, 100, 40460, 2147483648, 4294967295]
POOL_D = [0, 1, 33, 40, 175000, 350000, 1400000, 4294967295]
mv_dir = 1
last_move = 0.0
fuzz_fail = None
for k in range(150):
    if time.time() - last_move > 0.8:
        try:
            m.trapezoid_move(0.5 * mv_dir, 0.7)
            mv_dir = -mv_dir
            last_move = time.time()
        except ServoFatalError as e:
            fuzz_fail = f"move raised FatalError {e} at iter {k}"
            break
    try:
        m.set_pid_constants(random.choice(POOL_P), random.choice(POOL_I), random.choice(POOL_D))
    except ServoFatalError as e:
        fuzz_fail = f"set_pid raised FatalError {e} at iter {k}"
        break
    time.sleep(0.05)
if fuzz_fail is None:
    time.sleep(1.5)
    err = status_err()
    check(err == 0, f"no fatal error after 150 mid-motion gain changes (err={err})")
    m.set_pid_constants(2000, 25, 350000)
    time.sleep(0.3)
    m.trapezoid_move(0.5, 0.7); time.sleep(1.3)
    p = m.get_comprehensive_position()
    check(abs(p[0] - p[1]) < 0.01, f"tracking recovered with sane gains (|err|={abs(p[0]-p[1])*3276800:.0f} counts)")
else:
    check(False, fuzz_fail)

# ---------------- H4: live current-limit sweep ----------------
section("H4 live current-limit sweep (clamps recompute under load; low-V torque fix)")
enter_cl()
m.test_mode(3)
ok_rails = True
ok_outputs = True
lowv_above_voltage_ceiling = 0
for V in (25, 50, 100, 200, 390):
    m.set_maximum_motor_current(V, V)
    # the multipurpose buffer is one-shot: it may hold a sample written BEFORE this
    # limit change (with the previous clamp values) -- read and discard it
    try:
        m.read_multipurpose_buffer()
    except Exception:
        pass
    time.sleep(0.2)
    m.trapezoid_move(0.25, 0.35); time.sleep(0.75)   # demanding at low V
    rows = pid_samples(0.6)
    if not rows:
        ok_rails = False
        print(f"  V={V}: no samples!")
        continue
    authority = min(V << 19, 800000000)
    max_i = authority >> 2
    out_limit = max(V << 8, 16384)     # fw >= 0.15.3.4: angle term included
    peak_i = max(abs(r[2]) for r in rows)
    peak_out = max(abs(r[4]) for r in rows)
    ok_i = peak_i <= max_i
    ok_o = peak_out <= out_limit
    ok_rails = ok_rails and ok_i
    ok_outputs = ok_outputs and ok_o
    if V < 64:
        lowv_above_voltage_ceiling += sum(1 for r in rows if abs(r[4]) > (V << 8))
    print(f"  V={V}: peak|I|={peak_i}<={max_i}:{ok_i}  peak|out|={peak_out}<={out_limit}:{ok_o}")
check(ok_rails, "integral stayed within the voltage-scaled clamp at every limit")
check(ok_outputs, "output stayed within max(V<<8, 16384) at every limit")
check(status_err() == 0, "no fatal error during the sweep so far")
# TORQUE-FIX PROOF: a gentle move at V=25 doesn't demand enough output to reach the old
# V<<8=6400 ceiling, so force a snap (instant setpoint jump) -- the chase must now drive
# the output past 6400 (up to 16384), which was impossible before the 0.15.3.4 fix.
m.set_maximum_motor_current(25, 25)
try:
    m.read_multipurpose_buffer()   # discard possible stale one-shot sample
except Exception:
    pass
time.sleep(0.2)
m.move_with_velocity(0, 0.3)
m.move_with_velocity(10.0, 0.03)   # 0.3-rot setpoint jump
m.move_with_velocity(0, 0.1)
rows = pid_samples(2.0)
peak_out_snap = max(abs(r[4]) for r in rows) if rows else 0
check(6400 < peak_out_snap <= 16384,
      f"TORQUE FIX PROVEN: snap at V=25 drove |output| to {peak_out_snap} "
      f"(> old ceiling 6400, <= new limit 16384; lead-angle range restored)")
check(status_err() == 0, "no fatal error after the V=25 snap")
# positive check of the firmware's own range guard: V=400 > 390 must trip error 23
guard_ok = False
try:
    m.set_maximum_motor_current(400, 400)
    time.sleep(0.3)
    guard_ok = (status_err() == 23)
except ServoFatalError as e:
    guard_ok = ("23" in str(e))
except ServoTimeoutError:
    guard_ok = (status_err() == 23)
check(guard_ok, "out-of-range current limit (400 > 390) trips ERROR_MAX_PWM_VOLTAGE_TOO_HIGH (23)")
reset()  # clear the deliberate fatal error before the next section

# ---------------- H5: deviation-watchdog trip & recovery x5 ----------------
section("H5 deviation watchdog: trip error 45 and recover, 5 cycles")
# NOTE: the tight deviation limit must be set AFTER closed-loop entry + zero (the
# MOSFET-enable commutation-alignment transient would trip it otherwise -- documented
# firmware behavior, see the enable-mosfets gotcha in WORK_CHECKLIST TODO #8).
ok_all = True
for k in range(5):
    tripped = False
    try:
        enter_cl()                                     # normal entry, wide limit
        m.set_max_allowable_position_deviation(0.01)   # NOW tighten (32,768 counts)
        # a profiled move tracks too well (~3-6k counts lag) to trip 0.01 rot -- use a
        # SNAP: the setpoint jumps ~0.3 rot instantly, deviation rails at ~52k counts
        m.move_with_velocity(0, 0.2)
        m.move_with_velocity(10.0, 0.03)
        m.move_with_velocity(0, 0.1)
        for _ in range(30):
            time.sleep(0.1)
            try:
                if status_err() == 45:
                    tripped = True
                    break
            except (ServoFatalError, ServoTimeoutError):
                tripped = True
                break
    except (ServoFatalError, ServoTimeoutError):
        tripped = True
    except Exception as ex:
        print(f"  cycle {k}: unexpected {type(ex).__name__}: {ex}")
    if not tripped:
        ok_all = False
        print(f"  cycle {k}: watchdog did NOT trip")
        continue
    # recover
    try:
        reset()
        enter_cl()
        m.trapezoid_move(0.2, 0.5); time.sleep(1.0)
        if status_err() != 0:
            ok_all = False
            print(f"  cycle {k}: recovery move left fatal error")
    except Exception as ex:
        ok_all = False
        print(f"  cycle {k}: recovery failed: {ex}")
check(ok_all, "watchdog tripped and recovered cleanly in all 5 cycles")

# ---------------- H6: MOSFET kill mid-move x3 ----------------
section("H6 disable_mosfets mid-move, then full recovery, 3 cycles")
ok_all = True
for k in range(3):
    try:
        enter_cl()
        m.trapezoid_move(2, 1.5)
        time.sleep(0.5)
        m.disable_mosfets()
        time.sleep(0.5)
        reset()
        enter_cl()
        m.trapezoid_move(0.2, 0.5); time.sleep(1.0)
        p = m.get_comprehensive_position()
        if abs(p[0] - p[1]) > 0.01 or status_err() != 0:
            ok_all = False
            print(f"  cycle {k}: bad recovery state")
    except Exception as ex:
        ok_all = False
        print(f"  cycle {k}: {type(ex).__name__}: {ex}")
check(ok_all, "mid-move MOSFET kill recovered in all 3 cycles")

# ---------------- H7: queue stress ----------------
section("H7 queue stress: fill near capacity, drain, then verify the overflow guard")
# each trapezoid move enqueues up to 3 items; MOVEMENT_QUEUE_SIZE = 32
enter_cl()
try:
    for k in range(10):                      # 30 items: fits
        m.trapezoid_move(0.1 if k % 2 == 0 else -0.1, 0.15)
    time.sleep(3.0)
    p = m.get_comprehensive_position()
    check(abs(p[0] - p[1]) < 0.01 and status_err() == 0,
          f"30-item queue drained, tracking OK (|err|={abs(p[0]-p[1])*3276800:.0f} counts)")
except Exception as ex:
    check(False, f"queue fill/drain: {type(ex).__name__}: {ex}")
# positive check of the queue-full guard: spamming more than 32 items must trip error 17
guard17 = False
try:
    for k in range(20):                      # 60 items: must overflow
        m.trapezoid_move(0.05 if k % 2 == 0 else -0.05, 0.3)
except ServoFatalError as e:
    guard17 = ("17" in str(e))
except ServoTimeoutError:
    guard17 = (status_err() == 17)
if not guard17:
    try:
        guard17 = (status_err() == 17)
    except Exception:
        pass
check(guard17, "queue overflow trips ERROR_QUEUE_IS_FULL (17) as designed")
reset()
enter_cl()
m.trapezoid_move(0.2, 0.5); time.sleep(1.0)
check(status_err() == 0, "clean recovery after the deliberate queue overflow")

# ---------------- H8: 500-move soak ----------------
section("H8 endurance: 500 bidirectional moves (~7 min)")
enter_cl()
t0 = time.time()
soak_fail = None
for k in range(500):
    try:
        m.trapezoid_move(1.0 if k % 2 == 0 else -1.0, 0.5)
        time.sleep(0.78)
        if k % 100 == 99:
            e = status_err()
            p = m.get_comprehensive_position()
            print(f"  move {k+1}: |err|={abs(p[0]-p[1])*3276800:.0f} counts, fatal={e}", flush=True)
            if e != 0:
                soak_fail = f"fatal {e} at move {k}"
                break
    except Exception as ex:
        soak_fail = f"{type(ex).__name__} at move {k}: {ex}"
        break
check(soak_fail is None, soak_fail or f"500 moves in {time.time()-t0:.0f} s, no faults")

# ---------------- H9: test-mode-3 in open loop ----------------
section("H9 test mode 3 outside closed loop (must not populate or crash)")
reset()
m.enable_mosfets(); time.sleep(0.3)   # open-loop hold, NOT closed loop
m.test_mode(3)
time.sleep(0.3)
try:
    raw = bytes(m.read_multipurpose_buffer())
    got = True
except Exception:
    got = False
check(not got, "cmd 35 times out in open loop (PID not running, buffer stays empty)")
check(status_err() == 0, "no fatal error")

# ---------------- H10: final regressions ----------------
section("H10 final regression trio on this binary")
enter_cl()
m.test_mode(3)
rows = pid_samples(3.0)
import statistics
ss = rows[len(rows)//4:]
err_std = statistics.pstdev([r[0] for r in ss])
d_mean = statistics.mean([r[3] for r in ss])
check(err_std < 80, f"standstill error std {err_std:.1f} < 80")
check(abs(d_mean) < 40000, f"standstill D bias {d_mean:.0f} ~ 0")
enter_cl()
m.trapezoid_move(2, 2.0); time.sleep(4.5)
m.test_mode(3)
rows = pid_samples(1.0)
rms = math.sqrt(sum(r[0]**2 for r in rows) / len(rows))
check(rms < 200, f"post-move residual RMS {rms:.1f} < 200")

reset()
servomotor.close_serial_port()
print(f"\n{'='*60}")
if FAILURES:
    print(f"CAMPAIGN FAILED ({len(FAILURES)}):")
    for f in FAILURES:
        print("  -", f)
    sys.exit(1)
print("HARDWARE CAMPAIGN: ALL SECTIONS PASSED")
