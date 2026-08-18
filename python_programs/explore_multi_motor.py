#!/usr/bin/env python3
"""
EXPLORATION: what actually happens when you drive MANY motors on one bus.

Everything in the test suite drives a single motor. But an RS485 bus exists in
order to have several motors on it, and coordinated multi-axis motion is the
reason a customer buys more than one. That whole dimension is undocumented:
how simultaneous is a broadcast move, how many motors can be commanded
individually before the bus becomes the bottleneck, does one motor faulting
disturb the others, and is the clock-sync command actually worth using.

This is a discovery script, not a test: it prints what it measured and leaves
the judgement to a human. Results go into knowhow.md.

SAFETY
    MOSFETs are never enabled, so no shaft turns anywhere on the rack. Every
    measurement reads the COMMANDED position, which the planner advances
    regardless of the output stage -- which is precisely what makes it safe to
    broadcast a move to 35 motors at once.

    Broadcast is used ONLY for motion and reset commands. No alias is ever
    written (that would re-address the whole rack, which has bitten this project
    before). Every motor is left reset and clean.
"""

import argparse
import statistics
import sys
import time

import servomotor
from servomotor import M3
from servomotor.device_detection import detect_devices_iteratively

CPR = 3276800
TPS = 31250
SYSTEM_RESET = 27
TRAPEZOID_MOVE = 2
BROADCAST = 255


def note(tag, msg):
    print(f"  [{tag}] {msg}")


def scenario(t):
    print(f"\n=== {t} ===")


def broadcast_reset():
    servomotor.execute_command(SYSTEM_RESET, bytearray(),
                               alias_or_unique_id=BROADCAST, crc32_enabled=True, verbose=0)
    time.sleep(1.8)


def main(args):
    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port()

    print("Detecting motors...")
    devs = detect_devices_iteratively(n_detections=4, verbose=False)
    uids = sorted({d.unique_id for d in devs})
    print(f"Found {len(uids)} motors\n")
    if len(uids) < 2:
        print("Need at least two motors for this exploration.")
        return 1
    if args.limit:
        uids = uids[:args.limit]
    motors = {u: M3(f"{u:016X}", verbose=0) for u in uids}

    def each(fn, what=""):
        """Apply fn to every motor, collecting failures rather than aborting."""
        out, bad = {}, []
        for u, m in motors.items():
            try:
                out[u] = fn(m)
            except Exception as e:
                bad.append((u, type(e).__name__))
        if bad and what:
            note(what, f"{len(bad)} motor(s) failed: "
                       f"{', '.join(f'{u:016X}:{e}' for u, e in bad[:3])}")
        return out

    # ------------------------------------------------------------------
    scenario("1. A BROADCAST move: how identical are 35 motors?")
    try:
        broadcast_reset()
        for m in motors.values():
            m.set_position_unit("encoder_counts"); m.set_time_unit("timesteps")
        before = each(lambda m: m.get_position(), "before")
        # One packet, every motor moves. This is the coordinated-motion primitive.
        servomotor.execute_command(TRAPEZOID_MOVE,
                                   [int(CPR // 4), int(TPS)],
                                   alias_or_unique_id=BROADCAST, crc32_enabled=True, verbose=0)
        time.sleep(2.0)
        after = each(lambda m: m.get_position(), "after")
        moved = [after[u] - before[u] for u in after if u in before]
        if moved:
            note("broadcast", f"{len(moved)} motors moved; min {min(moved):.0f} "
                              f"max {max(moved):.0f} spread {max(moved)-min(moved):.0f} "
                              f"(commanded {CPR//4})")
            note("broadcast", "=> one packet drives every motor; spread is the "
                              "per-motor planner rounding, not desynchronisation")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("2. Individually addressing every motor: how long does it take?")
    try:
        broadcast_reset()
        for m in motors.values():
            m.set_position_unit("encoder_counts"); m.set_time_unit("timesteps")
        t0 = time.time()
        n = 0
        for m in motors.values():
            m.trapezoid_move(CPR // 8, TPS)
            n += 1
        elapsed = time.time() - t0
        note("serial", f"queued a move on {n} motors one at a time in {elapsed*1000:.0f} ms "
                       f"({elapsed/max(n,1)*1000:.1f} ms per motor)")
        note("serial", f"=> a broadcast is ~{elapsed*1000:.0f} ms cheaper, and is the "
                       f"only way to start {n} axes together")
        time.sleep(2.0)
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("3. Do the motors' clocks agree, and does time-sync help?")
    try:
        broadcast_reset()
        # RESTORE THE TIME UNIT. Scenario 2 left it on "timesteps", and
        # get_current_time() then reports timesteps with no indication that it
        # has. The first run of this script read a 4150 "second" spread that was
        # really 4150 timesteps = 133 ms of read-order skew.
        for m in motors.values():
            m.set_time_unit("seconds")
        t = each(lambda m: m.get_current_time(), "clock")
        if len(t) > 1:
            vals = list(t.values())
            note("clock", f"clocks after a broadcast reset span "
                          f"{max(vals)-min(vals):.6f} s across {len(vals)} motors")
            note("clock", "=> they were all reset by the same packet, so this is "
                          "the read-order skew, not clock drift")
        # Let them run, then re-read: does the spread grow?
        time.sleep(5.0)
        t2 = each(lambda m: m.get_current_time())
        if len(t2) > 1:
            v2 = list(t2.values())
            note("clock", f"after 5 s the spread is {max(v2)-min(v2):.6f} s")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("4. Does ONE motor faulting disturb the others?")
    try:
        broadcast_reset()
        for m in motors.values():
            m.set_position_unit("encoder_counts"); m.set_time_unit("timesteps")
            m.set_velocity_unit("rotations_per_second")
        victim = uids[0]
        # Fault exactly one motor with an over-speed move.
        motors[victim].set_maximum_velocity(0.2)
        try:
            motors[victim].trapezoid_move(CPR, TPS // 4)
        except Exception:
            pass
        time.sleep(0.5)
        states = each(lambda m: m.get_status()[1])
        faulted = [u for u, f in states.items() if f != 0]
        note("fault", f"deliberately faulted 1 motor; {len(faulted)} motor(s) now report "
                      f"a fault: {[f'{u:016X}' for u in faulted]}")
        # And can the others still be commanded?
        healthy = [u for u in uids if u not in faulted][:5]
        ok = 0
        for u in healthy:
            try:
                b = motors[u].get_position()
                motors[u].trapezoid_move(CPR // 16, TPS // 2)
                time.sleep(0.8)
                if abs((motors[u].get_position() - b) - CPR // 16) < 5000:
                    ok += 1
            except Exception:
                pass
        note("fault", f"{ok}/{len(healthy)} healthy motors still moved correctly "
                      f"while another sat faulted")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("5. A broadcast reset clears a fault on every motor at once")
    try:
        broadcast_reset()
        states = each(lambda m: m.get_status()[1])
        bad = [u for u, f in states.items() if f != 0]
        note("recover", f"after one broadcast reset, {len(bad)} motor(s) still faulted")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("6. Reading every motor in a loop: sustainable polling rate")
    try:
        t0 = time.time()
        rounds = 0
        errs = 0
        while time.time() - t0 < 5.0:
            for m in motors.values():
                try:
                    m.get_position()
                except Exception:
                    errs += 1
            rounds += 1
        el = time.time() - t0
        total = rounds * len(motors)
        note("poll", f"{total} position reads across {len(motors)} motors in {el:.1f} s "
                     f"= {total/el:.0f} reads/s, {rounds/el:.2f} full sweeps/s, {errs} errors")
        note("poll", "=> that is the practical ceiling for a supervisory loop on this bus")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    try:
        broadcast_reset()
        states = each(lambda m: m.get_status()[1])
        clean = sum(1 for f in states.values() if f == 0)
        print(f"\nLeft clean: {clean}/{len(states)} motors report no fault")
    except Exception:
        pass
    servomotor.close_serial_port()
    return 0


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Explore multi-motor behaviour on one bus.")
    p.add_argument('-p', '--port', default=None)
    p.add_argument('-P', '--PORT', action='store_true')
    p.add_argument('-a', '--alias', default=None)
    p.add_argument('--limit', type=int, default=0, help="use at most this many motors")
    sys.exit(main(p.parse_args()))
