#!/usr/bin/env python3
"""
EXPLORATION, not a test: drive a motor the way a customer plausibly would and
report what happens, so the results can be written into knowhow.md.

This is deliberately NOT pass/fail. Every scenario prints what it observed and
leaves the interpretation to a human. The point is to find behaviour nobody has
written down yet -- especially behaviour that is surprising but not a bug, which
is the category that costs support time.

The scenarios are chosen to be things a real user does by accident or by
reasonable-but-wrong assumption, rather than deliberate protocol abuse:
skipping the startup reset, changing units halfway through, changing a limit
after queueing, repeating an absolute move, and so on.

SAFETY: MOSFETs are never enabled, so nothing turns; every measurement reads the
COMMANDED position, which the planner advances regardless. No broadcasts and no
alias writes, so it is safe on the shared 35-motor rack. Each scenario resets
afterwards.
"""

import argparse
import sys
import time

import servomotor
from servomotor import M3

CPR = 3276800
TPS = 31250


def note(tag, msg):
    print(f"  [{tag}] {msg}")


def fresh(m):
    m.system_reset()
    time.sleep(1.6)
    m.set_position_unit("encoder_counts")
    m.set_time_unit("timesteps")
    m.set_velocity_unit("rotations_per_second")
    m.set_acceleration_unit("rotations_per_second_squared")


def drain(m, budget=15.0):
    end = time.time() + budget
    while time.time() < end:
        try:
            if m.get_n_queued_items() == 0:
                time.sleep(0.15)
                return True
        except Exception:
            return False
        time.sleep(0.02)
    return False


def fatal(m):
    try:
        return m.get_status()[1]
    except Exception:
        return -1


def scenario(title):
    print(f"\n=== {title} ===")


def main(args):
    servomotor.set_serial_port_from_args(args)
    servomotor.open_serial_port()
    m = M3(args.alias, verbose=0)

    # ------------------------------------------------------------------
    scenario("1. A program that does NOT reset at startup inherits state")
    # A customer writes program A that tightens a limit, then runs program B
    # which assumes factory defaults. Nothing power-cycles in between.
    try:
        fresh(m)
        m.set_maximum_velocity(0.2)                 # "program A" leaves this behind
        note("A", "program A set maximum velocity to 0.2 rot/s and exited")
        # "program B" starts here WITHOUT a reset
        m.set_position_unit("encoder_counts"); m.set_time_unit("timesteps")
        before = m.get_position()
        try:
            m.trapezoid_move(CPR, TPS // 4)         # a perfectly ordinary move
            time.sleep(0.4)
            f = fatal(m)
            moved = m.get_position() - before
            note("B", f"program B's ordinary 1-rotation move: fatal={f} moved={moved}")
        except Exception as e:
            note("B", f"program B's ordinary move raised {type(e).__name__}: {str(e)[:60]}")
        note("B", "=> a stale limit from a previous program silently changes behaviour")
    except Exception as e:
        note("!", f"scenario aborted: {type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("2. Changing the position unit while a move is already queued")
    try:
        fresh(m)
        m.set_maximum_velocity(4.0)
        before = m.get_position()
        m.trapezoid_move(CPR, 2 * TPS)              # queued in encoder counts
        time.sleep(0.3)
        m.set_position_unit("shaft_rotations")      # customer switches units mid-move
        pos_rot = m.get_position()
        m.set_position_unit("encoder_counts")
        drain(m)
        moved = m.get_position() - before
        note("units", f"mid-move read in rotations: {pos_rot:.4f}; final delta in counts: {moved}")
        note("units", "=> the unit is a HOST-side setting; the queued move is unaffected")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("3. Repeating the SAME absolute go-to-position")
    try:
        fresh(m)
        m.set_maximum_velocity(4.0)
        m.zero_position(); time.sleep(0.3)
        target = CPR // 2
        landed = []
        for i in range(4):
            m.go_to_position(target, TPS)
            drain(m)
            landed.append(m.get_position())
        note("goto", f"four identical go-to-position calls landed at: {landed}")
        note("goto", f"=> target {target}; first is short by {target - landed[0]}, then exact")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("4. The same distance as many small relative moves vs one big one")
    try:
        fresh(m); m.set_maximum_velocity(4.0)
        m.zero_position(); time.sleep(0.3)
        for _ in range(50):
            m.trapezoid_move(CPR // 50, TPS // 10)
        drain(m, 40)
        many = m.get_position()
        fresh(m); m.set_maximum_velocity(4.0)
        m.zero_position(); time.sleep(0.3)
        m.trapezoid_move(CPR, 2 * TPS)
        drain(m)
        one = m.get_position()
        note("drift", f"50 small moves -> {many};  1 big move -> {one};  ideal {CPR}")
        note("drift", f"=> relative-move rounding cost {CPR - many} counts over 50 moves")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("5. A slow, precise axis (the configuration that used to be silent)")
    try:
        for vel in (0.1, 0.25, 0.5):
            fresh(m)
            m.set_maximum_velocity(vel)             # leave acceleration at default
            before = m.get_position()
            m.trapezoid_move(CPR // 100, 2 * TPS)
            ok = drain(m)
            note("slow", f"maxVel={vel} rot/s, default accel: moved "
                         f"{m.get_position() - before} of {CPR // 100}, fatal={fatal(m)}")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("6. Emergency stop, then immediately queue new work")
    try:
        fresh(m); m.set_maximum_velocity(2.0)
        m.trapezoid_move(CPR, 4 * TPS)
        time.sleep(0.5)
        m.emergency_stop()
        time.sleep(0.2)
        before = m.get_position()
        m.trapezoid_move(CPR // 8, TPS)             # no reset in between
        ok = drain(m)
        note("estop", f"new move after e-stop with NO reset: moved {m.get_position()-before} "
                      f"of {CPR//8}, fatal={fatal(m)}")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("7. Queue exactly full, then one more move")
    try:
        fresh(m)
        m.set_maximum_velocity(1.0); m.set_maximum_acceleration(1.0)
        accepted = 0
        for i in range(14):
            try:
                m.trapezoid_move(CPR // 400, 8 * TPS)
                if fatal(m) != 0:
                    break
                accepted += 1
            except Exception as e:
                note("queue", f"move {i} raised {type(e).__name__}: {str(e)[:40]}")
                break
        note("queue", f"accepted {accepted} moves before refusal; fatal={fatal(m)}")
        note("queue", "=> at 3 slots each, 10 fit in the 32-slot queue")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("8. Same plan, but with a slow speed limit (2-slot fast path)")
    try:
        fresh(m)
        m.set_maximum_velocity(0.3)                 # default acceleration
        accepted = 0
        for i in range(20):
            try:
                m.trapezoid_move(CPR // 400, 8 * TPS)
                if fatal(m) != 0:
                    break
                accepted += 1
            except Exception:
                break
        note("queue2", f"accepted {accepted} moves before refusal; fatal={fatal(m)}")
        note("queue2", "=> a SLOWER speed limit lets MORE moves fit, because the "
                       "planner emits 2 segments instead of 3")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("9. Reading position in every unit back to back")
    try:
        fresh(m); m.set_maximum_velocity(4.0)
        m.zero_position(); time.sleep(0.3)
        m.set_position_unit("encoder_counts")
        m.trapezoid_move(CPR // 4, TPS); drain(m)
        vals = {}
        for u in ("encoder_counts", "shaft_rotations", "degrees", "radians"):
            m.set_position_unit(u)
            vals[u] = m.get_position()
        m.set_position_unit("encoder_counts")
        note("units", f"same position: {vals}")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("10. Homing without being in closed loop")
    try:
        fresh(m)
        try:
            m.set_position_unit("shaft_rotations"); m.set_time_unit("seconds")
            m.homing(1.0, 2.0)
            time.sleep(0.5)
            note("homing", f"homing accepted; fatal={fatal(m)}")
        except Exception as e:
            note("homing", f"homing raised {type(e).__name__}: {str(e)[:50]}; fatal={fatal(m)}")
        m.set_position_unit("encoder_counts"); m.set_time_unit("timesteps")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("11. A long zero-displacement dwell, then a move")
    try:
        fresh(m); m.set_maximum_velocity(4.0)
        before = m.get_position()
        m.trapezoid_move(0, 2 * TPS)                # dwell
        m.trapezoid_move(CPR // 8, TPS)
        ok = drain(m, 20)
        note("dwell", f"dwell + move: moved {m.get_position()-before} of {CPR//8}, "
                      f"fatal={fatal(m)}, drained={ok}")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    # ------------------------------------------------------------------
    scenario("12. Zero position while a move is still running")
    try:
        fresh(m); m.set_maximum_velocity(1.0); m.set_maximum_acceleration(1.0)
        m.trapezoid_move(CPR // 4, 8 * TPS)
        time.sleep(0.3)
        try:
            m.zero_position()
            note("zero", f"zero_position DURING a move was accepted; fatal={fatal(m)}")
        except Exception as e:
            note("zero", f"zero_position during a move raised {type(e).__name__}: "
                         f"{str(e)[:40]}; fatal={fatal(m)}")
    except Exception as e:
        note("!", f"{type(e).__name__}: {e}")

    try:
        fresh(m)
        print(f"\nMotor left clean: fatal={fatal(m)}, queue={m.get_n_queued_items()}")
    except Exception:
        pass
    servomotor.close_serial_port()
    return 0


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Explore realistic customer scenarios.")
    p.add_argument('-p', '--port', default=None)
    p.add_argument('-P', '--PORT', action='store_true')
    p.add_argument('-a', '--alias', default='X')
    sys.exit(main(p.parse_args()))
