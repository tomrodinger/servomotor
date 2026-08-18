#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_command_state_matrix.cpp
//
// WHICH COMMANDS ARE LEGAL IN WHICH STATE?
//
// A caller writing a supervisory loop has to know, for every command it might
// send, whether sending it right now is safe. The firmware answers that
// question differently depending on what the motor is doing -- `zero position`
// is fine when idle and raises ERROR_QUEUE_NOT_EMPTY mid-move, `start
// calibration` has four separate preconditions -- but the answers are scattered
// across the source and appear in no single place.
//
// This module builds the table. For each command it records the outcome in
// three machine states:
//
//     IDLE        -- reset, nothing queued, nothing running
//     MOVING      -- a long move in flight, queue non-empty
//     QUEUE FULL  -- the 32-slot queue saturated
//
// (The fourth state, FAULTED, is `tm_fatal_state_commands`, which found that
// only `get_status` answers there.)
//
// The properties asserted are the ones a caller actually depends on:
//
//   * every command produces a DEFINITE outcome in every state -- accepted or
//     refused, never a timeout or a hang
//   * the outcome is REPEATABLE: the same command in the same state gives the
//     same answer every time, so behaviour can be relied on rather than
//     discovered
//   * reads are legal in all three states -- monitoring must never be
//     state-dependent
//   * a refusal never leaves the machine broken: after any refusal, the motor
//     is still usable
//
// Note what is deliberately NOT asserted: which specific commands are legal
// mid-move. That is a design decision, not a correctness property, so the
// module RECORDS the table (printed in full) and asserts only the invariants
// above. A change in policy will show up in the printed table rather than as a
// spurious failure.
//
// SAFETY: MOSFETs stay OFF. `zeroPosition` and `emergencyStop` are exercised
// but nothing turns, since the planner advances only the COMMANDED position
// with the output stage off. Deliberately excluded: anything that broadcasts,
// writes an alias, starts calibration, or enters a test mode -- all unsafe on a
// shared rack. No broadcasts and no alias changes.
// ---------------------------------------------------------------------------

enum { N_CMDS = 12, N_STATES = 3 };

static const char* cmdName(int i) {
    switch (i) {
        case 0:  return "get status";
        case 1:  return "get position";
        case 2:  return "get n queued items";
        case 3:  return "get supply voltage";
        case 4:  return "get temperature";
        case 5:  return "ping";
        case 6:  return "set maximum velocity";
        case 7:  return "set maximum acceleration";
        case 8:  return "set PID constants";
        case 9:  return "zero position";
        case 10: return "trapezoid move";
        case 11: return "emergency stop";
    }
    return "?";
}

static const char* stateName(int s) {
    switch (s) {
        case 0: return "IDLE";
        case 1: return "MOVING";
        case 2: return "QUEUE FULL";
    }
    return "?";
}

// True if the command is purely a read (no state change intended).
static bool isRead(int i) { return i <= 5; }

// Issue command i. Returns true if it was accepted (no error reported).
static bool issue(Servomotor* m, int i) {
    switch (i) {
        case 0:  m->getStatus(); break;
        case 1:  m->getPositionRaw(); break;
        case 2:  m->getNQueuedItems(); break;
        case 3:  m->getSupplyVoltageRaw(); break;
        case 4:  m->getTemperatureRaw(); break;
        case 5:  { uint8_t p[10]; for (int k = 0; k < 10; k++) p[k] = (uint8_t)k;
                   m->ping(p); break; }
        case 6:  m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
                 m->setMaximumVelocity(3.0f); break;
        case 7:  m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
                 m->setMaximumAcceleration(1500.0f); break;
        case 8:  m->setPidConstants(1000, 100, 10000); break;
        case 9:  m->zeroPosition(); break;
        case 10: m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
                 m->setTimeUnit(TimeUnit::TIMESTEPS);
                 m->trapezoidMoveRaw(4000, TFH_TPS / 10); break;
        case 11: m->emergencyStop(); break;
    }
    return m->getError() == 0;
}

// Put the machine into the requested state. Returns false if it could not be
// established, so a probe is never run against the wrong state.
static bool enterState(Servomotor* m, int state) {
    tfhFreshOpen(m);
    if (state == 0) {
        return m->getNQueuedItems() == 0 && m->getError() == 0;
    }
    // Slow limits make each queued segment about a second long, so the state
    // persists for the whole probe instead of draining underneath it.
    m->setMaximumVelocity(1.0f);
    m->setMaximumAcceleration(1.0f);
    if (state == 1) {
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 12 * TFH_TPS);
        if (m->getError() != 0) return false;
        delay(150);
        return m->getNQueuedItems() > 0 && m->getError() == 0;
    }
    // state == 2: fill the queue to capacity without overflowing it.
    // 10 moves x 3 slots = 30, and an 11th would be 33 > 32.
    for (int k = 0; k < 10; k++) {
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 400), 12 * TFH_TPS);
        if (m->getError() != 0) return false;
    }
    uint8_t d = m->getNQueuedItems();
    return (m->getError() == 0) && d >= 30;
}

void tm_command_state_matrix(void) {
    Serial.println("tm_command_state_matrix: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // outcome[cmd][state]: 1 accepted, 0 refused, -1 state could not be set up
    static int outcome[N_CMDS][N_STATES];
    static bool repeatable[N_CMDS][N_STATES];

    // =====================================================================
    // 1. BUILD THE TABLE. Each cell is probed THREE times from a freshly
    //    established state, so the repeatability of the answer is measured
    //    rather than assumed -- an intermittent answer is itself a defect.
    // =====================================================================
    printf("\n--- 1. probing every command in every state ---\n");
    for (int c = 0; c < N_CMDS; c++) {
        for (int s = 0; s < N_STATES; s++) {
            int firstResult = -1;
            bool consistent = true;
            for (int trial = 0; trial < 3; trial++) {
                if (!enterState(m, s)) { firstResult = -1; consistent = false; break; }
                bool accepted = issue(m, c);
                // Settle, then clear whatever the probe may have started.
                delay(120);
                int r = accepted ? 1 : 0;
                if (trial == 0) firstResult = r;
                else if (r != firstResult) consistent = false;
                m->emergencyStop();
                delay(120);
            }
            outcome[c][s] = firstResult;
            repeatable[c][s] = consistent;
        }
    }

    // Print the whole table -- this IS the deliverable of the module.
    printf("\n   %-26s %-10s %-10s %-10s\n", "command", "IDLE", "MOVING", "QUEUE FULL");
    for (int c = 0; c < N_CMDS; c++) {
        printf("   %-26s", cmdName(c));
        for (int s = 0; s < N_STATES; s++) {
            const char* v = (outcome[c][s] == 1) ? "accepted"
                          : (outcome[c][s] == 0) ? "REFUSED" : "(no state)";
            printf(" %-10s", v);
        }
        if (!repeatable[c][0] || !repeatable[c][1] || !repeatable[c][2]) printf("  <- INCONSISTENT");
        printf("\n");
    }

    // =====================================================================
    // 2. EVERY CELL HAS A DEFINITE ANSWER. No timeouts, no hangs: the state
    //    was always establishable and the command always produced a verdict.
    // =====================================================================
    printf("\n--- 2. every command gives a definite answer in every state ---\n");
    {
        int undefined = 0;
        for (int c = 0; c < N_CMDS; c++)
            for (int s = 0; s < N_STATES; s++)
                if (outcome[c][s] < 0) {
                    undefined++;
                    printf("   %s in %s: no definite outcome\n", cmdName(c), stateName(s));
                }
        TEST_RESULT("all thirty-six command/state combinations produce a definite outcome",
                    undefined == 0);
    }

    // =====================================================================
    // 3. EVERY ANSWER IS REPEATABLE. A command that is sometimes accepted and
    //    sometimes refused in the same state cannot be programmed against.
    //    This is the assertion most likely to catch a real race.
    // =====================================================================
    printf("\n--- 3. every answer is repeatable across three trials ---\n");
    {
        int flaky = 0;
        for (int c = 0; c < N_CMDS; c++)
            for (int s = 0; s < N_STATES; s++)
                if (!repeatable[c][s]) {
                    flaky++;
                    printf("   %s in %s: answer varied between trials\n",
                           cmdName(c), stateName(s));
                }
        TEST_RESULT("no command/state combination gives an inconsistent answer", flaky == 0);
    }

    // =====================================================================
    // 4. READS ARE LEGAL IN EVERY STATE. Monitoring must not depend on what
    //    the machine happens to be doing -- a dashboard cannot be expected to
    //    know whether it is safe to ask.
    // =====================================================================
    printf("\n--- 4. reads are legal in every state ---\n");
    {
        for (int c = 0; c < N_CMDS; c++) {
            if (!isRead(c)) continue;
            bool everywhere = (outcome[c][0] == 1) && (outcome[c][1] == 1) && (outcome[c][2] == 1);
            if (!everywhere) {
                printf("   %s: idle=%d moving=%d full=%d\n", cmdName(c),
                       outcome[c][0], outcome[c][1], outcome[c][2]);
            }
            TEST_RESULT(std::string(cmdName(c)) + " is accepted in all three states",
                        everywhere);
        }
    }

    // =====================================================================
    // 5. THE MOTOR IS USABLE AFTER EVERY REFUSAL. A command being illegal in
    //    some state is fine; leaving the machine unusable is not. Each
    //    refusal found above is re-provoked and recovery is checked.
    // =====================================================================
    printf("\n--- 5. every refusal is recoverable ---\n");
    {
        int refusals = 0, recovered = 0;
        for (int c = 0; c < N_CMDS; c++) {
            for (int s = 0; s < N_STATES; s++) {
                if (outcome[c][s] != 0) continue;
                refusals++;
                if (!enterState(m, s)) continue;
                issue(m, c);
                delay(150);
                // Recovery: reset, then a real move must work.
                tfhFreshOpen(m);
                bool ok = false;
                int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 8), TFH_TPS / 2, &ok);
                if (ok && llabs((long long)(moved - TFH_CPR / 8)) < 200) recovered++;
                else printf("   did NOT recover after %s refused in %s\n",
                            cmdName(c), stateName(s));
            }
        }
        printf("   %d refusals found, %d recovered cleanly\n", refusals, recovered);
        TEST_RESULT("the machine recovers from every refusal in the table",
                    recovered == refusals);
    }

    // =====================================================================
    // 6. THE QUEUE-FULL STATE REALLY IS FULL, and one more move is refused.
    //    Without this, the whole QUEUE FULL column could be measuring a state
    //    that was never actually established.
    // =====================================================================
    printf("\n--- 6. the queue-full state is genuinely full ---\n");
    {
        bool established = enterState(m, 2);
        uint8_t depth = m->getNQueuedItems();
        // 30 slots used; one more 3-slot move would need 33 > 32.
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 400), 12 * TFH_TPS);
        bool accepted = (m->getError() == 0);
        delay(200);
        uint8_t fat = tfhFatal(m);
        printf("   depth %u, one more move: accepted=%d fatal=%u\n",
               (unsigned)depth, (int)accepted, (unsigned)fat);
        TEST_RESULT("the queue-full state was established", established && depth >= 30);
        TEST_RESULT("one move past capacity is refused rather than silently dropped",
                    (!accepted) || (fat != 0));
        tfhReset(m);
        TEST_RESULT("the machine recovers from the overflow", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the matrix sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
