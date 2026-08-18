#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_setting_timing.cpp
//
// WHEN does a setting take effect?
//
// Every settings module in the suite checks that a setter is ACCEPTED and that
// its value is REJECTED when out of range. None of them asks the question a
// caller actually has to answer before writing a motion sequence:
//
//     If I change the speed limit after queueing a move, does the queued move
//     run at the old limit or the new one?
//
// THE ACTUAL CONTRACT, read out of the firmware rather than assumed:
//
//   * Limits are PASS/FAIL CHECKS, never clamps. add_to_queue() compares the
//     derived acceleration and the predicted velocity against the limits and
//     calls fatal_error() if either is exceeded (motor_control.c:1795, :1900,
//     :1944). The firmware NEVER stretches a move to make it fit -- an
//     over-ambitious move is refused, not slowed down.
//   * The trapezoid SHAPE is computed once, at queue time, from the limits in
//     force at that moment. Raising a limit later does not speed a queued move
//     up.
//   * BUT the control loop ALSO re-checks the LIVE velocity against
//     max_velocity on every one of its 31250 ticks per second
//     (motor_control.c:2187) and raises ERROR_VEL_TOO_HIGH if it is exceeded.
//     So LOWERING max_velocity below the speed a move is already travelling at
//     faults that move -- even though the move was legal when it was queued.
//     Tightening the limit is therefore NOT safe mid-flight. That asymmetry is
//     surprising and is the single most useful thing this module records.
//
// This module pins the contract down in both directions:
//   * loosening a limit after queueing does NOT speed the queued move up
//   * lowering it BELOW the in-flight velocity faults the move (error 16)
//   * lowering it but staying ABOVE the in-flight velocity is safe
//   * an over-limit move is REFUSED, not stretched
//   * immediate-effect settings really are immediate
//
// SAFETY: MOSFETs stay OFF except where explicitly noted (nowhere in this
// module). Only queue depth and the COMMANDED position are read, and both
// advance regardless of the output stage, so nothing turns. No broadcasts and
// no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const int32_t CPR = 3276800;
static const int32_t TPS = 31250;

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
}

static bool drainQueue(Servomotor* m, uint32_t budgetMs) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) { delay(200); return true; }
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

void tm_setting_timing(void) {
    Serial.println("tm_setting_timing: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. LOOSENING A LIMIT DOES NOT ACCELERATE AN ALREADY-QUEUED MOVE.
    //    Queue a move whose duration is set by a tight speed limit, then
    //    raise the limit tenfold while it runs. The move must still take the
    //    time it was planned for. If it sped up, the trapezoid was being
    //    re-evaluated against live limits and no plan would be predictable.
    // =====================================================================
    printf("\n--- 1. raising the speed limit mid-move does not speed it up ---\n");
    {
        freshStart(m);
        m->setMaximumVelocity(0.5f);
        m->setMaximumAcceleration(50.0f);
        uint32_t t0 = millis();
        // 1 rotation at 0.5 rot/s cannot finish faster than ~2 s whatever we ask.
        m->trapezoidMoveRaw(CPR, 3 * TPS);
        bool queued = (m->getError() == 0);
        delay(300);
        m->setMaximumVelocity(5.0f);            // loosen it mid-flight
        bool loosened = (m->getError() == 0);
        bool drained = drainQueue(m, 15000);
        uint32_t elapsed = millis() - t0;
        printf("   queued for ~3 s, limit raised mid-move, took %lu ms\n",
               (unsigned long)elapsed);
        TEST_RESULT("raising the speed limit mid-move is accepted", queued && loosened);
        TEST_RESULT("the queued move still takes its planned duration",
                    drained && (fatalOf(m) == 0) && elapsed > 2500);
    }

    // =====================================================================
    // 2. LOWERING THE LIMIT MID-MOVE. The control loop re-checks the LIVE
    //    velocity against max_velocity every tick (motor_control.c:2187), so
    //    the outcome depends on whether the new limit is above or below the
    //    speed the move is already travelling at.
    //
    //    2a: lower the limit but stay ABOVE the in-flight velocity -> safe.
    //    2b: lower it BELOW the in-flight velocity -> ERROR_VEL_TOO_HIGH (16),
    //        even though the move was legal when it was queued. This is the
    //        asymmetry a caller has to know about: you cannot slow a machine
    //        down by tightening the limit, you fault it.
    // =====================================================================
    printf("\n--- 2. lowering the limit mid-move: safe above, fatal below ---\n");
    {
        // 2a. 1 rotation over 2 s peaks near 0.5 rot/s. Drop the ceiling from
        //     5 to 2 rot/s: still comfortably above, so nothing should happen.
        freshStart(m);
        m->setMaximumVelocity(5.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR, 2 * TPS);
        bool queued = (m->getError() == 0);
        delay(200);
        m->setMaximumVelocity(2.0f);            // still above the ~0.5 rot/s peak
        bool tightened = (m->getError() == 0);
        bool drained = drainQueue(m, 15000);
        int64_t moved = m->getPositionRaw() - before;
        uint8_t fat = fatalOf(m);
        printf("   2a: ceiling 5 -> 2 rot/s mid-move: moved %lld of %ld (fatal=%u)\n",
               (long long)moved, (long)CPR, (unsigned)fat);
        TEST_RESULT("lowering the ceiling but staying above the in-flight speed is accepted",
                    queued && tightened);
        TEST_RESULT("a move under a still-satisfied ceiling completes normally",
                    drained && fat == 0 && llabs((long long)(moved - CPR)) <= CPR / 100);

        // 2b. Same move, but drop the ceiling to 0.2 rot/s -- below the speed
        //     it is actually travelling at. The per-tick check must fire.
        freshStart(m);
        m->setMaximumVelocity(5.0f);
        m->setMaximumAcceleration(2000.0f);
        m->trapezoidMoveRaw(CPR, 2 * TPS);
        bool queued2 = (m->getError() == 0);
        delay(200);
        m->setMaximumVelocity(0.2f);            // below the in-flight velocity
        bool accepted2 = (m->getError() == 0);
        delay(500);
        uint8_t fat2 = fatalOf(m);
        printf("   2b: ceiling 5 -> 0.2 rot/s mid-move: fatal=%u (expect 16)\n",
               (unsigned)fat2);
        TEST_RESULT("the setter itself is still accepted when it undercuts the move",
                    queued2 && accepted2);
        TEST_RESULT("lowering the ceiling below the in-flight velocity raises "
                    "ERROR_VEL_TOO_HIGH (16)",
                    fat2 == 16);
    }

    // =====================================================================
    // 3. THE NEXT MOVE DOES OBEY THE NEW LIMIT. The flip side of 1 and 2: a
    //    limit change must be effective for everything queued afterwards, or
    //    the setter would be useless.
    // =====================================================================
    printf("\n--- 3. the next move obeys the new limit ---\n");
    {
        // Same move, timed under a fast limit and then under a slow one.
        freshStart(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        uint32_t tf0 = millis();
        m->trapezoidMoveRaw(CPR, TPS / 2);      // ask for 0.5 s; 4 rot/s allows it
        bool okFast = (m->getError() == 0) && drainQueue(m, 15000);
        uint32_t fastMs = millis() - tf0;

        freshStart(m);
        m->setMaximumVelocity(0.5f);
        m->setMaximumAcceleration(2000.0f);
        uint32_t ts0 = millis();
        m->trapezoidMoveRaw(CPR, TPS / 2);      // same ask; 0.5 rot/s forbids it
        bool acceptedSlow = (m->getError() == 0);
        bool drainedSlow = acceptedSlow && drainQueue(m, 15000);
        uint32_t slowMs = millis() - ts0;
        uint8_t fatSlow = fatalOf(m);

        printf("   1 rotation asked in 0.5 s: at 4 rot/s took %lu ms; at 0.5 rot/s -> "
               "accepted=%d fatal=%u took %lu ms\n",
               (unsigned long)fastMs, (int)acceptedSlow, (unsigned)fatSlow,
               (unsigned long)slowMs);
        TEST_RESULT("the move is fast under a loose limit", okFast && fastMs < 2000);
        // Under the tight limit the firmware must either stretch it or refuse
        // it -- what it must not do is run it fast anyway.
        TEST_RESULT("the same move under a tight limit is stretched or refused, not run fast",
                    (!acceptedSlow) || (fatSlow != 0) || (drainedSlow && slowMs > 1500));
    }

    // =====================================================================
    // 4. IMMEDIATE-EFFECT SETTINGS ARE IMMEDIATE. Motor current, PID
    //    constants and the deviation limit are not part of the queued shape,
    //    so they must apply the moment they are set -- including while a move
    //    is running. Checked by confirming they are accepted mid-move and
    //    that the move is unaffected.
    // =====================================================================
    printf("\n--- 4. non-queued settings apply immediately and do not disturb a move ---\n");
    {
        freshStart(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(100.0f);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR / 2, 2 * TPS);
        bool queued = (m->getError() == 0);
        delay(300);

        m->setMaximumMotorCurrent(200.0f, 200.0f);
        bool curOk = (m->getError() == 0);
        m->setPidConstants(1000, 100, 10000);
        bool pidOk = (m->getError() == 0);
        m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
        m->setMaxAllowablePositionDeviation(2.0f);
        bool devOk = (m->getError() == 0);
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);

        bool drained = drainQueue(m, 15000);
        int64_t moved = m->getPositionRaw() - before;
        uint8_t fat = fatalOf(m);
        printf("   mid-move settings: current=%d pid=%d deviation=%d; moved %lld of %ld\n",
               (int)curOk, (int)pidOk, (int)devOk, (long long)moved, (long)(CPR / 2));
        TEST_RESULT("motor current can be set while a move is running", queued && curOk);
        TEST_RESULT("PID constants can be set while a move is running", pidOk);
        TEST_RESULT("the deviation limit can be set while a move is running", devOk);
        TEST_RESULT("the running move is unaffected by those settings",
                    drained && fat == 0 && llabs((long long)(moved - CPR / 2)) <= CPR / 100);
    }

    // =====================================================================
    // 5. A LIMIT CHANGE BETWEEN TWO QUEUED MOVES. Queue A legally, tighten the
    //    limit, then queue B with the same parameters. Because limits are
    //    checks and not clamps, B must be REFUSED with a limit error rather
    //    than stretched to fit -- and because that refusal is a FATAL error,
    //    it takes the whole machine down, including the legal move A that was
    //    already in the queue. A caller building a plan needs to know that one
    //    rejected move aborts the ones before it.
    // =====================================================================
    printf("\n--- 5. an over-limit move is refused, and the refusal is fatal ---\n");
    {
        freshStart(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR / 2, TPS / 4);        // A: legal at 4 rot/s
        bool qa = (m->getError() == 0);
        uint8_t fatAfterA = fatalOf(m);
        m->setMaximumVelocity(0.5f);                  // now far too slow for the same ask
        bool changed = (m->getError() == 0);
        m->trapezoidMoveRaw(CPR / 2, TPS / 4);        // B: identical ask, now illegal
        delay(300);
        uint8_t fatAfterB = fatalOf(m);
        int64_t moved = m->getPositionRaw() - before;
        printf("   A accepted (fatal=%u); after B fatal=%u, moved %lld\n",
               (unsigned)fatAfterA, (unsigned)fatAfterB, (long long)moved);
        TEST_RESULT("the legal move A is accepted", qa && fatAfterA == 0);
        TEST_RESULT("the limit change between the two moves is accepted", changed);
        // 15 accel too high / 16 vel too high / 28 predicted velocity too high
        // are the queue-time limit refusals; which one fires depends on which
        // check the derived numbers hit first.
        TEST_RESULT("the over-limit move B is refused with a limit error, not stretched",
                    fatAfterB == 15 || fatAfterB == 16 || fatAfterB == 28);
        TEST_RESULT("the refusal aborts the queue rather than running A to completion",
                    llabs((long long)moved) < CPR / 4);
    }

    // =====================================================================
    // 6. SETTINGS PERSIST UNTIL RESET, AND RESET RESTORES THE DEFAULTS.
    //    Probed through the REFUSAL, since limits do not stretch: a move that
    //    is over the tightened limit must be refused while the limit is in
    //    force, and the very same move must be accepted once a reset has put
    //    the factory limit back. A limit that silently reverted, or one that
    //    survived a reset, would both break the "reset gives you a known
    //    machine" assumption that every module in this suite relies on.
    // =====================================================================
    printf("\n--- 6. settings persist until reset, then revert to defaults ---\n");
    {
        freshStart(m);
        m->setMaximumVelocity(0.5f);
        m->setMaximumAcceleration(2000.0f);
        // Do unrelated work; the limit must still be in force afterwards.
        for (int k = 0; k < 5; k++) { m->getStatus(); m->getPositionRaw(); }
        m->trapezoidMoveRaw(CPR, TPS / 4);      // 1 rotation in 0.25 s: way over 0.5 rot/s
        delay(300);
        uint8_t fatLimited = fatalOf(m);
        printf("   over-limit move while the 0.5 rot/s limit is set: fatal=%u\n",
               (unsigned)fatLimited);
        TEST_RESULT("a speed limit is still enforced after unrelated commands",
                    fatLimited == 15 || fatLimited == 16 || fatLimited == 28);

        // After a reset the factory default (9.3333 rot/s) is back, so the very
        // same move must now be accepted and complete.
        freshStart(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR, TPS / 4);
        bool accepted = (m->getError() == 0);
        bool drained = accepted && drainQueue(m, 15000);
        uint8_t fatDefault = fatalOf(m);
        int64_t moved = m->getPositionRaw() - before;
        printf("   same move after reset: accepted=%d fatal=%u moved=%lld\n",
               (int)accepted, (unsigned)fatDefault, (long long)moved);
        TEST_RESULT("a reset restores the factory speed limit, so the same move now runs",
                    drained && fatDefault == 0 &&
                    llabs((long long)(moved - CPR)) <= CPR / 100);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    freshStart(m);
    int64_t bFin = m->getPositionRaw();
    m->trapezoidMoveRaw(CPR / 4, TPS);
    bool finOk = (m->getError() == 0) && drainQueue(m, 8000);
    TEST_RESULT("an ordinary move still works after the timing sweep",
                finOk && (fatalOf(m) == 0) &&
                llabs((long long)(m->getPositionRaw() - bFin - CPR / 4)) < 200);

    freshStart(m);
}
