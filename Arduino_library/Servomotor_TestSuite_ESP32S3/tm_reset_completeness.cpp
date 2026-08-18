#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_reset_completeness.cpp
//
// DOES A RESET RESTORE *EVERYTHING*?
//
// Every module in this suite begins with a reset and assumes it produces a
// known machine. That assumption is load-bearing for the entire suite: if a
// reset left even one setting behind, tests would silently influence each other
// and a failure could be caused by whatever ran before it.
//
// `tm_reset_edges` checks that a reset SURVIVES difficult timing (back to back,
// mid-move, mid-fault). `tm_settings_persistence` checks that settings persist
// while they are supposed to. Neither asks the complementary question:
//
//     After a reset, is every settable thing actually back to its default?
//
// Answering it needs care, because most settings have NO getter -- you cannot
// read back the speed limit. So each one is probed BEHAVIOURALLY: set it to a
// value whose effect is observable, reset, and check the effect is gone. The
// probes are built around the fact that limits are pass/fail checks, so a move
// that is refused under a tightened limit and accepted under the default is a
// reliable one-bit readout of which limit is in force.
//
// What is covered, and how each is observed:
//   max velocity              -- a move that only the default ceiling permits
//   max acceleration          -- likewise
//   max position deviation    -- a large MOSFETs-off move only the raised
//                                limit permits (default trips error 45)
//   safety limits (the fence) -- a move outside a narrow fence
//   position / origin         -- read directly
//   the motion queue          -- read directly
//   latched fatal error       -- read directly
//   the device clock          -- read directly
//   CRC32 framing             -- an unprotected frame is refused again
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position is read, and the
// planner advances it regardless of the output stage, so nothing turns. No
// broadcasts and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// A move that the FACTORY DEFAULT limits comfortably permit: one rotation in
// one second needs about 1.6 rot/s peak, well under the 9.33 rot/s default,
// and stays inside the 2-rotation default deviation window.
static bool defaultMovePasses(Servomotor* m) {
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS);
    if (m->getError() != 0) return false;
    if (!tfhDrain(m, 12000)) return false;
    if (tfhFatal(m) != 0) return false;
    return llabs((long long)(m->getPositionRaw() - before - TFH_CPR)) <= TFH_CPR / 100;
}

void tm_reset_completeness(void) {
    Serial.println("tm_reset_completeness: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 0. THE BASELINE. Everything below compares against this: on a
    //    freshly-reset machine the reference move must pass. If it did not,
    //    every subsequent "the reset restored it" verdict would be vacuous.
    // =====================================================================
    printf("\n--- 0. the reference move passes on a fresh machine ---\n");
    {
        tfhReset(m);
        bool ok = defaultMovePasses(m);
        TEST_RESULT("one rotation in one second passes at the factory defaults", ok);
    }

    // =====================================================================
    // 1. MAX VELOCITY. Tighten it until the reference move is refused, reset,
    //    and require the reference move to pass again.
    // =====================================================================
    printf("\n--- 1. the speed limit is restored ---\n");
    {
        tfhReset(m);
        m->setMaximumVelocity(0.2f);
        bool setOk = (m->getError() == 0);
        bool refusedWhileTight = !defaultMovePasses(m);
        tfhReset(m);
        bool restored = defaultMovePasses(m);
        printf("   tightened=%d refused-while-tight=%d passes-after-reset=%d\n",
               (int)setOk, (int)refusedWhileTight, (int)restored);
        TEST_RESULT("tightening the speed limit does change behaviour",
                    setOk && refusedWhileTight);
        TEST_RESULT("a reset restores the default speed limit", restored);
    }

    // =====================================================================
    // 2. MAX ACCELERATION. Same shape, different limit.
    // =====================================================================
    printf("\n--- 2. the acceleration limit is restored ---\n");
    {
        tfhReset(m);
        m->setMaximumAcceleration(0.05f);
        bool setOk = (m->getError() == 0);
        bool refusedWhileTight = !defaultMovePasses(m);
        tfhReset(m);
        bool restored = defaultMovePasses(m);
        printf("   tightened=%d refused-while-tight=%d passes-after-reset=%d\n",
               (int)setOk, (int)refusedWhileTight, (int)restored);
        TEST_RESULT("tightening the acceleration limit does change behaviour",
                    setOk && refusedWhileTight);
        TEST_RESULT("a reset restores the default acceleration limit", restored);
    }

    // =====================================================================
    // 3. MAX ALLOWABLE POSITION DEVIATION. The default is two rotations, so
    //    with the MOSFETs off a four-rotation move trips error 45. Raise the
    //    limit so it passes, reset, and require it to trip again -- this one
    //    is checked in the OPPOSITE direction from the others, which is what
    //    makes it a real test of restoration rather than of the setter.
    // =====================================================================
    printf("\n--- 3. the deviation limit is restored ---\n");
    {
        // With the raised limit, a four-rotation move completes.
        tfhFreshOpen(m);                    // raises deviation limit + ceilings
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 4), 4 * TFH_TPS);
        bool acceptedRaised = (m->getError() == 0) && tfhDrain(m, 15000) &&
                              (tfhFatal(m) == 0);
        int64_t movedRaised = acceptedRaised ? (m->getPositionRaw() - before) : 0;

        // After a plain reset the default two-rotation window is back, so the
        // same move must now trip the deviation check.
        tfhReset(m);
        m->setMaximumVelocity(15.0f);
        m->setMaximumAcceleration(5000.0f);  // ceilings only; NOT the deviation
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 4), 4 * TFH_TPS);
        bool accepted = (m->getError() == 0);
        tfhDrain(m, 15000);
        uint8_t fat = tfhFatal(m);
        printf("   with the limit raised: moved %lld; after reset: accepted=%d fatal=%u\n",
               (long long)movedRaised, (int)accepted, (unsigned)fat);
        TEST_RESULT("a four-rotation MOSFETs-off move completes with the deviation limit raised",
                    acceptedRaised && llabs((long long)(movedRaised - TFH_CPR * 4)) <= TFH_CPR / 20);
        TEST_RESULT("a reset restores the two-rotation deviation default, so the same move trips",
                    fat != 0);
        tfhReset(m);
    }

    // =====================================================================
    // 4. THE SAFETY FENCE. Set a narrow fence, confirm it bites, reset, and
    //    confirm it is gone.
    // =====================================================================
    printf("\n--- 4. the safety fence is restored ---\n");
    {
        tfhReset(m);
        m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
        m->setSafetyLimits(-0.05f, 0.05f);
        bool setOk = (m->getError() == 0);
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        bool refusedInsideFence = !defaultMovePasses(m);
        tfhReset(m);
        bool restored = defaultMovePasses(m);
        printf("   fence set=%d blocked-while-fenced=%d passes-after-reset=%d\n",
               (int)setOk, (int)refusedInsideFence, (int)restored);
        TEST_RESULT("a narrow safety fence does block the reference move",
                    setOk && refusedInsideFence);
        TEST_RESULT("a reset removes the safety fence", restored);
    }

    // =====================================================================
    // 5. THE DIRECTLY-READABLE STATE. Position, queue, fatal error and clock
    //    are all observable, so they can be checked without a behavioural
    //    proxy. Each is disturbed as far as possible first.
    // =====================================================================
    printf("\n--- 5. position, queue, fault and clock are restored ---\n");
    {
        // Disturb everything at once: move a long way, queue more work, fault
        // the machine, and let the clock run.
        tfhFreshOpen(m);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 2), 2 * TFH_TPS);
        tfhDrain(m, 15000);
        int64_t wanderedTo = m->getPositionRaw();
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        for (int k = 0; k < 4; k++) m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), 8 * TFH_TPS);
        uint8_t depthBefore = m->getNQueuedItems();
        m->setMaximumVelocity(0.2f);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 4);    // force a fault
        delay(400);
        uint8_t faultBefore = tfhFatal(m);

        printf("   before the reset: position %lld, queue %u, fatal %u\n",
               (long long)wanderedTo, (unsigned)depthBefore, (unsigned)faultBefore);

        tfhReset(m);
        int64_t posAfter = m->getPositionRaw();
        bool posOk = (m->getError() == 0);
        uint8_t depthAfter = m->getNQueuedItems();
        uint8_t faultAfter = tfhFatal(m);
        uint64_t clockAfter = m->getCurrentTimeRaw();
        bool clockOk = (m->getError() == 0);

        printf("   after the reset: position %lld, queue %u, fatal %u, clock %llu us\n",
               (long long)posAfter, (unsigned)depthAfter, (unsigned)faultAfter,
               (unsigned long long)clockAfter);

        TEST_RESULT("the machine really was disturbed first",
                    llabs((long long)wanderedTo) > TFH_CPR && depthBefore > 0 &&
                    faultBefore != 0);
        TEST_RESULT("a reset returns the commanded position to zero",
                    posOk && llabs((long long)posAfter) <= 2);
        TEST_RESULT("a reset empties the motion queue", depthAfter == 0);
        TEST_RESULT("a reset clears the latched fatal error", faultAfter == 0);
        // The clock restarts from zero, so a few seconds at most after a reset.
        TEST_RESULT("a reset restarts the device clock near zero",
                    clockOk && clockAfter < 5000000ULL);
    }

    // =====================================================================
    // 6. CRC32 FRAMING. The firmware boots expecting a CRC, so a reset must
    //    put that back however it was left. A caller that reset and kept
    //    sending unprotected frames would otherwise lose the link silently.
    // =====================================================================
    printf("\n--- 6. the CRC32 framing default is restored ---\n");
    {
        tfhReset(m);
        m->enableCRC32();
        m->crc32Control(0);                 // firmware stops requiring a CRC
        bool disabled = (m->getError() == 0);
        m->disableCRC32();                  // library matches
        m->getStatus();
        bool worksWithout = (m->getError() == 0);

        m->systemReset();                   // sent WITHOUT a CRC, correct for now
        delay(1500);
        for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();

        m->getStatus();                     // still unprotected: must now fail
        int errUnprotected = m->getError();
        m->enableCRC32();
        m->getStatus();
        int errProtected = m->getError();
        printf("   crc disabled=%d worked-without=%d; after reset: without=%d with=%d\n",
               (int)disabled, (int)worksWithout, errUnprotected, errProtected);
        TEST_RESULT("CRC32 can be turned off and the link still works",
                    disabled && worksWithout);
        TEST_RESULT("after a reset an unprotected frame is refused again",
                    errUnprotected != 0);
        TEST_RESULT("after a reset a protected frame works", errProtected == 0);
    }

    // =====================================================================
    // 7. ALL AT ONCE. Disturb every setting simultaneously, reset, and
    //    require the reference move to pass. This is the assertion the whole
    //    suite implicitly relies on every time a module calls tfhReset().
    // =====================================================================
    printf("\n--- 7. everything disturbed at once, then reset ---\n");
    {
        bool allRestored = true;
        for (int cycle = 0; cycle < 3 && allRestored; cycle++) {
            tfhReset(m);
            m->setMaximumVelocity(0.2f);
            m->setMaximumAcceleration(0.05f);
            m->setPidConstants(1, 1, 1);
            m->setMaximumMotorCurrent(50.0f, 50.0f);
            m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
            m->setSafetyLimits(-0.02f, 0.02f);
            m->setMaxAllowablePositionDeviation(0.01f);
            m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
            // Confirm the machine really is crippled now.
            if (defaultMovePasses(m)) {
                allRestored = false;
                printf("   cycle %d: the reference move passed while crippled\n", cycle);
                break;
            }
            tfhReset(m);
            if (!defaultMovePasses(m)) {
                allRestored = false;
                printf("   cycle %d: the reference move did NOT pass after the reset\n", cycle);
                break;
            }
        }
        TEST_RESULT("three cycles of crippling every setting and resetting all recover",
                    allRestored);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
