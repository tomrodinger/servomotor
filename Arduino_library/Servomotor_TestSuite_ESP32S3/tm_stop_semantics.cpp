#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_stop_semantics.cpp
//
// WHAT "STOP" ACTUALLY MEANS, at every point in a move's life.
//
// tm_emergency_stop checks that the command works. What it does not establish
// is the contract a caller needs in order to trust it as a safety action:
//
//   * stopping BEFORE anything runs, MID-ramp, MID-coast, and AFTER the queue
//     has drained must all be safe and must all leave the same idle machine
//   * a stop must be IMMEDIATE -- the commanded position must stop advancing
//     within a short, bounded time, not coast on for another segment
//   * a stop must be IDEMPOTENT -- stopping an already-stopped machine is a
//     no-op, not an error, because a supervisor may fire it repeatedly
//   * after a stop the machine must accept new work WITHOUT a reset; if a stop
//     required a reset to recover it would be a fault, not a stop
//   * a stop must not silently rewrite the position
//
// The distinction that matters most is the last one plus the fourth: an
// emergency stop is not a fatal error. It aborts the plan but leaves the
// machine usable. If those two ever diverge, every supervisory controller
// built on this command breaks.
//
// SAFETY: MOSFETs stay OFF. Only queue depth and the COMMANDED position are
// read, and the planner advances the commanded position regardless of the
// output stage, so nothing turns. No broadcasts and no alias changes: safe on
// the shared 35-motor rack.
// ---------------------------------------------------------------------------

// How far the commanded position moves in the given window. Used to decide
// whether the machine is actually still.
static int64_t creepOver(Servomotor* m, uint32_t ms) {
    int64_t a = m->getPositionRaw();
    delay(ms);
    int64_t b = m->getPositionRaw();
    return b - a;
}

void tm_stop_semantics(void) {
    Serial.println("tm_stop_semantics: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. STOPPING AT EVERY PHASE OF A MOVE. A long move under slow limits has
    //    a ramp measured in seconds, so each phase can be hit deliberately
    //    rather than by luck.
    // =====================================================================
    printf("\n--- 1. stopping at each phase of a move ---\n");
    {
        struct { const char* name; uint32_t waitMs; } phase[] = {
            { "before the first tick", 0    },
            { "during the ramp up",    400  },
            { "during the coast",      2000 },
            { "during the ramp down",  6500 },
        };
        for (unsigned i = 0; i < sizeof(phase) / sizeof(phase[0]); i++) {
            tfhFreshOpen(m);
            m->setMaximumVelocity(1.0f);
            m->setMaximumAcceleration(1.0f);     // 1 s ramps: phases are reachable
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), 8 * TFH_TPS);
            bool queued = (m->getError() == 0);
            if (phase[i].waitMs) delay(phase[i].waitMs);

            m->emergencyStop();
            bool stopAccepted = (m->getError() == 0);
            delay(300);

            uint8_t q = m->getNQueuedItems();
            uint8_t fat = tfhFatal(m);
            int64_t creep = creepOver(m, 500);

            printf("   stop %-24s: queued=%d accepted=%d queue=%u fatal=%u creep=%lld\n",
                   phase[i].name, (int)queued, (int)stopAccepted,
                   (unsigned)q, (unsigned)fat, (long long)creep);

            TEST_RESULT(std::string("emergency stop ") + phase[i].name + " is accepted",
                        queued && stopAccepted);
            TEST_RESULT(std::string("emergency stop ") + phase[i].name + " empties the queue",
                        q == 0);
            TEST_RESULT(std::string("emergency stop ") + phase[i].name +
                            " leaves the machine still",
                        llabs((long long)creep) <= 2);
            // The key one: a stop is not a fault.
            TEST_RESULT(std::string("emergency stop ") + phase[i].name +
                            " does not latch a fatal error",
                        fat == 0);
        }
    }

    // =====================================================================
    // 2. A STOP IS PROMPT. Not merely "eventually still" but still within a
    //    short bounded window -- otherwise it is a deceleration request, not
    //    a stop, and cannot be relied on as a safety action.
    // =====================================================================
    printf("\n--- 2. a stop takes effect promptly ---\n");
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 2), 4 * TFH_TPS);
        bool queued = (m->getError() == 0);
        delay(600);                                // well into the coast
        int64_t atStop = m->getPositionRaw();
        m->emergencyStop();
        bool accepted = (m->getError() == 0);
        delay(200);                                // a short, bounded window
        int64_t shortlyAfter = m->getPositionRaw();
        delay(800);
        int64_t muchLater = m->getPositionRaw();

        int64_t overshoot = shortlyAfter - atStop;
        int64_t afterwards = muchLater - shortlyAfter;
        printf("   overshoot within 200 ms: %lld counts; further travel after that: %lld\n",
               (long long)overshoot, (long long)afterwards);
        TEST_RESULT("the stop is accepted mid-coast", queued && accepted);
        // Half a rotation of overshoot in 200 ms would mean it never stopped.
        TEST_RESULT("the machine is stationary within 200 ms of the stop",
                    llabs((long long)afterwards) <= 2);
        TEST_RESULT("the overshoot is bounded, not a whole further segment",
                    llabs((long long)overshoot) < TFH_CPR);
    }

    // =====================================================================
    // 3. A STOP IS IDEMPOTENT. A supervisor may fire it repeatedly, or fire it
    //    at a machine that is already idle. Neither may produce an error.
    // =====================================================================
    printf("\n--- 3. stopping repeatedly, and stopping an idle machine ---\n");
    {
        tfhFreshOpen(m);
        bool allOk = true;
        for (int i = 0; i < 10; i++) {
            m->emergencyStop();
            if (m->getError() != 0) { allOk = false; printf("   idle stop %d errored\n", i); break; }
            delay(60);
        }
        uint8_t fat = tfhFatal(m);
        TEST_RESULT("ten emergency stops on an idle machine are all accepted", allOk);
        TEST_RESULT("stopping an idle machine does not latch a fatal error", fat == 0);

        // And repeatedly during a move.
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), 8 * TFH_TPS);
        delay(300);
        bool repOk = true;
        for (int i = 0; i < 5; i++) {
            m->emergencyStop();
            if (m->getError() != 0) { repOk = false; break; }
            delay(80);
        }
        TEST_RESULT("five stops in a row during a move are all accepted", repOk);
        TEST_RESULT("the queue is empty after the repeated stops",
                    m->getNQueuedItems() == 0);
        TEST_RESULT("no fatal error results from the repeated stops", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 4. THE MACHINE IS USABLE AFTER A STOP, WITHOUT A RESET. This is what
    //    separates a stop from a fault. If new work needed a reset first, a
    //    supervisory controller could not use this command at all.
    // =====================================================================
    printf("\n--- 4. new work is accepted after a stop, with no reset ---\n");
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), 8 * TFH_TPS);
        delay(500);
        m->emergencyStop();
        delay(300);

        // No reset here on purpose.
        bool ok = false;
        int64_t before = m->getPositionRaw();
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   after a stop (no reset): move accepted=%d moved=%lld of %lld\n",
               (int)ok, (long long)moved, (long long)(TFH_CPR / 4));
        (void)before;
        TEST_RESULT("a new move is accepted immediately after a stop, with no reset", ok);
        TEST_RESULT("that move delivers its full displacement",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) <= TFH_CPR / 100);

        // And several stop/move cycles in a row.
        bool cyclesOk = true;
        for (int i = 0; i < 5 && cyclesOk; i++) {
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 2 * TFH_TPS);
            if (m->getError() != 0) { cyclesOk = false; break; }
            delay(200);
            m->emergencyStop();
            if (m->getError() != 0) { cyclesOk = false; break; }
            delay(200);
            if (m->getNQueuedItems() != 0) { cyclesOk = false; break; }
        }
        TEST_RESULT("five move-then-stop cycles run without a reset between them", cyclesOk);
        TEST_RESULT("no fatal error accumulates over the cycles", tfhFatal(m) == 0);
    }

    // =====================================================================
    // 5. A STOP DOES NOT REWRITE THE POSITION. It abandons the remaining
    //    travel, but wherever the machine got to must still be reported
    //    faithfully. A stop that zeroed or rounded the position would
    //    silently corrupt a coordinate system that the caller still trusts.
    // =====================================================================
    printf("\n--- 5. a stop does not rewrite the position ---\n");
    {
        tfhFreshOpen(m);
        m->zeroPosition();
        delay(200);
        m->setMaximumVelocity(2.0f);
        m->setMaximumAcceleration(1000.0f);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, 4 * TFH_TPS);
        delay(900);
        int64_t justBefore = m->getPositionRaw();
        m->emergencyStop();
        delay(400);
        int64_t justAfter = m->getPositionRaw();
        // And it must still be there a moment later.
        delay(500);
        int64_t later = m->getPositionRaw();

        printf("   position: %lld before the stop, %lld after, %lld later\n",
               (long long)justBefore, (long long)justAfter, (long long)later);
        TEST_RESULT("the position is not reset to zero by a stop",
                    llabs((long long)justAfter) > TFH_CPR / 100);
        TEST_RESULT("the position after the stop is close to where it was",
                    llabs((long long)(justAfter - justBefore)) < TFH_CPR);
        TEST_RESULT("the position stays put after the stop",
                    llabs((long long)(later - justAfter)) <= 2);
        // Partial travel: it started at zero and stopped partway, so it must be
        // somewhere strictly between the origin and the full commanded distance.
        TEST_RESULT("the stop abandoned the remaining travel rather than completing it",
                    justAfter > 0 && justAfter < TFH_CPR);
    }

    // =====================================================================
    // 6. A STOP CLEARS A MULTI-MOVE PLAN ENTIRELY, not just the segment that
    //    was running. A stop that only dropped the current segment would let
    //    the machine resume into the next one.
    // =====================================================================
    printf("\n--- 6. a stop clears the whole plan, not just the running segment ---\n");
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        for (int i = 0; i < 5; i++) m->trapezoidMoveRaw((int32_t)(TFH_CPR / 20), 4 * TFH_TPS);
        bool queued = (m->getError() == 0);
        uint8_t depthBefore = m->getNQueuedItems();
        delay(400);
        m->emergencyStop();
        delay(400);
        uint8_t depthAfter = m->getNQueuedItems();
        int64_t creep = creepOver(m, 800);
        printf("   depth %u -> stop -> %u, creep over 800 ms: %lld\n",
               (unsigned)depthBefore, (unsigned)depthAfter, (long long)creep);
        TEST_RESULT("a five-move plan really was queued", queued && depthBefore >= 10);
        TEST_RESULT("one stop clears every queued segment", depthAfter == 0);
        TEST_RESULT("the machine does not resume into a later segment",
                    llabs((long long)creep) <= 2);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the stop sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
