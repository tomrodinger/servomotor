#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_queue_underrun.cpp
//
// ERROR_RUN_OUT_OF_QUEUE_ITEMS (18): the error the 0.15.12.0 fix exists to
// prevent.
//
// A velocity segment HOLDS its velocity when its time expires. If the queue
// then empties while the velocity is still nonzero, handle_queued_movements()
// finds itself moving with nothing left to execute and raises error 18. That is
// correct and necessary -- the alternative is a motor that runs away silently --
// but it is also the exact failure the one-tick-ramp race used to cause on
// perfectly ordinary moves, about one move in five.
//
// Several modules mention error 18 in passing. None owns it. This one does:
//
//   * it fires when a velocity segment expires with no successor
//   * it does NOT fire when the sequence ends at zero velocity
//   * it does NOT fire on an ordinary trapezoid move, at any ramp length --
//     including the delta_t1 <= 1 regime that used to trigger it
//   * it does not fire merely because the queue is empty; a machine at rest
//     with an empty queue is the normal idle state
//   * once raised it behaves like any other fatal error and a reset clears it
//
// The third of those is the regression guard that matters. It is written as a
// REPEATED trial deliberately: the old bug was probabilistic, so a single pass
// proves nothing. Twenty-five trials in the former race regime give the old
// failure mode a better than 99.6% chance of showing up if it returns.
//
// SAFETY: MOSFETs stay OFF. Velocity moves here are brief and small, and only
// the COMMANDED position is read, which the planner advances regardless of the
// output stage -- so nothing turns even when a segment is deliberately left
// running. Every subsection ends with a reset. No broadcasts and no alias
// changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_RUN_OUT_OF_QUEUE_ITEMS = 18;

void tm_queue_underrun(void) {
    Serial.println("tm_queue_underrun: BEGIN\n");
    Servomotor* m = tfGetMotor();

    const int32_t vel = (int32_t)convertVelocity(1.0f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                 ConversionDirection::TO_INTERNAL);
    const uint32_t shortDur = (uint32_t)convertTime(0.15f, TimeUnit::SECONDS,
                                                    ConversionDirection::TO_INTERNAL);

    // =====================================================================
    // 1. IT FIRES when a velocity segment expires with nothing behind it.
    //    This is the defining case; if it did not fire, a motor could run on
    //    indefinitely with an empty queue and no indication.
    // =====================================================================
    printf("\n--- 1. a velocity segment with no successor raises error 18 ---\n");
    {
        tfhFreshOpen(m);
        m->moveWithVelocityRaw(vel, shortDur);
        bool accepted = (m->getError() == 0);
        delay(900);                              // well past the 0.15 s segment
        uint8_t fat = tfhFatal(m);
        printf("   velocity segment left to expire -> fatal %u (expected %u)\n",
               (unsigned)fat, (unsigned)ERR_RUN_OUT_OF_QUEUE_ITEMS);
        TEST_RESULT("a lone velocity segment is accepted", accepted);
        TEST_RESULT("letting it expire raises a fatal error", fat != 0 && fat != 0xFF);
        TEST_RESULT("and that error is ERROR_RUN_OUT_OF_QUEUE_ITEMS (18)",
                    fat == ERR_RUN_OUT_OF_QUEUE_ITEMS);
    }

    // =====================================================================
    // 2. IT DOES NOT FIRE when the sequence ends at zero velocity. The same
    //    sequence plus an explicit stop must be completely clean -- otherwise
    //    the error would be unavoidable and velocity moves unusable.
    // =====================================================================
    printf("\n--- 2. ending at zero velocity is clean ---\n");
    {
        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        m->moveWithVelocityRaw(vel, shortDur);
        m->moveWithVelocityRaw(0, 1);            // the mandatory stop
        bool accepted = (m->getError() == 0);
        bool drained = accepted && tfhDrain(m, 8000);
        delay(600);                              // give it every chance to fault
        uint8_t fat = tfhFatal(m);
        int64_t moved = (fat == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   velocity segment plus a stop -> fatal %u, moved %lld counts\n",
               (unsigned)fat, (long long)moved);
        TEST_RESULT("a velocity segment followed by a stop is accepted", accepted && drained);
        TEST_RESULT("it raises no error at all", fat == 0);
        TEST_RESULT("and it actually travelled", llabs((long long)moved) > TFH_CPR / 100);

        // It must also stay stopped.
        int64_t p1 = m->getPositionRaw();
        delay(600);
        int64_t p2 = m->getPositionRaw();
        TEST_RESULT("the machine stays stopped after the zero-velocity segment",
                    llabs((long long)(p2 - p1)) <= 2);
    }

    // =====================================================================
    // 3. THE REGRESSION GUARD. An ordinary trapezoid move must NEVER raise
    //    error 18, at any ramp length. Swept across the whole low end of
    //    delta_t1 = maxVelocity/maxAcceleration, including the delta_t1 <= 1
    //    regime that the 0.15.12.0 two-slot fast path was introduced to make
    //    safe, and repeated so a probabilistic failure cannot hide.
    // =====================================================================
    printf("\n--- 3. ordinary trapezoid moves never underrun, at any ramp length ---\n");
    {
        const float vels[] = { 0.1f, 0.3f, 0.5f, 0.8f, 1.5f, 4.0f };
        for (unsigned i = 0; i < sizeof(vels) / sizeof(vels[0]); i++) {
            int underruns = 0, otherFaults = 0, clean = 0;
            const int REPS = 8;
            for (int r = 0; r < REPS; r++) {
                tfhReset(m);
                m->setMaximumVelocity(vels[i]);   // with the factory default accel
                m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), 2 * TFH_TPS);
                if (m->getError() != 0) { otherFaults++; continue; }
                if (!tfhDrain(m, 10000)) { otherFaults++; continue; }
                uint8_t fat = tfhFatal(m);
                if (fat == ERR_RUN_OUT_OF_QUEUE_ITEMS) underruns++;
                else if (fat != 0) otherFaults++;
                else clean++;
            }
            printf("   maxVel=%.1f: %d clean, %d underrun, %d other (of %d)\n",
                   (double)vels[i], clean, underruns, otherFaults, REPS);
            TEST_RESULT(std::string("maxVel=") + std::to_string((double)vels[i]).substr(0, 3) +
                            " never underruns on an ordinary move",
                        underruns == 0);
            TEST_RESULT(std::string("maxVel=") + std::to_string((double)vels[i]).substr(0, 3) +
                            " completes every ordinary move cleanly",
                        clean == REPS);
        }
    }

    // =====================================================================
    // 4. THE FORMER RACE REGIME, HAMMERED. maxVelocity/maxAcceleration of one
    //    tick is where the old failure lived. Twenty-five trials: the old
    //    ~20% failure rate would show up with probability 1 - 0.8^25 > 99.6%.
    // =====================================================================
    printf("\n--- 4. the one-tick-ramp regime, twenty-five trials ---\n");
    {
        const int REPS = 25;
        int underruns = 0, clean = 0, other = 0;
        for (int r = 0; r < REPS; r++) {
            tfhReset(m);
            m->setMaximumVelocity(0.5f);         // 0.5 / 12000 -> a one-tick ramp
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), 2 * TFH_TPS);
            if (m->getError() != 0) { other++; continue; }
            if (!tfhDrain(m, 10000)) { other++; continue; }
            uint8_t fat = tfhFatal(m);
            if (fat == ERR_RUN_OUT_OF_QUEUE_ITEMS) underruns++;
            else if (fat != 0) other++;
            else clean++;
        }
        printf("   %d trials: %d clean, %d underrun, %d other\n",
               REPS, clean, underruns, other);
        TEST_RESULT("twenty-five one-tick-ramp moves produce zero underruns", underruns == 0);
        TEST_RESULT("all twenty-five complete cleanly", clean == REPS);
    }

    // =====================================================================
    // 5. AN EMPTY QUEUE AT REST IS NOT AN UNDERRUN. The error is about moving
    //    with nothing queued, not about having nothing queued. A machine
    //    sitting idle must never fault, however long it sits.
    // =====================================================================
    printf("\n--- 5. an idle machine with an empty queue never underruns ---\n");
    {
        tfhReset(m);
        bool everFaulted = false;
        uint8_t depth = m->getNQueuedItems();
        for (int i = 0; i < 30; i++) {
            if (tfhFatal(m) != 0) { everFaulted = true; break; }
            delay(100);
        }
        printf("   idle for 3 s with queue depth %u: faulted=%d\n",
               (unsigned)depth, (int)everFaulted);
        TEST_RESULT("the queue really is empty while idle", depth == 0);
        TEST_RESULT("three seconds of idling raises no underrun", !everFaulted);
    }

    // =====================================================================
    // 6. RECOVERY. Error 18 must behave like every other fatal error: a reset
    //    clears it and the machine is fully usable again. Repeated, because a
    //    fault raised from inside the motion loop is a plausible place for
    //    state to be left behind.
    // =====================================================================
    printf("\n--- 6. recovery from an underrun ---\n");
    {
        bool allOk = true;
        for (int cycle = 0; cycle < 5 && allOk; cycle++) {
            tfhFreshOpen(m);
            m->moveWithVelocityRaw(vel, shortDur);
            delay(800);
            if (tfhFatal(m) != ERR_RUN_OUT_OF_QUEUE_ITEMS) {
                allOk = false;
                printf("   cycle %d did not underrun as expected\n", cycle);
                break;
            }
            tfhReset(m);
            if (tfhFatal(m) != 0) { allOk = false; printf("   cycle %d did not clear\n", cycle); break; }
            if (m->getNQueuedItems() != 0) { allOk = false; printf("   cycle %d left the queue dirty\n", cycle); break; }
        }
        TEST_RESULT("five underrun-and-recover cycles each fault and each clear", allOk);

        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("a real move works again after recovering from an underrun",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    // =====================================================================
    // 7. A LONG VELOCITY CHAIN IS FINE AS LONG AS IT ENDS AT ZERO. The
    //    positive counterpart to section 1: velocity moves are perfectly
    //    usable, and this shows the whole pattern working end to end.
    // =====================================================================
    printf("\n--- 7. a long velocity chain ending at zero is clean ---\n");
    {
        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        const uint32_t seg = (uint32_t)convertTime(0.08f, TimeUnit::SECONDS,
                                                   ConversionDirection::TO_INTERNAL);
        bool accepted = true;
        for (int i = 0; i < 8 && accepted; i++) {
            m->moveWithVelocityRaw((i % 2 == 0) ? vel : vel / 2, seg);
            if (m->getError() != 0) accepted = false;
        }
        m->moveWithVelocityRaw(0, 1);
        if (m->getError() != 0) accepted = false;
        bool drained = accepted && tfhDrain(m, 12000);
        delay(500);
        uint8_t fat = tfhFatal(m);
        int64_t moved = (fat == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   nine-segment velocity chain -> fatal %u, moved %lld counts\n",
               (unsigned)fat, (long long)moved);
        TEST_RESULT("a nine-segment velocity chain is accepted and completes",
                    accepted && drained);
        TEST_RESULT("it raises no underrun", fat == 0);
        TEST_RESULT("and it travelled a sensible distance",
                    llabs((long long)moved) > TFH_CPR / 20);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
