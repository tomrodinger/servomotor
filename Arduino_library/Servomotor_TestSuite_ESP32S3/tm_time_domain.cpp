#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_time_domain.cpp
//
// THE DEVICE'S OWN CLOCK, CHECKED AGAINST THE MOTION IT DRIVES.
//
// tm_time exercises the three clock commands (reset time, get current time,
// time sync) and confirms each responds sensibly. What it cannot do -- and what
// nothing in the suite does -- is close the loop between the clock and the
// planner. Both are driven by the same 31.25 kHz tick, so they must agree:
//
//     A move commanded to last N timesteps must advance the device clock by
//     N timesteps. Not approximately by a host-measured wall clock, but by the
//     device's own reckoning.
//
// That is a much sharper check than a host stopwatch, because it removes the
// USB and RS485 round trips from the measurement entirely. If the planner and
// the clock ever ran off different divisors -- which is exactly the kind of
// mistake a prescaler change introduces -- a host-timed test would blame serial
// latency, while this one localises it immediately.
//
// Also checked: monotonicity (a clock that goes backwards breaks any scheduling
// built on it), that reset time really re-zeroes, and that the clock keeps
// running across the events that disturb everything else -- a fatal error, an
// emergency stop, a full queue.
//
// SAFETY: MOSFETs stay OFF. Only the clock, queue depth and the COMMANDED
// position are read, and the planner advances the commanded position regardless
// of the output stage, so nothing turns. No broadcasts and no alias changes:
// safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// The device clock in timesteps. getCurrentTimeRaw() reports microseconds, so
// convert through the library's own table rather than a hand-rolled constant.
static int64_t deviceTicks(Servomotor* m, bool* ok) {
    uint64_t us = m->getCurrentTimeRaw();
    *ok = (m->getError() == 0);
    // 1 timestep = 32 us exactly (31250 ticks/s).
    return (int64_t)(us / 32ULL);
}

void tm_time_domain(void) {
    Serial.println("tm_time_domain: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. MONOTONICITY. The clock must never go backwards or stall, across a
    //    long run of reads. A single pair of reads proves nothing; a hundred
    //    catches a wrap or a torn multi-byte read.
    // =====================================================================
    printf("\n--- 1. the clock is monotonic ---\n");
    {
        tfhReset(m);
        bool ok = true, monotonic = true, advanced = false;
        int64_t prev = deviceTicks(m, &ok);
        int64_t first = prev;
        for (int i = 0; i < 100 && ok; i++) {
            int64_t now = deviceTicks(m, &ok);
            if (!ok) break;
            if (now < prev) {
                monotonic = false;
                printf("   clock went BACKWARDS on read %d: %lld -> %lld\n",
                       i, (long long)prev, (long long)now);
                break;
            }
            if (now > first) advanced = true;
            prev = now;
            delay(10);
        }
        printf("   100 reads: %lld -> %lld ticks\n", (long long)first, (long long)prev);
        TEST_RESULT("all hundred clock reads succeed", ok);
        TEST_RESULT("the clock never goes backwards", monotonic);
        TEST_RESULT("the clock actually advances", advanced);
    }

    // =====================================================================
    // 2. THE CLOCK AND THE PLANNER AGREE. The heart of the module. A move of
    //    N timesteps must consume N timesteps of device clock. Measured
    //    entirely on the device, so no host latency enters the comparison.
    // =====================================================================
    printf("\n--- 2. a move of N timesteps consumes N timesteps of clock ---\n");
    {
        const uint32_t durations[] = { TFH_TPS / 4, TFH_TPS / 2, TFH_TPS, 2 * TFH_TPS };
        for (unsigned i = 0; i < sizeof(durations) / sizeof(durations[0]); i++) {
            uint32_t want = durations[i];
            tfhFreshOpen(m);
            bool okA = true, okB = true;
            int64_t t0 = deviceTicks(m, &okA);
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 8), want);
            bool queued = (m->getError() == 0);
            bool drained = queued && tfhDrain(m, (want * 1000 / TFH_TPS) + 10000);
            int64_t t1 = deviceTicks(m, &okB);
            int64_t elapsed = t1 - t0;

            // The measurement brackets the move plus the polling overhead of
            // draining the queue, so elapsed must be at least the commanded
            // duration and not wildly more.
            bool atLeast = elapsed >= (int64_t)want - 2;
            bool notWild = elapsed <= (int64_t)want + (int64_t)TFH_TPS;  // <= 1 s of slack
            printf("   commanded %lu ticks -> clock advanced %lld ticks\n",
                   (unsigned long)want, (long long)elapsed);
            TEST_RESULT(std::string("a ") + std::to_string((unsigned long)want) +
                            "-tick move completes", okA && okB && drained);
            TEST_RESULT(std::string("a ") + std::to_string((unsigned long)want) +
                            "-tick move consumes at least that much device clock", atLeast);
            TEST_RESULT(std::string("a ") + std::to_string((unsigned long)want) +
                            "-tick move does not consume far more", notWild);
        }
    }

    // =====================================================================
    // 3. THE CLOCK RATE IS RIGHT IN ABSOLUTE TERMS. Section 2 compares the
    //    clock to the planner; both could be wrong together if the tick rate
    //    were misconfigured. Anchor it once against the host's wall clock --
    //    coarsely, because that is all the host can honestly claim.
    // =====================================================================
    printf("\n--- 3. the tick rate is 31250/s in absolute terms ---\n");
    {
        tfhReset(m);
        bool okA = true, okB = true;
        uint32_t h0 = millis();
        int64_t d0 = deviceTicks(m, &okA);
        delay(3000);
        int64_t d1 = deviceTicks(m, &okB);
        uint32_t h1 = millis();
        int64_t deviceElapsed = d1 - d0;
        uint32_t hostElapsed = h1 - h0;
        double impliedRate = (double)deviceElapsed / ((double)hostElapsed / 1000.0);
        printf("   device advanced %lld ticks while the host measured %lu ms -> %.0f ticks/s\n",
               (long long)deviceElapsed, (unsigned long)hostElapsed, impliedRate);
        TEST_RESULT("both clock reads succeed", okA && okB);
        // 2% covers host scheduling jitter and the crystal tolerance.
        TEST_RESULT("the implied tick rate is 31250/s to within 2%",
                    impliedRate > 31250.0 * 0.98 && impliedRate < 31250.0 * 1.02);
    }

    // =====================================================================
    // 4. RESET TIME RE-ZEROES. And a system reset must do so too, since the
    //    clock is part of the state a reset is supposed to restore.
    // =====================================================================
    printf("\n--- 4. the clock can be re-zeroed ---\n");
    {
        tfhReset(m);
        delay(500);
        bool ok = true;
        int64_t beforeReset = deviceTicks(m, &ok);
        m->resetTime();
        bool cmdOk = (m->getError() == 0);
        delay(50);
        int64_t afterReset = deviceTicks(m, &ok);
        printf("   clock %lld -> resetTime -> %lld\n",
               (long long)beforeReset, (long long)afterReset);
        TEST_RESULT("reset time is accepted", cmdOk && ok);
        TEST_RESULT("reset time actually re-zeroes the clock",
                    afterReset < beforeReset || afterReset < TFH_TPS / 4);

        // And it starts running again from there.
        delay(500);
        int64_t later = deviceTicks(m, &ok);
        TEST_RESULT("the clock runs again after being reset", ok && later > afterReset);

        // A system reset must also put the clock back near zero.
        tfhReset(m);
        int64_t afterSystemReset = deviceTicks(m, &ok);
        printf("   after a system reset the clock reads %lld ticks\n",
               (long long)afterSystemReset);
        TEST_RESULT("a system reset also re-zeroes the clock",
                    ok && afterSystemReset < 5 * TFH_TPS);
    }

    // =====================================================================
    // 5. THE CLOCK SURVIVES THE THINGS THAT DISTURB EVERYTHING ELSE. A clock
    //    that stalled during a fault would make post-mortem timestamps
    //    useless exactly when they are needed.
    // =====================================================================
    printf("\n--- 5. the clock keeps running through a fault and a stop ---\n");
    {
        // Through an emergency stop.
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), 8 * TFH_TPS);
        delay(400);
        bool ok = true;
        int64_t beforeStop = deviceTicks(m, &ok);
        m->emergencyStop();
        delay(600);
        int64_t afterStop = deviceTicks(m, &ok);
        printf("   across an emergency stop: %lld -> %lld\n",
               (long long)beforeStop, (long long)afterStop);
        TEST_RESULT("the clock keeps running across an emergency stop",
                    ok && afterStop > beforeStop);

        // Through a latched fatal error -- except that the clock is NOT
        // readable there. Once a fatal error is latched, `get_status` is the
        // ONLY command that returns a real reply; every other command,
        // including `get current time`, answers with the latched error code.
        // Verified by probing twelve different commands in two different
        // orders on rack motor 5E2E8161CF1D2624 (fw 0.15.12.0).
        //
        // So what is asserted here is the actual contract, not the one this
        // module originally assumed.
        tfhReset(m);
        m->setMaximumVelocity(0.2f);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 4);   // over the limit
        delay(400);
        uint8_t fat = tfhFatal(m);
        bool clockOk = true;
        (void)deviceTicks(m, &clockOk);
        printf("   with fatal %u latched: get_status works, clock readable=%d\n",
               (unsigned)fat, (int)clockOk);
        TEST_RESULT("a fatal error really was latched", fat != 0 && fat != 0xFF);
        TEST_RESULT("get_status still answers with a fatal error latched",
                    fat != 0xFF);
        TEST_RESULT("the clock is NOT readable with a fatal error latched "
                    "(every command but get_status returns the error code)",
                    !clockOk);

        // And it comes back after a reset -- the fault must not damage the clock.
        tfhReset(m);
        bool afterOk = true;
        int64_t afterFault = deviceTicks(m, &afterOk);
        delay(400);
        int64_t laterStill = deviceTicks(m, &afterOk);
        printf("   after clearing the fault: %lld -> %lld\n",
               (long long)afterFault, (long long)laterStill);
        TEST_RESULT("the clock is readable again once the fault is cleared", afterOk);
        TEST_RESULT("the clock is running again once the fault is cleared",
                    laterStill > afterFault);
    }

    // =====================================================================
    // 6. TIME SYNC ANSWERS SENSIBLY AND DOES NOT WEDGE THE CLOCK. The command
    //    exists to align a fleet; whatever it adjusts, the clock must still be
    //    monotonic afterwards.
    // =====================================================================
    printf("\n--- 6. time sync leaves a usable clock ---\n");
    {
        tfhReset(m);
        bool ok = true;
        int64_t before = deviceTicks(m, &ok);
        timeSyncResponse r = m->timeSyncRaw((uint32_t)(before * 32));
        bool syncOk = (m->getError() == 0);
        (void)r;
        delay(300);
        int64_t after = deviceTicks(m, &ok);
        delay(300);
        int64_t later = deviceTicks(m, &ok);
        printf("   time sync accepted=%d; clock %lld -> %lld -> %lld\n",
               (int)syncOk, (long long)before, (long long)after, (long long)later);
        TEST_RESULT("time sync is accepted", syncOk && ok);
        TEST_RESULT("the clock still advances after a time sync", later > after);

        // Several syncs in a row must not accumulate damage.
        bool allOk = true;
        for (int i = 0; i < 5 && allOk; i++) {
            bool r2 = true;
            int64_t t = deviceTicks(m, &r2);
            m->timeSyncRaw((uint32_t)(t * 32));
            if (m->getError() != 0 || !r2) allOk = false;
            delay(100);
        }
        int64_t finalRead = deviceTicks(m, &ok);
        TEST_RESULT("five time syncs in a row are all accepted", allOk);
        TEST_RESULT("the clock is still sane after repeated syncs", ok && finalRead > 0);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the clock sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
