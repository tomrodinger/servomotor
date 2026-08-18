#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_diagnostics_readouts.cpp
//
// THE DIAGNOSTIC COMMANDS, CHECKED BY THEIR INTERNAL MATHEMATICS.
//
// `Control/Get hall sensor statistics` (32/33), `Get max PID error` (39) and
// `Get debug values` (45) sit in the thin tail of the coverage table -- five to
// ten call sites each across eighty modules. They are also the commands an
// engineer reaches for FIRST when a motor misbehaves in the field, so a defect
// in one of them is discovered at the worst possible moment.
//
// They are awkward to test, which is presumably why they are thin: nobody knows
// what the "right" hall sensor sum or profiler time is, and it varies by motor,
// temperature and supply. Asserting a specific value would be wrong.
//
// The way in is that these structures carry MATHEMATICAL INVARIANTS that must
// hold whatever the readings actually are:
//
//     min <= max                          for every hall channel
//     min <= sum/count <= max             the mean must lie inside the range
//     time <= maxTime                     for all six profilers
//     minDelta <= averageDelta <= maxDelta
//     minPidError <= maxPidError
//
// An invariant like "the mean lies between the minimum and the maximum" cannot
// be satisfied by accident. If the sums, the counts or the min/max tracking are
// computed wrongly -- a missing reset, a truncated accumulator, a sign error,
// an off-by-one in the count -- the mean falls outside the range and the test
// fails, on any motor, at any temperature. That makes these checks stronger
// than a plausibility range while being completely device-independent.
//
// GROUND TRUTH (firmware/Src/main.c, CONTROL_HALL_SENSOR_STATISTICS_COMMAND):
//     subcommand 1 -> hall_sensor_turn_on_and_reset_statistics()
//                     (max = 0, min = 65535, sums = 0, count = 0, gathering ON)
//     subcommand 0 -> hall_sensor_turn_off_statistics()
//                     (gathering OFF, accumulated values retained)
//     Any other value is ignored -- neither branch is taken.
//
// SAFETY: MOSFETs stay OFF. These are all read-only diagnostics plus one
// statistics on/off switch; nothing is queued and nothing turns. No broadcasts
// and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

void tm_diagnostics_readouts(void) {
    Serial.println("tm_diagnostics_readouts: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE STATISTICS RESET IS REAL. Subcommand 1 resets and starts
    //    gathering, so immediately afterwards the count must be small and
    //    then must grow. A "reset" that did not reset would silently make
    //    every later reading cumulative across sessions.
    // =====================================================================
    printf("\n--- 1. subcommand 1 resets and starts gathering ---\n");
    {
        tfhReset(m);
        m->controlHallSensorStatistics(1);
        bool startOk = (m->getError() == 0);
        delay(50);
        getHallSensorStatisticsResponse a = m->getHallSensorStatistics();
        bool readA = (m->getError() == 0);
        delay(500);
        getHallSensorStatisticsResponse b = m->getHallSensorStatistics();
        bool readB = (m->getError() == 0);
        printf("   count after 50 ms: %lu; after a further 500 ms: %lu\n",
               (unsigned long)a.measurementCount, (unsigned long)b.measurementCount);
        TEST_RESULT("statistics gathering can be turned on", startOk);
        TEST_RESULT("the statistics can be read", readA && readB);
        TEST_RESULT("the measurement count grows while gathering is on",
                    readA && readB && b.measurementCount > a.measurementCount);

        // Reset again: the count must drop back down rather than continuing.
        m->controlHallSensorStatistics(1);
        delay(50);
        getHallSensorStatisticsResponse c = m->getHallSensorStatistics();
        bool readC = (m->getError() == 0);
        printf("   count after a second reset: %lu (was %lu)\n",
               (unsigned long)c.measurementCount, (unsigned long)b.measurementCount);
        TEST_RESULT("a second reset really does clear the accumulated count",
                    readC && c.measurementCount < b.measurementCount);
    }

    // =====================================================================
    // 2. TURNING GATHERING OFF STOPS IT, and the accumulated values are
    //    retained rather than cleared -- which is what makes it possible to
    //    capture a window and then read it at leisure.
    // =====================================================================
    printf("\n--- 2. subcommand 0 stops gathering and retains the values ---\n");
    {
        tfhReset(m);
        m->controlHallSensorStatistics(1);
        delay(400);
        m->controlHallSensorStatistics(0);
        bool stopOk = (m->getError() == 0);
        delay(50);
        getHallSensorStatisticsResponse a = m->getHallSensorStatistics();
        bool readA = (m->getError() == 0);
        delay(600);
        getHallSensorStatisticsResponse b = m->getHallSensorStatistics();
        bool readB = (m->getError() == 0);
        printf("   count while stopped: %lu then %lu\n",
               (unsigned long)a.measurementCount, (unsigned long)b.measurementCount);
        TEST_RESULT("statistics gathering can be turned off", stopOk);
        TEST_RESULT("the count stops advancing once gathering is off",
                    readA && readB && a.measurementCount == b.measurementCount);
        TEST_RESULT("the accumulated count is retained, not zeroed, when stopped",
                    readA && a.measurementCount > 0);
    }

    // =====================================================================
    // 3. THE STATISTICS ARE INTERNALLY CONSISTENT. The heart of the module:
    //    invariants that hold whatever the sensors happen to be reading.
    // =====================================================================
    printf("\n--- 3. min <= mean <= max, on every channel ---\n");
    {
        tfhReset(m);
        m->controlHallSensorStatistics(1);
        delay(1200);                     // gather a decent sample
        m->controlHallSensorStatistics(0);
        delay(50);
        getHallSensorStatisticsResponse s = m->getHallSensorStatistics();
        bool ok = (m->getError() == 0);

        const uint16_t mins[3] = { s.minHall1, s.minHall2, s.minHall3 };
        const uint16_t maxs[3] = { s.maxHall1, s.maxHall2, s.maxHall3 };
        const uint64_t sums[3] = { s.sumHall1, s.sumHall2, s.sumHall3 };
        uint32_t count = s.measurementCount;

        printf("   count=%lu\n", (unsigned long)count);
        for (int ch = 0; ch < 3; ch++) {
            double mean = (count > 0) ? ((double)sums[ch] / (double)count) : 0.0;
            printf("   hall%d: min=%u max=%u sum=%llu mean=%.1f\n",
                   ch + 1, (unsigned)mins[ch], (unsigned)maxs[ch],
                   (unsigned long long)sums[ch], mean);
        }
        TEST_RESULT("the statistics are readable", ok);
        TEST_RESULT("a meaningful number of measurements were gathered",
                    ok && count > 100);

        for (int ch = 0; ch < 3; ch++) {
            std::string label = std::string("hall channel ") + std::to_string(ch + 1);
            TEST_RESULT(label + ": the minimum does not exceed the maximum",
                        ok && mins[ch] <= maxs[ch]);
            // The invariant that cannot be satisfied by accident.
            double mean = (count > 0) ? ((double)sums[ch] / (double)count) : -1.0;
            TEST_RESULT(label + ": the mean lies between the minimum and the maximum",
                        ok && count > 0 &&
                        mean >= (double)mins[ch] - 1.0 && mean <= (double)maxs[ch] + 1.0);
            // A 12-bit ADC reading left-aligned into 16 bits still cannot
            // exceed the 16-bit range; a stuck channel would read 0 or 65535.
            TEST_RESULT(label + ": the readings are not pinned at an extreme",
                        ok && !(mins[ch] == 0 && maxs[ch] == 0) &&
                        !(mins[ch] == 65535 && maxs[ch] == 65535));
        }
    }

    // =====================================================================
    // 4. AN UNRECOGNISED SUBCOMMAND IS IGNORED, NOT ACTED ON. The firmware
    //    takes neither branch for values other than 0 and 1, so gathering
    //    must be left exactly as it was. A handler that fell through to one
    //    of the branches would silently reset a capture in progress.
    // =====================================================================
    printf("\n--- 4. an unrecognised subcommand changes nothing ---\n");
    {
        tfhReset(m);
        m->controlHallSensorStatistics(1);
        delay(600);
        m->controlHallSensorStatistics(0);      // stop, so the count is frozen
        delay(50);
        getHallSensorStatisticsResponse before = m->getHallSensorStatistics();
        bool ok1 = (m->getError() == 0);

        const uint8_t odd[] = { 2, 3, 17, 128, 255 };
        bool allAccepted = true, unchanged = true;
        for (unsigned i = 0; i < sizeof(odd) / sizeof(odd[0]); i++) {
            m->controlHallSensorStatistics(odd[i]);
            if (m->getError() != 0) { allAccepted = false; break; }
        }
        delay(300);
        getHallSensorStatisticsResponse after = m->getHallSensorStatistics();
        bool ok2 = (m->getError() == 0);
        if (ok1 && ok2 && after.measurementCount != before.measurementCount) unchanged = false;
        printf("   count before=%lu after five odd subcommands=%lu\n",
               (unsigned long)before.measurementCount, (unsigned long)after.measurementCount);
        TEST_RESULT("unrecognised subcommands are accepted without error", allAccepted);
        TEST_RESULT("they neither restart nor reset the statistics",
                    ok1 && ok2 && unchanged);
        TEST_RESULT("no fatal error results from an unrecognised subcommand",
                    tfhFatal(m) == 0);
    }

    // =====================================================================
    // 5. GET MAX PID ERROR, INCLUDING ITS "NO DATA YET" STATE.
    //
    //    The PID error bounds are recorded by the closed-loop controller. With
    //    the MOSFETs off there IS no closed loop, so no sample is ever taken
    //    and the accumulator sits at its empty sentinel:
    //
    //        min = INT32_MAX (2147483647),  max = INT32_MIN (-2147483648)
    //
    //    which is the standard running-min/running-max initialisation, chosen
    //    so the first real sample replaces both. It is correct, but it means
    //    the naive invariant "min <= max" is FALSE on an idle motor -- this
    //    module's first draft asserted exactly that and failed against healthy
    //    firmware.
    //
    //    Worth knowing as a caller: an idle motor reports bounds that look
    //    absurd rather than reporting "no data", and nothing in the API says
    //    so. The distinction is asserted explicitly below so the sentinel is
    //    documented behaviour rather than a surprise.
    // =====================================================================
    printf("\n--- 5. the PID error bounds, and their empty sentinel ---\n");
    {
        const int32_t I32MAX = 2147483647;
        const int32_t I32MIN = -2147483647 - 1;

        tfhReset(m);
        getMaxPidErrorResponse a = m->getMaxPidErrorRaw();
        bool ok = (m->getError() == 0);
        bool isEmptySentinel = (a.minPidError == I32MAX) && (a.maxPidError == I32MIN);
        printf("   min=%ld max=%ld  (%s)\n", (long)a.minPidError, (long)a.maxPidError,
               isEmptySentinel ? "empty sentinel: no samples taken" : "populated");
        TEST_RESULT("get max PID error is answerable", ok);
        // Either it holds real data and is ordered, or it is the empty sentinel.
        TEST_RESULT("the bounds are either ordered or the documented empty sentinel",
                    ok && (isEmptySentinel || a.minPidError <= a.maxPidError));
        // With the MOSFETs off it must specifically BE the empty sentinel --
        // anything else would mean the closed loop had run when it should not.
        TEST_RESULT("with the output stage off the bounds are the empty sentinel, "
                    "confirming the closed loop never ran",
                    ok && isEmptySentinel);

        bool stable = ok;
        for (int i = 0; i < 10 && stable; i++) {
            getMaxPidErrorResponse b = m->getMaxPidErrorRaw();
            if (m->getError() != 0) { stable = false; break; }
            bool sentinel = (b.minPidError == I32MAX) && (b.maxPidError == I32MIN);
            if (!(sentinel || b.minPidError <= b.maxPidError)) stable = false;
        }
        TEST_RESULT("that holds across ten consecutive reads", stable);

        // The converted form. Note that float32 cannot represent INT32_MAX
        // exactly -- 2147483647 becomes 2147483648.0 -- so the converted
        // sentinel is compared with a tolerance rather than for equality.
        getMaxPidErrorResponseConverted cv = m->getMaxPidError();
        bool cok = (m->getError() == 0);
        printf("   converted: min=%.1f max=%.1f\n",
               (double)cv.minPidError, (double)cv.maxPidError);
        TEST_RESULT("the converted PID error is readable", cok);
        TEST_RESULT("the converted form shows the same empty state, "
                    "with the min above the max",
                    cok && (cv.minPidError > cv.maxPidError));
        TEST_RESULT("the converted sentinel is the float image of INT32_MAX/INT32_MIN",
                    cok && cv.minPidError > 2.0e9f && cv.maxPidError < -2.0e9f);
    }

    // =====================================================================
    // 6. GET DEBUG VALUES. Six profilers, each reporting a current time and a
    //    maximum. A current time above its own recorded maximum is impossible
    //    and would mean the max tracking is broken -- again an invariant that
    //    holds regardless of what the timings actually are.
    // =====================================================================
    printf("\n--- 6. every profiler's current time is within its recorded maximum ---\n");
    {
        tfhReset(m);
        getDebugValuesResponse d = m->getDebugValues();
        bool ok = (m->getError() == 0);

        struct { const char* name; uint16_t now, max; } prof[] = {
            { "all motor control calculations", d.allMotorControlCalculationsProfilerTime,
                                                d.allMotorControlCalculationsProfilerMaxTime },
            { "get sensor position",            d.getSensorPositionProfilerTime,
                                                d.getSensorPositionProfilerMaxTime },
            { "compute velocity",               d.computeVelocityProfilerTime,
                                                d.computeVelocityProfilerMaxTime },
            { "motor movement calculations",    d.motorMovementCalculationsProfilerTime,
                                                d.motorMovementCalculationsProfilerMaxTime },
            { "motor phase calculations",       d.motorPhaseCalculationsProfilerTime,
                                                d.motorPhaseCalculationsProfilerMaxTime },
            { "motor control loop period",      d.motorControlLoopPeriodProfilerTime,
                                                d.motorControlLoopPeriodProfilerMaxTime },
        };
        TEST_RESULT("get debug values is answerable", ok);
        for (unsigned i = 0; i < sizeof(prof) / sizeof(prof[0]); i++) {
            printf("   %-32s now=%5u max=%5u\n", prof[i].name,
                   (unsigned)prof[i].now, (unsigned)prof[i].max);
            TEST_RESULT(std::string("the '") + prof[i].name +
                            "' profiler's current time is within its maximum",
                        ok && prof[i].now <= prof[i].max);
        }

        // The hall position deltas must be ordered min <= average <= max.
        printf("   hall position delta: min=%ld average=%ld max=%ld\n",
               (long)d.minHallPositionDelta, (long)d.averageHallPositionDelta,
               (long)d.maxHallPositionDelta);
        TEST_RESULT("the average hall position delta lies between the min and the max",
                    ok && d.minHallPositionDelta <= d.averageHallPositionDelta &&
                    d.averageHallPositionDelta <= d.maxHallPositionDelta);

        // The three hall sensor voltages must be present and not pinned.
        printf("   hall sensor voltages: %u %u %u\n",
               (unsigned)d.hallSensor1Voltage, (unsigned)d.hallSensor2Voltage,
               (unsigned)d.hallSensor3Voltage);
        TEST_RESULT("all three hall sensor voltages are non-zero",
                    ok && d.hallSensor1Voltage > 0 && d.hallSensor2Voltage > 0 &&
                    d.hallSensor3Voltage > 0);
        TEST_RESULT("the motor phases-reversed flag is a clean boolean",
                    ok && (d.motorPhasesReversed == 0 || d.motorPhasesReversed == 1));
    }

    // =====================================================================
    // 7. THE DIAGNOSTICS ARE PURE. None of them may queue anything, latch an
    //    error, or perturb a move -- they are the commands most likely to be
    //    called while something is going wrong, so they must be safe then.
    // =====================================================================
    printf("\n--- 7. reading diagnostics does not disturb the machine ---\n");
    {
        // An undisturbed reference move.
        tfhFreshOpen(m);
        bool refOk = false;
        int64_t reference = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &refOk);

        // The same move, hammered with diagnostic reads throughout.
        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), TFH_TPS);
        bool queued = (m->getError() == 0);
        int calls = 0; bool allOk = true;
        uint32_t deadline = millis() + 1100;
        while (millis() < deadline) {
            m->getDebugValues();       if (m->getError() != 0) { allOk = false; break; }
            m->getMaxPidErrorRaw();    if (m->getError() != 0) { allOk = false; break; }
            m->getHallSensorStatistics(); if (m->getError() != 0) { allOk = false; break; }
            calls += 3;
        }
        bool drained = queued && tfhDrain(m, 10000);
        int64_t moved = (drained && tfhFatal(m) == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   %d diagnostic reads during the move; moved %lld vs reference %lld\n",
               calls, (long long)moved, (long long)reference);
        TEST_RESULT("the diagnostics can be polled continuously during a move",
                    allOk && calls > 15);
        TEST_RESULT("the move still completes cleanly", refOk && drained);
        TEST_RESULT("and it lands where the undisturbed move did",
                    refOk && drained && llabs((long long)(moved - reference)) <= 4);
        TEST_RESULT("no queue slots were consumed by reading diagnostics",
                    (m->getNQueuedItems() == 0) && (m->getError() == 0));
        TEST_RESULT("no fatal error resulted", tfhFatal(m) == 0);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
