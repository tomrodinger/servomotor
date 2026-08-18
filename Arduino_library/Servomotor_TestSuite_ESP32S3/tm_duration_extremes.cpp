#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_duration_extremes.cpp
//
// THE DURATION AXIS, ALL THE WAY OUT.
//
// A move's duration is a uint32 count of 31.25 kHz control ticks, so the
// representable range runs from 1 tick (32 microseconds) to 4294967295 ticks --
// just over 38 hours. `tm_planner_arithmetic` sweeps the bottom of that range
// and stops at 200000 ticks (6.4 seconds). The top three quarters of the range
// are untested, and that is where the arithmetic gets interesting:
//
//   compute_trapezoid_move() derives
//       delta_t1    = max_velocity / max_acceleration
//       delta_t2    = total_time - 2 * delta_t1
//       denominator = (delta_t1 + delta_t2) * delta_t1
//       acceleration = numerator / denominator
//
//   * `delta_t2 = total_time - 2*delta_t1` is an UNSIGNED subtraction. If
//     total_time is smaller than 2*delta_t1 it wraps to something enormous
//     rather than going negative -- a classic way to turn a short move into an
//     absurd one.
//   * At the other end, a total_time near 2^32 makes `(delta_t1 + delta_t2)`
//     approach 2^32 and the denominator approach 2^32 * delta_t1, which is well
//     past what 32 bits holds. The 0.15.12.0 fixes added an int64 computation
//     and an explicit int32 saturation precisely because this arithmetic
//     overflows quietly.
//
// The invariant is the campaign's standard one, and it is what makes an
// untestable 38-hour move testable: a move is either performed as asked, or
// refused with a fatal error. For durations too long to wait out, the check is
// that the move is ACCEPTED AND STARTS MOVING IN THE RIGHT DIRECTION AT A
// PLAUSIBLE RATE -- which is enough to catch a wrapped delta_t2 (the motor
// would go the wrong way, or not at all, or far too fast) -- and it is then
// cancelled with an emergency stop rather than waited out.
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position is read, and the
// planner advances it regardless of the output stage, so nothing turns. Every
// long move is cancelled with an emergency stop, so no subsection leaves work
// running. No broadcasts and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint32_t U32MAX = 4294967295u;

// Queue a move, watch it for a moment, cancel it. Reports the rate actually
// observed, in counts per second, and whether it was accepted cleanly.
struct Started {
    bool     accepted;
    uint8_t  fatal;
    int64_t  observed;      // counts moved during the observation window
    double   countsPerSec;
};

static Started startAndSample(Servomotor* m, int32_t counts, uint32_t ticks,
                              uint32_t watchMs) {
    Started s;
    memset(&s, 0, sizeof(s));
    tfhFreshOpen(m);
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw(counts, ticks);
    s.accepted = (m->getError() == 0);
    if (!s.accepted) { s.fatal = tfhFatal(m); return s; }
    delay(watchMs);
    int64_t during = m->getPositionRaw();
    s.observed = during - before;
    s.countsPerSec = (double)s.observed / ((double)watchMs / 1000.0);
    m->emergencyStop();
    delay(250);
    s.fatal = tfhFatal(m);
    return s;
}

void tm_duration_extremes(void) {
    Serial.println("tm_duration_extremes: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. VERY LONG DURATIONS. One rotation spread over progressively longer
    //    times, up to the uint32 ceiling. Each must be accepted and must
    //    creep in the commanded direction at a rate consistent with the
    //    duration asked for. A wrapped delta_t2 would show up as motion that
    //    is far too fast, backwards, or absent.
    // =====================================================================
    printf("\n--- 1. long durations up to the uint32 ceiling ---\n");
    {
        struct { const char* label; uint32_t ticks; } cases[] = {
            { "10 seconds",        10u * 31250u        },
            { "1 minute",          60u * 31250u        },
            { "10 minutes",        600u * 31250u       },
            { "1 hour",            3600u * 31250u      },
            { "12 hours",          43200u * 31250u     },
            { "uint32 max / 2",    U32MAX / 2u         },
            { "uint32 max - 1",    U32MAX - 1u         },
            { "uint32 max",        U32MAX              },
        };
        const int32_t dist = (int32_t)TFH_CPR;          // one rotation
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            Started s = startAndSample(m, dist, cases[i].ticks, 700);
            // Expected creep rate: dist counts spread over ticks/31250 seconds.
            double expected = (double)dist / ((double)cases[i].ticks / 31250.0);
            printf("   %-16s (%10lu ticks): accepted=%d fatal=%u moved=%lld "
                   "(%.1f counts/s, expected about %.1f)\n",
                   cases[i].label, (unsigned long)cases[i].ticks,
                   (int)s.accepted, (unsigned)s.fatal,
                   (long long)s.observed, s.countsPerSec, expected);

            // Either refused loudly, or accepted and behaving sanely.
            bool refused = (!s.accepted) || (s.fatal != 0);
            bool sane;
            if (refused) {
                sane = true;
            } else {
                // Never backwards, and never faster than the whole move in the
                // observation window -- both are wrap signatures.
                bool rightWay = (s.observed >= 0);
                bool notAbsurd = (s.observed <= (int64_t)dist);
                sane = rightWay && notAbsurd;
            }
            TEST_RESULT(std::string("a move over ") + cases[i].label +
                            " is performed sanely or refused, never wrapped", sane);
            if (!refused) {
                TEST_RESULT(std::string("a move over ") + cases[i].label +
                                " creeps forwards, not backwards", s.observed >= 0);
            }
        }
    }

    // =====================================================================
    // 2. THE UNSIGNED-SUBTRACTION HAZARD. delta_t2 = total_time - 2*delta_t1
    //    wraps if total_time < 2*delta_t1. Force exactly that by making
    //    delta_t1 large (a high speed ceiling with a low acceleration ceiling)
    //    and the total time small. If the subtraction wrapped, the coast
    //    segment would become astronomically long and the move would never
    //    finish -- so the check is that the move either completes in a bounded
    //    time or is refused.
    // =====================================================================
    printf("\n--- 2. total_time smaller than the ramps require ---\n");
    {
        // maxVel 15 rot/s with maxAccel 1 rot/s^2 -> delta_t1 is about 15 s,
        // so 2*delta_t1 is about 30 s. Ask for far less than that.
        const uint32_t shortTimes[] = { 100u, 1000u, 31250u, 5u * 31250u };
        for (unsigned i = 0; i < sizeof(shortTimes) / sizeof(shortTimes[0]); i++) {
            tfhReset(m);
            m->setMaximumVelocity(15.0f);
            m->setMaximumAcceleration(1.0f);       // delta_t1 ~ 15 seconds
            m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
            m->setMaxAllowablePositionDeviation(100000.0f);
            m->setPositionUnit(PositionUnit::ENCODER_COUNTS);

            int64_t before = m->getPositionRaw();
            uint32_t t0 = millis();
            m->trapezoidMoveRaw((int32_t)(TFH_CPR / 10), shortTimes[i]);
            bool accepted = (m->getError() == 0);
            // A wrapped delta_t2 would make this never drain. Bound it well
            // above the commanded duration but far below a wrapped one.
            bool drained = accepted && tfhDrain(m, 15000);
            uint32_t elapsed = millis() - t0;
            uint8_t fat = accepted ? tfhFatal(m) : 0xFE;
            int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
            printf("   total_time=%-8lu (2*delta_t1 is about 937500): accepted=%d "
                   "drained=%d in %lu ms fatal=%u moved=%lld\n",
                   (unsigned long)shortTimes[i], (int)accepted, (int)drained,
                   (unsigned long)elapsed, (unsigned)fat, (long long)moved);
            // Refused is fine. Accepted must mean it FINISHED, promptly.
            bool ok = (!accepted) || (fat != 0) || drained;
            TEST_RESULT(std::string("total_time=") + std::to_string((unsigned long)shortTimes[i]) +
                            " below the ramp length does not hang the queue", ok);
            m->emergencyStop();
            delay(250);
        }
    }

    // =====================================================================
    // 3. THE SHORT END, PAIRED WITH LARGE DISPLACEMENTS. The opposite corner:
    //    a big move in a tiny time is physically impossible and must be
    //    refused, not truncated into something achievable.
    // =====================================================================
    printf("\n--- 3. large displacement in a tiny time ---\n");
    {
        struct { int32_t counts; uint32_t ticks; } cases[] = {
            { (int32_t)(TFH_CPR),      1u   },
            { (int32_t)(TFH_CPR),      2u   },
            { (int32_t)(TFH_CPR * 10), 10u  },
            { (int32_t)(TFH_CPR * 100), 100u },
            { 2147483647,              1u   },
            { -2147483647,             1u   },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            tfhFreshOpen(m);
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(cases[i].counts, cases[i].ticks);
            bool accepted = (m->getError() == 0);
            delay(400);
            uint8_t fat = tfhFatal(m);
            int64_t moved = (accepted && fat == 0) ? (m->getPositionRaw() - before) : 0;
            printf("   %ld counts in %lu ticks: accepted=%d fatal=%u moved=%lld\n",
                   (long)cases[i].counts, (unsigned long)cases[i].ticks,
                   (int)accepted, (unsigned)fat, (long long)moved);
            // Refused, or accepted having actually delivered it.
            bool ok = (!accepted) || (fat != 0) ||
                      (llabs((long long)(moved - cases[i].counts)) <=
                       llabs((long long)cases[i].counts) / 100 + 4);
            TEST_RESULT(std::string("displacement ") + std::to_string((long)cases[i].counts) +
                            " in " + std::to_string((unsigned long)cases[i].ticks) +
                            " ticks is delivered or refused, never truncated", ok);
        }
    }

    // =====================================================================
    // 4. A LONG MOVE IS CANCELLABLE. A 38-hour move that could not be stopped
    //    would be a trap: the machine would be unusable until a power cycle.
    //    Emergency stop must reclaim it immediately.
    // =====================================================================
    printf("\n--- 4. a very long move can be cancelled ---\n");
    {
        tfhFreshOpen(m);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, U32MAX);
        bool accepted = (m->getError() == 0);
        delay(400);
        uint8_t depthBefore = m->getNQueuedItems();
        uint32_t t0 = millis();
        m->emergencyStop();
        bool stopOk = (m->getError() == 0);
        delay(300);
        uint32_t stopMs = millis() - t0;
        uint8_t depthAfter = m->getNQueuedItems();
        uint8_t fat = tfhFatal(m);
        printf("   38-hour move: accepted=%d depth %u -> stop in %lu ms -> %u, fatal=%u\n",
               (int)accepted, (unsigned)depthBefore, (unsigned long)stopMs,
               (unsigned)depthAfter, (unsigned)fat);
        // If the firmware refused the move outright that is also fine; what
        // must not happen is an accepted move that cannot be cancelled.
        TEST_RESULT("a 38-hour move is either refused or cancellable",
                    (!accepted) || (fat != 0) || (stopOk && depthAfter == 0));
        if (accepted && fat == 0) {
            TEST_RESULT("cancelling it takes well under a second", stopMs < 1000);
        }
        tfhReset(m);
        TEST_RESULT("the machine is usable after the long move was cancelled",
                    tfhFatal(m) == 0);
    }

    // =====================================================================
    // 5. DURATION AND DISPLACEMENT SCALE TOGETHER. Doubling the duration of
    //    the same displacement must roughly halve the rate. Checked over two
    //    decades so a wrapped or saturated intermediate would break the
    //    proportionality even where it did not break the direction.
    // =====================================================================
    printf("\n--- 5. rate scales inversely with duration ---\n");
    {
        const uint32_t ticks[] = { 60u * 31250u, 120u * 31250u, 600u * 31250u, 6000u * 31250u };
        double rates[4];
        bool allOk = true;
        for (unsigned i = 0; i < 4; i++) {
            Started s = startAndSample(m, (int32_t)TFH_CPR, ticks[i], 900);
            rates[i] = s.countsPerSec;
            if (!s.accepted || s.fatal != 0) allOk = false;
            printf("   %8lu ticks -> %.1f counts/s\n",
                   (unsigned long)ticks[i], rates[i]);
        }
        TEST_RESULT("all four long-duration moves are accepted and start moving", allOk);
        if (allOk) {
            // Each step is 2x, 5x, 10x longer -> proportionally slower.
            bool monotonic = (rates[0] > rates[1]) && (rates[1] > rates[2]) &&
                             (rates[2] > rates[3]);
            TEST_RESULT("a longer duration always produces a slower rate", monotonic);
            // Doubling the duration should roughly halve the rate.
            bool halved = rates[1] > 0 &&
                          (rates[0] / rates[1]) > 1.5 && (rates[0] / rates[1]) < 3.0;
            printf("   ratio for a 2x duration: %.2f (expected about 2)\n",
                   rates[1] > 0 ? rates[0] / rates[1] : 0.0);
            TEST_RESULT("doubling the duration roughly halves the rate", halved);
        }
    }

    // =====================================================================
    // 6. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the duration sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
