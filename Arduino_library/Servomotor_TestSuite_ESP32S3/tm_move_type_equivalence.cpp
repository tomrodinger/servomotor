#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_move_type_equivalence.cpp
//
// FOUR WAYS TO COMMAND THE SAME MOTION. THEY MUST AGREE.
//
// The firmware offers four ways to make the shaft travel a given distance:
//
//   1. `trapezoid move`   -- relative displacement over a duration
//   2. `go to position`   -- absolute target over a duration
//   3. a VELOCITY chain   -- hold a speed for a time, then stop
//   4. an ACCELERATION chain -- ramp up, coast, ramp down by hand
//
// The first two share the planner (both call queue_trapezoid_segments in
// firmware 0.15.12.0). The last two are what the planner EMITS. So all four are
// ultimately the same integrator being driven from different entry points, and
// a given net displacement must come out the same whichever entry point is used.
//
// tm_move_composition checks 1 against 2. Nothing checks either against the
// hand-built 3 and 4 -- which is the comparison that would actually catch a
// scaling mistake, because velocity and acceleration are stored at DIFFERENT
// fixed-point scales (VELOCITY_SHIFT_LEFT 12 vs ACCELERATION_SHIFT_LEFT 8).
// The 0.15.12.0 fast path had to compute the velocity directly at its own scale
// rather than deriving it from the acceleration, precisely to avoid throwing
// away those four bits; this module is the check that the two scales still line
// up with each other.
//
// All four are driven through the library's own unit converters rather than
// hand-rolled shift constants, so the test would fail if the LIBRARY's scaling
// disagreed with the firmware's too.
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position is read, and the
// planner advances it regardless of the output stage, so nothing turns. Every
// velocity chain ends with an explicit zero-velocity stop. No broadcasts and no
// alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// A velocity chain: hold `rotPerSec` for `seconds`, then stop.
// Net displacement should be rotPerSec * seconds rotations.
static int64_t byVelocity(Servomotor* m, float rotPerSec, float seconds, bool* ok) {
    tfhFreshOpen(m);
    int64_t before = m->getPositionRaw();
    int32_t v = (int32_t)convertVelocity(rotPerSec, VelocityUnit::ROTATIONS_PER_SECOND,
                                         ConversionDirection::TO_INTERNAL);
    uint32_t t = (uint32_t)convertTime(seconds, TimeUnit::SECONDS,
                                       ConversionDirection::TO_INTERNAL);
    m->moveWithVelocityRaw(v, t);
    if (m->getError() != 0) { *ok = false; return 0; }
    m->moveWithVelocityRaw(0, 1);                 // mandatory stop
    if (m->getError() != 0) { *ok = false; return 0; }
    if (!tfhDrain(m, (uint32_t)(seconds * 1000.0f) + 10000)) { *ok = false; return 0; }
    if (tfhFatal(m) != 0) { *ok = false; return 0; }
    *ok = true;
    return m->getPositionRaw() - before;
}

// A hand-built acceleration chain shaped like a trapezoid: ramp up for tRamp at
// +a, coast for tCoast, ramp down for tRamp at -a, then stop.
// Peak velocity is a * tRamp; net displacement is a*tRamp*(tRamp + tCoast).
static int64_t byAcceleration(Servomotor* m, float accel, float tRamp, float tCoast,
                              bool* ok) {
    tfhFreshOpen(m);
    int64_t before = m->getPositionRaw();
    int32_t a = (int32_t)convertAcceleration(accel,
                    AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED,
                    ConversionDirection::TO_INTERNAL);
    uint32_t tr = (uint32_t)convertTime(tRamp,  TimeUnit::SECONDS,
                                        ConversionDirection::TO_INTERNAL);
    uint32_t tc = (uint32_t)convertTime(tCoast, TimeUnit::SECONDS,
                                        ConversionDirection::TO_INTERNAL);
    m->moveWithAccelerationRaw(a, tr);
    if (m->getError() != 0) { *ok = false; return 0; }
    m->moveWithAccelerationRaw(0, tc);
    if (m->getError() != 0) { *ok = false; return 0; }
    m->moveWithAccelerationRaw(-a, tr);
    if (m->getError() != 0) { *ok = false; return 0; }
    m->moveWithVelocityRaw(0, 1);                 // guarantee it ends at rest
    if (m->getError() != 0) { *ok = false; return 0; }
    if (!tfhDrain(m, (uint32_t)((2 * tRamp + tCoast) * 1000.0f) + 10000)) { *ok = false; return 0; }
    if (tfhFatal(m) != 0) { *ok = false; return 0; }
    *ok = true;
    return m->getPositionRaw() - before;
}

void tm_move_type_equivalence(void) {
    Serial.println("tm_move_type_equivalence: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. A VELOCITY CHAIN TRAVELS ITS PHYSICAL DISTANCE. Establish the
    //    velocity scale against physics before comparing anything to it: if
    //    this is wrong, every comparison below would be wrong together.
    // =====================================================================
    printf("\n--- 1. a velocity chain travels speed x time ---\n");
    {
        struct { float rps; float secs; } cases[] = {
            { 1.0f, 0.5f }, { 2.0f, 0.5f }, { 0.5f, 2.0f }, { 4.0f, 0.25f },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            bool ok = false;
            int64_t moved = byVelocity(m, cases[i].rps, cases[i].secs, &ok);
            int64_t want = (int64_t)((double)cases[i].rps * (double)cases[i].secs
                                     * (double)TFH_CPR);
            printf("   %.1f rot/s for %.2f s -> %lld counts (expected %lld)\n",
                   (double)cases[i].rps, (double)cases[i].secs,
                   (long long)moved, (long long)want);
            TEST_RESULT(std::string("a velocity chain at ") +
                            std::to_string((double)cases[i].rps).substr(0, 3) +
                            " rot/s completes", ok);
            // 1% covers the single-tick stop segment and integer rounding.
            TEST_RESULT(std::string("a velocity chain at ") +
                            std::to_string((double)cases[i].rps).substr(0, 3) +
                            " rot/s travels speed x time",
                        ok && llabs((long long)(moved - want)) <= want / 100 + 4);
        }
    }

    // =====================================================================
    // 2. AN ACCELERATION CHAIN TRAVELS ITS PHYSICAL DISTANCE. Same, for the
    //    other scale. Net displacement of a symmetric trapezoid built by hand
    //    is peakVelocity * (tRamp + tCoast), with peak = accel * tRamp.
    // =====================================================================
    printf("\n--- 2. an acceleration chain travels its integrated distance ---\n");
    {
        struct { float a; float tr; float tc; } cases[] = {
            { 4.0f, 0.25f, 0.5f },   // peak 1.0 rot/s
            { 8.0f, 0.25f, 0.5f },   // peak 2.0 rot/s
            { 2.0f, 0.5f,  1.0f },   // peak 1.0 rot/s
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            bool ok = false;
            int64_t moved = byAcceleration(m, cases[i].a, cases[i].tr, cases[i].tc, &ok);
            double peak = (double)cases[i].a * (double)cases[i].tr;
            double rot  = peak * ((double)cases[i].tr + (double)cases[i].tc);
            int64_t want = (int64_t)(rot * (double)TFH_CPR);
            printf("   a=%.1f tRamp=%.2f tCoast=%.2f (peak %.2f rot/s) -> %lld "
                   "(expected about %lld)\n",
                   (double)cases[i].a, (double)cases[i].tr, (double)cases[i].tc,
                   peak, (long long)moved, (long long)want);
            TEST_RESULT(std::string("acceleration chain ") + std::to_string(i) + " completes", ok);
            // Looser than the velocity case: the discrete integrator adds a
            // half-tick of area at each of the two ramp boundaries.
            TEST_RESULT(std::string("acceleration chain ") + std::to_string(i) +
                            " travels its integrated distance",
                        ok && llabs((long long)(moved - want)) <= want / 20 + 100);
        }
    }

    // =====================================================================
    // 3. THE TWO SCALES AGREE WITH EACH OTHER. The heart of the module: an
    //    acceleration chain engineered to reach exactly the same peak speed
    //    and hold it for the same time as a velocity chain must travel the
    //    same distance. A mismatch between VELOCITY_SHIFT_LEFT and
    //    ACCELERATION_SHIFT_LEFT would show up here as a clean factor.
    // =====================================================================
    printf("\n--- 3. the velocity and acceleration scales agree ---\n");
    {
        // Acceleration chain: a = 4 rot/s^2 for 0.25 s -> peak 1 rot/s,
        // coast 1.0 s, ramp down. Net = 1.0 * (0.25 + 1.0) = 1.25 rotations.
        bool okA = false;
        int64_t byAcc = byAcceleration(m, 4.0f, 0.25f, 1.0f, &okA);
        // Velocity chain covering the same 1.25 rotations at 1 rot/s.
        bool okV = false;
        int64_t byVel = byVelocity(m, 1.0f, 1.25f, &okV);

        printf("   acceleration route: %lld counts; velocity route: %lld counts\n",
               (long long)byAcc, (long long)byVel);
        TEST_RESULT("both routes complete", okA && okV);
        // If the two fixed-point scales disagreed the ratio would be a power of
        // two, not a few percent. 8% absorbs the integrator's ramp rounding.
        TEST_RESULT("the two routes agree to within 8%",
                    okA && okV && byVel != 0 &&
                    llabs((long long)(byAcc - byVel)) <= llabs((long long)byVel) / 12 + 200);
        TEST_RESULT("they are not out by a factor of two or more",
                    okA && okV && byVel != 0 &&
                    llabs((long long)byAcc) < llabs((long long)byVel) * 2 &&
                    llabs((long long)byAcc) > llabs((long long)byVel) / 2);
    }

    // =====================================================================
    // 4. THE PLANNER AGREES WITH THE HAND-BUILT CHAINS. A trapezoid move over
    //    the same distance must land in the same place as the velocity chain
    //    that covers it. This closes the loop: the high-level command and the
    //    primitives it decomposes into are consistent.
    // =====================================================================
    printf("\n--- 4. the planner agrees with the primitives it emits ---\n");
    {
        const int32_t dist = (int32_t)(TFH_CPR + TFH_CPR / 4);   // 1.25 rotations

        tfhFreshOpen(m);
        bool okT = false;
        int64_t byTrap = tfhMove(m, dist, 2 * TFH_TPS, &okT);

        tfhFreshOpen(m);
        bool okG = false;
        int64_t startG = m->getPositionRaw();
        m->goToPositionRaw((int32_t)(startG + dist), 2 * TFH_TPS);
        okG = (m->getError() == 0) && tfhDrain(m, 12000) && (tfhFatal(m) == 0);
        int64_t byGoto = okG ? (m->getPositionRaw() - startG) : 0;

        bool okV = false;
        int64_t byVel = byVelocity(m, 1.0f, 1.25f, &okV);

        printf("   trapezoid %lld, goto %lld, velocity chain %lld (asked %ld)\n",
               (long long)byTrap, (long long)byGoto, (long long)byVel, (long)dist);
        TEST_RESULT("all three routes complete", okT && okG && okV);
        TEST_RESULT("the trapezoid move delivers what was asked",
                    okT && llabs((long long)(byTrap - dist)) <= dist / 100 + 4);
        TEST_RESULT("go to position delivers the same",
                    okG && llabs((long long)(byGoto - dist)) <= dist / 100 + 4);
        TEST_RESULT("the velocity chain covers the same ground",
                    okV && llabs((long long)(byVel - dist)) <= dist / 50 + 100);
        TEST_RESULT("all three agree with each other",
                    okT && okG && okV &&
                    llabs((long long)(byTrap - byGoto)) <= 4 &&
                    llabs((long long)(byTrap - byVel)) <= dist / 50 + 100);
    }

    // =====================================================================
    // 5. SIGN SYMMETRY ACROSS ALL FOUR ROUTES. Each route must mirror exactly
    //    under negation. Sign handling is per-route (the planner negates its
    //    own acceleration; the chains take a signed parameter directly), so
    //    this is four independent checks, not one.
    // =====================================================================
    printf("\n--- 5. every route is sign-symmetric ---\n");
    {
        const int32_t dist = (int32_t)(TFH_CPR / 2);

        tfhFreshOpen(m); bool o1 = false; int64_t tp = tfhMove(m, dist, TFH_TPS, &o1);
        tfhFreshOpen(m); bool o2 = false; int64_t tn = tfhMove(m, -dist, TFH_TPS, &o2);
        TEST_RESULT("the trapezoid route mirrors under negation",
                    o1 && o2 && llabs((long long)(tp + tn)) <= 4);

        tfhFreshOpen(m);
        bool o3 = false; int64_t gp = 0, gn = 0;
        {
            int64_t s = m->getPositionRaw();
            m->goToPositionRaw((int32_t)(s + dist), TFH_TPS);
            o3 = (m->getError() == 0) && tfhDrain(m, 10000) && (tfhFatal(m) == 0);
            gp = o3 ? (m->getPositionRaw() - s) : 0;
        }
        tfhFreshOpen(m);
        bool o4 = false;
        {
            int64_t s = m->getPositionRaw();
            m->goToPositionRaw((int32_t)(s - dist), TFH_TPS);
            o4 = (m->getError() == 0) && tfhDrain(m, 10000) && (tfhFatal(m) == 0);
            gn = o4 ? (m->getPositionRaw() - s) : 0;
        }
        TEST_RESULT("the absolute route mirrors under negation",
                    o3 && o4 && llabs((long long)(gp + gn)) <= 4);

        bool o5 = false, o6 = false;
        int64_t vp = byVelocity(m,  1.0f, 0.5f, &o5);
        int64_t vn = byVelocity(m, -1.0f, 0.5f, &o6);
        printf("   velocity route: +%lld / %lld\n", (long long)vp, (long long)vn);
        TEST_RESULT("the velocity route mirrors under negation",
                    o5 && o6 && llabs((long long)(vp + vn)) <= TFH_CPR / 100 + 4);

        bool o7 = false, o8 = false;
        int64_t ap = byAcceleration(m,  4.0f, 0.25f, 0.5f, &o7);
        int64_t an = byAcceleration(m, -4.0f, 0.25f, 0.5f, &o8);
        printf("   acceleration route: +%lld / %lld\n", (long long)ap, (long long)an);
        TEST_RESULT("the acceleration route mirrors under negation",
                    o7 && o8 && llabs((long long)(ap + an)) <= TFH_CPR / 20 + 100);
    }

    // =====================================================================
    // 6. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the equivalence sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
