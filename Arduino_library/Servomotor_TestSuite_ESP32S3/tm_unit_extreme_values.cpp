#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_unit_extreme_values.cpp
//
// EVERY USER UNIT, DRIVEN TO ITS EXTREMES.
//
// tm_unit_equivalence proves the units AGREE WITH EACH OTHER at ordinary
// magnitudes: a quarter turn is a quarter turn however it is spelled. That is a
// RELATIVE property, and it is silent about the place users actually get hurt --
// the ends of the range. tm_unit_conversions and tm_units_roundtrip are pure
// arithmetic with no motor involved. tm_float_edges pokes NaN/Inf at ONE command
// (Trapezoid move, cmd 2) in ONE unit (shaft rotations).
//
// This module is the ABSOLUTE, per-unit, per-axis version. For position, time,
// velocity and acceleration it commands values that are very large, very small,
// denormal, exactly zero, negative zero and negative, and asserts one invariant:
//
//     PERFORMED AS ASKED, OR REFUSED WITH A FATAL ERROR.
//     NEVER SILENTLY SOMETHING ELSE.
//
// "As asked" is deliberately NOT "as arithmetically ideal". Every converting
// method in the library funnels the user's value through a FLOAT32 multiply and
// then a C cast to the integer wire field, e.g. Arduino_library/Servomotor.cpp:
//
//     float displacement_internal = ::convertPosition(displacement, m_positionUnit, TO_INTERNAL);
//     trapezoidMoveRaw((int32_t)(displacement_internal), (uint32_t)(duration_internal));
//
// float32 carries 24 mantissa bits, and several conversion factors are not
// exact binary fractions (CONVERSION_FACTOR_DEGREES 9102.222222222f,
// CONVERSION_FACTOR_RADIANS 521518.917523523f). So the honest expectation for
// 1.0 degree is 9102 counts -- the TRUNCATION of 9102.222..., not 9102 rounded
// from something, and certainly not 9103. Every prediction below is therefore
// computed by calling the LIBRARY'S OWN convert* function and applying the same
// cast, so the test asserts what is REPRESENTABLE rather than what is ideal.
// A test that asserted the ideal would fail on correct firmware.
//
// The four ends that are checked, and why each one is a real hazard:
//
//   * ROUNDS TO ZERO. A value too small to reach one wire unit becomes 0. For a
//     DISPLACEMENT that is harmless (a dwell). For a DURATION it is fatal error
//     34: add_trapezoid_move_to_queue rejects total_time == 0 rather than
//     dividing by zero (firmware/Src/motor_control.c:2051-2053), and
//     Move with velocity / Move with acceleration reject it too
//     (firmware/Src/main.c:527-541, :608-622). For a LIMIT it is also 34:
//     set_max_velocity and set_max_acceleration refuse a zero ceiling
//     (motor_control.c:3343-3345, :3362-3364) because a zero ceiling would turn
//     every later move into a silent zero-motion dwell.
//   * DENORMAL. 1e-40f is subnormal; the ESP32-S3 FPU may flush it to zero.
//     Either way the cast yields 0, so the outcome must be identical to an exact
//     zero -- that equivalence is what is asserted, not the intermediate float.
//   * TOP OF THE WIRE FIELD. Displacement is int32 (about 655 shaft rotations)
//     and duration is uint32. Near 2^31 a float32 quantises to multiples of 128,
//     so the count actually commanded is NOT the arithmetic product. It must
//     still be delivered in full and in the right direction, with no wrap.
//   * CLAMP VS REFUSAL. The two are NOT interchangeable and the firmware uses
//     both, in different places. A ceiling is CLAMPED (set_max_velocity caps at
//     EXPERIMENTAL_MAX_VELOCITY = 2 x MAX_VELOCITY, motor_control.c:3349-3351,
//     about 18.67 rot/s). A MOVE is never clamped -- an over-ambitious move is
//     REFUSED with fatal 15, 16 or 28. Section 7 pins both halves down.
//
// SAFETY. The MOSFETs stay OFF for the whole module and are never enabled: every
// measurement reads the COMMANDED position (Get position, cmd 34), which the
// planner advances whether or not the output stage is energised, so nothing
// turns on any motor at any point. Large displacements are commanded only after
// tfhFreshOpen(), which raises the deviation window off its two-rotation default
// (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION, motor_control.c:97) -- otherwise
// the watchdog, not the unit under test, would be what is measured. Deliberately
// induced fatal errors are cleared by tfhReset(). No broadcasts, no alias
// changes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// Fatal error codes, from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_ACCEL_TOO_HIGH          = 15;
static const uint8_t ERR_VEL_TOO_HIGH            = 16;
static const uint8_t ERR_PREDICTED_VEL_TOO_HIGH  = 28;
static const uint8_t ERR_PARAMETER_OUT_OF_RANGE  = 34;

static int64_t absll(int64_t v) { return v < 0 ? -v : v; }

// ---------------------------------------------------------------------------
// How close the delivered displacement has to be to the predicted one.
//
// The planner splits a trapezoid into integer segments and truncates the
// per-tick acceleration, so it loses a fraction of a count per move: measured
// during this campaign at about 1 count in 1.6-4 million (409599 of 409600,
// 4095999 of 4096000, 2147483640 of 2147483647). A flat 5 counts plus 30 ppm is
// comfortably above that and still thousands of times tighter than any real
// defect this module could hit -- a wrapped int32 misses by ~4e9, a wrong unit
// by a factor of 32, 360 or 1000.
// ---------------------------------------------------------------------------
static int64_t posTol(int64_t predicted) { return 5 + absll(predicted) / 32768; }

// The library's own conversions, applied exactly as the library applies them.
static int32_t predCounts(float v, PositionUnit u) {
    return (int32_t)::convertPosition(v, u, ConversionDirection::TO_INTERNAL);
}
static float rawCounts(float v, PositionUnit u) {
    return ::convertPosition(v, u, ConversionDirection::TO_INTERNAL);
}
// Guarded: a float outside the destination range makes the C cast undefined, so
// never perform one here. Every value this module actually commands has been
// checked to land inside the field; the guard only keeps the PREDICTION honest.
static uint32_t predTicks(float v, TimeUnit u) {
    float f = ::convertTime(v, u, ConversionDirection::TO_INTERNAL);
    if (!(f > 0.0f)) return 0;
    if (f >= 4294967296.0f) return 0xFFFFFFFFu;
    return (uint32_t)f;
}
static uint32_t predVelCeiling(float v, VelocityUnit u) {
    float f = ::convertVelocity(v, u, ConversionDirection::TO_INTERNAL);
    if (!(f > 0.0f)) return 0;
    if (f >= 4294967296.0f) return 0xFFFFFFFFu;
    return (uint32_t)f;
}
static int32_t predVelMove(float v, VelocityUnit u) {
    float f = ::convertVelocity(v, u, ConversionDirection::TO_INTERNAL);
    if (f >= 2147483648.0f) return INT32_MAX;
    if (f <= -2147483648.0f) return INT32_MIN;
    return (int32_t)f;
}
static int32_t predAccel(float v, AccelerationUnit u) {
    float f = ::convertAcceleration(v, u, ConversionDirection::TO_INTERNAL);
    if (f >= 2147483648.0f) return INT32_MAX;
    if (f <= -2147483648.0f) return INT32_MIN;
    return (int32_t)f;
}

// A velocity segment of `wire` units held for `ticks` advances the commanded
// position by wire * ticks / 2^20: the queue item stores velocity << 12 in the
// 2^-32-counts-per-tick fixed point (VELOCITY_SHIFT_LEFT, motor_control.h:65).
static int64_t countsFromVelocitySegment(int32_t wire, uint32_t ticks) {
    return ((int64_t)wire * (int64_t)ticks) / 1048576;
}

struct MoveOutcome {
    int     enqErr;     // reply to the queueing command; 0 == accepted
    uint8_t fatal;      // latched fatal code afterwards (0xFF if unreadable)
    bool    completed;  // drained with no fault
    int64_t moved;      // raw commanded-position delta
};

// One converting Trapezoid move (cmd 2) expressed in the given units.
static MoveOutcome unitTrapezoid(Servomotor* m, bool openCeilings,
                                 PositionUnit pu, float disp,
                                 TimeUnit tu, float dur, uint32_t budgetMs) {
    MoveOutcome o = { 0, 0, false, 0 };
    if (openCeilings) tfhFreshOpen(m); else tfhReset(m);
    int64_t before = m->getPositionRaw();
    m->setPositionUnit(pu);
    m->setTimeUnit(tu);
    m->trapezoidMove(disp, dur);
    o.enqErr = m->getError();
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    if (o.enqErr != 0) { o.fatal = tfhFatal(m); return o; }
    o.completed = tfhDrain(m, budgetMs);
    o.fatal = tfhFatal(m);
    if (o.completed && o.fatal == 0) o.moved = m->getPositionRaw() - before;
    else o.completed = false;
    return o;
}

void tm_unit_extreme_values(void) {
    Serial.println("tm_unit_extreme_values: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. SMALL NONZERO DISPLACEMENTS ARE TRUNCATED TOWARD ZERO, EXACTLY AS
    //    convertPosition PREDICTS.
    //
    //    This is the sharpest place to measure the conversion, because at a few
    //    thousand counts the planner's own rounding is under a single count, so
    //    the whole discrepancy belongs to the float32 pipeline. 1 degree is
    //    9102.222... counts and the wire field is an integer: the device must
    //    move 9102, not 9103 and not 9102.222 rounded to 9102 by luck. The
    //    negative twin catches a conversion that truncates toward minus infinity
    //    on one side only.
    // =====================================================================
    printf("\n--- 1. small displacements truncate toward zero, as convert* predicts ---\n");
    {
        struct { const char* name; PositionUnit unit; float value; } cases[] = {
            { "1 degree",        PositionUnit::DEGREES,         1.0f    },
            { "-1 degree",       PositionUnit::DEGREES,        -1.0f    },
            { "0.001 radians",   PositionUnit::RADIANS,         0.001f  },
            { "0.001 rotations", PositionUnit::SHAFT_ROTATIONS, 0.001f  },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            int32_t predicted = predCounts(cases[i].value, cases[i].unit);
            float   raw       = rawCounts(cases[i].value, cases[i].unit);
            MoveOutcome o = unitTrapezoid(m, false, cases[i].unit, cases[i].value,
                                          TimeUnit::SECONDS, 1.0f, 9000);
            printf("   %-16s float32 -> %.4f counts, cast -> %ld, device moved %lld (enq=%d fatal=%u)\n",
                   cases[i].name, (double)raw, (long)predicted,
                   (long long)o.moved, o.enqErr, (unsigned)o.fatal);
            TEST_RESULT(std::string(cases[i].name) +
                            " moves exactly the number of counts the float32 conversion yields",
                        o.completed && absll(o.moved - (int64_t)predicted) <= posTol(predicted));
        }
        // Pin the two float32 facts themselves, so a regenerated conversion
        // table that changed a factor cannot slip past unnoticed.
        int32_t deg1 = predCounts(1.0f, PositionUnit::DEGREES);
        int32_t rot1 = predCounts(0.001f, PositionUnit::SHAFT_ROTATIONS);
        printf("   truncation check: 1 degree -> %ld (ideal 9102.222), 0.001 rotations -> %ld (ideal 3276.8)\n",
               (long)deg1, (long)rot1);
        TEST_RESULT("the conversion truncates rather than rounds (9102 not 9103, 3276 not 3277)",
                    deg1 == 9102 && rot1 == 3276);
    }

    // =====================================================================
    // 2. SUB-COUNT, DENORMAL, ZERO AND NEGATIVE-ZERO DISPLACEMENTS ARE EXACT
    //    NO-OPS.
    //
    //    A user nudging an axis by a micro-degree, or feeding through a value
    //    that underflowed somewhere upstream, must get a dwell -- accepted, no
    //    motion, no fault. The dangerous alternatives are a fault (an axis that
    //    stops because someone asked for nothing) and a wild jump (a cast that
    //    wrapped). Because the commanded displacement is exactly 0 the planner
    //    has nothing to round, so the delivered motion is asserted at EXACTLY
    //    zero counts rather than within a tolerance.
    // =====================================================================
    printf("\n--- 2. sub-count, denormal and zero displacements are exact no-ops ---\n");
    {
        struct { const char* name; PositionUnit unit; float value; } cases[] = {
            { "0.9 encoder counts",         PositionUnit::ENCODER_COUNTS,  0.9f   },
            { "-0.9 encoder counts",        PositionUnit::ENCODER_COUNTS, -0.9f   },
            { "1e-7 rotations",             PositionUnit::SHAFT_ROTATIONS, 1e-7f  },
            { "1e-40 rotations (denormal)", PositionUnit::SHAFT_ROTATIONS, 1e-40f },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            int32_t predicted = predCounts(cases[i].value, cases[i].unit);
            MoveOutcome o = unitTrapezoid(m, false, cases[i].unit, cases[i].value,
                                          TimeUnit::SECONDS, 0.5f, 9000);
            printf("   %-28s predicted %ld counts, device moved %lld (enq=%d fatal=%u)\n",
                   cases[i].name, (long)predicted, (long long)o.moved,
                   o.enqErr, (unsigned)o.fatal);
            TEST_RESULT(std::string(cases[i].name) +
                            " is accepted as a dwell and moves exactly zero counts",
                        o.completed && o.fatal == 0 && o.moved == 0);
        }
        // Negative zero and a denormal in a second unit are checked as
        // predictions only: they must reach the wire as the same plain zero.
        int32_t negZero  = predCounts(-0.0f, PositionUnit::SHAFT_ROTATIONS);
        int32_t denormRad = predCounts(1e-40f, PositionUnit::RADIANS);
        int32_t exactZero = predCounts(0.0f, PositionUnit::ENCODER_COUNTS);
        printf("   -0.0 rotations -> %ld ; 1e-40 radians -> %ld ; 0.0 counts -> %ld\n",
               (long)negZero, (long)denormRad, (long)exactZero);
        TEST_RESULT("negative zero and denormals reach the wire as a plain zero displacement",
                    negZero == 0 && denormRad == 0 && exactZero == 0);
    }

    // =====================================================================
    // 3. LARGE DISPLACEMENTS, IN EVERY POSITION UNIT, DELIVERED IN FULL.
    //
    //    30 shaft rotations is 98,304,000 counts -- well past 2^24, so float32
    //    can no longer represent every integer in the range and the four
    //    spellings genuinely disagree (the degrees spelling lands 8 counts away
    //    from the rotations spelling). Each is asserted against ITS OWN
    //    prediction, which is the only defensible expectation; they are then
    //    required to agree with each other only to within that float32 spread.
    //
    //    The second half goes to the top of the int32 wire field: 655.35
    //    rotations is about 2,147,450,752 counts, a few thousand short of
    //    INT32_MAX, where a float32 quantises to multiples of 128. This is the
    //    place an int32 wrap would show up as a full-scale move in the WRONG
    //    DIRECTION, which is why the direction is asserted separately.
    // =====================================================================
    printf("\n--- 3. large displacements in every position unit ---\n");
    {
        struct { const char* name; PositionUnit unit; float value; } cases[] = {
            { "30 rotations",        PositionUnit::SHAFT_ROTATIONS, 30.0f          },
            { "10800 degrees",       PositionUnit::DEGREES,         10800.0f       },
            { "188.4955592 radians", PositionUnit::RADIANS,         188.4955592f   },
            { "98304000 counts",     PositionUnit::ENCODER_COUNTS,  98304000.0f    },
        };
        int32_t predicted[4];
        for (unsigned i = 0; i < 4; i++) {
            predicted[i] = predCounts(cases[i].value, cases[i].unit);
            uint32_t ticks = tfhTicksFor((int64_t)predicted[i]);
            float    secs  = (float)ticks / (float)TFH_TPS;
            MoveOutcome o = unitTrapezoid(m, true, cases[i].unit, cases[i].value,
                                          TimeUnit::SECONDS, secs, ticks / 31 + 20000);
            printf("   %-20s predicted %ld counts, device moved %lld (enq=%d fatal=%u)\n",
                   cases[i].name, (long)predicted[i], (long long)o.moved,
                   o.enqErr, (unsigned)o.fatal);
            TEST_RESULT(std::string(cases[i].name) +
                            " delivers the full predicted displacement",
                        o.completed &&
                        absll(o.moved - (int64_t)predicted[i]) <= posTol(predicted[i]));
        }
        int32_t lo = predicted[0], hi = predicted[0];
        for (unsigned i = 1; i < 4; i++) {
            if (predicted[i] < lo) lo = predicted[i];
            if (predicted[i] > hi) hi = predicted[i];
        }
        printf("   float32 spread across the four spellings of 30 rotations: %ld counts\n",
               (long)(hi - lo));
        TEST_RESULT("the four spellings of 30 rotations agree to within float32 resolution",
                    (hi - lo) <= 64);

        // The top of the int32 displacement field, reached through a unit.
        {
            const float bigRot = 655.35f;
            float   raw       = rawCounts(bigRot, PositionUnit::SHAFT_ROTATIONS);
            int32_t bigPred   = predCounts(bigRot, PositionUnit::SHAFT_ROTATIONS);
            bool    inField   = (raw > 0.0f) && (raw < 2147483000.0f);
            uint32_t ticks    = tfhTicksFor((int64_t)bigPred);
            float    secs     = (float)ticks / (float)TFH_TPS;
            MoveOutcome o = { 0, 0, false, 0 };
            if (inField) {
                o = unitTrapezoid(m, true, PositionUnit::SHAFT_ROTATIONS, bigRot,
                                  TimeUnit::SECONDS, secs, ticks / 31 + 40000);
            }
            printf("   655.35 rotations: float32 -> %.1f, cast -> %ld, over %.1f s, device moved %lld (enq=%d fatal=%u)\n",
                   (double)raw, (long)bigPred, (double)secs,
                   (long long)o.moved, o.enqErr, (unsigned)o.fatal);
            TEST_RESULT("a displacement just under INT32_MAX, expressed in rotations, completes",
                        inField && o.completed && o.fatal == 0);
            TEST_RESULT("and it is delivered in full, forward, with no int32 wrap",
                        inField && o.completed && o.moved > 0 &&
                        absll(o.moved - (int64_t)bigPred) <= posTol(bigPred));
        }
    }

    // =====================================================================
    // 4. A DURATION THAT ROUNDS BELOW ONE TICK IS REFUSED, NOT PERFORMED.
    //
    //    One control tick is 32 microseconds, so anything under that converts to
    //    zero timesteps. Zero is the one duration the planner cannot use: it
    //    would divide by zero in compute_trapezoid_move. The firmware therefore
    //    rejects it up front with fatal 34 (motor_control.c:2051). That refusal
    //    is the invariant -- an accepted-but-never-moving move would be the
    //    silent failure this whole module exists to rule out, and is exactly
    //    what firmware before 0.15.4.0 did.
    //
    //    The negative duration is the awkward one and is treated honestly. The
    //    library performs (uint32_t) on a NEGATIVE float, which the C standard
    //    leaves undefined; ARM/Xtensa saturate it to 0 while some hosts wrap it
    //    to a near-2^32 tick count. So the assertion is the property that must
    //    hold either way: the commanded move is never simply carried out.
    // =====================================================================
    printf("\n--- 4. durations below one tick are refused, never silently performed ---\n");
    {
        struct { const char* name; TimeUnit unit; float value; } cases[] = {
            { "1e-6 seconds",     TimeUnit::SECONDS,      1e-6f },
            { "1 microsecond",    TimeUnit::MICROSECONDS, 1.0f  },
            { "31 microseconds",  TimeUnit::MICROSECONDS, 31.0f },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            MoveOutcome o = unitTrapezoid(m, false, PositionUnit::ENCODER_COUNTS, 1000.0f,
                                          cases[i].unit, cases[i].value, 6000);
            printf("   %-16s -> %lu timesteps: enq=%d fatal=%u\n",
                   cases[i].name, (unsigned long)predTicks(cases[i].value, cases[i].unit),
                   o.enqErr, (unsigned)o.fatal);
            TEST_RESULT(std::string(cases[i].name) +
                            " is refused with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                        o.enqErr != 0 && o.fatal == ERR_PARAMETER_OUT_OF_RANGE);
        }
        uint32_t t1 = predTicks(1e-6f, TimeUnit::SECONDS);
        uint32_t t2 = predTicks(1e-40f, TimeUnit::SECONDS);      // denormal
        uint32_t t3 = predTicks(0.0f, TimeUnit::SECONDS);        // exact zero
        uint32_t t4 = predTicks(1e-8f, TimeUnit::MINUTES);
        printf("   sub-tick durations -> %lu, %lu, %lu, %lu timesteps\n",
               (unsigned long)t1, (unsigned long)t2, (unsigned long)t3, (unsigned long)t4);
        TEST_RESULT("a denormal, an exact zero and a sub-tick value all convert to zero timesteps",
                    t1 == 0 && t2 == 0 && t3 == 0 && t4 == 0);

        // A negative duration: refused, or accepted as something enormous --
        // but never performed as the ordinary move that was asked for.
        {
            tfhReset(m);
            int64_t before = m->getPositionRaw();
            m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
            m->setTimeUnit(TimeUnit::SECONDS);
            m->trapezoidMove(100000.0f, -1.0f);
            int enq = m->getError();
            m->setTimeUnit(TimeUnit::TIMESTEPS);
            bool safe;
            if (enq != 0) {
                uint8_t fat = tfhFatal(m);
                printf("   -1.0 seconds: refused, enq=%d fatal=%u\n", enq, (unsigned)fat);
                safe = (fat == ERR_PARAMETER_OUT_OF_RANGE);
            } else {
                delay(1200);
                uint8_t q = m->getNQueuedItems();
                bool qOk = (m->getError() == 0);
                int64_t here = m->getPositionRaw();
                int64_t moved = (m->getError() == 0) ? (here - before) : 0;
                if (!qOk) q = 0;
                printf("   -1.0 seconds: accepted; after 1.2 s queue=%u moved=%lld of 100000\n",
                       (unsigned)q, (long long)moved);
                // Still busy, or stopped short: either way it did not perform
                // "100000 counts in one second".
                safe = (q > 0) || (absll(moved) < 90000);
                m->emergencyStop();
            }
            TEST_RESULT("a negative duration is never carried out as the ordinary move that was asked for",
                        safe);
            tfhReset(m);
        }
    }

    // =====================================================================
    // 5. THE ONE-TICK FLOOR IS REAL AND IS HONOURED.
    //
    //    32 microseconds is exactly one timestep, the smallest duration the
    //    machine has. It must be accepted (the boundary belongs to the legal
    //    side) while 31 microseconds, checked above, is not. And a real
    //    displacement crammed into that single tick must be REFUSED rather than
    //    truncated into a different move: compute_trapezoid_move saturates the
    //    computed acceleration at INT32_MAX precisely so add_to_queue's limit
    //    check rejects it loudly (motor_control.c:510-515, rejected at :1795).
    // =====================================================================
    printf("\n--- 5. the one-tick duration floor ---\n");
    {
        uint32_t us32 = predTicks(32.0f, TimeUnit::MICROSECONDS);
        uint32_t ms32 = predTicks(0.032f, TimeUnit::MILLISECONDS);
        printf("   32 us -> %lu timesteps ; 0.032 ms -> %lu timesteps\n",
               (unsigned long)us32, (unsigned long)ms32);

        MoveOutcome a = unitTrapezoid(m, false, PositionUnit::ENCODER_COUNTS, 0.0f,
                                      TimeUnit::MICROSECONDS, 32.0f, 6000);
        printf("   zero displacement over 32 us: enq=%d fatal=%u moved=%lld\n",
               a.enqErr, (unsigned)a.fatal, (long long)a.moved);
        TEST_RESULT("32 microseconds is exactly one timestep and a zero-displacement move over it is accepted",
                    us32 == 1 && ms32 == 1 && a.completed && a.fatal == 0 && a.moved == 0);

        MoveOutcome b = unitTrapezoid(m, false, PositionUnit::ENCODER_COUNTS, 0.0f,
                                      TimeUnit::MILLISECONDS, 0.032f, 6000);
        printf("   zero displacement over 0.032 ms: enq=%d fatal=%u moved=%lld\n",
               b.enqErr, (unsigned)b.fatal, (long long)b.moved);
        TEST_RESULT("the same one-tick duration spelled in milliseconds behaves identically",
                    b.completed && b.fatal == 0 && b.moved == 0);

        MoveOutcome c = unitTrapezoid(m, false, PositionUnit::ENCODER_COUNTS, 1000.0f,
                                      TimeUnit::MICROSECONDS, 32.0f, 6000);
        printf("   1000 counts over 32 us: enq=%d fatal=%u\n", c.enqErr, (unsigned)c.fatal);
        TEST_RESULT("1000 counts in a single tick is refused with ERROR_ACCEL_TOO_HIGH (15), not truncated",
                    c.enqErr != 0 && c.fatal == ERR_ACCEL_TOO_HIGH);
        tfhReset(m);
    }

    // =====================================================================
    // 6. A SPEED CEILING THAT ROUNDS TO ZERO IS REFUSED, NEVER SILENTLY ZERO.
    //
    //    This is the most valuable refusal in the firmware and the least
    //    obvious. A zero max_velocity does not stop the machine: it makes
    //    compute_trapezoid_move derive a zero ramp, so every subsequent move is
    //    accepted with a success reply, occupies the queue, and never turns the
    //    shaft. set_max_velocity therefore rejects zero outright
    //    (motor_control.c:3343-3345). Any unit whose factor is small enough will
    //    produce that zero from a perfectly innocent-looking user value -- 0.01
    //    counts/second converts to 0.335, which casts to 0.
    // =====================================================================
    printf("\n--- 6. speed ceilings that round to zero are refused ---\n");
    {
        struct { const char* name; VelocityUnit unit; float value; } cases[] = {
            { "0 rotations/second",  VelocityUnit::ROTATIONS_PER_SECOND, 0.0f  },
            { "1e-9 RPM",            VelocityUnit::RPM,                  1e-9f },
            { "0.01 counts/second",  VelocityUnit::COUNTS_PER_SECOND,    0.01f },
        };
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            tfhReset(m);
            m->setVelocityUnit(cases[i].unit);
            m->setMaximumVelocity(cases[i].value);
            int enq = m->getError();
            uint8_t fat = tfhFatal(m);
            m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
            printf("   %-20s -> wire %lu: enq=%d fatal=%u\n",
                   cases[i].name, (unsigned long)predVelCeiling(cases[i].value, cases[i].unit),
                   enq, (unsigned)fat);
            TEST_RESULT(std::string("a maximum velocity of ") + cases[i].name +
                            " is refused with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                        enq != 0 && fat == ERR_PARAMETER_OUT_OF_RANGE);
        }
        uint32_t w1 = predVelCeiling(1e-9f, VelocityUnit::DEGREES_PER_SECOND);
        uint32_t w2 = predVelCeiling(1e-40f, VelocityUnit::ROTATIONS_PER_SECOND);  // denormal
        uint32_t w3 = predVelCeiling(1e-7f, VelocityUnit::COUNTS_PER_TIMESTEP);
        printf("   1e-9 deg/s -> %lu ; 1e-40 rot/s -> %lu ; 1e-7 counts/tick -> %lu\n",
               (unsigned long)w1, (unsigned long)w2, (unsigned long)w3);
        TEST_RESULT("tiny and denormal speed ceilings all reach the wire as exactly zero",
                    w1 == 0 && w2 == 0 && w3 == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE TWO ENDS OF THE SPEED CEILING: SMALLEST ACCEPTED, LARGEST CLAMPED.
    //
    //    At the bottom, one wire unit -- 2^-20 counts per timestep, about 0.03
    //    counts per second -- is nonzero and so is accepted. What matters is
    //    what happens NEXT: an ordinary quarter-turn move under that ceiling
    //    must be REFUSED, not quietly stretched or ignored. Limits in this
    //    firmware are pass/fail checks, never clamps.
    //
    //    At the top the rule inverts, and this is the asymmetry users trip over.
    //    set_max_velocity CLAMPS: asking for 30 rot/s silently yields
    //    EXPERIMENTAL_MAX_VELOCITY, about 18.67 rot/s (motor_control.c:3349).
    //    The clamp is proved behaviourally -- a 16 rot/s velocity move is
    //    accepted (so the ceiling did rise above its 9.33 rot/s default) while
    //    19.5 rot/s is still refused (so it did NOT rise to the 30 that was
    //    asked for). The 16 rot/s case is commanded NEGATIVE so that a large
    //    signed velocity is exercised in the same breath.
    // =====================================================================
    printf("\n--- 7. the smallest accepted speed ceiling, and the clamp at the top ---\n");
    {
        // Bottom end: exactly one wire unit.
        tfhFreshOpen(m);
        m->setVelocityUnit(VelocityUnit::COUNTS_PER_TIMESTEP);
        m->setMaximumVelocity(9.5367431640625e-7f);           // 2^-20 counts/tick
        int setErr = m->getError();
        uint8_t setFat = tfhFatal(m);
        m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
        printf("   smallest nonzero ceiling: wire %lu, enq=%d fatal=%u\n",
               (unsigned long)predVelCeiling(9.5367431640625e-7f, VelocityUnit::COUNTS_PER_TIMESTEP),
               setErr, (unsigned)setFat);
        TEST_RESULT("the smallest nonzero speed ceiling (one wire unit) is accepted",
                    setErr == 0 && setFat == 0);

        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), (uint32_t)TFH_TPS);
        int moveErr = m->getError();
        uint8_t moveFat = tfhFatal(m);
        printf("   quarter turn under that ceiling: enq=%d fatal=%u\n", moveErr, (unsigned)moveFat);
        TEST_RESULT("and an ordinary move under it is refused with a limit error (15, 16 or 28), not silently performed",
                    moveErr != 0 &&
                    (moveFat == ERR_VEL_TOO_HIGH || moveFat == ERR_ACCEL_TOO_HIGH ||
                     moveFat == ERR_PREDICTED_VEL_TOO_HIGH));

        // Top end: an over-large ceiling is clamped, and a move above the clamp
        // is still refused.
        tfhFreshOpen(m);
        m->setMaximumVelocity(30.0f);                          // clamped to ~18.67 rot/s
        int32_t wire16 = predVelMove(-16.0f, VelocityUnit::ROTATIONS_PER_SECOND);
        int64_t before = m->getPositionRaw();
        m->moveWithVelocity(-16.0f, 9375.0f);                  // 0.3 s at -16 rot/s
        int fastErr = m->getError();
        m->moveWithVelocity(0.0f, 3125.0f);                    // the mandatory stop segment
        bool drained = (fastErr == 0) && (m->getError() == 0) && tfhDrain(m, 15000);
        uint8_t fastFat = tfhFatal(m);
        int64_t moved = (drained && fastFat == 0) ? (m->getPositionRaw() - before) : 0;
        int64_t predicted = countsFromVelocitySegment(wire16, 9375);
        printf("   -16 rot/s for 9375 ticks: wire %ld, predicted %lld, moved %lld (enq=%d fatal=%u)\n",
               (long)wire16, (long long)predicted, (long long)moved, fastErr, (unsigned)fastFat);
        TEST_RESULT("a 30 rot/s ceiling lets a -16 rot/s move run and deliver its predicted negative displacement",
                    drained && fastFat == 0 && moved < 0 &&
                    absll(moved - predicted) <= posTol(predicted));

        tfhFreshOpen(m);
        m->setMaximumVelocity(30.0f);
        m->moveWithVelocity(19.5f, 3125.0f);
        int overErr = m->getError();
        uint8_t overFat = tfhFatal(m);
        printf("   19.5 rot/s under the same 30 rot/s request: wire %ld, enq=%d fatal=%u\n",
               (long)predVelMove(19.5f, VelocityUnit::ROTATIONS_PER_SECOND), overErr, (unsigned)overFat);
        TEST_RESULT("but 19.5 rot/s is still refused with ERROR_VEL_TOO_HIGH (16): the 30 was clamped, not granted",
                    overErr != 0 && overFat == ERR_VEL_TOO_HIGH);
        tfhReset(m);
    }

    // =====================================================================
    // 8. ACCELERATION AT BOTH ENDS.
    //
    //    Acceleration has the same zero-ceiling hazard as velocity, for the same
    //    reason: trapezoid planning DIVIDES by max_acceleration, so
    //    set_max_acceleration refuses zero (motor_control.c:3362-3364). And the
    //    acceleration factors are the smallest in the table -- 1 microradian per
    //    second squared is 0.009 on the wire -- so an innocuous-looking value
    //    rounds away to nothing.
    //
    //    The move end is exercised through MULTIMOVE (cmd 29) rather than two
    //    separate Move-with-acceleration commands. That is not decoration: a
    //    high acceleration reaches the speed limit within about 26 ticks
    //    (0.8 ms), far less than an RS485 round trip, so a separately-sent
    //    decelerating segment could never arrive before the queue emptied at
    //    speed and raised fatal 18. Multimove queues the accelerate/decelerate
    //    pair atomically, which is the only way to test a large acceleration
    //    without the underrun masking the result.
    //
    //    The ramp length is a two-sided constraint and ACC_SEG_TICKS sits in
    //    the middle of it:
    //      * UPPER bound -- the accumulated velocity after the up-ramp is
    //        (wire << 8) * ticks and add_to_queue refuses it once it passes
    //        max_velocity (motor_control.c:1802). With the factory default
    //        ceiling (MAX_RPM 560 -> 4203359652212) and 11000 rot/s^2
    //        (wire 619244928 -> 158526701568 per tick) that caps the ramp at
    //        26 ticks. 20 ticks is 75% of the ceiling: comfortably inside, and
    //        the arithmetic is exact integer arithmetic with no jitter.
    //      * LOWER bound -- the two segments are queued by two consecutive
    //        add_to_queue() calls while the 31.25 kHz ISR is already draining
    //        the first one. If the first segment is shorter than the gap
    //        between the calls, the queue empties at full speed and raises
    //        fatal 18 -- the documented one-tick-ramp race
    //        (motor_control.c:1991-2012), which was measured at ~20% of moves
    //        when the first segment was a single 32 us tick. 20 ticks is
    //        640 us, twenty times that gap, so the race cannot be what this
    //        test measures.
    // =====================================================================
    printf("\n--- 8. acceleration at both ends ---\n");
    {
        // Ticks per ramp segment. See the two-sided constraint above: must stay
        // under 26 (the velocity ceiling) and well over 1 (the queue-underrun
        // race). Time unit is TIMESTEPS throughout this section.
        const int64_t ACC_SEG_TICKS = 20;
        const float   ACC_SEG_DUR   = (float)ACC_SEG_TICKS;

        // Zero and round-to-zero ceilings.
        tfhReset(m);
        m->setMaximumAcceleration(0.0f);
        int zeroErr = m->getError();
        uint8_t zeroFat = tfhFatal(m);
        printf("   0 rot/s^2 ceiling: enq=%d fatal=%u\n", zeroErr, (unsigned)zeroFat);
        TEST_RESULT("a maximum acceleration of exactly zero is refused with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    zeroErr != 0 && zeroFat == ERR_PARAMETER_OUT_OF_RANGE);

        tfhReset(m);
        m->setAccelerationUnit(AccelerationUnit::RADIANS_PER_SECOND_SQUARED);
        m->setMaximumAcceleration(1e-6f);
        int tinyErr = m->getError();
        uint8_t tinyFat = tfhFatal(m);
        m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
        printf("   1e-6 rad/s^2 ceiling: wire %ld, enq=%d fatal=%u\n",
               (long)predAccel(1e-6f, AccelerationUnit::RADIANS_PER_SECOND_SQUARED),
               tinyErr, (unsigned)tinyFat);
        TEST_RESULT("an acceleration ceiling too small to reach one wire unit is refused with 34, not treated as zero",
                    tinyErr != 0 && tinyFat == ERR_PARAMETER_OUT_OF_RANGE);

        // A legal high acceleration, queued atomically so it cannot underrun.
        tfhReset(m);
        m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
        int32_t wireOk = predAccel(11000.0f, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
        multimoveListConverted_t okPair[2] = { {  11000.0f, ACC_SEG_DUR },
                                               { -11000.0f, ACC_SEG_DUR } };
        int64_t beforeAcc = m->getPositionRaw();
        m->multimove(2, 0x00000000u, okPair);        // both bits clear == acceleration
        int okErr = m->getError();
        bool okDrained = (okErr == 0) && tfhDrain(m, 8000);
        uint8_t okFat = tfhFatal(m);
        int64_t okMoved = (okDrained && okFat == 0) ? (m->getPositionRaw() - beforeAcc) : 0;
        // Ramping up over n ticks and back down over n advances the position by
        // exactly n^2 tick-velocities of (wire << 8) in 2^-32 counts -- the up
        // ramp contributes a*n(n+1)/2 and the down ramp a*n^2 - a*n(n+1)/2 --
        // i.e. wire * n^2 / 2^24 counts. Integer arithmetic on both sides, so
        // this is an exact prediction; the tolerance below is slack, not need.
        int64_t okPredicted = (int64_t)wireOk * (ACC_SEG_TICKS * ACC_SEG_TICKS) / 16777216;
        printf("   +/-11000 rot/s^2 x %lld ticks: wire %ld, predicted %lld counts, moved %lld (enq=%d fatal=%u)\n",
               (long long)ACC_SEG_TICKS, (long)wireOk, (long long)okPredicted,
               (long long)okMoved, okErr, (unsigned)okFat);
        TEST_RESULT("11000 rot/s^2 is inside the factory ceiling, is accepted, and moves its predicted distance",
                    okDrained && okFat == 0 && absll(okMoved - okPredicted) <= 8);

        // Above the factory ceiling: refused, never reinterpreted.
        tfhReset(m);
        m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
        multimoveListConverted_t hotPair[2] = { {  13000.0f, ACC_SEG_DUR },
                                                { -13000.0f, ACC_SEG_DUR } };
        m->multimove(2, 0x00000000u, hotPair);
        int hotErr = m->getError();
        uint8_t hotFat = tfhFatal(m);
        printf("   +/-13000 rot/s^2: wire %ld, enq=%d fatal=%u\n",
               (long)predAccel(13000.0f, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED),
               hotErr, (unsigned)hotFat);
        TEST_RESULT("13000 rot/s^2 is above the factory ceiling and is refused with ERROR_ACCEL_TOO_HIGH (15)",
                    hotErr != 0 && hotFat == ERR_ACCEL_TOO_HIGH);

        // The rated 12000 rot/s^2 is a knife edge: float32 rounds the product a
        // few dozen wire units ABOVE the firmware's own MAX_ACCELERATION, so it
        // may land on either side. Which side is not the point -- the point is
        // that the machine answers definitely, rather than accepting the command
        // and executing a different acceleration.
        tfhReset(m);
        m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
        multimoveListConverted_t edgePair[2] = { {  12000.0f, ACC_SEG_DUR },
                                                 { -12000.0f, ACC_SEG_DUR } };
        m->multimove(2, 0x00000000u, edgePair);
        int edgeErr = m->getError();
        uint8_t edgeFat = tfhFatal(m);
        printf("   the rated 12000 rot/s^2: wire %ld, enq=%d fatal=%u\n",
               (long)predAccel(12000.0f, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED),
               edgeErr, (unsigned)edgeFat);
        TEST_RESULT("the rated 12000 rot/s^2 gets a definite answer: accepted, or refused with 15",
                    (edgeErr == 0 && edgeFat == 0) ||
                    (edgeErr != 0 && edgeFat == ERR_ACCEL_TOO_HIGH));
        tfhReset(m);
    }

    // =====================================================================
    // 9. THE MACHINE IS UNHARMED BY ALL OF IT.
    //
    //    Every extreme above either completed or faulted, and a fault latches
    //    until a reset. If any of them left the planner, the queue or a limit in
    //    a corrupted state, an ordinary move afterwards is where it would show.
    // =====================================================================
    printf("\n--- 9. an ordinary move still works afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   quarter turn after the sweep: ok=%d moved=%lld of %lld\n",
               (int)ok, (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary quarter-turn move still works after every extreme value",
                    ok && absll(moved - TFH_CPR / 4) <= posTol(TFH_CPR / 4));

        tfhReset(m);
        TEST_RESULT("the motor is left clean, with no latched fault", tfhFatal(m) == 0);
    }
}
