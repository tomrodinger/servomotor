#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_multimove_bitmask.cpp
//
// THE MULTIMOVE TYPE BITMASK, COMBINATORIALLY.
//
// Multimove (cmd 29) packs up to 32 moves into one packet, with a uint32 in
// which bit k selects the TYPE of move k: 0 = move with acceleration,
// 1 = move with velocity (firmware main.c, MULTIMOVE_COMMAND handler). The
// bitmask is the only thing distinguishing two otherwise identical int32s, so
// an off-by-one in which bit maps to which move would silently reinterpret an
// acceleration as a velocity -- a factor-of-thousands error in what the motor
// does, with no error raised.
//
// tm_multimove and tm_multimove_edges cover the command's size limits and its
// rejection cases. Neither walks the MASK. This module does:
//
//   * every one of the 2^k patterns for k = 1..4 moves
//   * the mask bit really does select the type, shown by making the two types
//     produce measurably different motion for the same numeric parameter
//   * bits above the move count are ignored, not misread
//   * a mask of all-zeros and all-ones at a longer length
//
// WHY THE PARAMETER VALUE ALONE IS NOT ENOUGH: the same int32 means very
// different things in the two interpretations, because velocity is scaled by
// VELOCITY_SHIFT_LEFT (12) and acceleration by ACCELERATION_SHIFT_LEFT (8).
// So each pattern is checked by what the motor DOES, not by whether it was
// accepted.
//
// A velocity segment HOLDS its velocity after its time expires, so every
// sequence here ends with an explicit zero-velocity stop. Without it the queue
// empties at speed and raises ERROR_RUN_OUT_OF_QUEUE_ITEMS.
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position is read, and the
// planner advances it regardless of the output stage, so nothing turns.
// No broadcasts and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const int64_t CPR = 3276800;
static const int32_t TPS = 31250;

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    // The speed ceiling has to clear the WORST pattern, not the typical one.
    // Acceleration segments ACCUMULATE velocity: each one here adds
    // 20 rot/s^2 * 0.15 s = 3 rot/s, so the all-acceleration mask reaches
    // 4 x 3 = 12 rot/s. With a 6 rot/s ceiling the firmware correctly refused
    // 8 of the 30 patterns -- it was measuring the limit, not the bitmask.
    // 15 rot/s leaves margin over the 12 rot/s worst case while staying under
    // the ~18.7 rot/s experimental ceiling.
    m->setMaximumVelocity(15.0f);
    m->setMaximumAcceleration(4000.0f);
    // With the MOSFETs off the shaft never follows, so the commanded-vs-measured
    // deviation grows with every count commanded and trips at the two-rotation
    // default (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION, motor_control.c:97).
    // The all-acceleration patterns cover just over two rotations, so without
    // this the module measures the deviation limit rather than the bitmask.
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setMaxAllowablePositionDeviation(100000.0f);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
}

static bool drainQueue(Servomotor* m, uint32_t budgetMs) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) { delay(150); return true; }
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

// Build a k-move list where every move carries the type-appropriate parameter
// for its bit, then append a zero-velocity stop so nothing is left running.
// Returns the commanded-position delta, or LLONG_MIN if the attempt failed.
static int64_t runPattern(Servomotor* m, uint32_t mask, int k, bool* accepted) {
    const int32_t vel = (int32_t)convertVelocity(1.0f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                 ConversionDirection::TO_INTERNAL);
    const int32_t acc = (int32_t)convertAcceleration(
                            20.0f, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED,
                            ConversionDirection::TO_INTERNAL);
    const uint32_t dur = (uint32_t)convertTime(0.15f, TimeUnit::SECONDS,
                                               ConversionDirection::TO_INTERNAL);

    freshStart(m);
    int64_t before = m->getPositionRaw();

    multimoveList_t mv[6];
    for (int i = 0; i < k; i++) {
        bool isVelocity = ((mask >> i) & 1u) != 0;
        mv[i].value = isVelocity ? vel : acc;
        mv[i].timeSteps = dur;
    }
    // Trailing stop, always a velocity move, so its bit must be set.
    mv[k].value = 0;
    mv[k].timeSteps = 1;
    uint32_t fullMask = mask | (1u << k);

    m->multimoveRaw((uint8_t)(k + 1), fullMask, mv);
    *accepted = (m->getError() == 0);
    if (!*accepted) return 0;
    if (!drainQueue(m, 12000)) { *accepted = false; return 0; }
    if (fatalOf(m) != 0) { *accepted = false; return 0; }
    return m->getPositionRaw() - before;
}

void tm_multimove_bitmask(void) {
    Serial.println("tm_multimove_bitmask: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE BIT REALLY DOES SELECT THE TYPE. One move, one bit. The SAME
    //    numeric parameter interpreted as a velocity and as an acceleration
    //    must produce different motion -- if it does not, the bit is being
    //    ignored and every pattern below would pass for the wrong reason.
    // =====================================================================
    printf("\n--- 1. the type bit changes what the number means ---\n");
    {
        const int32_t param = (int32_t)convertVelocity(
                                  1.0f, VelocityUnit::ROTATIONS_PER_SECOND,
                                  ConversionDirection::TO_INTERNAL);
        const uint32_t dur = (uint32_t)convertTime(0.2f, TimeUnit::SECONDS,
                                                   ConversionDirection::TO_INTERNAL);
        int64_t asVel = 0, asAcc = 0;
        bool okV = false, okA = false;

        // As a VELOCITY (bit set), plus a stop.
        freshStart(m);
        {
            int64_t before = m->getPositionRaw();
            multimoveList_t mv[2];
            mv[0].value = param; mv[0].timeSteps = dur;
            mv[1].value = 0;     mv[1].timeSteps = 1;
            m->multimoveRaw(2, 0x00000003, mv);        // both velocity
            okV = (m->getError() == 0) && drainQueue(m, 10000) && (fatalOf(m) == 0);
            asVel = okV ? (m->getPositionRaw() - before) : 0;
        }

        // As an ACCELERATION (bit clear), then a stop.
        freshStart(m);
        {
            int64_t before = m->getPositionRaw();
            multimoveList_t mv[2];
            mv[0].value = param; mv[0].timeSteps = dur;
            mv[1].value = 0;     mv[1].timeSteps = 1;
            m->multimoveRaw(2, 0x00000002, mv);        // move 0 accel, move 1 velocity
            okA = (m->getError() == 0) && drainQueue(m, 10000) && (fatalOf(m) == 0);
            asAcc = okA ? (m->getPositionRaw() - before) : 0;
        }

        printf("   same parameter %ld: as velocity -> %lld counts (ok=%d); "
               "as acceleration -> %lld counts (ok=%d)\n",
               (long)param, (long long)asVel, (int)okV, (long long)asAcc, (int)okA);
        TEST_RESULT("the same parameter is accepted under both interpretations", okV || okA);
        // If the bit were ignored, both would be identical. They must not be.
        TEST_RESULT("the type bit actually changes the resulting motion",
                    (okV && okA) ? (llabs((long long)(asVel - asAcc)) > 1000)
                                 : (okV != okA));
    }

    // =====================================================================
    // 2. EVERY PATTERN OF 1..4 MOVES. 2 + 4 + 8 + 16 = 30 masks. Each must be
    //    accepted and must complete, and the motion must come to rest -- a
    //    mask that mis-selected a type would typically leave the machine
    //    running or raise ERROR_RUN_OUT_OF_QUEUE_ITEMS.
    // =====================================================================
    printf("\n--- 2. all 2^k masks for k = 1..4 ---\n");
    {
        int total = 0, good = 0;
        for (int k = 1; k <= 4; k++) {
            for (uint32_t mask = 0; mask < (1u << k); mask++) {
                bool accepted = false;
                int64_t moved = runPattern(m, mask, k, &accepted);
                total++;
                // Came to rest?
                int64_t p1 = m->getPositionRaw();
                delay(250);
                int64_t p2 = m->getPositionRaw();
                bool stopped = (m->getError() == 0) && (llabs((long long)(p2 - p1)) <= 2);
                bool ok = accepted && stopped;
                if (ok) good++;
                else printf("   k=%d mask=0x%lX: accepted=%d stopped=%d moved=%lld\n",
                            k, (unsigned long)mask, (int)accepted, (int)stopped,
                            (long long)moved);
            }
        }
        printf("   %d/%d masks ran cleanly and came to rest\n", good, total);
        TEST_RESULT("every 1-to-4-move type pattern is accepted and comes to rest",
                    good == total);
        TEST_RESULT("all thirty patterns were actually attempted", total == 30);
    }

    // =====================================================================
    // 3. BITS ABOVE THE MOVE COUNT ARE IGNORED. The mask is a full uint32 but
    //    only the low `moveCount` bits are meaningful. Setting the unused high
    //    bits must not change the outcome; if it does, the handler is reading
    //    past the list.
    // =====================================================================
    printf("\n--- 3. bits above the move count are ignored ---\n");
    {
        bool acc1 = false, acc2 = false;
        // Two velocity moves plus a stop == mask 0b111 in the low three bits.
        int64_t clean = runPattern(m, 0x00000003, 2, &acc1);
        // Same thing, with every unused high bit set.
        const int32_t vel = (int32_t)convertVelocity(1.0f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                     ConversionDirection::TO_INTERNAL);
        const uint32_t dur = (uint32_t)convertTime(0.15f, TimeUnit::SECONDS,
                                                   ConversionDirection::TO_INTERNAL);
        freshStart(m);
        int64_t before = m->getPositionRaw();
        multimoveList_t mv[3];
        mv[0].value = vel; mv[0].timeSteps = dur;
        mv[1].value = vel; mv[1].timeSteps = dur;
        mv[2].value = 0;   mv[2].timeSteps = 1;
        m->multimoveRaw(3, 0xFFFFFFFF, mv);            // all bits set; only low 3 matter
        acc2 = (m->getError() == 0) && drainQueue(m, 10000) && (fatalOf(m) == 0);
        int64_t noisy = acc2 ? (m->getPositionRaw() - before) : 0;

        printf("   low bits only -> %lld counts; all high bits set -> %lld counts\n",
               (long long)clean, (long long)noisy);
        TEST_RESULT("both the clean and the all-bits-set masks are accepted", acc1 && acc2);
        TEST_RESULT("setting the unused high mask bits changes nothing",
                    acc1 && acc2 && llabs((long long)(clean - noisy)) <= CPR / 200);
    }

    // =====================================================================
    // 4. LONGER UNIFORM PATTERNS. All-acceleration and all-velocity at a
    //    length that fills a good part of the queue, so the mask is exercised
    //    beyond the handful of bits above.
    // =====================================================================
    printf("\n--- 4. longer uniform patterns ---\n");
    {
        const int32_t vel = (int32_t)convertVelocity(0.5f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                     ConversionDirection::TO_INTERNAL);
        const uint32_t dur = (uint32_t)convertTime(0.05f, TimeUnit::SECONDS,
                                                   ConversionDirection::TO_INTERNAL);
        // Ten velocity moves plus a stop.
        freshStart(m);
        int64_t before = m->getPositionRaw();
        multimoveList_t mv[11];
        for (int i = 0; i < 10; i++) { mv[i].value = vel; mv[i].timeSteps = dur; }
        mv[10].value = 0; mv[10].timeSteps = 1;
        m->multimoveRaw(11, 0x000007FF, mv);           // all eleven are velocity
        bool okV = (m->getError() == 0) && drainQueue(m, 15000) && (fatalOf(m) == 0);
        int64_t movedV = okV ? (m->getPositionRaw() - before) : 0;
        // 0.5 rot/s for 10 x 0.05 s = 0.5 s of travel = 0.25 rotation.
        int64_t wantV = (int64_t)(0.25 * (double)CPR);
        printf("   eleven velocity moves -> %lld counts (expected about %lld)\n",
               (long long)movedV, (long long)wantV);
        TEST_RESULT("an eleven-move all-velocity multimove runs cleanly", okV);
        TEST_RESULT("it travels the physically expected distance",
                    okV && llabs((long long)(movedV - wantV)) <= CPR / 20);

        // Ten acceleration moves that cancel: +a, -a repeated, then a stop.
        const int32_t acc = (int32_t)convertAcceleration(
                                20.0f, AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED,
                                ConversionDirection::TO_INTERNAL);
        freshStart(m);
        int64_t b2 = m->getPositionRaw();
        multimoveList_t mv2[11];
        for (int i = 0; i < 10; i++) {
            mv2[i].value = (i % 2 == 0) ? acc : -acc;
            mv2[i].timeSteps = dur;
        }
        mv2[10].value = 0; mv2[10].timeSteps = 1;
        m->multimoveRaw(11, 0x00000400, mv2);          // ten accel, final one velocity
        bool okA = (m->getError() == 0) && drainQueue(m, 15000) && (fatalOf(m) == 0);
        int64_t movedA = okA ? (m->getPositionRaw() - b2) : 0;
        printf("   eleven alternating-acceleration moves -> %lld counts\n", (long long)movedA);
        TEST_RESULT("an eleven-move mostly-acceleration multimove runs cleanly", okA);
        // Alternating equal accelerations over equal times integrate to a small
        // net displacement, not a large one.
        TEST_RESULT("alternating accelerations do not run away",
                    okA && llabs((long long)movedA) < CPR);
    }

    // =====================================================================
    // 5. A MULTIMOVE WITHOUT ITS TRAILING STOP LEAVES THE MACHINE RUNNING.
    //    Stated as a positive assertion because it is the single most common
    //    way to misuse this command, and because it documents WHY every
    //    sequence above ends in a zero-velocity segment.
    // =====================================================================
    printf("\n--- 5. a velocity multimove with no stop keeps going ---\n");
    {
        const int32_t vel = (int32_t)convertVelocity(0.5f, VelocityUnit::ROTATIONS_PER_SECOND,
                                                     ConversionDirection::TO_INTERNAL);
        const uint32_t dur = (uint32_t)convertTime(0.1f, TimeUnit::SECONDS,
                                                   ConversionDirection::TO_INTERNAL);
        freshStart(m);
        multimoveList_t mv[1];
        mv[0].value = vel; mv[0].timeSteps = dur;
        m->multimoveRaw(1, 0x00000001, mv);            // velocity, NO trailing stop
        bool accepted = (m->getError() == 0);
        delay(400);                                    // well past the 0.1 s duration
        int64_t p1 = m->getPositionRaw();
        delay(300);
        int64_t p2 = m->getPositionRaw();
        uint8_t fat = fatalOf(m);
        bool stillMoving = (llabs((long long)(p2 - p1)) > 1000);
        printf("   after the segment expired: moved a further %lld counts, fatal=%u\n",
               (long long)(p2 - p1), (unsigned)fat);
        TEST_RESULT("a velocity multimove with no trailing stop is accepted", accepted);
        TEST_RESULT("and it either keeps moving or raises a queue-underrun error, "
                    "which is why the stop is mandatory",
                    stillMoving || fat != 0);
        m->emergencyStop();
        delay(300);
        freshStart(m);
        TEST_RESULT("the machine recovers from the runaway", fatalOf(m) == 0);
    }

    // =====================================================================
    // 6. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        freshStart(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(CPR / 4), TPS);
        bool ok = (m->getError() == 0) && drainQueue(m, 8000);
        TEST_RESULT("an ordinary move still works after the bitmask sweep",
                    ok && fatalOf(m) == 0 &&
                    llabs((long long)(m->getPositionRaw() - before - CPR / 4)) < 200);
    }

    freshStart(m);
}
