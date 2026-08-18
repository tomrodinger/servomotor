#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_error_taxonomy.cpp
//
// ONE ERROR AT A TIME, EACH IDENTIFIED BY ITS EXACT CODE, EACH RECOVERED FROM.
//
// The suite has modules that induce individual faults (tm_fatal_state_matrix
// takes the machine into ERROR_CONTROL_LOOP_TOOK_TOO_LONG, tm_safety_fence
// exercises the fence, tm_calibration_rejections covers the calibration
// preconditions). What is missing is the systematic sweep: a single place that
// walks a range of DISTINCT provocations and, for each one, asserts the same
// three things:
//
//     1. it actually faults -- the illegal thing is not quietly permitted
//     2. it faults with the SPECIFIC documented code, not merely "some error"
//     3. a system reset fully clears it and the machine works again
//
// Point 2 is the one that is usually skipped and the one that matters most to a
// caller: an error code is the only diagnostic a user gets over RS485, so a
// command that raises a plausible-but-wrong code sends them hunting in the
// wrong place. Point 3 is what makes an error recoverable rather than a trip to
// the bench.
//
// The provocations are chosen to be ones a real caller could stumble into --
// asking for too much speed, filling the queue, moving past a fence they set,
// passing a zero duration -- not exotic protocol abuse. tm_payload_size_attacks
// and tm_bad_command_byte already cover the malformed-packet side.
//
// DELIBERATELY NOT PROVOKED HERE:
//   * test modes 10..13, which disable interrupts and spin forever; only a
//     power cycle recovers, so an automated suite must never send them
//   * a multimove with a move count above 32, which smashes the stack before
//     reaching its own fatal_error() call (a known, separately tracked bug) --
//     not something to trigger on a shared rack
//   * anything needing the MOSFETs on, a free shaft, or a specific load
//
// SAFETY: MOSFETs stay OFF throughout. Only the COMMANDED position, queue depth
// and status are read, and the planner advances the commanded position
// regardless of the output stage, so nothing turns. No broadcasts and no alias
// changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const int32_t CPR = 3276800;
static const int32_t TPS = 31250;

// Codes from python_programs/servomotor/error_codes.json.
static const uint8_t ERR_QUEUE_NOT_EMPTY   = 8;
static const uint8_t ERR_ACCEL_TOO_HIGH    = 15;
static const uint8_t ERR_VEL_TOO_HIGH      = 16;
static const uint8_t ERR_QUEUE_IS_FULL     = 17;
static const uint8_t ERR_TURN_POINT_OOZ    = 26;
static const uint8_t ERR_PRED_POS_OOZ      = 27;
static const uint8_t ERR_PRED_VEL_TOO_HIGH = 28;
static const uint8_t ERR_PARAM_OUT_OF_RANGE= 34;

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
        if (q == 0) { delay(150); return true; }
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

// After a fault: reset, confirm the error is gone, and confirm an ordinary move
// works again. Recovery is not "the status byte cleared" -- it is "the machine
// is usable".
static bool recovers(Servomotor* m) {
    freshStart(m);
    if (fatalOf(m) != 0) return false;
    if (m->getNQueuedItems() != 0) return false;
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw(CPR / 16, TPS / 2);
    if (m->getError() != 0) return false;
    if (!drainQueue(m, 8000)) return false;
    if (fatalOf(m) != 0) return false;
    return llabs((long long)(m->getPositionRaw() - before - CPR / 16)) <= 200;
}

// Run one provocation and report the three verdicts.
static void provoke(Servomotor* m, const char* label, uint8_t wantCode,
                    void (*setup)(Servomotor*), void (*trigger)(Servomotor*)) {
    freshStart(m);
    if (setup) setup(m);
    trigger(m);
    delay(400);
    uint8_t got = fatalOf(m);
    printf("   %-42s -> fatal %u (expected %u)\n", label, (unsigned)got, (unsigned)wantCode);
    TEST_RESULT(std::string(label) + ": faults rather than being quietly permitted",
                got != 0 && got != 0xFF);
    TEST_RESULT(std::string(label) + ": reports the documented code " +
                    std::to_string((unsigned)wantCode),
                got == wantCode);
    TEST_RESULT(std::string(label) + ": a reset fully restores the machine", recovers(m));
}

// --- setups -----------------------------------------------------------------
static void setupSlowLimit(Servomotor* m)   { m->setMaximumVelocity(0.2f); }
static void setupSlowAccel(Servomotor* m)   { m->setMaximumAcceleration(0.05f); }
static void setupTightFence(Servomotor* m) {
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setSafetyLimits(-0.05f, 0.05f);      // a fence a tenth of a turn wide
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
}

// --- triggers ---------------------------------------------------------------
static void trigOverSpeed(Servomotor* m)  { m->trapezoidMoveRaw(CPR, TPS / 4); }
static void trigOverAccel(Servomotor* m)  { m->trapezoidMoveRaw(CPR, TPS / 4); }
static void trigZeroTime(Servomotor* m)   { m->trapezoidMoveRaw(CPR / 100, 0); }
static void trigZeroTimeGoto(Servomotor* m){ m->goToPositionRaw(CPR / 100, 0); }
static void trigPastFence(Servomotor* m)  { m->trapezoidMoveRaw(CPR, 2 * TPS); }
static void trigFillQueue(Servomotor* m) {
    // 33 slots' worth: with long ramps each move costs 3 and nothing drains.
    m->setMaximumVelocity(1.0f);
    m->setMaximumAcceleration(1.0f);
    for (int k = 0; k < 12; k++) m->trapezoidMoveRaw(CPR / 400, 8 * TPS);
}
static void trigZeroWhileMoving(Servomotor* m) {
    m->setMaximumVelocity(1.0f);
    m->setMaximumAcceleration(1.0f);
    m->trapezoidMoveRaw(CPR / 4, 8 * TPS);
    delay(150);
    m->zeroPosition();                      // illegal while the queue is busy
}

void tm_error_taxonomy(void) {
    Serial.println("tm_error_taxonomy: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. LIMIT VIOLATIONS. Asking for more speed or more acceleration than
    //    the configured ceiling allows. These are the errors an ordinary
    //    caller meets first, and the codes differ depending on which ceiling
    //    the derived numbers hit -- which is exactly why the code matters.
    // =====================================================================
    printf("\n--- 1. limit violations ---\n");
    {
        // Over the speed ceiling. Which of 15/16/28 fires depends on which
        // check the derived numbers reach first, so record the code rather
        // than guessing, then assert it is one of the documented three and is
        // STABLE across repetitions (an unstable code is itself a defect).
        uint8_t codes[5];
        for (int i = 0; i < 5; i++) {
            freshStart(m);
            setupSlowLimit(m);
            trigOverSpeed(m);
            delay(300);
            codes[i] = fatalOf(m);
        }
        bool stable = true;
        for (int i = 1; i < 5; i++) if (codes[i] != codes[0]) stable = false;
        printf("   over-speed move, 5 repetitions -> codes %u %u %u %u %u\n",
               (unsigned)codes[0], (unsigned)codes[1], (unsigned)codes[2],
               (unsigned)codes[3], (unsigned)codes[4]);
        TEST_RESULT("an over-speed move always faults", codes[0] != 0 && codes[0] != 0xFF);
        TEST_RESULT("an over-speed move reports a documented limit code (15/16/28)",
                    codes[0] == ERR_ACCEL_TOO_HIGH || codes[0] == ERR_VEL_TOO_HIGH ||
                    codes[0] == ERR_PRED_VEL_TOO_HIGH);
        TEST_RESULT("the code is the same on every repetition, not a coin flip", stable);
        TEST_RESULT("the machine recovers from an over-speed fault", recovers(m));

        // Over the acceleration ceiling.
        freshStart(m);
        setupSlowAccel(m);
        trigOverAccel(m);
        delay(300);
        uint8_t accelCode = fatalOf(m);
        printf("   over-acceleration move -> fatal %u\n", (unsigned)accelCode);
        TEST_RESULT("an over-acceleration move faults",
                    accelCode != 0 && accelCode != 0xFF);
        TEST_RESULT("an over-acceleration move reports a documented limit code (15/16/28)",
                    accelCode == ERR_ACCEL_TOO_HIGH || accelCode == ERR_VEL_TOO_HIGH ||
                    accelCode == ERR_PRED_VEL_TOO_HIGH);
        TEST_RESULT("the machine recovers from an over-acceleration fault", recovers(m));
    }

    // =====================================================================
    // 2. ZERO DURATION. A move must last at least one control tick. Both move
    //    commands take a duration, so both are checked -- a guard present in
    //    one wrapper and missing in the other is a classic asymmetry.
    // =====================================================================
    printf("\n--- 2. zero duration ---\n");
    provoke(m, "trapezoid move with zero duration", ERR_PARAM_OUT_OF_RANGE,
            NULL, trigZeroTime);
    provoke(m, "go to position with zero duration", ERR_PARAM_OUT_OF_RANGE,
            NULL, trigZeroTimeGoto);

    // =====================================================================
    // 3. THE SAFETY FENCE. Set a narrow fence, then command a move that would
    //    cross it. The refusal happens at QUEUE time from the predicted
    //    endpoint, so it must arrive before anything moves.
    // =====================================================================
    printf("\n--- 3. moving past a safety fence ---\n");
    {
        freshStart(m);
        setupTightFence(m);
        int64_t before = m->getPositionRaw();
        trigPastFence(m);
        delay(400);
        uint8_t code = fatalOf(m);
        int64_t moved = (code == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   1 rotation through a 0.1-rotation fence -> fatal %u\n", (unsigned)code);
        TEST_RESULT("a move that would leave the safety zone faults",
                    code != 0 && code != 0xFF);
        TEST_RESULT("it reports a safety-zone code (26 or 27)",
                    code == ERR_TURN_POINT_OOZ || code == ERR_PRED_POS_OOZ);
        TEST_RESULT("nothing moved before the refusal", llabs((long long)moved) < CPR / 8);
        TEST_RESULT("the machine recovers from a fence violation", recovers(m));

        // The counterpart: a move that stays INSIDE the fence must be allowed.
        // A fence that rejected everything would pass the checks above while
        // being useless.
        freshStart(m);
        setupTightFence(m);
        int64_t b2 = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR / 100, TPS);        // 0.01 rotation, well inside
        bool accepted = (m->getError() == 0);
        bool drained = accepted && drainQueue(m, 8000);
        uint8_t f2 = fatalOf(m);
        int64_t moved2 = m->getPositionRaw() - b2;
        printf("   0.01 rotation inside the same fence -> accepted=%d fatal=%u moved=%lld\n",
               (int)accepted, (unsigned)f2, (long long)moved2);
        TEST_RESULT("a move that stays inside the fence is allowed through",
                    drained && f2 == 0 &&
                    llabs((long long)(moved2 - CPR / 100)) <= CPR / 1000 + 2);
    }

    // =====================================================================
    // 4. QUEUE FULL. Twelve three-slot moves is 36 slots against a 32-slot
    //    queue, so the queue must fill and refuse. tm_queue_edges finds the
    //    boundary; what is asserted here is the CODE and the recovery.
    // =====================================================================
    printf("\n--- 4. overflowing the motion queue ---\n");
    provoke(m, "queueing more than the queue can hold", ERR_QUEUE_IS_FULL,
            NULL, trigFillQueue);

    // =====================================================================
    // 5. AN OPERATION THAT REQUIRES AN IDLE MACHINE. Zeroing the position
    //    while moves are queued is meaningless -- the origin would shift
    //    under the moves already planned against it -- so the firmware
    //    refuses. Same class as the calibration preconditions, different
    //    command.
    // =====================================================================
    printf("\n--- 5. zeroing the position while the queue is busy ---\n");
    provoke(m, "zero position with moves still queued", ERR_QUEUE_NOT_EMPTY,
            NULL, trigZeroWhileMoving);

    // =====================================================================
    // 6. THE SAME OPERATION IS LEGAL WHEN THE PRECONDITION HOLDS. Without
    //    this, section 5 would be satisfied by a command that always failed.
    // =====================================================================
    printf("\n--- 6. the same operations succeed once their precondition holds ---\n");
    {
        freshStart(m);
        m->zeroPosition();                          // idle: must be fine
        bool zeroOk = (m->getError() == 0);
        delay(200);
        uint8_t f = fatalOf(m);
        int64_t p = m->getPositionRaw();
        printf("   zero position while idle -> error=%d fatal=%u position=%lld\n",
               (int)!zeroOk, (unsigned)f, (long long)p);
        TEST_RESULT("zero position is accepted when the queue is empty", zeroOk && f == 0);
        TEST_RESULT("and it actually sets the position to zero", llabs((long long)p) <= 2);

        // A move within the default limits, after all the fault-injection, is
        // the real proof that none of this left residue behind.
        freshStart(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(CPR / 4, TPS);
        bool ok = (m->getError() == 0) && drainQueue(m, 8000);
        TEST_RESULT("an ordinary move works after the whole taxonomy sweep",
                    ok && fatalOf(m) == 0 &&
                    llabs((long long)(m->getPositionRaw() - before - CPR / 4)) < 200);
    }

    // =====================================================================
    // 7. ERRORS DO NOT ACCUMULATE. Provoking several faults in a row, each
    //    with a reset in between, must leave the machine in the same state as
    //    provoking one. A latched bit that survived reset would show up as a
    //    fault code that is no longer the one just induced.
    // =====================================================================
    printf("\n--- 7. repeated fault-and-recover cycles leave no residue ---\n");
    {
        bool allClean = true;
        for (int cycle = 0; cycle < 6 && allClean; cycle++) {
            freshStart(m);
            setupSlowLimit(m);
            trigOverSpeed(m);
            delay(250);
            if (fatalOf(m) == 0) { allClean = false; printf("   cycle %d did not fault\n", cycle); break; }
            freshStart(m);
            if (fatalOf(m) != 0) { allClean = false; printf("   cycle %d did not clear\n", cycle); break; }
            if (m->getNQueuedItems() != 0) { allClean = false; printf("   cycle %d left the queue dirty\n", cycle); break; }
        }
        TEST_RESULT("six fault-and-recover cycles each fault and each clear", allClean);
        TEST_RESULT("the machine is fully usable after six fault cycles", recovers(m));
    }

    freshStart(m);
}
