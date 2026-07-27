#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_actuator_edges.cpp
//
// EDGE-CASE probing of the two "device-signalling / actuator" commands whose
// behavior is easy to get wrong, hunting for bugs at their boundaries:
//   * cmd 40  Vibrate   (the "vibration level" byte)
//   * cmd 41  Identify   (LED blink; interaction with a running move; re-issue)
//
// This is the edge-case complement to tm_actuators.cpp (which does the plain
// at-rest happy path). Every assertion below encodes FIRMWARE GROUND TRUTH.
//
// ==== GROUND TRUTH that CONTRADICTS the spec sketch (firmware wins) ==========
// The spec sketch asked to "read the Vibrate description for the valid level
// range, assert each valid level, and the behavior of an INVALID level (6/255
// -> error? fatal?)". That premise DOES NOT HOLD on the M17 (the bench + the
// 39-motor rack are all M17):
//
//   firmware/Src/motor_control.c vibrate() has its ENTIRE body wrapped in
//   `#ifdef PRODUCT_NAME_M1`. On M17/M23/M2 the function is EMPTY -> a pure
//   no-op. The command handler (firmware/Src/main.c VIBRATE_COMMAND) still
//   size-checks the 1-byte payload (copy_input_parameters_and_check_size) and
//   returns a normal no-error response. And even on M1 the level byte is only
//   tested zero-vs-nonzero (motor_control.c:1179) -- the amplitude is the fixed
//   constant VIBRATE_MAGNITUDE, so there is NO "level range" and NO "invalid
//   level" on ANY product. The JSON Description agrees:
//   "The value is only tested for zero versus nonzero ... does not scale
//   intensity ... other products accept the command but do nothing."
//
// So on M17 the GROUND-TRUTH behavior of an "invalid" level like 6 or 255 is:
// SILENTLY OK -- a no-op that returns success, NOT an error, NOT fatal. That is
// exactly what this module asserts (and proves the no-op really moved nothing).
// The only error path (fatal 51, ERROR_COMMAND_SIZE_WRONG) requires a WRONG
// payload SIZE, which the typed library API (uint8_t) can never emit, so it is
// out of reach here and intentionally not probed (would need raw injection).
// =============================================================================
//
// Identify (cmd 41), firmware ground truth (main.c IDENTIFY_COMMAND ~line 951):
//   * takes NO payload; just sets green_led_action_counter=0, n_identify_flashes
//     =30 and returns success immediately. It does NOT touch the motion queue,
//     motion state, or MOSFETs -> safe to send mid-move.
//   * Re-issuing while flashing simply re-arms the counters -> always accepted,
//     never fatal (restarts the 30-flash sequence).
//
// Error codes referenced (verified in python_programs/servomotor/error_codes.json):
//   * 51 ERROR_COMMAND_SIZE_WRONG   (not triggered here; see note above)
//   * 8/19/22 (Vibrate's M1-only faults) are unreachable on M17; not probed.
//
// SAFETY:
//   * The vibrate sweep runs at rest with MOSFETs OFF; on M17 it is a no-op and
//     provably causes no motion (asserted).
//   * The one commanded move is a single gentle trapezoid (0.5 rot over 2 s,
//     peak ~0.5 rot/s) that ends at zero velocity by construction; current is
//     capped to 200 mA. Well within the <=1 rot / <=3 rot/s gentle guideline.
//   * ONE motor, unique-ID (extended) addressed via tfGetMotor(); every frame is
//     single-target and collision-safe on the 39-motor rack. Nothing is
//     broadcast. No raw bus injection. No test-mode (cmd 36) values at all.
//   * All wait loops are millis()-bounded.
// ---------------------------------------------------------------------------

static const int      NORMAL_RESET_DELAY_MS   = 1500;   // wait after a plain reset
static const uint16_t STATUS_MOSFETS_ENABLED  = (1u << 1);  // device_status.h bit 1
static const float    MOVE_ROT                = 0.5f;   // gentle relative move (rot)
static const float    MOVE_DUR                = 2.0f;   // seconds (peak ~0.5 rot/s)

// Drain any stray RX bytes (bounded, never hangs).
static void drainSerial() {
    uint32_t t0 = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - t0 > 2000) break;
    }
}

// System reset + settle + drain. Leaves the device clean and responsive.
static void cleanReset(Servomotor* motor) {
    motor->systemReset();            // unique-ID addressed; no response during reset
    delay(NORMAL_RESET_DELAY_MS);    // do NOT check errors: nothing answers mid-reset
    drainSerial();
}

// Read the latched fatal error code via Get status (answered normally even in
// the fatal-error state). Returns 0xFF on a comms-layer failure.
static uint8_t readFatal(Servomotor* motor) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return 0xFF;
    }
    return st.fatalErrorCode;
}

// Read the status flags word; returns false on comms failure.
static bool readFlags(Servomotor* motor, uint16_t* flagsOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) return false;
    *flagsOut = st.statusFlags;
    return true;
}

// Verify a 10-byte ping round-trips unchanged (device alive AND answering).
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 11 + 3);
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) {
        printf("  ping comms error %d\n", motor->getError());
        return false;
    }
    return memcmp(r.responsePayload, payload, 10) == 0;
}

static int64_t absDiff(int64_t a, int64_t b) { return (a > b) ? (a - b) : (b - a); }

void tm_actuator_edges(void) {
    Serial.println("test_ard_actuator_edges: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean, known starting state (reset + settle + drain). ----
    cleanReset(motor);
    uint8_t startFatal = readFatal(motor);
    printf("Fatal error code after reset: %d\n", startFatal);
    TEST_RESULT("No fatal error after initial reset", startFatal == 0);

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setCurrentUnit(CurrentUnit::MILLIAMPS);

    // ---- Determine product so we never induce real motion via Vibrate. ----
    // vibrate() is a real, motion-producing command ONLY on M1 (product code
    // "M1" + non-digit padding). On M17 the code is "M17" (code[2]=='7'), so the
    // command is a pure no-op. Gate the nonzero-level sweep behind "not M1 real".
    getProductInfoResponse info = motor->getProductInfo();
    bool infoOk = (motor->getError() == 0);
    char code[9]; memcpy(code, info.productCode, 8); code[8] = '\0';
    bool m1Real = infoOk && code[0] == 'M' && code[1] == '1' &&
                  !(code[2] >= '0' && code[2] <= '9');
    bool vibrateIsNoOp = infoOk && !m1Real;
    printf("productCode = \"%s\"  (vibrate no-op product = %d)\n",
           code, (int)vibrateIsNoOp);
    TEST_RESULT("Product info read OK (needed to gate vibrate safely)", infoOk);

    // countsPerRotation: used for position tolerances below.
    getProductSpecsResponse specs = motor->getProductSpecs();
    bool specsOk = (motor->getError() == 0);
    uint32_t cpr = specsOk ? specs.countsPerRotation : 0;
    printf("countsPerRotation = %u\n", cpr);
    TEST_RESULT("Product specs read OK (countsPerRotation)", specsOk && cpr > 0);
    int64_t posTol = specsOk ? (int64_t)(cpr / 20) : 5000;  // ~0.05 rotation

    // =======================================================================
    // SECTION A: VIBRATE level edges (M17 ground truth = no-op success).
    //
    // At rest, MOSFETs OFF. Sweep a spread of "levels" incl. the spec's
    // "invalid" 6 and 255. GROUND TRUTH on M17: every value returns success,
    // no fatal, and moves NOTHING. Contradicts the spec sketch's "invalid level
    // -> error/fatal"; firmware wins (see header note).
    // =======================================================================
    printf("\n--- SECTION A: Vibrate level edges ---\n");

    int64_t vibStartPos = motor->getPositionRaw();
    bool vibStartPosOk = (motor->getError() == 0);
    printf("Position before vibrate sweep: %lld counts (posOk=%d)\n",
           (long long)vibStartPos, (int)vibStartPosOk);

    // Baseline: vibrate(0) (turn-off / no-op) always accepted on every product.
    motor->vibrate(0);
    bool vib0Comms = (motor->getError() == 0);
    uint8_t vib0Fatal = readFatal(motor);
    TEST_RESULT("Vibrate(0) at rest returns success (comms ok, no fatal)",
                vib0Comms && vib0Fatal == 0);

    if (vibrateIsNoOp) {
        // Sweep a representative spread of nonzero "levels" including 6 & 255.
        static const uint8_t LEVELS[] = {1, 2, 3, 4, 5, 6, 64, 127, 255};
        const int NLEV = (int)(sizeof(LEVELS) / sizeof(LEVELS[0]));
        bool allAccepted = true;
        for (int i = 0; i < NLEV; i++) {
            motor->vibrate(LEVELS[i]);        // "on" (no-op on M17)
            bool onComms = (motor->getError() == 0);
            delay(150);                        // brief, as specified
            uint8_t onFatal = readFatal(motor);
            motor->vibrate(0);                 // "off"
            bool offComms = (motor->getError() == 0);
            uint8_t offFatal = readFatal(motor);
            bool ok = onComms && offComms && onFatal == 0 && offFatal == 0;
            if (!ok) {
                printf("  level %u: onComms=%d onFatal=%d offComms=%d offFatal=%d\n",
                       LEVELS[i], (int)onComms, (int)onFatal, (int)offComms, (int)offFatal);
                allAccepted = false;
            }
        }
        TEST_RESULT("Vibrate accepts every level 1..255 as no-op success (M17)",
                    allAccepted);

        // The spec's "invalid" levels, called out explicitly: NOT an error.
        motor->vibrate(6);
        bool l6Comms = (motor->getError() == 0);
        uint8_t l6Fatal = readFatal(motor);
        motor->vibrate(0);
        // GROUND TRUTH (motor_control.c: value only tested zero-vs-nonzero; body
        // is M1-only): level 6 is silently OK on M17, NOT ERROR_* -- contradicts
        // the spec sketch's "invalid level -> error" guess.
        TEST_RESULT("Vibrate level 6 is silently OK on M17 (no error/fatal, not invalid)",
                    l6Comms && l6Fatal == 0);

        motor->vibrate(255);
        bool l255Comms = (motor->getError() == 0);
        uint8_t l255Fatal = readFatal(motor);
        motor->vibrate(0);
        TEST_RESULT("Vibrate level 255 is silently OK on M17 (no error/fatal)",
                    l255Comms && l255Fatal == 0);

        // Prove it is truly a NO-OP: MOSFETs were never enabled and nothing moved.
        // (On M1 vibrate would enable MOSFETs + switch to open-loop + move.)
        uint16_t flags = 0xFFFF;
        bool flagsOk = readFlags(motor, &flags);
        printf("Post-sweep flags=0x%04X\n", (unsigned)flags);
        TEST_RESULT("Vibrate no-op did NOT enable MOSFETs on M17",
                    flagsOk && (flags & STATUS_MOSFETS_ENABLED) == 0);

        int64_t vibEndPos = motor->getPositionRaw();
        bool vibEndPosOk = (motor->getError() == 0);
        int64_t moved = (vibStartPosOk && vibEndPosOk)
                            ? absDiff(vibEndPos, vibStartPos) : (posTol + 1);
        printf("Position after vibrate sweep: %lld counts (moved %lld, tol %lld)\n",
               (long long)vibEndPos, (long long)moved, (long long)posTol);
        TEST_RESULT("Vibrate no-op caused no motion on M17",
                    vibStartPosOk && vibEndPosOk && moved <= posTol);
    } else {
        // Not a no-op product (would be a real M1 vibration = motion). Do NOT
        // send nonzero levels here; only the safe off value is exercised above.
        printf("  (product is not a vibrate no-op product; skipping nonzero-level "
               "sweep to avoid inducing motion)\n");
        TEST_RESULT("Vibrate nonzero-level sweep skipped on non-no-op product (safety)",
                    true);
    }

    // Device must remain clean/responsive after the vibrate section.
    TEST_RESULT("Device responsive (ping) after vibrate section",
                pingRoundTrips(motor));

    // =======================================================================
    // SECTION B: IDENTIFY concurrent with a queued MOVE.
    //
    // GROUND TRUTH: identify does not touch the motion queue/state/MOSFETs, so
    // BOTH proceed -- the move must run to completion on target while the LED
    // blinks, and the bus must stay responsive mid-blink.
    // =======================================================================
    printf("\n--- SECTION B: Identify during a queued move ---\n");

    motor->enableMosfets();
    bool enComms = (motor->getError() == 0);
    TEST_RESULT("enableMosfets returns success (comms ok)", enComms);
    delay(300);  // let the commutation-alignment transient settle before zeroing

    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    motor->setMaximumMotorCurrent(200.0f, 200.0f);  // gentle
    checkMotorError(*motor, "setMaximumMotorCurrent");

    int64_t moveStartPos = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw move-start");
    printf("Move start position: %lld counts\n", (long long)moveStartPos);

    // Queue a gentle relative move (ends at zero velocity by construction).
    motor->trapezoidMove(MOVE_ROT, MOVE_DUR);
    checkMotorError(*motor, "trapezoidMove");

    // Fire identify while the move is running.
    motor->identify();
    bool idComms = (motor->getError() == 0);
    uint8_t idFatal = readFatal(motor);
    printf("identify() mid-move: comms ok=%d fatal=%d\n", (int)idComms, (int)idFatal);
    TEST_RESULT("Identify accepted mid-move (comms ok)", idComms);
    TEST_RESULT("Identify mid-move leaves no fatal error", idFatal == 0);

    // Identify must NOT have flushed / disturbed the motion queue: the move is
    // still pending (2 s move, identify was immediate).
    uint8_t nqAfterId = motor->getNQueuedItems();
    bool nqComms = (motor->getError() == 0);
    printf("Queue depth right after identify: %u (expect > 0)\n", nqAfterId);
    TEST_RESULT("Move still queued after identify (queue undisturbed)",
                nqComms && nqAfterId > 0);

    // Bus responsive mid-blink (~0.8 s into the 2 s move / 2.7 s blink).
    delay(800);
    TEST_RESULT("Device responsive (ping) mid-blink during move",
                pingRoundTrips(motor));

    // Let the move complete (bounded), then settle and check it landed on target.
    uint8_t nq = 0xFF;
    unsigned long t0 = millis();
    while (millis() - t0 < 8000) {
        nq = motor->getNQueuedItems();
        if (motor->getError() == 0 && nq == 0) break;
        delay(10);
    }
    delay(300);  // settle at the target
    TEST_RESULT("Queue drained (move completed) within timeout", nq == 0);

    int64_t moveEndPos = motor->getPositionRaw();
    bool moveEndOk = (motor->getError() == 0);
    int64_t expected = moveStartPos + (int64_t)((double)MOVE_ROT * (double)cpr);
    int64_t landErr = moveEndOk ? absDiff(moveEndPos, expected) : (posTol + 1);
    printf("Move end pos=%lld expected=%lld err=%lld tol=%lld\n",
           (long long)moveEndPos, (long long)expected,
           (long long)landErr, (long long)posTol);
    TEST_RESULT("Move completed on target despite concurrent identify",
                moveEndOk && specsOk && landErr <= posTol);

    uint8_t bFatal = readFatal(motor);
    TEST_RESULT("No fatal error after concurrent identify+move", bFatal == 0);

    // =======================================================================
    // SECTION C: two IDENTIFY commands back-to-back (mid-flash re-issue).
    //
    // GROUND TRUTH: re-issuing while flashing just re-arms the counters -> the
    // second is accepted and restarts the sequence; never fatal; motion state
    // and queue are untouched.
    // =======================================================================
    printf("\n--- SECTION C: back-to-back identify ---\n");

    int64_t posBeforeIds = motor->getPositionRaw();
    bool posBeforeOk = (motor->getError() == 0);

    motor->identify();
    bool id1Comms = (motor->getError() == 0);
    delay(100);                 // still well within the ~2.7 s flash window
    motor->identify();          // second, mid-flash -> restarts the sequence
    bool id2Comms = (motor->getError() == 0);
    uint8_t idcFatal = readFatal(motor);
    printf("back-to-back identify: id1=%d id2=%d fatal=%d\n",
           (int)id1Comms, (int)id2Comms, (int)idcFatal);
    TEST_RESULT("First identify accepted", id1Comms);
    TEST_RESULT("Second identify (mid-flash re-issue) accepted, restarts sequence",
                id2Comms);
    TEST_RESULT("Back-to-back identify leaves no fatal error", idcFatal == 0);

    // Identify is purely cosmetic: queue empty, motor did not move.
    uint8_t nqC = motor->getNQueuedItems();
    bool nqCComms = (motor->getError() == 0);
    TEST_RESULT("Identify did not enqueue anything (queue still empty)",
                nqCComms && nqC == 0);

    int64_t posAfterIds = motor->getPositionRaw();
    bool posAfterOk = (motor->getError() == 0);
    int64_t idMoved = (posBeforeOk && posAfterOk)
                          ? absDiff(posAfterIds, posBeforeIds) : (posTol + 1);
    printf("Position across identifies: before=%lld after=%lld moved=%lld tol=%lld\n",
           (long long)posBeforeIds, (long long)posAfterIds,
           (long long)idMoved, (long long)posTol);
    TEST_RESULT("Identify did not move the motor",
                posBeforeOk && posAfterOk && idMoved <= posTol);

    TEST_RESULT("Device responsive (ping) after back-to-back identify",
                pingRoundTrips(motor));

    // ---- Leave the device clean and at rest. ----
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    cleanReset(motor);
}
