#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_broadcast.cpp
//
// Arduino-library on-device test for BROADCAST (alias 255) command semantics.
//
// Broadcast is the write-only addressing mode that targets EVERY device on the
// bus at once. The firmware sends NO response to a broadcast packet, so it is
// the only multi-target mode that is collision-safe on the 39-motor rack: the
// bus never sees 39 simultaneous replies. This module exercises the three
// write-style broadcasts that are safe to fan out to a whole bus:
//
//   (a) broadcast System reset (cmd 27)   -- no reply; device(s) reset
//   (b) broadcast Enable MOSFETs (cmd 1) + tiny broadcast Trapezoid move
//       (cmd 2) + broadcast Disable MOSFETs (cmd 0)
//   (c) broadcast System reset (cmd 27) to leave everything clean
//
// Because broadcasts get no reply, the library's getResponse() call inside each
// void command TIMES OUT and records COMMUNICATION_ERROR_TIMEOUT (-1) in the
// motor's error field. That is EXPECTED, not a failure -- we tolerate 0 (a
// stray/success read) or -1 (timeout) on every broadcast send, and we verify
// the actual EFFECT of each broadcast through the unique-ID-addressed target
// motor (single responder -> collision-safe even on the rack).
//
// Addressing strategy: tfGetMotor() is a single Servomotor object addressed by
// unique ID. We capture that unique ID once, flip the same object to alias 255
// (useAlias(255)) to emit each broadcast, then flip it back to the unique ID
// (useUniqueId(uid)) for every verification read. Reads (Get status, Get
// position, Get n queued items) always go to the single target, never to 255.
//
// SAFETY:
//   * The only motion is one relative Trapezoid move of 0.02 rotation over
//     0.5 s. Trapezoid moves end at rest by design (no zero-velocity fatal 18).
//   * Displacement is tiny (0.02 rot << 1 rot) and gentle.
//   * MOSFETs are enabled, the move completes, then MOSFETs are disabled while
//     the queue is empty and the shaft is at rest (no deviation fault).
//   * No test-mode command (cmd 36) is ever sent.
//   * The module both begins and ends with a broadcast System reset, so it is
//     self-cleaning and leaves the device(s) disabled and reset.
// ---------------------------------------------------------------------------

static const uint8_t BROADCAST_ALIAS = 255;   // RS485 broadcast address
static const int     COMMS_TIMEOUT   = COMMUNICATION_ERROR_TIMEOUT;  // -1

// STATUS_MOSFETS_ENABLED_FLAG_BIT is bit 1 in the Get status statusFlags word
// (see common_source_files/device_status.h). Mask == 1 << 1 == 0x0002.
static const uint16_t MOSFETS_ENABLED_MASK = (uint16_t)(1u << 1);

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;   // 3276800

// A broadcast send is "OK" if it reports success (0, e.g. a stray read) or the
// expected no-response timeout (-1). Anything else is a genuine comms fault.
static bool commsOkOrTimeout(int e) {
    return (e == 0) || (e == COMMS_TIMEOUT);
}

// Drain the target's motion queue (unique-ID addressed), then settle briefly.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (motor->getError() != 0) { delay(10); continue; }
        if (n == 0) break;
        delay(10);
    }
    delay(150);  // small settle margin
}

void tm_broadcast(void) {
    Serial.println("test_ard_broadcast: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // Defensive: drain any stale bytes a previous module may have left in the
    // RX stream (e.g. straggler detect-responses on a 39-motor bus) so our
    // first transaction starts from a clean, framed stream.
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)

    // The target's unique ID -- captured once so we can flip between broadcast
    // (alias 255) for sends and unique-ID addressing for verification reads.
    const uint64_t uid = motor->usingThisUniqueId();

    // =======================================================================
    // PART A: broadcast System reset (cmd 27).
    // No reply is expected; the send should report success OR timeout. The
    // TARGET must be responsive and fault-free after the reset settles.
    // =======================================================================
    printf("\n--- A: broadcast System reset ---\n");

    motor->useAlias(BROADCAST_ALIAS);
    motor->systemReset();
    int aResetComms = motor->getError();
    motor->useUniqueId(uid);           // restore target addressing
    printf("broadcast systemReset comms=%d (expect 0 or %d)\n", aResetComms, COMMS_TIMEOUT);
    TEST_RESULT("A broadcast systemReset reports success or timeout",
                commsOkOrTimeout(aResetComms));

    delay(1500);  // reset + 250 ms bootloader window + app launch

    getStatusResponse stA = motor->getStatus();
    bool aResp = (motor->getError() == 0);
    printf("post-reset target getStatus: comms=%d flags=0x%04X fatal=%d\n",
           motor->getError(), aResp ? stA.statusFlags : 0,
           aResp ? stA.fatalErrorCode : -1);
    TEST_RESULT("A target responsive after broadcast systemReset", aResp);
    TEST_RESULT("A no fatal error after broadcast systemReset",
                aResp && (stA.fatalErrorCode == 0));

    // =======================================================================
    // PART B: broadcast Enable MOSFETs -> tiny broadcast Trapezoid move ->
    // broadcast Disable MOSFETs. Every effect is verified via the target.
    // =======================================================================
    printf("\n--- B: broadcast enable + move + disable ---\n");

    // Unit selection is host-side object state (address-independent).
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    // --- B1: broadcast Enable MOSFETs ---
    motor->useAlias(BROADCAST_ALIAS);
    motor->enableMosfets();
    int bEnableComms = motor->getError();
    motor->useUniqueId(uid);
    printf("broadcast enableMosfets comms=%d\n", bEnableComms);
    TEST_RESULT("B broadcast enableMosfets reports success or timeout",
                commsOkOrTimeout(bEnableComms));

    delay(500);  // let the commutation-alignment transient settle (~0.3 s)

    getStatusResponse stEn = motor->getStatus();
    bool enComms = (motor->getError() == 0);
    bool mosfetsEnabled = enComms && ((stEn.statusFlags & MOSFETS_ENABLED_MASK) != 0);
    printf("after enable: comms=%d flags=0x%04X -> enabled=%d\n",
           motor->getError(), enComms ? stEn.statusFlags : 0, mosfetsEnabled ? 1 : 0);
    TEST_RESULT("B target MOSFETs ENABLED after broadcast enableMosfets",
                mosfetsEnabled);

    // Baseline position AFTER enable settle (enable can twitch a few degrees;
    // baselining here isolates the move displacement from that transient).
    int64_t baseCounts = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw baseline");
    printf("baseline position: %lld counts\n", (long long)baseCounts);

    // --- B2: tiny broadcast Trapezoid move, +0.02 rotation over 0.5 s ---
    const float MOVE_ROT = 0.02f;
    const int64_t EXPECT_DELTA = (int64_t)(0.02 * (double)COUNTS_PER_ROTATION);  // 65536
    const int64_t MOVE_TOL     = (int64_t)(0.01 * (double)COUNTS_PER_ROTATION);  // 32768

    motor->useAlias(BROADCAST_ALIAS);
    motor->trapezoidMove(MOVE_ROT, 0.5f);
    int bMoveComms = motor->getError();
    motor->useUniqueId(uid);
    printf("broadcast trapezoidMove comms=%d\n", bMoveComms);
    TEST_RESULT("B broadcast trapezoidMove reports success or timeout",
                commsOkOrTimeout(bMoveComms));

    waitForIdle(motor);  // trapezoid ends at rest; drain the target's queue

    int64_t endCounts = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw end");
    int64_t delta = endCounts - baseCounts;
    printf("end position: %lld counts (delta=%lld, expect ~%lld +/-%lld)\n",
           (long long)endCounts, (long long)delta,
           (long long)EXPECT_DELTA, (long long)MOVE_TOL);
    TEST_RESULT("B target moved ~0.02 rot from broadcast trapezoidMove",
                llabs(delta - EXPECT_DELTA) <= MOVE_TOL);

    // --- B3: broadcast Disable MOSFETs (queue empty, shaft at rest) ---
    motor->useAlias(BROADCAST_ALIAS);
    motor->disableMosfets();
    int bDisableComms = motor->getError();
    motor->useUniqueId(uid);
    printf("broadcast disableMosfets comms=%d\n", bDisableComms);
    TEST_RESULT("B broadcast disableMosfets reports success or timeout",
                commsOkOrTimeout(bDisableComms));

    delay(150);

    getStatusResponse stDis = motor->getStatus();
    bool disComms = (motor->getError() == 0);
    bool mosfetsDisabled = disComms && ((stDis.statusFlags & MOSFETS_ENABLED_MASK) == 0);
    printf("after disable: comms=%d flags=0x%04X -> disabled=%d\n",
           motor->getError(), disComms ? stDis.statusFlags : 0, mosfetsDisabled ? 1 : 0);
    TEST_RESULT("B target MOSFETs DISABLED after broadcast disableMosfets",
                mosfetsDisabled);

    // =======================================================================
    // PART C: final broadcast System reset, then confirm the target is
    // responsive and clean (leaves the bus reset + MOSFETs disabled).
    // =======================================================================
    printf("\n--- C: final broadcast System reset ---\n");

    motor->useAlias(BROADCAST_ALIAS);
    motor->systemReset();
    int cResetComms = motor->getError();
    motor->useUniqueId(uid);
    printf("final broadcast systemReset comms=%d\n", cResetComms);

    delay(1500);

    getStatusResponse stC = motor->getStatus();
    bool cResp = (motor->getError() == 0);
    printf("final target getStatus: comms=%d flags=0x%04X fatal=%d\n",
           motor->getError(), cResp ? stC.statusFlags : 0,
           cResp ? stC.fatalErrorCode : -1);
    TEST_RESULT("C target responsive after final broadcast systemReset", cResp);
    TEST_RESULT("C no fatal error after final broadcast systemReset",
                cResp && (stC.fatalErrorCode == 0));
}
