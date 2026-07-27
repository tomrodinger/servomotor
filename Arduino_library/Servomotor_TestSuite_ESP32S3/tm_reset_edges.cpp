#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_reset_edges.cpp
//
// RESET ROBUSTNESS edge cases. Three sub-scenarios, each ending verified-
// healthy with a working small move:
//
//   (a) Three System resets (cmd 27) back-to-back, ~1.6 s apart: after every
//       reset the device must be clean -- status flags == 0 AND fatal error
//       code == 0.
//   (b) System reset DELIVERED DURING an executing queued move: the MCU reboots
//       cleanly (NVIC_SystemReset in main.c SYSTEM_RESET_COMMAND), and after the
//       ~1.6 s boot window the status is clean and the COMMANDED position frame
//       has been RE-ZEROED. (See ground-truth note below.)
//   (c) System reset DELIVERED DURING closed-loop holding: the device reboots
//       into OPEN LOOP -- the closed-loop status bit must be clear afterward.
//
// GROUND TRUTH (verified in firmware / headers, cited inline):
//
//   * Status flag bit layout -- common_source_files/device_status.h:
//       bit0 in-bootloader, bit1 MOSFETs-enabled, bit2 CLOSED-LOOP,
//       bit3 calibrating, bit4 homing, bit5 go-to-closed-loop, bit6 motor-busy.
//     get_motor_status_flags() (motor_control.c:3360) sets bit2 iff
//       motor_control_mode == CLOSED_LOOP_POSITION_CONTROL.
//     A freshly reset, idle, MOSFETs-off device therefore reports flags == 0.
//
//   * Post-reset commanded position == 0. Get position (main.c GET_POSITION_
//     COMMAND:443) returns get_motor_position() == current_position_i96.position
//     (motor_control.c:3332-3336). current_position_i96 is a file-scope static
//     initialised to {0} (motor_control.c:153); the firmware boot does NOT seed
//     the commanded position from the hall sensor. So after ANY reset -- even
//     mid-move with the shaft physically displaced -- getPositionRaw() reads
//     exactly 0. (This is what the spec calls "re-zeroed per firmware
//     behaviour": commanded position resets to 0, NOT to the hall position.)
//
//   * getStatus and systemReset framing: getStatus round-trips normally on a
//     healthy device; systemReset sends a no-error packet then reboots
//     (main.c:623-628), so there is no reliable response and errors are not
//     checked immediately after it.
//
// SAFETY:
//   * All motion is gentle open-loop/closed-loop trapezoids (<= 0.3 rotation,
//     <= 0.1 rot/s) that end at rest by design; interrupting one with a reset
//     leaves the shaft coasting to a stop with MOSFETs off -- no residual
//     commanded velocity, no async fatal error.
//   * NO test-mode (cmd 36) values are used anywhere -- 10..13 permanent-hang
//     risk is structurally impossible here.
//   * Every command uses tfGetMotor() (unique-ID / extended addressing), so it
//     is collision-safe on the 39-motor rack; nothing is broadcast.
//   * NO raw Serial1.write() frames are emitted; only drain-reads.
//   * Every reset is followed by delay(1600) + RX drain before the next step so
//     no command lands in the bootloader window.
// ---------------------------------------------------------------------------

static const int RESET_DELAY_MS      = 1600;  // >=1.5 s boot window, spec's 1.6 s
static const int ENABLE_SETTLE_MS    = 300;   // let enable/commutation transient settle
static const int CLOSED_LOOP_TIMEOUT_MS = 6000;
static const int QUEUE_DRAIN_TIMEOUT_MS = 8000;

static const uint16_t STATUS_MOSFETS_ENABLED_BIT = (1u << 1);  // device_status.h bit1
static const uint16_t STATUS_CLOSED_LOOP_BIT     = (1u << 2);  // device_status.h bit2

static const float SMALL_MOVE_ROT   = 0.05f;  // rotations for the "small move works" check
static const float SMALL_MOVE_DUR   = 0.5f;   // seconds
static const float MOVE_POS_TOL     = 0.02f;  // rotations tolerance on commanded endpoint

static const float INFLIGHT_ROT     = 0.3f;   // scenario (b) move to interrupt
static const float INFLIGHT_DUR     = 3.0f;   // seconds (0.1 rot/s, gentle)

// Drain any bytes lingering on the motor RX line.
static void drainRx() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// Reset to a clean, known state and drain. No response during reset -> no error check.
static void resetClean(Servomotor* motor) {
    motor->systemReset();
    delay(RESET_DELAY_MS);
    drainRx();
}

// Read status flags + fatal error code. Returns false on a comms-layer failure.
static bool readStatus(Servomotor* motor, uint16_t* flagsOut, uint8_t* codeOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *flagsOut = st.statusFlags;
    *codeOut  = st.fatalErrorCode;
    return true;
}

// A device is "clean" iff no status flags are set and no fatal error is latched.
static bool statusClean(Servomotor* motor) {
    uint16_t flags = 0xFFFF;
    uint8_t  code  = 0xFF;
    bool ok = readStatus(motor, &flags, &code);
    printf("  status: commsOk=%d flags=0x%04X code=%d\n",
           (int)ok, (unsigned)flags, (int)code);
    return ok && flags == 0 && code == 0;
}

// Poll the motion queue until empty or timeout. Returns final depth (0xFF on comms error).
static uint8_t waitForQueueDrain(Servomotor* motor) {
    uint8_t n = 0xFF;
    uint32_t deadline = millis() + QUEUE_DRAIN_TIMEOUT_MS;
    while ((int32_t)(millis() - deadline) < 0) {
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(150);  // settle margin
    return n;
}

// Verify a gentle open-loop move works on a freshly-reset device.
// Enables MOSFETs, commands a tiny trapezoid, waits for drain, then checks:
//   - no fatal error latched,
//   - queue drained to 0,
//   - commanded position advanced to ~SMALL_MOVE_ROT (starts at 0 post-reset).
// Assumes the device is clean/at-rest on entry; leaves MOSFETs enabled (caller
// resets clean afterward).
static bool smallMoveWorks(Servomotor* motor) {
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    motor->enableMosfets();
    if (motor->getError() != 0) { printf("  smallMove: enableMosfets err %d\n", motor->getError()); return false; }
    delay(ENABLE_SETTLE_MS);  // commutation-alignment transient

    // Post-reset commanded position is 0 (ground truth above); confirm baseline.
    float startPos = motor->getPosition();
    if (motor->getError() != 0) { printf("  smallMove: getPosition(start) err %d\n", motor->getError()); return false; }

    motor->trapezoidMove(SMALL_MOVE_ROT, SMALL_MOVE_DUR);
    if (motor->getError() != 0) { printf("  smallMove: trapezoidMove err %d\n", motor->getError()); return false; }

    uint8_t drained = waitForQueueDrain(motor);
    if (drained != 0) { printf("  smallMove: queue did not drain (n=%d)\n", (int)drained); return false; }

    // No fatal error may have latched.
    uint16_t flags = 0xFFFF; uint8_t code = 0xFF;
    if (!readStatus(motor, &flags, &code)) return false;
    if (code != 0) { printf("  smallMove: fatal code %d latched\n", (int)code); return false; }

    float endPos = motor->getPosition();
    if (motor->getError() != 0) { printf("  smallMove: getPosition(end) err %d\n", motor->getError()); return false; }

    float delta = endPos - startPos;
    printf("  smallMove: start=%.4f end=%.4f delta=%.4f (expect ~%.4f)\n",
           startPos, endPos, delta, SMALL_MOVE_ROT);
    return approxEqual(delta, SMALL_MOVE_ROT, MOVE_POS_TOL);
}

void tm_reset_edges(void) {
    Serial.println("test_ard_reset_edges: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean, known starting state. ----
    resetClean(motor);
    TEST_RESULT("R Clean at entry (flags==0, code==0)", statusClean(motor));

    // =======================================================================
    // (a) Three System resets back-to-back, ~1.6 s apart. Healthy after each.
    // =======================================================================
    printf("\n--- (a) three back-to-back system resets ---\n");
    bool allCleanA = true;
    for (int i = 0; i < 3; i++) {
        resetClean(motor);
        bool clean = statusClean(motor);
        char name[80];
        snprintf(name, sizeof(name), "R-a Reset #%d leaves device clean", i + 1);
        TEST_RESULT(name, clean);
        if (!clean) allCleanA = false;
    }
    printf("(a) all three resets clean: %d\n", (int)allCleanA);

    // Small move must work after the reset triple.
    bool moveA = smallMoveWorks(motor);
    TEST_RESULT("R-a Small move works after reset triple", moveA);
    resetClean(motor);

    // =======================================================================
    // (b) System reset DURING an executing queued move. Clean reboot + the
    //     commanded position frame re-zeroes to 0 (ground truth above).
    // =======================================================================
    printf("\n--- (b) reset during a queued move ---\n");
    resetClean(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets (b)");
    delay(ENABLE_SETTLE_MS);

    // Launch a gentle multi-second move so it is genuinely mid-flight.
    motor->trapezoidMove(INFLIGHT_ROT, INFLIGHT_DUR);
    checkMotorError(*motor, "trapezoidMove (b)");

    // Confirm the move is actually in flight before we interrupt it: a trapezoid
    // queues up to 3 items (accel/coast/decel), so depth must be > 0 right now.
    uint8_t inflightDepth = motor->getNQueuedItems();
    bool inflight = (motor->getError() == 0) && (inflightDepth > 0);
    printf("(b) queue depth mid-move = %d (expect > 0)\n", (int)inflightDepth);
    TEST_RESULT("R-b Move is in flight before reset (queue > 0)", inflight);

    delay(500);  // let it travel a bit (~0.05 rot) so the shaft is displaced

    // Reset MID-MOVE. Device coasts to rest with MOSFETs off; commanded pos -> 0.
    resetClean(motor);

    // Status must be clean after the mid-move reboot.
    bool cleanB = statusClean(motor);
    TEST_RESULT("R-b Device clean after mid-move reset", cleanB);

    // Commanded position frame re-zeroed: getPositionRaw() == 0 exactly.
    // (current_position_i96 static == 0 on boot; not seeded from hall.)
    int64_t posRaw = motor->getPositionRaw();
    bool posOk = (motor->getError() == 0);
    printf("(b) getPositionRaw() after reset = %lld (expect 0)\n", (long long)posRaw);
    TEST_RESULT("R-b Commanded position re-zeroed after reset", posOk && posRaw == 0);

    // Small move must work afterward.
    bool moveB = smallMoveWorks(motor);
    TEST_RESULT("R-b Small move works after mid-move reset", moveB);
    resetClean(motor);

    // =======================================================================
    // (c) System reset DURING closed-loop holding. Reboots to OPEN loop:
    //     the closed-loop status bit must be clear afterward.
    // =======================================================================
    printf("\n--- (c) reset during closed-loop holding ---\n");
    resetClean(motor);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets (c)");
    delay(ENABLE_SETTLE_MS);

    // Before go_to_closed_loop the closed-loop bit must be clear (open loop).
    uint16_t preFlags = 0xFFFF; uint8_t preCode = 0xFF;
    bool preOk = readStatus(motor, &preFlags, &preCode);
    bool openBefore = preOk && ((preFlags & STATUS_CLOSED_LOOP_BIT) == 0);
    printf("(c) flags before go_to_closed_loop = 0x%04X (closed-loop bit must be 0)\n",
           (unsigned)preFlags);
    TEST_RESULT("R-c Open loop before go_to_closed_loop", openBefore);

    // Enter closed loop and poll for the closed-loop bit (bounded by timeout).
    motor->goToClosedLoop();
    checkMotorError(*motor, "goToClosedLoop (c)");

    bool closedReached = false;
    uint16_t clFlags = 0; uint8_t clCode = 0;
    uint32_t deadline = millis() + CLOSED_LOOP_TIMEOUT_MS;
    while ((int32_t)(millis() - deadline) < 0) {
        if (readStatus(motor, &clFlags, &clCode) && (clFlags & STATUS_CLOSED_LOOP_BIT)) {
            closedReached = true;
            break;
        }
        delay(50);
    }
    printf("(c) closed-loop reached=%d flags=0x%04X\n", (int)closedReached, (unsigned)clFlags);
    TEST_RESULT("R-c Closed-loop holding established", closedReached);

    // Reset WHILE holding closed loop.
    resetClean(motor);

    // After the reboot the closed-loop bit must be clear (device in open loop),
    // and more broadly the status must be fully clean.
    uint16_t postFlags = 0xFFFF; uint8_t postCode = 0xFF;
    bool postOk = readStatus(motor, &postFlags, &postCode);
    bool clBitClear = postOk && ((postFlags & STATUS_CLOSED_LOOP_BIT) == 0);
    printf("(c) flags after reset = 0x%04X code=%d (closed-loop bit must be 0)\n",
           (unsigned)postFlags, (int)postCode);
    TEST_RESULT("R-c Closed-loop bit clear after reset (open loop)", clBitClear);
    TEST_RESULT("R-c Device fully clean after closed-loop reset",
                postOk && postFlags == 0 && postCode == 0);

    // Small move must work afterward.
    bool moveC = smallMoveWorks(motor);
    TEST_RESULT("R-c Small move works after closed-loop reset", moveC);

    // ---- Leave the device clean and at rest. ----
    resetClean(motor);
}
