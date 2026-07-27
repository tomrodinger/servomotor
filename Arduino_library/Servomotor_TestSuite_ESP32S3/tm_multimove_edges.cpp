#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_multimove_edges.cpp
//
// Edge / boundary cases for the Multimove command (cmd 29), all verified
// against firmware ground truth (firmware/Src/main.c MULTIMOVE_COMMAND handler,
// firmware/Src/motor_control.c, python_programs/servomotor/error_codes.json).
//
// Cases:
//   (a) 33-move list  -> fatal 24 ERROR_MULTIMOVE_MORE_THAN_32_MOVES, device
//       stays alive (answers Get status, recoverable by System reset).
//   (b) 1-move list ending at zero velocity -> accepted, drains cleanly, no
//       fatal 18.
//   (c) 1-move list ending at NONZERO velocity -> accepted (success reply), but
//       when the queue drains the firmware raises fatal 18
//       ERROR_RUN_OUT_OF_QUEUE_ITEMS asynchronously. Poll Get status until it
//       appears (bounded), then recover.
//   (d) count-0 list -> firmware no-op that returns success (see analysis at
//       case (d)); nothing is queued, no error.
//
// GROUND TRUTH for case (a) — firmware/Src/main.c:
//     uint8_t n_moves_in_this_command = ((int8_t*)payload)[0];
//     if(n_moves_in_this_command > MAX_MULTIMOVES) {           // MAX_MULTIMOVES == 32
//         fatal_error(ERROR_MULTIMOVE_MORE_THAN_32_MOVES);     // code 24
//     }
//   The count guard runs BEFORE copy_input_parameters_and_check_size(), so a
//   count byte of 33 faults with code 24 regardless of the rest of the payload
//   (this is the BUG-2 stack-overflow guard). error_codes.json: code 24 ==
//   ERROR_MULTIMOVE_MORE_THAN_32_MOVES.
//
//   The Arduino multimove()/multimoveRaw() API CANNOT be used to send 33 moves:
//   multimoveRaw does memcpy(payload.moveList /*[32]*/, moveList, 33*8) and
//   multimove writes convertedList[32] in a 0..count loop — both overflow a
//   32-entry host-side buffer (corrupting the ESP32 controller, not the motor).
//   So case (a) is sent as a RAW extended-addressed frame (address byte 254 +
//   8-byte unique ID), built to mirror Communication::sendCommandCore exactly,
//   WITH CRC32 (device + library both default to CRC enabled; never toggled).
//   Extended addressing keeps the frame targeted at one motor on the shared
//   39-motor bus; it can never be mistaken for an alias-88 frame.
//
// GROUND TRUTH for case (c) — firmware/Src/motor_control.c motor_movement_
//   calculations() (called every tick from TIM16_IRQHandler, independent of
//   MOSFET enable state): when the queue empties with decelerate_to_stop_active
//   and current_velocity_i64 != 0 it calls fatal_error(ERROR_RUN_OUT_OF_QUEUE_
//   ITEMS) (code 18). Because the trigger is tick-based and MOSFET-independent,
//   this whole module runs with the MOSFETs DISABLED: zero physical motion, yet
//   both the queue-time (fatal 24) and drain-time (fatal 18) paths still fire.
//
// GROUND TRUTH for case (d) — firmware/Src/main.c: count 0 is NOT > 32, so no
//   fatal 24; expected_size = sizeof(move_type_bits) + 0 = 4, the size check
//   passes for the library's 5-byte payload (1 count + 4 move_type_bits), the
//   queue loop runs zero times, and rs485_transmit_no_error_packet() replies
//   success. Net: a no-op returning success, nothing queued, no error.
//
// SAFETY:
//   * MOSFETs stay DISABLED the entire module -> no torque, no motion.
//   * Single-target commands go via tfGetMotor() (unique-ID / extended
//     addressing). The one raw frame is extended-addressed to the target's
//     unique ID. Nothing is broadcast; nothing solicits multi-device responses.
//   * Test-mode values 10..13 are never used (no test-mode commands at all).
//   * After each deliberately-induced fatal error (cases a, c): System reset +
//     delay(1500) + drain before proceeding.
//   * Every poll loop is bounded by a millis() deadline.
// ---------------------------------------------------------------------------

static const int      RESET_DELAY_MS   = 1500;   // no response during reset
static const int      LATCH_MS         = 300;    // let a fatal state latch
static const uint32_t FATAL18_POLL_MS  = 5000;   // bound for the async fatal-18 wait
static const uint8_t  CMD_MULTIMOVE    = 29;     // Commands.h: MULTIMOVE = 29

// ---- helpers ---------------------------------------------------------------

// Drain any pending RX bytes, bounded so a chattering line cannot hang us.
static void drainRx() {
    uint32_t deadline = millis() + 500;
    while (millis() < deadline) {
        if (Serial1.available()) {
            Serial1.read();
        } else {
            break;
        }
    }
}

// Clean reset: System reset (unique-ID addressed) + settle + drain.
static void resetDrain(Servomotor* motor) {
    motor->systemReset();
    delay(RESET_DELAY_MS);
    drainRx();
}

// Read the latched fatal error code via Get status (answered even while a fatal
// error is latched). Returns false only if the query itself failed at comms.
static bool readFatalCode(Servomotor* motor, uint8_t* codeOut, uint16_t* flagsOut) {
    getStatusResponse st = motor->getStatus();
    if (motor->getError() != 0) {
        printf("  (getStatus comms error: %d)\n", motor->getError());
        return false;
    }
    *codeOut  = st.fatalErrorCode;
    *flagsOut = st.statusFlags;
    return true;
}

// Build and transmit a raw, extended-addressed MULTIMOVE frame carrying a
// single count byte (moveCount) and NOTHING else, WITH CRC32 — mirroring
// Communication::sendCommandCore framing. For moveCount > 32 the firmware
// faults with fatal 24 before it ever validates the payload length, so a
// 1-byte payload is sufficient and is the lowest-risk frame to inject.
//   frame = sizeByte | 254 | uniqueId[8 LE] | 29 | moveCount | crc32[4 LE]
static void injectRawMultimoveCount(Servomotor* motor, uint8_t moveCount) {
    uint64_t uid = motor->usingThisUniqueId();

    // total = size(1) + addr(1+8) + cmd(1) + payload(1) + crc(4) = 16
    const uint16_t totalPacketSize = 1 + 1 + 8 + 1 + 1 + 4;

    uint8_t frame[16];
    int n = 0;
    frame[n++] = encodeFirstByte((uint8_t)totalPacketSize);  // (size<<1)|1, size<=127 -> single byte
    frame[n++] = EXTENDED_ADDRESSING;                        // 254
    for (int i = 0; i < 8; i++) {
        frame[n++] = (uint8_t)((uid >> (8 * i)) & 0xFF);     // unique ID, little-endian (as the library sends it)
    }
    frame[n++] = CMD_MULTIMOVE;                              // 29
    frame[n++] = moveCount;                                  // the oversized count
    uint32_t crc = calculate_crc32(frame, n);               // CRC over sizeByte..payload, exactly like sendCommandCore
    for (int i = 0; i < 4; i++) {
        frame[n++] = (uint8_t)((crc >> (8 * i)) & 0xFF);     // CRC32, little-endian
    }

    drainRx();
    Serial1.write(frame, (size_t)n);
    Serial1.flush();
}

// Configure the standard rotation/second/seconds unit set used by the cases.
static void setStdUnits(Servomotor* motor) {
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);
}

// ---------------------------------------------------------------------------

void tm_multimove_edges(void) {
    Serial.println("test_ard_multimove_edges: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean, known start. MOSFETs stay disabled for the whole module. ----
    resetDrain(motor);
    setStdUnits(motor);

    uint8_t  code  = 0xFF;
    uint16_t flags = 0xFFFF;
    bool ok = readFatalCode(motor, &code, &flags);
    TEST_RESULT("Clean start: no fatal error, comms OK", ok && code == 0);

    // =======================================================================
    // (a) 33-move list -> fatal 24, device stays alive.
    // =======================================================================
    printf("\n--- (a) raw 33-move Multimove -> expect fatal 24 ---\n");
    injectRawMultimoveCount(motor, 33);   // count byte 33 > MAX_MULTIMOVES(32)
    delay(LATCH_MS);
    drainRx();                            // fatal_error sends no response; clear any stray bytes

    code = 0xFF; flags = 0xFFFF;
    bool aRead = readFatalCode(motor, &code, &flags);
    printf("  after 33-move inject: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)aRead, (int)code, (unsigned)flags);
    // error_codes.json: code 24 == ERROR_MULTIMOVE_MORE_THAN_32_MOVES.
    TEST_RESULT("33-move Multimove latches fatal code 24", aRead && code == 24);
    // Device is still alive: Get status was answered above (aRead true) -> the
    // oversized count faulted cleanly, it did not brick the device.
    TEST_RESULT("Device still answers Get status after fatal 24", aRead);

    // Recover and confirm the device is fully clean again.
    resetDrain(motor);
    code = 0xFF; flags = 0xFFFF;
    bool aClear = readFatalCode(motor, &code, &flags);
    TEST_RESULT("System reset clears fatal 24", aClear && code == 0);

    // =======================================================================
    // (b) 1-move list ending at zero velocity -> accepted, drains, no fatal 18.
    // =======================================================================
    printf("\n--- (b) 1-move Multimove ending at zero velocity ---\n");
    setStdUnits(motor);
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition (b)");
    int64_t posBefore = motor->getPositionRaw();

    const uint8_t  bCount = 1;
    const uint32_t bTypes = 0b1;          // bit0 = velocity move
    multimoveListConverted_t bList[bCount] = {
        {0.0f, 0.1f},                     // velocity 0 rot/s for 0.1 s -> ends at zero velocity
    };
    motor->multimove(bCount, bTypes, bList);
    bool bAccepted = (motor->getError() == 0);
    TEST_RESULT("1-move (zero-velocity) Multimove accepted", bAccepted);

    // Let the single 0.1 s item execute and the queue drain.
    delay(400);

    // Queue must have emptied cleanly with velocity 0: no fatal 18, code stays 0.
    code = 0xFF; flags = 0xFFFF;
    bool bRead = readFatalCode(motor, &code, &flags);
    printf("  after drain: commsOk=%d fatalCode=%d\n", (int)bRead, (int)code);
    TEST_RESULT("Zero-velocity 1-move drains without any fatal error",
                bRead && code == 0);

    uint8_t bQueued = motor->getNQueuedItems();
    TEST_RESULT("Queue empty after zero-velocity 1-move", bQueued == 0);

    // MOSFETs disabled + velocity 0 -> commanded position did not advance.
    int64_t posAfter = motor->getPositionRaw();
    TEST_RESULT("Zero-velocity move leaves position unchanged", posAfter == posBefore);

    // =======================================================================
    // (c) 1-move ending at NONZERO velocity -> accepted, async fatal 18.
    // =======================================================================
    printf("\n--- (c) 1-move Multimove ending at NONZERO velocity -> expect async fatal 18 ---\n");
    setStdUnits(motor);
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition (c)");

    // Safety guard: this un-terminated velocity move is only physically safe
    // because the MOSFETs are OFF (post-reset default). Assert that explicitly
    // so a future edit that enables them upstream cannot silently make this a
    // real spinning runout.
    {
        getStatusResponse stGuard = motor->getStatus();
        TEST_RESULT("(c) precondition: MOSFETs are DISABLED before un-terminated move",
                    motor->getError() == 0 && (stGuard.statusFlags & (1u << 1)) == 0);
    }

    const uint8_t  cCount = 1;
    const uint32_t cTypes = 0b1;          // bit0 = velocity move
    multimoveListConverted_t cList[cCount] = {
        {0.5f, 0.3f},                     // 0.5 rot/s for 0.3 s, then queue empties WHILE moving
    };
    motor->multimove(cCount, cTypes, cList);
    bool cAccepted = (motor->getError() == 0);
    printf("  multimove accepted (success reply before drain): err=%d\n", motor->getError());
    // The success reply is sent at enqueue time, long before the 0.3 s item drains.
    TEST_RESULT("Nonzero-velocity 1-move accepted at enqueue time", cAccepted);

    // Poll Get status until the async fatal 18 latches (bounded).
    uint8_t  cCode  = 0;
    uint16_t cFlags = 0;
    bool sawFatal18 = false;
    uint32_t deadline = millis() + FATAL18_POLL_MS;
    while (millis() < deadline) {
        if (readFatalCode(motor, &cCode, &cFlags) && cCode == 18) {
            sawFatal18 = true;
            break;
        }
        delay(100);
    }
    printf("  polled fatalCode=%d (expect 18), sawFatal18=%d\n",
           (int)cCode, (int)sawFatal18);
    // error_codes.json: code 18 == ERROR_RUN_OUT_OF_QUEUE_ITEMS.
    TEST_RESULT("Queue drain at nonzero velocity raises async fatal 18", sawFatal18);

    // Recover.
    resetDrain(motor);
    code = 0xFF; flags = 0xFFFF;
    bool cClear = readFatalCode(motor, &code, &flags);
    TEST_RESULT("System reset clears fatal 18", cClear && code == 0);

    // =======================================================================
    // (d) count-0 list -> firmware no-op returning success (see analysis above).
    // =======================================================================
    printf("\n--- (d) count-0 Multimove -> expect no-op success ---\n");
    setStdUnits(motor);
    uint8_t dQueuedBefore = motor->getNQueuedItems();

    const uint8_t  dCount = 0;
    const uint32_t dTypes = 0;            // no moves; bitfield unused
    multimoveListConverted_t dList[1] = { {0.0f, 0.0f} };  // unused (count 0); placeholder
    motor->multimove(dCount, dTypes, dList);
    bool dAccepted = (motor->getError() == 0);
    printf("  count-0 multimove: err=%d\n", motor->getError());
    TEST_RESULT("Count-0 Multimove returns success (no-op)", dAccepted);

    // Nothing was queued.
    uint8_t dQueuedAfter = motor->getNQueuedItems();
    TEST_RESULT("Count-0 Multimove queues nothing",
                dQueuedAfter == dQueuedBefore && dQueuedAfter == 0);

    // No fatal error latched.
    code = 0xFF; flags = 0xFFFF;
    bool dRead = readFatalCode(motor, &code, &flags);
    TEST_RESULT("Count-0 Multimove leaves no fatal error", dRead && code == 0);

    // ---- Clean end: motor already at rest (MOSFETs never enabled); reset. ----
    resetDrain(motor);
    printf("\n=== tm_multimove_edges complete ===\n");
}
