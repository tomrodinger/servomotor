#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_fatal_state_matrix.cpp
//
// Behaviour of the command dispatch WHILE a fatal error is latched.
//
// GROUND TRUTH (read from firmware, not from the JSON descriptions):
//   When fatal_error() is entered it __disable_irq()'s and spins forever in a
//   while(1) loop (common_source_files/error_handling.c:169..270). Inside that
//   loop the ONLY packet handler that runs is the reduced handle_packet()
//   (error_handling.c:62..112), which recognises exactly two commands:
//       * SYSTEM_RESET_COMMAND  -> acks + requests a real reset       (l.83..91)
//       * GET_STATUS_COMMAND    -> replies with the full status frame (l.92..103)
//   EVERY other command falls into the default case (l.104..110) and the device
//   replies with an ERROR packet carrying device_status->error_code (the latched
//   fatal code) via rs485_transmit_error_packet() -- it is NOT silent, and it is
//   NOT a normal reply. The getStatus() docstring in Servomotor.cpp:696 states
//   this exact contract ("one of only two commands still answered normally after
//   a fatal error ... every other command then receives an error packet").
//
//   How the library surfaces that error packet: rs485_transmit_error_packet()
//   (common_source_files/RS485.c:549) sends a response frame whose single payload
//   byte is the error code. Communication.cpp:405..413 reads that byte as
//   remoteErrorCode and, being non-zero, RETURNS it verbatim as a POSITIVE value;
//   every command method stores it in _errno (e.g. ping Servomotor.cpp:1283,
//   getPositionRaw:1381+, enableMosfets:150, trapezoidMoveRaw:171). All the
//   library's own comms errors are NEGATIVE (Communication.h:11..19), so a
//   getError() of exactly +30 unambiguously means "device reported firmware
//   fatal code 30". That is the strong, bug-catching assertion this module makes.
//
//   Injected code: Test mode 44 -> fatal_error(44-14) == fatal_error(30) ==
//   ERROR_CONTROL_LOOP_TOOK_TOO_LONG (firmware main.c:909..910; code 30 verified
//   in python_programs/servomotor/error_codes.json:443..444). This is the
//   known-safe injection (recoverable via systemReset; NOT the 10..13 LED hang).
//
//   Flags in the fatal state: fatal_error() refreshes the status snapshot to
//   (flags & in-bootloader-bit) (error_handling.c:227). We are not in the
//   bootloader, so getStatus() must report statusFlags == 0 (the BUG-7 fix,
//   confirmed by the getStatus docstring "the flags all read 0 as of 0.15.4.0").
//
// SAFETY:
//   * NO motion is commanded while the fatal error is latched: in that state
//     handle_packet() never queues anything (trapezoidMove hits the default
//     error path, it does NOT enqueue), so there is nothing to drain and no
//     async fatal-18 risk. The only real motion is one tiny (<=0.05 rot, 0.5 s)
//     trapezoid at the very end to prove the recovered device moves normally;
//     it ends at rest and the queue is drained before asserting.
//   * Test-mode values 10..13 (permanent LED-test hang) are NEVER sent; only 44
//     (recoverable fatal) is injected. After the deliberate fatal error we
//     systemReset + delay(1500) + drain before doing anything else.
//   * tfGetMotor() is unique-ID (extended) addressed, so every frame here is
//     collision-safe on the 39-motor rack; nothing is broadcast that would draw
//     replies from multiple devices. No raw Serial1.write() frames are emitted.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset (no reply during reset)
static const int FATAL_RESET_DELAY_MS  = 2000;  // extra margin recovering from a fatal error
static const int INJECT_SETTLE_MS      = 300;   // let the fatal state latch
static const int SETTLE_MS             = 300;   // commutation-alignment settle after enable

static const uint8_t INJECT_TEST_MODE  = 44;    // -> fatal_error(30)  (main.c:909)
static const uint8_t EXPECT_FATAL_CODE  = 30;   // ERROR_CONTROL_LOOP_TOOK_TOO_LONG (error_codes.json:443)

// Drain any stray bytes left on the RS485 line so the next reply is read cleanly.
static void drainBus() {
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}

// Reset to a known-clean state, wait out the (reply-less) reset window, drain.
static void cleanReset(Servomotor* motor, int ms) {
    motor->systemReset();
    delay(ms);
    drainBus();
}

// Read the latched fatal code + flags via Get status (one of the two commands
// answered normally in the fatal state). commsOk == (getError()==0).
static bool readStatus(Servomotor* motor, uint8_t* codeOut, uint16_t* flagsOut, bool* commsOk) {
    getStatusResponse st = motor->getStatus();
    *commsOk = (motor->getError() == 0);
    *codeOut = st.fatalErrorCode;
    *flagsOut = st.statusFlags;
    return *commsOk;
}

// Prove the device is alive at the application level: a 0..9 ping payload must
// echo back unchanged. Returns true only on a full, error-free round-trip.
static bool pingRoundTrips(Servomotor* motor) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)i;
    pingResponse r = motor->ping(payload);
    if (motor->getError() != 0) return false;
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// Poll the motion queue until it drains to 0 (bounded), then settle.
static uint8_t waitForIdle(Servomotor* motor) {
    uint8_t n = 0xFF;
    uint32_t start = millis();
    while (millis() - start < 15000) {   // hard timeout guard
        n = motor->getNQueuedItems();
        if (motor->getError() == 0 && n == 0) break;
        delay(10);
    }
    delay(150);
    return n;
}

void tm_fatal_state_matrix(void) {
    Serial.println("test_ard_fatal_state_matrix: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    cleanReset(motor, NORMAL_RESET_DELAY_MS);

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    uint8_t  code  = 0xFF;
    uint16_t flags = 0xFFFF;
    bool     comms = false;

    readStatus(motor, &code, &flags, &comms);
    printf("Baseline: commsOk=%d fatalCode=%d flags=0x%04X\n", (int)comms, (int)code, (unsigned)flags);
    TEST_RESULT("FSM Baseline: no fatal error after reset", comms && code == 0);

    // =======================================================================
    // Inject fatal code 30 (Test mode 44). The testMode command itself will be
    // answered by the deferred fatal error response; we don't assert on its own
    // return (matches tm_error_injection), just let the state latch.
    // =======================================================================
    printf("\n--- Inject Test mode %d -> fatal code %d ---\n",
           (int)INJECT_TEST_MODE, (int)EXPECT_FATAL_CODE);
    motor->testMode(INJECT_TEST_MODE);
    delay(INJECT_SETTLE_MS);
    drainBus();  // discard the injection's own deferred error reply before probing

    // -----------------------------------------------------------------------
    // (1) Get status STILL WORKS in the fatal state: normal reply (getError==0),
    //     reporting EXACTLY the latched code with flags refreshed to 0.
    //     firmware: error_handling.c:92..103 ; flags: error_handling.c:227
    // -----------------------------------------------------------------------
    code = 0xFF; flags = 0xFFFF; comms = false;
    readStatus(motor, &code, &flags, &comms);
    printf("In fatal: getStatus commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)comms, (int)code, (unsigned)flags);
    TEST_RESULT("FSM getStatus answered normally in fatal state", comms);              // getError()==0
    TEST_RESULT("FSM getStatus reports latched fatal code 30", comms && code == EXPECT_FATAL_CODE);
    TEST_RESULT("FSM getStatus flags cleared to 0 (BUG-7 fix)", comms && flags == 0);

    // -----------------------------------------------------------------------
    // (2) getPosition -> default error path: the device replies with an ERROR
    //     packet carrying the latched fatal code, so getError() == +30 (a
    //     POSITIVE firmware code, not a negative comms error, not silence).
    //     firmware: error_handling.c:104..108 ; surfaced Communication.cpp:412
    // -----------------------------------------------------------------------
    motor->getPositionRaw();
    int posErr = motor->getError();
    printf("In fatal: getPositionRaw getError=%d (expect %d)\n", posErr, (int)EXPECT_FATAL_CODE);
    TEST_RESULT("FSM getPosition returns error packet carrying fatal code 30",
                posErr == (int)EXPECT_FATAL_CODE);

    // -----------------------------------------------------------------------
    // (3) ping -> default error path: same error packet; getError() == +30 and
    //     the payload is NOT echoed (the library returns a zeroed default).
    // -----------------------------------------------------------------------
    uint8_t pingPayload[10];
    for (int i = 0; i < 10; i++) pingPayload[i] = (uint8_t)i;
    pingResponse pr = motor->ping(pingPayload);
    int pingErr = motor->getError();
    bool echoed = (memcmp(pr.responsePayload, pingPayload, 10) == 0);
    printf("In fatal: ping getError=%d echoed=%d\n", pingErr, (int)echoed);
    TEST_RESULT("FSM ping returns error packet carrying fatal code 30",
                pingErr == (int)EXPECT_FATAL_CODE);
    TEST_RESULT("FSM ping payload NOT echoed in fatal state", !echoed);

    // -----------------------------------------------------------------------
    // (4) enableMosfets -> default error path (no MOSFET enable happens; the
    //     motor stays powered off, as fatal_error() already disabled them).
    // -----------------------------------------------------------------------
    motor->enableMosfets();
    int enErr = motor->getError();
    printf("In fatal: enableMosfets getError=%d\n", enErr);
    TEST_RESULT("FSM enableMosfets returns error packet carrying fatal code 30",
                enErr == (int)EXPECT_FATAL_CODE);

    // -----------------------------------------------------------------------
    // (5) trapezoidMove -> default error path. Crucially this does NOT enqueue a
    //     move in the fatal state (handle_packet() has no TRAPEZOID case), so no
    //     motion can result; getError() == +30.
    // -----------------------------------------------------------------------
    motor->trapezoidMove(0.05f, 0.5f);   // rejected before any queueing
    int moveErr = motor->getError();
    printf("In fatal: trapezoidMove getError=%d\n", moveErr);
    TEST_RESULT("FSM trapezoidMove returns error packet carrying fatal code 30",
                moveErr == (int)EXPECT_FATAL_CODE);

    // =======================================================================
    // System reset clears EVERYTHING; the device becomes fully responsive.
    // =======================================================================
    printf("\n--- systemReset clears the fatal state ---\n");
    cleanReset(motor, FATAL_RESET_DELAY_MS);

    code = 0xFF; flags = 0xFFFF; comms = false;
    readStatus(motor, &code, &flags, &comms);
    printf("After reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
           (int)comms, (int)code, (unsigned)flags);
    TEST_RESULT("FSM systemReset clears fatal code (back to 0)", comms && code == 0);
    TEST_RESULT("FSM getStatus comms OK after reset", comms);
    TEST_RESULT("FSM device responsive (ping round-trips) after reset", pingRoundTrips(motor));

    // =======================================================================
    // A NORMAL move works again after recovery.
    // =======================================================================
    printf("\n--- normal move works after recovery ---\n");
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);

    motor->enableMosfets();
    int recEnErr = motor->getError();
    TEST_RESULT("FSM enableMosfets succeeds after recovery", recEnErr == 0);
    delay(SETTLE_MS);  // let the commutation-alignment transient settle

    int64_t startPos = motor->getPositionRaw();
    bool startPosOk = (motor->getError() == 0);

    motor->trapezoidMove(0.05f, 0.5f);   // gentle: 0.05 rot over 0.5 s, ends at rest
    int recMoveErr = motor->getError();
    TEST_RESULT("FSM normal trapezoid move accepted after recovery", recMoveErr == 0);

    uint8_t drained = waitForIdle(motor);
    printf("Queue depth after normal move drains = %d\n", (int)drained);

    // No async fatal error resulted from the normal move.
    code = 0xFF; flags = 0xFFFF; comms = false;
    readStatus(motor, &code, &flags, &comms);
    TEST_RESULT("FSM no fatal error after recovered normal move", comms && code == 0);

    // The move actually advanced the commanded position (proves real execution).
    int64_t endPos = motor->getPositionRaw();
    bool endPosOk = (motor->getError() == 0);
    printf("Position start=%lld end=%lld\n", (long long)startPos, (long long)endPos);
    TEST_RESULT("FSM recovered move advanced the position",
                startPosOk && endPosOk && (endPos - startPos) > 1000);  // 0.05 rot >> 1000 microsteps

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest.
    // -----------------------------------------------------------------------
    motor->disableMosfets();
    cleanReset(motor, NORMAL_RESET_DELAY_MS);
}
