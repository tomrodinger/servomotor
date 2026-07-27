#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_rapid_fire.cpp
//
// Zero-delay command-storm stress test for the Arduino library's request/
// response turnaround. Every Servomotor method here is BLOCKING: it transmits
// a frame and then reads the device's response (or times out) before returning.
// That means back-to-back calls with no host-side delay are naturally paced by
// the round-trip, and must NOT provoke ERROR_COMMAND_OVERFLOW (error_codes.json
// code 5), which only occurs when a byte arrives while both device receive
// buffers are still full -- impossible when the host waits for each response.
// This module proves that invariant holds under sustained rapid fire.
//
// PART A - 50 back-to-back Get status calls with NO delay: every call must
//          succeed at the comms layer and report fatalErrorCode == 0.
//
// PART B - 20 alternating Enable/Disable MOSFETs calls with NO delay. Both are
//          immediate (not queued) and idempotent. After each call, Get status
//          must show the MOSFETs-enabled status bit tracking exactly the last
//          op. The sequence starts with Enable (even index) and ends with
//          Disable (odd index 19), so the final state is DISABLED.
//            Status bit source of truth (common_source_files/device_status.h):
//              #define STATUS_MOSFETS_ENABLED_FLAG_BIT 1
//            get_motor_status_flags() sets (1 << 1) iff is_mosfets_enabled()
//            (firmware/Src/motor_control.c:3364-3366). Mask = 0x0002.
//
// PART C - 30 iterations cycling Get status / Get position / Get temperature /
//          Get supply voltage with NO delay (120 calls total): zero comms
//          errors across the whole burst.
//
// Reports total wall time (millis) and effective commands/sec for each part.
//
// SAFETY:
//   * No motion is ever commanded, so there is no motion queue to drain and no
//     async fatal-error (e.g. ERROR_POSITION_DEVIATION_TOO_LARGE) risk. Enable
//     MOSFETs causes only the ~0.3 s commutation-alignment snap (bench up to a
//     few degrees) -- well within gentle-motion limits, and the shaft is free.
//   * No Test mode (cmd 36) values are used anywhere. Values 10..13 never sent.
//   * tfGetMotor() is unique-ID addressed (extended addressing), so every
//     command is collision-safe on the 39-motor rack; nothing is broadcast and
//     no multi-device responses are solicited. No raw bus frames are written.
//   * The device is reset clean and left at rest (MOSFETs disabled) at the end.
// ---------------------------------------------------------------------------

static const int      NORMAL_RESET_DELAY_MS = 1500;   // wait after a plain reset
static const uint16_t MOSFETS_ENABLED_BIT   = (1 << 1); // STATUS_MOSFETS_ENABLED_FLAG_BIT == 1

static const int      N_STATUS_STORM        = 50;     // Part A
static const int      N_TOGGLE_OPS          = 20;     // Part B (even->enable, odd->disable)
static const int      N_MIXED_CYCLES        = 30;     // Part C (4 commands each)

// Overall wall-clock safety guard: every library call has an internal ~1 s
// timeout; the worst-case total here is ~210 calls, so cap the module well
// above the expected fast path but below anything that would eat the slot.
static const unsigned long MODULE_GUARD_MS  = 240000UL;


// Cleanup tail shared by the normal exit and the guard-timeout early exits:
// leave the motor disabled + reset with a drained RX stream.
static void rfCleanup(Servomotor* motor) {
    motor->disableMosfets();
    motor->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
}

void tm_rapid_fire(void) {
    Serial.println("test_ard_rapid_fire: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    const unsigned long moduleStart = millis();

    // -----------------------------------------------------------------------
    // Clean, known starting state.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);            // no response during reset; don't check errors
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)  // drain any stragglers

    {
        getStatusResponse st = motor->getStatus();
        bool ok = (motor->getError() == 0);
        printf("After reset: commsOk=%d fatalCode=%d flags=0x%04X\n",
               (int)ok, (int)st.fatalErrorCode, (unsigned)st.statusFlags);
        TEST_RESULT("RF No fatal error / clean comms after reset",
                    ok && st.fatalErrorCode == 0);
    }

    // =======================================================================
    // PART A: 50 back-to-back Get status calls, NO delay. All must succeed at
    // the comms layer and report no latched fatal error.
    // =======================================================================
    printf("\n--- PART A: %d back-to-back getStatus, no delay ---\n", N_STATUS_STORM);

    int  aCommErrors  = 0;
    int  aFatalSeen   = 0;
    unsigned long aStart = millis();
    for (int i = 0; i < N_STATUS_STORM; i++) {
        getStatusResponse st = motor->getStatus();
        if (motor->getError() != 0) {
            aCommErrors++;
            printf("  getStatus[%d] comms error %d\n", i, motor->getError());
        } else if (st.fatalErrorCode != 0) {
            aFatalSeen++;
            printf("  getStatus[%d] unexpected fatal code %d\n", i, (int)st.fatalErrorCode);
        }
        if (millis() - moduleStart > MODULE_GUARD_MS) { printf("  guard timeout in Part A\n"); rfCleanup(motor); return; }
    }
    unsigned long aElapsed = millis() - aStart;
    double aRate = aElapsed > 0 ? (1000.0 * N_STATUS_STORM) / (double)aElapsed : 0.0;
    printf("Part A: %d calls in %lu ms -> %.1f cmd/s (commErrors=%d, fatalSeen=%d)\n",
           N_STATUS_STORM, aElapsed, aRate, aCommErrors, aFatalSeen);

    TEST_RESULT("RF-A All 50 back-to-back getStatus succeeded (no comms error)",
                aCommErrors == 0);
    TEST_RESULT("RF-A No latched fatal error during status storm",
                aFatalSeen == 0);

    // =======================================================================
    // PART B: 20 alternating Enable/Disable MOSFETs, NO delay. The status
    // MOSFETs-enabled bit must track the last op exactly; final state disabled.
    // =======================================================================
    printf("\n--- PART B: %d alternating enable/disable MOSFETs, no delay ---\n", N_TOGGLE_OPS);

    int  bCommErrors  = 0;   // comms errors on the toggle op itself
    int  bStatusErrors = 0;  // comms errors on the follow-up status read
    int  bBitMismatch = 0;   // status bit did not match the last op
    bool bFinalDisabled = false;
    unsigned long bStart = millis();
    for (int i = 0; i < N_TOGGLE_OPS; i++) {
        bool enabling = (i % 2 == 0);   // even -> enable, odd -> disable; ends disabled
        if (enabling) motor->enableMosfets();
        else          motor->disableMosfets();
        if (motor->getError() != 0) {
            bCommErrors++;
            printf("  toggle[%d] (%s) comms error %d\n",
                   i, enabling ? "enable" : "disable", motor->getError());
        }

        // Read back the status bit and confirm it matches the op just issued.
        getStatusResponse st = motor->getStatus();
        if (motor->getError() != 0) {
            bStatusErrors++;
            printf("  toggle[%d] status read comms error %d\n", i, motor->getError());
        } else {
            bool bitSet = (st.statusFlags & MOSFETS_ENABLED_BIT) != 0;
            if (bitSet != enabling) {
                bBitMismatch++;
                printf("  toggle[%d] bit mismatch: expected %d got %d (flags=0x%04X)\n",
                       i, (int)enabling, (int)bitSet, (unsigned)st.statusFlags);
            }
            if (i == N_TOGGLE_OPS - 1) bFinalDisabled = !bitSet;
        }
        if (millis() - moduleStart > MODULE_GUARD_MS) { printf("  guard timeout in Part B\n"); rfCleanup(motor); return; }
    }
    unsigned long bElapsed = millis() - bStart;
    int bTotalCalls = N_TOGGLE_OPS * 2;  // toggle + status per iteration
    double bRate = bElapsed > 0 ? (1000.0 * bTotalCalls) / (double)bElapsed : 0.0;
    printf("Part B: %d ops (+%d status reads) in %lu ms -> %.1f cmd/s "
           "(toggleErr=%d, statusErr=%d, bitMismatch=%d)\n",
           N_TOGGLE_OPS, N_TOGGLE_OPS, bElapsed, bRate,
           bCommErrors, bStatusErrors, bBitMismatch);

    TEST_RESULT("RF-B All enable/disable toggles succeeded (no comms error)",
                bCommErrors == 0 && bStatusErrors == 0);
    TEST_RESULT("RF-B Status MOSFETs bit tracked every op",
                bBitMismatch == 0);
    TEST_RESULT("RF-B Final state is MOSFETs disabled",
                bFinalDisabled);

    // Make sure the MOSFETs really are off before moving on (idempotent).
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");

    // =======================================================================
    // PART C: 30 iterations cycling getStatus / getPositionRaw /
    // getTemperature / getSupplyVoltage, NO delay. Zero comms errors expected.
    // =======================================================================
    printf("\n--- PART C: %d cycles of 4 mixed reads, no delay ---\n", N_MIXED_CYCLES);

    int  cCommErrors = 0;
    int  cCalls      = 0;
    unsigned long cStart = millis();
    for (int i = 0; i < N_MIXED_CYCLES; i++) {
        motor->getStatus();
        cCalls++;
        if (motor->getError() != 0) { cCommErrors++; printf("  cyc[%d] getStatus err %d\n", i, motor->getError()); }

        motor->getPositionRaw();
        cCalls++;
        if (motor->getError() != 0) { cCommErrors++; printf("  cyc[%d] getPositionRaw err %d\n", i, motor->getError()); }

        motor->getTemperature();
        cCalls++;
        if (motor->getError() != 0) { cCommErrors++; printf("  cyc[%d] getTemperature err %d\n", i, motor->getError()); }

        motor->getSupplyVoltage();
        cCalls++;
        if (motor->getError() != 0) { cCommErrors++; printf("  cyc[%d] getSupplyVoltage err %d\n", i, motor->getError()); }

        if (millis() - moduleStart > MODULE_GUARD_MS) { printf("  guard timeout in Part C\n"); rfCleanup(motor); return; }
    }
    unsigned long cElapsed = millis() - cStart;
    double cRate = cElapsed > 0 ? (1000.0 * cCalls) / (double)cElapsed : 0.0;
    printf("Part C: %d mixed reads in %lu ms -> %.1f cmd/s (commErrors=%d)\n",
           cCalls, cElapsed, cRate, cCommErrors);

    TEST_RESULT("RF-C Zero comms errors across 120 mixed rapid reads",
                cCommErrors == 0);

    // -----------------------------------------------------------------------
    // Overall throughput report.
    // -----------------------------------------------------------------------
    int totalCalls = N_STATUS_STORM + (N_TOGGLE_OPS * 2) + cCalls;
    unsigned long totalElapsed = aElapsed + bElapsed + cElapsed;
    double totalRate = totalElapsed > 0 ? (1000.0 * totalCalls) / (double)totalElapsed : 0.0;
    printf("\nTOTAL: %d commands in %lu ms of busy time -> %.1f cmd/s effective\n",
           totalCalls, totalElapsed, totalRate);

    // -----------------------------------------------------------------------
    // Leave the device clean and at rest (MOSFETs off).
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    for (uint32_t tf_drain_t0 = millis(); Serial1.available() && (millis() - tf_drain_t0) < 500; ) Serial1.read();  // bounded drain (hang-proof)
}
