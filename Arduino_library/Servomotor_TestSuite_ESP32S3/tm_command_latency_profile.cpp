#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_command_latency_profile.cpp
//
// HOW LONG EVERY COMMAND TAKES, AND THAT NONE OF THEM MONOPOLISES THE BUS.
//
// THE PROPERTY:
//
//     Every command on this device is a short, bounded transaction. The time a
//     command takes is explained by the number of bytes it puts on the wire and
//     nothing else. No command blocks the firmware's foreground loop, no
//     command's cost drifts upward over a long run, and every command finishes
//     with an order of magnitude to spare against the library's one-second
//     receive timeout.
//
// WHY THE SUITE NEEDS THIS. Every other module in the suite asks "is the answer
// correct?". None of them asks "how long did the answer take?". That leaves a
// whole class of regression invisible: a firmware change that made one command
// sit in the main loop for tens of milliseconds -- a busy-wait, a flash write, a
// long __disable_irq section, an accidentally synchronous transmit -- would
// still return the right value, so every functional assertion in the suite would
// still pass. The only symptom would be a machine that had become sluggish, and
// nothing here would notice. This module is the performance-regression guard.
//
// The claim is grounded in how the firmware is actually built. Commands are
// serviced in the FOREGROUND: main()'s `while(1)` (firmware/Src/main.c:1444)
// polls `rs485_has_a_packet()` (:1451) and calls `process_packet()` (:1456,
// defined at :208), which is one big switch that formats a reply and hands it to
// `rs485_finalize_and_transmit_packet()`. Motion runs in the 31.25 kHz control
// interrupt, at a HIGHER NVIC priority than the receiver
// (`NVIC_SetPriority(TIM16_IRQn, 1)`, firmware/Src/motor_control.c:380, versus
// `NVIC_SetPriority(USART1_IRQn, 2)`, common_source_files/RS485.c:165). So a
// slow command handler cannot delay motion -- but it CAN delay the next command,
// because there is exactly one foreground loop and it is not re-entrant. That is
// precisely the failure mode measured here.
//
// WHAT IT ASSERTS:
//   1. The timing rig itself is sound (the clock resolves a known delay, and two
//      independent timing methods agree).
//   2. Each of eighteen safe commands, over 120 consecutive calls, answers every
//      call with no communication error and a median far inside the timeout.
//   3. Nothing monopolises the bus: no command's median is a large multiple of
//      any other's; the small-payload commands are all within a small factor of
//      each other; the state-changing commands (emergency stop, zero position,
//      disable MOSFETs) are no slower than the cheapest read, which is what rules
//      out a hidden flash write; and the one genuinely slower command,
//      Get debug values, is slower by an amount consistent with its 112-byte
//      payload rather than with blocking.
//   4. The spread is bounded: no bimodal stalls, mean tracks median, and not one
//      call anywhere in the module comes near the library's timeout.
//   5. Latency does not degrade over the run: second half versus first half for
//      every command, plus a re-profile of two commands at the very end after
//      ~2600 transactions have gone by.
//
// HOW IT DIFFERS FROM WHAT IS ALREADY HERE. Four modules already send a lot of
// commands, and none of them times anything per-command:
//   * `tm_rapid_fire` proves a command STORM does not overflow the receive
//     buffers. It reports one aggregate commands/sec figure for a whole burst.
//   * `tm_ping_stress` proves Ping's echo is byte-exact over 100 payloads. It
//     prints a round-trip min/max for Ping alone and asserts nothing about it.
//   * `tm_concurrent_command_pressure` proves traffic cannot corrupt MOTION --
//     the measured quantity is the endpoint of a move, not the cost of a command.
//   * `tm_soak` proves nothing accumulates over hundreds of moves; its subject is
//     the planner, not the link.
// This module is the only one whose measured quantity is TIME PER COMMAND, and
// the only one that compares commands against each other.
//
// CAVEAT, STATED UP FRONT: these are HOST-SIDE round-trip times. Each figure is
// the whole path -- the host building and writing the frame, the USB-serial or
// UART link, the wire at 230400 baud, the firmware's foreground loop, the reply,
// and the host's parse -- plus whatever scheduler and USB latency the host adds.
// On the ESP32-S3 that overhead is small; in host mode on a Mac a USB-serial
// adapter's latency timer can dominate everything else. So the ABSOLUTE numbers
// are a property of the whole setup, not of the firmware, and the thresholds
// here are deliberately generous. The RELATIVE comparisons are the real test:
// host overhead is a roughly constant per-transaction cost, so it inflates every
// command equally and cannot manufacture the "one command is much worse than the
// others" signature this module is looking for. Timing is also quantised to the
// millisecond, because millis() is the only clock the test framework exposes and
// ArduinoEmulator.h provides no micros(); that is why calls are timed in batches
// of ten, which recovers 0.1 ms resolution on the per-call figures.
//
// TWO HOST-SIDE COSTS ARE LARGE AND ARE NOT EXPLAINED BY BYTES ON THE WIRE, so
// the opening claim above should be read as "explained by the wire once the
// fixed per-transaction cost is subtracted", which is exactly what the relative
// comparisons do:
//   * Every library call begins with an unconditional debug line --
//     `Serial.println("[Motor] ... called.")`, 144 of them in Servomotor.cpp,
//     with no VERBOSE guard. On-device that is a USB CDC write; in host mode
//     ArduinoEmulator's ConsoleSerial writes it to std::cout with std::endl,
//     i.e. a flush per call. Either can stall for milliseconds if the reader is
//     busy, and the stall lands INSIDE the timed region.
//   * In host mode one request is FIVE separate write() calls (size byte,
//     extended-address byte, 8-byte unique ID, command byte, CRC32) or SIX when
//     there is a payload -- Communication.cpp:177..250 -- and
//     ArduinoEmulator.h:196/213 follows every one of them with tcdrain(). So the
//     host-side floor scales with the number of write CHUNKS, not with bytes,
//     and payload-bearing commands pay one extra tcdrain for reasons that have
//     nothing to do with the firmware.
// Both of these are why the absolute budgets below are stated as fractions of
// the library timeout rather than as tight numbers, and why the spread and
// mean/median checks carry an absolute slack term as well as a ratio.
//
// The predicted wire time printed alongside each measurement comes from the
// actual framing: a request is 1 size byte + 9 extended-address bytes + 1 command
// byte + payload + 4 CRC32 bytes; a data reply is a 3-byte header + payload +
// 4 CRC32 bytes; a no-error reply is a hard-coded 6 bytes
// (NO_ERROR_RESPONSE_CRC32_ENABLED, common_source_files/RS485.h:23). CRC32 is on
// by default (`static uint8_t crc32_enabled = 1;`, common_source_files/RS485.c:49)
// and nothing here turns it off. The baud is 230400 (RS485.c:159), 10 bits per
// byte, so 43.4 us per byte. Payload sizes are taken with sizeof() from the
// generated structs in Servomotor.h so the prediction cannot go stale.
//
// The timeout being compared against is the library's:
//     #define TIMEOUT_MS 1000     // Arduino_library/Communication.cpp:6
// enforced in the response wait loop at Communication.cpp:545 and in
// receiveBytes() at :724.
//
// SAFETY:
//   * MOSFETs stay OFF for the whole module, and NOTHING IS EVER QUEUED. Not one
//     motion command is sent -- no trapezoid move, no velocity or acceleration
//     move, no multimove, no homing, no go-to-closed-loop, no enable. The shaft
//     cannot turn because nothing ever asks it to.
//   * The three state-changing commands profiled are all no-ops on an idle,
//     de-energised machine. `emergency_stop()` disables the MOSFETs and clears an
//     already-empty queue (firmware/Src/motor_control.c:3405). `zero_position()`
//     assigns zero to the position accumulators (:3311) -- note it raises
//     ERROR_QUEUE_NOT_EMPTY if anything is queued, which is why section 1 asserts
//     the queue is empty before the table runs and why the module never queues.
//     `disableMosfets` is idempotent.
//   * Rack-safe: addressed by unique ID through tfGetMotor() (extended
//     addressing). No broadcast, no Set device alias, no Test mode, no
//     calibration, no firmware upgrade, no raw frames.
//   * `Get communication statistics` is always called with resetCounter = 0, so
//     it observes the error counters without clearing them and cannot disturb
//     `tm_communication_statistics`.
//   * ONE of the twelve getters is not read-only: `Get max PID error` clears the
//     min/max accumulators every time it is read (motor_control.c:3480-3490), so
//     this module leaves them at their empty sentinel. That is harmless -- the
//     closing `tfhReset(m)` restores them anyway, and every module in the suite
//     resets on entry -- but it must not be described as a pure read.
//   * Every section ends with tfhReset(m), and the module leaves the motor clean.
// ---------------------------------------------------------------------------

// ---- timing rig configuration ---------------------------------------------
static const int TMCLP_BATCH   = 10;                          // calls per timed batch
static const int TMCLP_BATCHES = 12;                          // batches per command
static const int TMCLP_CALLS   = TMCLP_BATCH * TMCLP_BATCHES; // 120 samples per command

static const uint32_t TMCLP_LIB_TIMEOUT_MS   = 1000;   // Communication.cpp:6
// Both budgets below are stated as a FRACTION OF THE LIBRARY TIMEOUT, not as a
// tight number. Anything tighter is a claim about the host's USB stack rather
// than about the firmware: in host mode a request costs five or six
// write()+tcdrain() round trips (see the caveat above), so a slow adapter can
// put a perfectly healthy command into the tens of milliseconds without any
// firmware involvement. What these still catch is the thing worth catching --
// a command that has become a large fraction of the timeout, i.e. one that is
// blocking rather than transacting.
static const uint32_t TMCLP_MEDIAN_BUDGET_US = 200000; // a median must stay under a fifth of the timeout
static const uint32_t TMCLP_WORST_CALL_MS    = 500;    // no single call past half the timeout
static const uint32_t TMCLP_BAUD             = 230400; // RS485.c:159

// Framing overheads, in bytes on the wire.
static const uint32_t TMCLP_REQ_OVERHEAD  = 1 + 9 + 1 + 4; // size + ext address + command + CRC32
static const uint32_t TMCLP_DATA_OVERHEAD = 3 + 4;         // header[3] + CRC32
static const uint32_t TMCLP_NOERR_BYTES   = 6;             // RS485.h:23

struct CmdSpec {
    const char* name;
    uint8_t     id;         // command number in python_programs/servomotor/motor_commands.json
    uint16_t    reqPayload; // payload bytes host -> device
    uint16_t    rspPayload; // payload bytes device -> host (data replies only)
    bool        dataReply;  // false => the 6-byte "no error" reply
    // A getter -- something whose purpose is to return a value. NOT a promise of
    // purity: Get max PID error (cmd 39) CLEARS min_PID_error/max_PID_error as it
    // reads them (motor_control.c:3480-3490), and Get communication statistics
    // would clear its counters if it were ever passed a nonzero resetCounter,
    // which this module never does. Neither writes flash, which is all the
    // grouping in section 3 relies on.
    bool        isRead;
};

// Eighteen commands, every one of them safe on a shared rack with the output
// stage off. Command numbers verified against motor_commands.json.
static const CmdSpec TMCLP_CMDS[] = {
    { "Get status",                           16, 0, sizeof(getStatusResponse),                  true,  true  },
    { "Get position",                         34, 0, sizeof(getPositionResponse),                true,  true  },
    { "Get n queued items",                   11, 0, sizeof(getNQueuedItemsResponse),            true,  true  },
    { "Get hall sensor position",             15, 0, sizeof(getHallSensorPositionResponse),      true,  true  },
    { "Get comprehensive position",           37, 0, sizeof(getComprehensivePositionResponse),   true,  true  },
    { "Get supply voltage",                   38, 0, sizeof(getSupplyVoltageResponse),           true,  true  },
    { "Get temperature",                      42, 0, sizeof(getTemperatureResponse),             true,  true  },
    { "Get max PID error",                    39, 0, sizeof(getMaxPidErrorResponse),             true,  true  },
    { "Get firmware version",                 25, 0, sizeof(getFirmwareVersionResponse),         true,  true  },
    { "Ping",                                 31, sizeof(pingPayload), sizeof(pingResponse),     true,  true  },
    { "Get communication statistics",         47, sizeof(getCommunicationStatisticsPayload),
                                                  sizeof(getCommunicationStatisticsResponse),    true,  true  },
    { "Get debug values",                     45, 0, sizeof(getDebugValuesResponse),             true,  true  },
    { "Set maximum velocity",                  3, sizeof(setMaximumVelocityPayload),     0,      false, false },
    { "Set maximum acceleration",              5, sizeof(setMaximumAccelerationPayload), 0,      false, false },
    { "Set max allowable position deviation", 44, sizeof(setMaxAllowablePositionDeviationPayload),
                                                  0,                                            false, false },
    { "Zero position",                        13, 0, 0,                                          false, false },
    { "Emergency stop",                       12, 0, 0,                                          false, false },
    { "Disable MOSFETs",                       0, 0, 0,                                          false, false },
};
static const int TMCLP_NCMD = (int)(sizeof(TMCLP_CMDS) / sizeof(TMCLP_CMDS[0]));

// Indices used by name later on.
static const int TMCLP_IDX_STATUS   = 0;
static const int TMCLP_IDX_POSITION = 1;
static const int TMCLP_IDX_DEBUG    = 11;

// A command counts as "small payload" if its whole transaction fits in this many
// bytes. Everything except Get debug values does.
static const uint32_t TMCLP_SMALL_BYTES = 50;

struct LatencyResult {
    uint32_t batchUs[TMCLP_BATCHES]; // per-call microseconds, one figure per batch
    uint32_t meanUs;                 // whole-run wall time / calls
    uint32_t medianUs;               // median of the batch figures
    uint32_t minBatchUs;
    uint32_t maxBatchUs;
    uint32_t secondWorstBatchUs;     // robust to a single host hiccup
    uint32_t firstHalfUs;            // per-call mean over the first half of the calls
    uint32_t secondHalfUs;           // ... and the second half
    uint32_t worstCallMs;            // largest single observed round trip
    uint32_t totalMs;                // whole-run wall time
    uint32_t batchSumMs;             // sum of the individual batch times
    int      errors;                 // calls that came back with a comms error
};

static LatencyResult gRes[TMCLP_NCMD];
static uint8_t       gPingPayload[10];
static uint32_t      gModuleWorstCallMs;
static uint32_t      gTotalCalls;

static uint32_t tmclpWireBytes(const CmdSpec& s) {
    uint32_t req = TMCLP_REQ_OVERHEAD + (uint32_t)s.reqPayload;
    uint32_t rsp = s.dataReply ? (TMCLP_DATA_OVERHEAD + (uint32_t)s.rspPayload) : TMCLP_NOERR_BYTES;
    return req + rsp;
}

// 10 bits per byte on the line, so bytes * 10 / baud seconds.
static uint32_t tmclpWireMicros(const CmdSpec& s) {
    return (uint32_t)((uint64_t)tmclpWireBytes(s) * 10ULL * 1000000ULL / (uint64_t)TMCLP_BAUD);
}

static void tmclpSort(uint32_t* v, int n) {
    for (int i = 1; i < n; i++) {
        uint32_t key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }
}

// Issue exactly one instance of command `idx`. Return values are deliberately
// discarded: this module measures the transaction, not the answer, and every
// other module in the suite already checks the answers.
static void tmclpIssue(Servomotor* m, int idx) {
    switch (idx) {
        case 0:  m->getStatus();                                            break;
        case 1:  m->getPositionRaw();                                       break;
        case 2:  m->getNQueuedItems();                                      break;
        case 3:  m->getHallSensorPositionRaw();                             break;
        case 4:  m->getComprehensivePositionRaw();                          break;
        case 5:  m->getSupplyVoltageRaw();                                  break;
        case 6:  m->getTemperatureRaw();                                    break;
        case 7:  m->getMaxPidErrorRaw();                                    break;
        case 8:  m->getFirmwareVersion();                                   break;
        case 9:  m->ping(gPingPayload);                                     break;
        // resetCounter = 0: observe the counters, never clear them. The cast is
        // only for clarity -- the two overloads are getCommunicationStatistics(
        // uint8_t) and getCommunicationStatistics(uint64_t, uint8_t)
        // (Servomotor.h:542-543), so a single argument already picks the first
        // one unambiguously.
        case 10: m->getCommunicationStatistics((uint8_t)0);                 break;
        case 11: m->getDebugValues();                                       break;
        case 12: m->setMaximumVelocity(4.0f);                               break;
        case 13: m->setMaximumAcceleration(2000.0f);                        break;
        case 14: m->setMaxAllowablePositionDeviationRaw((int64_t)(TFH_CPR * 2)); break;
        case 15: m->zeroPosition();                                         break;
        case 16: m->emergencyStop();                                        break;
        case 17: m->disableMosfets();                                       break;
        default: break;
    }
}

// Run `calls` instances of one command, timing the whole run, each batch of ten,
// and each individual call. The batch figures give 0.1 ms resolution on the
// distribution; the per-call figures, coarse as they are, are all that is needed
// to catch a single call that stalled for tens of milliseconds.
static void tmclpProfile(Servomotor* m, int idx, LatencyResult* r) {
    memset(r, 0, sizeof(*r));
    r->minBatchUs = 0xFFFFFFFFu;

    uint32_t runStart = millis();
    uint32_t halfMark = runStart;

    for (int b = 0; b < TMCLP_BATCHES; b++) {
        uint32_t batchStart = millis();
        for (int c = 0; c < TMCLP_BATCH; c++) {
            uint32_t callStart = millis();
            tmclpIssue(m, idx);
            uint32_t callMs = millis() - callStart;
            if (m->getError() != 0) r->errors++;
            if (callMs > r->worstCallMs) r->worstCallMs = callMs;
            gTotalCalls++;
        }
        uint32_t batchMs = millis() - batchStart;
        r->batchSumMs += batchMs;
        r->batchUs[b] = (uint32_t)((uint64_t)batchMs * 1000ULL / (uint64_t)TMCLP_BATCH);
        if (b == (TMCLP_BATCHES / 2) - 1) halfMark = millis();
    }

    uint32_t runEnd = millis();
    r->totalMs      = runEnd - runStart;
    r->meanUs       = (uint32_t)((uint64_t)r->totalMs * 1000ULL / (uint64_t)TMCLP_CALLS);
    r->firstHalfUs  = (uint32_t)((uint64_t)(halfMark - runStart) * 1000ULL / (uint64_t)(TMCLP_CALLS / 2));
    r->secondHalfUs = (uint32_t)((uint64_t)(runEnd - halfMark) * 1000ULL / (uint64_t)(TMCLP_CALLS / 2));

    uint32_t sorted[TMCLP_BATCHES];
    for (int b = 0; b < TMCLP_BATCHES; b++) sorted[b] = r->batchUs[b];
    tmclpSort(sorted, TMCLP_BATCHES);
    r->minBatchUs         = sorted[0];
    r->medianUs           = sorted[TMCLP_BATCHES / 2];
    r->secondWorstBatchUs = sorted[TMCLP_BATCHES - 2];
    r->maxBatchUs         = sorted[TMCLP_BATCHES - 1];

    if (r->worstCallMs > gModuleWorstCallMs) gModuleWorstCallMs = r->worstCallMs;
}

// Guard every ratio against a zero denominator.
static uint32_t tmclpNz(uint32_t v) { return v ? v : 1u; }

void tm_command_latency_profile(void) {
    Serial.println("tm_command_latency_profile: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // Re-initialise all file-scope state: the host harness can run modules
    // back-to-back inside one process.
    memset(gRes, 0, sizeof(gRes));
    gModuleWorstCallMs = 0;
    gTotalCalls = 0;
    for (int i = 0; i < 10; i++) gPingPayload[i] = (uint8_t)(0x5A + i);

    // =====================================================================
    // 1. THE TIMING RIG. Every number in this module is a difference of two
    //    millis() readings, so before measuring anything the clock and the two
    //    timing methods have to be shown to work. A rig that silently reported
    //    zero, or that disagreed with itself, would make the whole module
    //    vacuous -- and a vacuous performance test is worse than none, because
    //    it looks like coverage.
    // =====================================================================
    printf("\n--- 1. the timing rig, and a quiet starting machine ---\n");
    {
        tfhReset(m);
        m->disableMosfets();
        delay(200);

        uint32_t t0 = millis();
        delay(200);
        uint32_t observed = millis() - t0;
        printf("   a commanded 200 ms delay measured %lu ms\n", (unsigned long)observed);
        TEST_RESULT("the millisecond clock resolves a known 200 ms delay",
                    observed >= 195 && observed <= 400);

        // Nothing may be queued. zero_position() (motor_control.c:3311) raises
        // ERROR_QUEUE_NOT_EMPTY if anything is, and the whole module depends on
        // an idle machine.
        uint8_t queued = m->getNQueuedItems();
        bool queueRead = (m->getError() == 0);
        uint8_t fatal0 = tfhFatal(m);
        printf("   starting state: queue=%u fatal=%u\n", (unsigned)queued, (unsigned)fatal0);
        TEST_RESULT("the machine starts idle: queue empty and no latched fault",
                    queueRead && queued == 0 && fatal0 == 0);

        // Section 3 addresses the table by POSITION. Reordering the table would
        // silently make those comparisons compare the wrong pair of commands and
        // still "pass", so pin the three indices to the names they must name.
        bool idxOk = (strcmp(TMCLP_CMDS[TMCLP_IDX_STATUS].name,   "Get status")       == 0) &&
                     (strcmp(TMCLP_CMDS[TMCLP_IDX_POSITION].name, "Get position")     == 0) &&
                     (strcmp(TMCLP_CMDS[TMCLP_IDX_DEBUG].name,    "Get debug values") == 0);
        TEST_RESULT("the table indices used by the later comparisons still name the right commands",
                    idxOk);

        // Calibration run: the same 120-call profile the table uses, on
        // Get status, used here only to check the rig against itself.
        LatencyResult cal;
        tmclpProfile(m, TMCLP_IDX_STATUS, &cal);
        uint32_t diff = (cal.batchSumMs > cal.totalMs) ? (cal.batchSumMs - cal.totalMs)
                                                       : (cal.totalMs - cal.batchSumMs);
        printf("   120 Get status calls: whole-run %lu ms, sum of batches %lu ms, mean %lu us/call\n",
               (unsigned long)cal.totalMs, (unsigned long)cal.batchSumMs, (unsigned long)cal.meanUs);
        TEST_RESULT("the calibration burst completes every call without a comms error",
                    cal.errors == 0);
        // Each batch time carries up to 1 ms of quantisation error, so twelve of
        // them may drift from the whole-run figure by that much in total.
        TEST_RESULT("timing a run as a whole and timing it batch by batch agree",
                    diff <= (uint32_t)TMCLP_BATCHES + cal.totalMs / 20);
    }

    // =====================================================================
    // 2. THE PROFILE. 120 consecutive calls of each of eighteen commands, back
    //    to back with no host-side pause, so the figure measured is the true
    //    turnaround of the request/reply pair and not the host's pacing.
    //
    //    Deliberately NOT reset between commands: a reset costs 1.5 s and would
    //    hand the firmware an idle bus to recover in. The point is to keep the
    //    link saturated from the first command to the last.
    // =====================================================================
    printf("\n--- 2. round-trip profile: %d commands x %d calls ---\n", TMCLP_NCMD, TMCLP_CALLS);
    printf("   %-3s %-38s %5s %8s %9s %8s %8s %8s %7s %4s\n",
           "cmd", "command", "bytes", "wire_us", "mean_us", "bmin_us", "bmed_us", "bmax_us",
           "worst", "err");
    {
        for (int i = 0; i < TMCLP_NCMD; i++) {
            const CmdSpec& s = TMCLP_CMDS[i];
            tmclpProfile(m, i, &gRes[i]);
            printf("   %-3u %-38s %5lu %8lu %9lu %8lu %8lu %8lu %5lums %4d\n",
                   (unsigned)s.id, s.name,
                   (unsigned long)tmclpWireBytes(s), (unsigned long)tmclpWireMicros(s),
                   (unsigned long)gRes[i].meanUs, (unsigned long)gRes[i].minBatchUs,
                   (unsigned long)gRes[i].medianUs, (unsigned long)gRes[i].maxBatchUs,
                   (unsigned long)gRes[i].worstCallMs, gRes[i].errors);
            TEST_RESULT(std::string(s.name) + " (cmd " + std::to_string((int)s.id) +
                            ") answers all " + std::to_string(TMCLP_CALLS) +
                            " calls with no error and a median well inside the library timeout",
                        gRes[i].errors == 0 && gRes[i].medianUs < TMCLP_MEDIAN_BUDGET_US);
        }
    }

    // =====================================================================
    // 3. NOTHING MONOPOLISES THE BUS. The regression this module exists to
    //    catch does not look like a wrong answer; it looks like ONE command
    //    costing far more than its neighbours. So the comparisons are between
    //    commands, which also cancels out whatever fixed per-transaction
    //    overhead the host contributes.
    // =====================================================================
    printf("\n--- 3. no command costs far more than the others ---\n");
    {
        uint32_t fastest = 0xFFFFFFFFu, slowest = 0;
        int fastestIdx = 0, slowestIdx = 0;
        uint32_t smallFastest = 0xFFFFFFFFu, smallSlowest = 0;
        uint32_t readFastest = 0xFFFFFFFFu;
        for (int i = 0; i < TMCLP_NCMD; i++) {
            uint32_t v = gRes[i].medianUs;
            if (v < fastest) { fastest = v; fastestIdx = i; }
            if (v > slowest) { slowest = v; slowestIdx = i; }
            if (tmclpWireBytes(TMCLP_CMDS[i]) <= TMCLP_SMALL_BYTES) {
                if (v < smallFastest) smallFastest = v;
                if (v > smallSlowest) smallSlowest = v;
            }
            if (TMCLP_CMDS[i].isRead && v < readFastest) readFastest = v;
        }
        printf("   fastest: %s at %lu us; slowest: %s at %lu us (ratio %.2f)\n",
               TMCLP_CMDS[fastestIdx].name, (unsigned long)fastest,
               TMCLP_CMDS[slowestIdx].name, (unsigned long)slowest,
               (double)slowest / (double)tmclpNz(fastest));
        printf("   small-payload group (<= %lu bytes): %lu us .. %lu us (ratio %.2f)\n",
               (unsigned long)TMCLP_SMALL_BYTES, (unsigned long)smallFastest,
               (unsigned long)smallSlowest,
               (double)smallSlowest / (double)tmclpNz(smallFastest));

        TEST_RESULT("no command's median is more than ten times the fastest command's",
                    slowest <= 10u * tmclpNz(fastest));
        TEST_RESULT("the small-payload commands are all within a factor of four of each other",
                    smallSlowest <= 4u * tmclpNz(smallFastest));

        // A hidden flash write would show up here: an STM32G0 page erase plus
        // program is tens of milliseconds, and the only command that legitimately
        // does one is Set device alias (main.c:563), which this module never
        // sends. The three state-changers here only assign to variables.
        uint32_t worstStateChanger = 0;
        const char* worstStateName = "";
        for (int i = 0; i < TMCLP_NCMD; i++) {
            if (!TMCLP_CMDS[i].isRead && gRes[i].medianUs > worstStateChanger) {
                worstStateChanger = gRes[i].medianUs;
                worstStateName = TMCLP_CMDS[i].name;
            }
        }
        printf("   slowest state-changing command: %s at %lu us, against the cheapest read at %lu us\n",
               worstStateName, (unsigned long)worstStateChanger, (unsigned long)readFastest);
        TEST_RESULT("the setters and the stop/zero/disable commands are no dearer than a cheap read, "
                    "so none of them persists to flash or blocks",
                    worstStateChanger <= (5u * tmclpNz(readFastest)) / 2u);

        // Get debug values is the one command with a large reply: 112 payload
        // bytes against Get status's 3. At 230400 baud those extra bytes are
        // worth several milliseconds all by themselves, so it SHOULD be dearer.
        // What must not happen is it being dearer by far more than that.
        //
        // The two commands send an IDENTICAL 15-byte request (no payload, so the
        // same five write()+tcdrain() chunks in host mode); the only difference
        // is 109 extra reply bytes, i.e. 4.73 ms of unavoidable wire time. That
        // makes the lower bound safe. The UPPER bound is deliberately loose --
        // eight times the predicted cost -- because the bigger reply can cross a
        // USB-serial adapter's latency-timer boundary an extra time, which adds
        // tens of milliseconds that the firmware had no part in. A genuine
        // blocking handler shows up as far more than this, and the median budget
        // in section 2 bounds it from above in any case.
        uint32_t statusUs = gRes[TMCLP_IDX_STATUS].medianUs;
        uint32_t debugUs  = gRes[TMCLP_IDX_DEBUG].medianUs;
        uint32_t excess   = (debugUs > statusUs) ? (debugUs - statusUs) : 0;
        uint32_t predicted = tmclpWireMicros(TMCLP_CMDS[TMCLP_IDX_DEBUG]) -
                             tmclpWireMicros(TMCLP_CMDS[TMCLP_IDX_STATUS]);
        printf("   Get debug values costs %lu us more than Get status; its %lu extra payload bytes "
               "predict %lu us\n",
               (unsigned long)excess,
               (unsigned long)(sizeof(getDebugValuesResponse) - sizeof(getStatusResponse)),
               (unsigned long)predicted);
        TEST_RESULT("the one large-reply command is dearer only by about what its extra bytes cost "
                    "on the wire, not by a blocking amount",
                    excess >= 500u && excess <= 40000u);
    }

    // =====================================================================
    // 4. THE SPREAD IS BOUNDED. An average hides a stall. A command that was
    //    usually fast and occasionally blocked for 100 ms would have a healthy
    //    median and would still ruin a real machine, so the tail is checked
    //    separately from the centre.
    //
    //    The ratio check uses the SECOND-worst batch rather than the worst,
    //    deliberately. In host mode these numbers include the operating
    //    system's scheduling, and over 200-odd batches a single outlier is
    //    expected; two in the same command's twelve is not. The absolute
    //    bounds below are what guard the true tail.
    //
    //    BOTH checks carry an absolute slack term as well as a ratio, and that
    //    is not padding. On the ESP32-S3 the cheapest command's whole batch of
    //    ten is about 13 ms, so a ratio-only bound of 4x leaves under 40 ms of
    //    headroom for the entire batch -- one USB CDC flush stall inside the
    //    library's unconditional per-call debug println would break it on
    //    healthy firmware. The slack (5 ms per call on the spread, 3 ms per call
    //    on the centre) absorbs that while still failing on the signature that
    //    matters: a command that blocks for tens of milliseconds REPEATEDLY.
    // =====================================================================
    printf("\n--- 4. the spread is bounded and nothing approaches the timeout ---\n");
    {
        bool spreadOk = true, centreOk = true;
        int worstSpreadIdx = 0; double worstSpread = 0.0;
        int worstCentreIdx = 0; double worstCentre = 0.0;
        for (int i = 0; i < TMCLP_NCMD; i++) {
            double spread = (double)gRes[i].secondWorstBatchUs / (double)tmclpNz(gRes[i].medianUs);
            if (spread > worstSpread) { worstSpread = spread; worstSpreadIdx = i; }
            if (gRes[i].secondWorstBatchUs > 4u * tmclpNz(gRes[i].medianUs) + 5000u) spreadOk = false;

            double centre = (double)gRes[i].meanUs / (double)tmclpNz(gRes[i].medianUs);
            if (centre > worstCentre) { worstCentre = centre; worstCentreIdx = i; }
            if (gRes[i].meanUs > (3u * tmclpNz(gRes[i].medianUs)) / 2u + 3000u) centreOk = false;
        }
        printf("   widest spread: %s at %.2fx its median batch\n",
               TMCLP_CMDS[worstSpreadIdx].name, worstSpread);
        printf("   worst mean/median ratio: %s at %.2f\n",
               TMCLP_CMDS[worstCentreIdx].name, worstCentre);
        TEST_RESULT("no command has a slow batch that stands far outside its own median", spreadOk);
        TEST_RESULT("for every command the mean tracks the median, so there is no heavy tail",
                    centreOk);

        printf("   worst single round trip anywhere in the module: %lu ms, against a %lu ms library timeout\n",
               (unsigned long)gModuleWorstCallMs, (unsigned long)TMCLP_LIB_TIMEOUT_MS);
        printf("   total commands sent so far: %lu\n", (unsigned long)gTotalCalls);
        TEST_RESULT("not one call anywhere in the module reached the library's one-second timeout",
                    gModuleWorstCallMs < TMCLP_LIB_TIMEOUT_MS);
        TEST_RESULT("the worst single round trip stays under half of that timeout",
                    gModuleWorstCallMs < TMCLP_WORST_CALL_MS);
    }

    // =====================================================================
    // 5. LATENCY DOES NOT DEGRADE. A leak that cost a little more time on every
    //    command -- a growing buffer, a list walked from the head, a counter
    //    that never wraps -- would pass every check above, because every check
    //    above compares commands measured at roughly the same moment. The only
    //    way to see it is to compare EARLY against LATE.
    // =====================================================================
    printf("\n--- 5. latency does not drift over the run ---\n");
    {
        bool halvesOk = true;
        int worstIdx = 0; double worstRatio = 0.0;
        for (int i = 0; i < TMCLP_NCMD; i++) {
            double ratio = (double)gRes[i].secondHalfUs / (double)tmclpNz(gRes[i].firstHalfUs);
            if (ratio > worstRatio) { worstRatio = ratio; worstIdx = i; }
            if (gRes[i].secondHalfUs > 2u * tmclpNz(gRes[i].firstHalfUs) + 5000u) halvesOk = false;
        }
        printf("   worst second-half/first-half ratio: %s at %.2f (%lu us -> %lu us)\n",
               TMCLP_CMDS[worstIdx].name, worstRatio,
               (unsigned long)gRes[worstIdx].firstHalfUs,
               (unsigned long)gRes[worstIdx].secondHalfUs);
        TEST_RESULT("for every command the last sixty calls cost about what the first sixty did",
                    halvesOk);

        // Re-profile after everything else has run, with no reset in between:
        // roughly 2200 transactions now separate this measurement from the one
        // in the table.
        LatencyResult again;
        tmclpProfile(m, TMCLP_IDX_STATUS, &again);
        printf("   Get status: %lu us/call at the start of the table, %lu us/call after %lu commands\n",
               (unsigned long)gRes[TMCLP_IDX_STATUS].meanUs, (unsigned long)again.meanUs,
               (unsigned long)gTotalCalls);
        TEST_RESULT("Get status is no slower after two thousand commands than it was at the start",
                    again.errors == 0 &&
                    again.meanUs <= (8u * tmclpNz(gRes[TMCLP_IDX_STATUS].meanUs)) / 5u + 2000u);

        LatencyResult posAgain;
        tmclpProfile(m, TMCLP_IDX_POSITION, &posAgain);
        printf("   Get position: %lu us/call at the start of the table, %lu us/call now\n",
               (unsigned long)gRes[TMCLP_IDX_POSITION].meanUs, (unsigned long)posAgain.meanUs);
        TEST_RESULT("Get position is likewise no slower at the end of the run than at the start",
                    posAgain.errors == 0 &&
                    posAgain.meanUs <= (8u * tmclpNz(gRes[TMCLP_IDX_POSITION].meanUs)) / 5u + 2000u);
    }

    // =====================================================================
    // 6. CLEAN EXIT. Roughly two and a half thousand commands have now been
    //    sent without a single reset in the middle, which is itself worth
    //    asserting: none of them latched a fault and the machine is still
    //    answering.
    // =====================================================================
    printf("\n--- 6. the machine survived the whole profile ---\n");
    {
        uint8_t fatal = tfhFatal(m);
        uint8_t queued = m->getNQueuedItems();
        bool readOk = (m->getError() == 0);
        printf("   after %lu commands: fatal=%u queue=%u\n",
               (unsigned long)gTotalCalls, (unsigned)fatal, (unsigned)queued);
        TEST_RESULT("none of the thousands of commands sent here latched a fault or queued anything",
                    fatal == 0 && readOk && queued == 0);

        tfhReset(m);
        TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
    }
}
