#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_concurrent_command_pressure.cpp
//
// CORRECTNESS UNDER SUSTAINED COMMAND PRESSURE, NOT MERELY SURVIVAL.
//
// THE PROPERTY:
//
//     Throughput pressure on the RS485 link cannot corrupt motion or framing.
//     A move executes to the same endpoint whether the bus is silent or
//     saturated; every response is well formed and is the answer to the request
//     that asked for it; the receiver's error counters stay at zero; and the
//     motion queue only ever drains.
//
// WHY IT SHOULD HOLD, from the firmware. Motion and communication live on
// opposite sides of an interrupt boundary and are explicitly prioritised so
// that traffic can never delay motion:
//
//   * The planner runs ONLY in the 31.25 kHz control interrupt.
//     TIM16_IRQHandler (firmware/Src/motor_control.c:3075) calls
//     motor_movement_calculations() (:2611), which calls
//     handle_queued_movements() (:2125). That is the only place a queue slot is
//     ever consumed (n_items_in_queue-- at :2136 and :2144).
//   * Commands are handled in the FOREGROUND. main()'s while(1) loop
//     (firmware/Src/main.c:1444) polls rs485_has_a_packet() and calls
//     process_packet() (:1451-1456). Nothing in that path touches the planner's
//     time base.
//   * The priorities make the ordering explicit rather than accidental:
//     NVIC_SetPriority(TIM16_IRQn, 1) (motor_control.c:380) against
//     NVIC_SetPriority(USART1_IRQn, 2) (common_source_files/RS485.c:165, whose
//     comment says outright that it "needs to be lower than the motor control
//     interrupt"). Lower number wins on a Cortex-M, so the control ISR preempts
//     the receiver, never the other way round.
//
// So the claim is falsifiable in a specific way: if command handling ever
// blocked, masked or starved the control ISR, a flooded move would land SHORT of
// an unflooded one. That is the measurement this module makes, at four different
// flood rates, against a reference measured three times.
//
// HOW THIS IS DISTINCT FROM WHAT ALREADY EXISTS:
//   * tm_rapid_fire is a SURVIVAL test with NO motion at all: back-to-back calls
//     must not raise ERROR_COMMAND_OVERFLOW and the MOSFET status bit must track
//     the last toggle. It never runs a move, so it cannot say anything about
//     motion being disturbed.
//   * tm_comm_during_motion polls at ONE fixed 10 ms cadence through a velocity
//     profile and asserts survival properties (zero comm errors, pings echo,
//     position rises). It has no undisturbed reference to compare against, so
//     "the move ended where it should have" is not among its claims.
//   * tm_readonly_purity asks a per-COMMAND question -- is each individual
//     getter pure? -- by flooding with one command type at a time at one rate.
//     This module asks a per-RATE question about a MIX: does the LOAD LEVEL
//     matter, and does anything come back malformed or mismatched to its
//     request when it does not?
//   Concretely, nothing else in the suite sweeps the flood rate, checks the
//   communication error counters (command 47) around a move, watches the queue
//   depth for monotonicity, or hunts for a response arriving one behind its
//   request.
//
// TWO CHECKS WORTH SINGLING OUT, because generic "no error" testing misses them:
//
//   1. OFF-BY-ONE DESYNCHRONISATION. The nastiest failure a saturated
//      request/response link can have is not a dropped frame -- that shows up as
//      an error -- but a response getting matched to the WRONG request, after
//      which every answer is stale yet superficially valid. Ping (command 31)
//      echoes a 10-byte payload, so a payload that changes on every call makes
//      that detectable: a mismatch is checked against the PREVIOUS payload, and
//      a hit is reported as a desynchronisation rather than as generic
//      corruption.
//   2. VARIABLE-LENGTH RESPONSES. Get product description (command 24) returns a
//      string whose length is not known in advance. Variable-length responses
//      are exactly where the Arduino library had real framing bugs
//      (BUG-25/26/27), and they are the first thing that should break if
//      pressure disturbed the receive path. Its bytes are captured once as a
//      baseline and required to be identical on every one of the hundreds of
//      reads under load.
//
// SAFETY. The MOSFETs are NEVER enabled and no closed-loop command is sent, so
// nothing on the shared 35-motor rack turns at any point. Every distance here is
// read from the COMMANDED position (command 34 / command 37), which the planner
// advances whether or not the output stage is energised -- that is precisely
// what makes a two-rotation move measurable to the count with a stationary
// shaft. tfhFreshOpen() raises the deviation window first (it defaults to two
// rotations, motor_control.c:97, and with the output stage off the deviation IS
// the commanded displacement, so anything near two turns would otherwise trip
// fatal error 45). Every command is unique-ID addressed: nothing is broadcast,
// no alias is touched, no raw frames are injected, and no test mode is entered.
// ---------------------------------------------------------------------------

typedef getCommunicationStatisticsResponse Stats;

// Firmware constants this module depends on, each cited so a future change to
// the firmware shows up here as a compile-time-obvious mismatch rather than a
// mysterious failure.
static const uint8_t  QUEUE_CAPACITY      = 32;    // MOVEMENT_QUEUE_SIZE, motor_control.h:31
static const uint16_t STATUS_DEFINED_BITS = 0x007F; // bits 0..6, device_status.h:16-22

// The move under test: two rotations over four seconds (0.5 rotations/second,
// far below the 15 rot/s ceiling tfhFreshOpen sets). Long enough that a flood
// spans the whole of it, small enough to stay well inside the raised deviation
// window.
static const int32_t  MOVE_COUNTS = (int32_t)(TFH_CPR * 2);
static const uint32_t MOVE_TICKS  = (uint32_t)(TFH_TPS * 4);

// The commanded position is exact with the output stage off, so the tolerance
// only has to absorb the planner's integer rounding at the endpoint.
static const int64_t POSITION_SLACK  = 16;
static const int64_t REFERENCE_SLACK = 4;   // "a couple of counts", as specified

// Inter-command gaps defining the four flood rates, fastest first. Zero means
// "as fast as the library will go", which is naturally paced by the blocking
// request/response turnaround.
static const int FLOOD_GAPS_MS[] = { 0, 4, 12, 30 };
enum { N_RATES = 4 };

// How long the flood runs on PAST the end of the move. This is not padding: the
// only way the in-flood sampling can ever see the queue at zero is for a
// "get n queued items" turn of the nine-command rotation to land in this tail.
// One full rotation at the slowest gap is nine round trips plus 9 x 30 ms, and a
// round trip is a few milliseconds on the ESP32-S3 but can be tens of
// milliseconds through a USB-serial adapter in host mode (an FTDI latency timer
// alone defaults to 16 ms per direction). 500 ms left that a coin flip on the
// host; 1000 ms fits at least two rotations even at 25 ms per command. The
// post-loop read below removes the remaining dependence on it entirely.
static const uint32_t FLOOD_TAIL_MS = 1000;

// Allowance for the ELECTRICAL counters over the whole four-rate sweep. See the
// assertion in section 4 for why this is not zero.
static const uint32_t ELECTRICAL_GLITCH_BUDGET = 4;

// A flood must be a flood. Even the slowest rate issues far more than this over
// four seconds; the bar exists so that a run which silently issued almost
// nothing is reported as such instead of trivially passing everything.
static const int MIN_COMMANDS_PER_FLOOD = 40;

enum { N_FLOOD_READERS = 9 };
enum { DESC_BUF = 128 };

// Identity captured once on a quiet bus; every read under load is required to
// reproduce it exactly.
struct Baseline {
    uint64_t uniqueId;
    char     desc[DESC_BUF];
    uint16_t descLen;
    bool     ok;
};

// Well-formedness violations, accumulated across ALL four floods. Each counter
// names one specific way a response could be wrong beyond simply failing, so a
// failure points straight at the layer responsible.
struct Wellformed {
    int pingMismatches;         // echoed payload differed from the one sent
    int pingDesyncs;            // ...and matched the PREVIOUS payload: off by one
    int identityMismatches;     // get product info reported a different unique ID
    int descriptionMismatches;  // the variable-length description changed
    int statusUndefinedBits;    // a status bit above bit 6 was set
    int statusUnexpectedFlags;  // a defined bit was set that should not be
    int depthOutOfRange;        // queue depth byte larger than the queue
    int positionOutOfBand;      // commanded position outside what the move spans
    int positionWentBackwards;  // commanded position moved the wrong way
    int comprehensiveDisagrees; // command 37 disagreed with command 34
    int voltageOutOfBand;
    int temperatureOutOfBand;
};

struct FloodResult {
    int      gapMs;
    bool     queued;
    int      commands;
    int      commErrors;
    int      malformed;
    uint32_t elapsedMs;
    bool     haveDepth;
    uint8_t  firstDepth;
    uint8_t  maxDepth;
    bool     depthMonotonic;
    bool     depthReachedZero;
    bool     completed;
    uint8_t  fatal;
    int64_t  moved;
    bool     countersOk;
    Stats    counters;
};

// The three counters decided by packet CONTENT (RS485.c:326, :257, :402). These
// are exactly reproducible: on clean traffic they must be zero, full stop.
static bool softwareCountersZero(const Stats& s) {
    return s.crc32ErrorCount == 0 && s.packetDecodeErrorCount == 0 &&
           s.firstBitErrorCount == 0;
}

// The three ELECTRICAL counters (framing, overrun, noise: RS485.c:362, :367,
// :372). Kept separate on purpose -- they describe the cable, not the protocol.
static uint32_t hardwareTotal(const Stats& s) {
    return s.framingErrorCount + s.overrunErrorCount + s.noiseErrorCount;
}

static void printStats(const char* label, const Stats& s) {
    printf("   %-18s crc32=%u decode=%u firstBit=%u | framing=%u overrun=%u noise=%u\n",
           label,
           (unsigned)s.crc32ErrorCount, (unsigned)s.packetDecodeErrorCount,
           (unsigned)s.firstBitErrorCount, (unsigned)s.framingErrorCount,
           (unsigned)s.overrunErrorCount, (unsigned)s.noiseErrorCount);
}

// Command 47, "Get communication statistics" (motor_commands.json; handler
// firmware/Src/main.c:1128). resetFlag != 0 snapshots the counters and THEN
// zeroes them, so a resetting read loses nothing.
static bool readStats(Servomotor* m, uint8_t resetFlag, Stats* out) {
    Stats s = m->getCommunicationStatistics(resetFlag);
    if (m->getError() != 0) {
        printf("   getCommunicationStatistics(%u) comms error %d\n",
               (unsigned)resetFlag, m->getError());
        return false;
    }
    *out = s;
    return true;
}

// A known machine with the ceilings open and every unit this module reads
// pinned. enableCRC32() is library-side only (Servomotor.cpp:60) and realigns
// the host's framing with the firmware, which always boots with CRC32 on --
// tfhReset restores the motion units but says nothing about framing.
static void prepare(Servomotor* m) {
    m->enableCRC32();
    tfhFreshOpen(m);
    m->enableCRC32();
    m->setVoltageUnit(VoltageUnit::VOLTS);
    m->setTemperatureUnit(TemperatureUnit::CELSIUS);
}

// ---------------------------------------------------------------------------
// Queue one precisely-known move, then hammer the bus with a rotating mix of
// nine reads for the whole of its duration, checking every response as it
// arrives. Returns with the queue drained and the outcome recorded.
//
// Command 47 is deliberately NOT in the mix: it is the instrument measuring the
// flood, and it is the one "read" in the API that mutates what it reads, so
// including it would contaminate the measurement.
// ---------------------------------------------------------------------------
static void runFlood(Servomotor* m, int gapMs, const Baseline* base,
                     FloodResult* r, Wellformed* w) {
    memset(r, 0, sizeof(*r));
    r->gapMs = gapMs;
    r->depthMonotonic = true;

    prepare(m);

    // Zero the counters so that whatever they read afterwards was produced by
    // this flood and nothing else.
    Stats cleared;
    memset(&cleared, 0, sizeof(cleared));
    bool zeroed = readStats(m, 1, &cleared);

    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) { r->countersOk = false; return; }

    m->trapezoidMoveRaw(MOVE_COUNTS, MOVE_TICKS);   // command 2, Trapezoid move
    r->queued = (m->getError() == 0);
    if (!r->queued) return;

    const uint32_t moveMs = (uint32_t)((uint64_t)MOVE_TICKS * 1000ULL / (uint64_t)TFH_TPS);
    const uint32_t t0     = millis();
    const uint32_t deadline = t0 + moveMs + FLOOD_TAIL_MS;  // outlast the move deliberately

    int64_t lastPos   = before;
    uint8_t lastDepth = 0;
    uint8_t prevPayload[10];
    bool    havePrev  = false;
    int     seq       = 0;

    while (millis() < deadline) {
        switch (seq % N_FLOOD_READERS) {

        case 0: {   // command 16, Get status
            getStatusResponse st = m->getStatus();
            if (m->getError() != 0) { r->commErrors++; break; }
            if ((st.statusFlags & (uint16_t)~STATUS_DEFINED_BITS) != 0) {
                w->statusUndefinedBits++; r->malformed++;
            }
            // MOSFETs off, not homing, not calibrating, not closed loop, and
            // motor_busy is set only for long actions -- so every defined bit
            // must be clear (get_motor_status_flags, motor_control.c:3442-3466).
            if (st.statusFlags != 0) { w->statusUnexpectedFlags++; r->malformed++; }
            if (st.fatalErrorCode != 0 && r->fatal == 0) r->fatal = st.fatalErrorCode;
            break;
        }

        case 1: {   // command 34, Get position (the COMMANDED position)
            int64_t p = m->getPositionRaw();
            if (m->getError() != 0) { r->commErrors++; break; }
            if (p < before - POSITION_SLACK ||
                p > before + (int64_t)MOVE_COUNTS + POSITION_SLACK) {
                w->positionOutOfBand++; r->malformed++;
            }
            // A positive trapezoid never reverses, so this is exact, not a
            // tolerance: any decrease is a wrong answer.
            if (p < lastPos) { w->positionWentBackwards++; r->malformed++; }
            else lastPos = p;
            break;
        }

        case 2: {   // command 11, Get n queued items
            uint8_t q = m->getNQueuedItems();
            if (m->getError() != 0) { r->commErrors++; break; }
            if (q > QUEUE_CAPACITY) { w->depthOutOfRange++; r->malformed++; }
            if (!r->haveDepth) {
                r->firstDepth = q; r->maxDepth = q; r->haveDepth = true;
            }
            else {
                if (q > lastDepth) r->depthMonotonic = false;
                if (q > r->maxDepth) r->maxDepth = q;
            }
            if (q == 0) r->depthReachedZero = true;
            lastDepth = q;
            break;
        }

        case 3: {   // command 31, Ping -- the desynchronisation detector
            uint8_t payload[10];
            for (int k = 0; k < 10; k++)
                payload[k] = (uint8_t)((seq * 13 + k * 7 + 1) & 0xFF);
            pingResponse pr = m->ping(payload);
            if (m->getError() != 0) { r->commErrors++; }
            else if (memcmp(pr.responsePayload, payload, 10) != 0) {
                w->pingMismatches++; r->malformed++;
                if (havePrev && memcmp(pr.responsePayload, prevPayload, 10) == 0)
                    w->pingDesyncs++;    // answered the request BEFORE this one
            }
            memcpy(prevPayload, payload, 10);
            havePrev = true;
            break;
        }

        case 4: {   // command 38, Get supply voltage
            float v = m->getSupplyVoltage();
            if (m->getError() != 0) { r->commErrors++; break; }
            if (!(v > 5.0f && v < 60.0f)) { w->voltageOutOfBand++; r->malformed++; }
            break;
        }

        case 5: {   // command 42, Get temperature
            float t = m->getTemperature();
            if (m->getError() != 0) { r->commErrors++; break; }
            if (!(t > -20.0f && t < 90.0f)) { w->temperatureOutOfBand++; r->malformed++; }
            break;
        }

        case 6: {   // command 37, Get comprehensive position
            getComprehensivePositionResponse cp = m->getComprehensivePositionRaw();
            if (m->getError() != 0) { r->commErrors++; break; }
            // The same quantity command 34 reports, in a differently shaped
            // response: a cross-check that the right bytes reached the right
            // field.
            if (cp.commandedPosition < before - POSITION_SLACK ||
                cp.commandedPosition > before + (int64_t)MOVE_COUNTS + POSITION_SLACK) {
                w->comprehensiveDisagrees++; r->malformed++;
            }
            break;
        }

        case 7: {   // command 24, Get product description -- VARIABLE LENGTH
            char buf[DESC_BUF];
            uint16_t n = 0;
            memset(buf, 0, sizeof(buf));
            m->getProductDescription(buf, sizeof(buf), &n);
            if (m->getError() != 0) { r->commErrors++; break; }
            if (!base->ok || n != base->descLen ||
                memcmp(buf, base->desc, base->descLen) != 0) {
                w->descriptionMismatches++; r->malformed++;
            }
            break;
        }

        case 8: {   // command 22, Get product info
            getProductInfoResponse pi = m->getProductInfo();
            if (m->getError() != 0) { r->commErrors++; break; }
            if (!base->ok || pi.uniqueId != base->uniqueId) {
                w->identityMismatches++; r->malformed++;
            }
            break;
        }
        }

        r->commands++;
        seq++;
        if (gapMs > 0) delay((unsigned long)gapMs);
    }

    r->elapsedMs = millis() - t0;

    // One depth read taken OUTSIDE the timed loop, used for nothing except
    // answering "did the queue actually drain?". Whether a case-2 turn of the
    // rotation happens to fall after the move ended is a property of the host's
    // serial latency, not of the firmware, so making the drain claim depend on
    // it would be a coin flip dressed up as a measurement. The MONOTONICITY half
    // of the claim is still gathered entirely under load, which is the part that
    // is actually about pressure.
    uint8_t finalDepth = m->getNQueuedItems();
    if (m->getError() == 0) {
        if (r->haveDepth && finalDepth > lastDepth) r->depthMonotonic = false;
        if (finalDepth == 0) r->depthReachedZero = true;
    }

    // NEVER a fixed wait: poll the queue to empty with a bounded deadline.
    r->completed = tfhDrain(m, moveMs + 15000);
    uint8_t latched = tfhFatal(m);
    if (latched != 0 && latched != 0xFF) r->fatal = latched;

    if (r->completed && r->fatal == 0) {
        r->moved = m->getPositionRaw() - before;
        // Once a fatal error is latched, get_status is the only command that
        // answers, so only read the counters on a healthy machine.
        r->countersOk = zeroed && readStats(m, 0, &r->counters);
    }
}

void tm_concurrent_command_pressure(void) {
    Serial.println("tm_concurrent_command_pressure: BEGIN\n");
    Servomotor* m = tfGetMotor();

    Wellformed w;
    memset(&w, 0, sizeof(w));

    Baseline base;
    memset(&base, 0, sizeof(base));

    // =====================================================================
    // 1. THE UNDISTURBED REFERENCE. Everything downstream is a comparison
    //    against this number, so it is measured three times and required to
    //    be repeatable before it is trusted. The identity baselines are taken
    //    here too, on a quiet bus, so that "unchanged under load" means
    //    unchanged from a value that was read when nothing else was going on.
    // =====================================================================
    printf("\n--- 1. the undisturbed reference, measured on a quiet bus ---\n");
    int64_t reference = 0;
    {
        prepare(m);

        // Identity, captured once. getProductDescription is the variable-length
        // response; its exact bytes become the yardstick for the flood.
        getProductInfoResponse pi = m->getProductInfo();
        bool idOk = (m->getError() == 0);
        base.uniqueId = idOk ? pi.uniqueId : 0;
        uint16_t n = 0;
        m->getProductDescription(base.desc, DESC_BUF, &n);
        bool descOk = (m->getError() == 0) && (n > 0) && (n < DESC_BUF);
        base.descLen = descOk ? n : 0;
        base.ok = idOk && descOk;
        printf("   uniqueId=0x%016llX  description=%u bytes: \"%s\"\n",
               (unsigned long long)base.uniqueId, (unsigned)base.descLen,
               base.ok ? base.desc : "");
        TEST_RESULT("the identity and the variable-length description read cleanly on a quiet bus",
                    base.ok);

        Stats fresh;
        memset(&fresh, 0, sizeof(fresh));
        bool statsOk = readStats(m, 1, &fresh);
        if (statsOk) printStats("after reset:", fresh);
        TEST_RESULT("a freshly reset motor reports no communication errors at all",
                    statsOk && softwareCountersZero(fresh) && hardwareTotal(fresh) == 0);

        int64_t lo = 0, hi = 0;
        bool refOk = true;
        for (int r = 0; r < 3; r++) {
            prepare(m);
            bool ok = false;
            int64_t moved = tfhMove(m, MOVE_COUNTS, MOVE_TICKS, &ok);
            if (!ok) { refOk = false; break; }
            if (r == 0) { lo = hi = moved; }
            else { if (moved < lo) lo = moved; if (moved > hi) hi = moved; }
            reference = moved;
        }
        printf("   reference: %lld counts of a commanded %ld (spread over 3 runs: %lld)\n",
               (long long)reference, (long)MOVE_COUNTS, (long long)(hi - lo));
        TEST_RESULT("the reference move completes cleanly three times in a row", refOk);
        TEST_RESULT("the undisturbed reference is repeatable to within two counts",
                    refOk && (hi - lo) <= 2);
        TEST_RESULT("the reference move delivers the displacement it was commanded",
                    refOk && llabs((long long)(reference - MOVE_COUNTS)) <= 8);
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE SAME MOVE UNDER FOUR DIFFERENT FLOOD RATES. One move, queued and
    //    then buried under a rotating mix of nine reads for its whole
    //    duration. Sweeping the rate is the point: a single rate can only say
    //    "this much traffic was survivable", whereas a sweep says the OUTCOME
    //    does not depend on how hard the bus is being driven. Every response
    //    is inspected as it arrives; the per-rate assertions below cover
    //    throughput, framing, the endpoint and the queue, and the detailed
    //    well-formedness tallies are asserted together in section 3.
    // =====================================================================
    printf("\n--- 2. the same move under four flood rates ---\n");
    FloodResult results[N_RATES];
    {
        for (int i = 0; i < N_RATES; i++) {
            runFlood(m, FLOOD_GAPS_MS[i], &base, &results[i], &w);
            const FloodResult& r = results[i];
            double rate = (r.elapsedMs > 0)
                        ? (1000.0 * (double)r.commands) / (double)r.elapsedMs : 0.0;

            printf("   gap %2d ms: %4d commands in %5lu ms (%.1f cmd/s), commErr=%d "
                   "malformed=%d\n",
                   r.gapMs, r.commands, (unsigned long)r.elapsedMs, rate,
                   r.commErrors, r.malformed);
            printf("             queue depth %u -> 0 (max seen %u, monotonic=%d, "
                   "reached zero=%d)\n",
                   (unsigned)r.firstDepth, (unsigned)r.maxDepth,
                   (int)r.depthMonotonic, (int)r.depthReachedZero);
            printf("             moved %lld vs reference %lld (delta %lld), fatal=%u\n",
                   (long long)r.moved, (long long)reference,
                   (long long)(r.moved - reference), (unsigned)r.fatal);
            if (r.countersOk) printStats("counters:", r.counters);

            std::string at = std::string("at a ") + std::to_string(r.gapMs) +
                             " ms inter-command gap, ";

            TEST_RESULT(at + "the bus really was kept busy for the whole move",
                        r.queued && r.commands >= MIN_COMMANDS_PER_FLOOD);
            TEST_RESULT(at + "every single command was answered",
                        r.queued && r.commErrors == 0);
            // The headline claim: pressure did not change the outcome.
            TEST_RESULT(at + "the move lands exactly where the undisturbed reference landed",
                        r.completed && r.fatal == 0 &&
                        llabs((long long)(r.moved - reference)) <= REFERENCE_SLACK);
            TEST_RESULT(at + "the queue depth only ever falls, and reaches zero",
                        r.haveDepth && r.depthMonotonic && r.depthReachedZero &&
                        r.maxDepth <= QUEUE_CAPACITY);
            // Clean traffic must leave the error counters alone however much of
            // it there is: these are ERROR counters, not traffic counters.
            TEST_RESULT(at + "the receiver logged no CRC32, decode or first-bit errors",
                        r.countersOk && softwareCountersZero(r.counters));

            tfhReset(m);
        }
    }

    // =====================================================================
    // 3. EVERY RESPONSE WAS WELL FORMED, AND WAS THE ANSWER TO ITS OWN
    //    REQUEST. Aggregated over all four floods -- well over a thousand
    //    responses -- because these failures are rare-event failures: one bad
    //    frame in a thousand is a real defect and a per-rate assertion would
    //    only dilute it. "No error" is a much weaker claim than any of these:
    //    a stale-but-valid response passes an error check and fails here.
    // =====================================================================
    printf("\n--- 3. every response under load was well formed ---\n");
    {
        int total = 0, errs = 0, malformed = 0;
        for (int i = 0; i < N_RATES; i++) {
            total += results[i].commands;
            errs += results[i].commErrors;
            malformed += results[i].malformed;
        }
        printf("   %d responses inspected across the four floods: %d errors, %d malformed\n",
               total, errs, malformed);
        printf("   ping: %d mismatched (%d of them one-behind), identity: %d, "
               "description: %d\n",
               w.pingMismatches, w.pingDesyncs, w.identityMismatches,
               w.descriptionMismatches);
        printf("   status: %d undefined bits, %d unexpected flags; depth: %d out of range\n",
               w.statusUndefinedBits, w.statusUnexpectedFlags, w.depthOutOfRange);
        printf("   position: %d out of band, %d backwards, %d disagreements; "
               "voltage %d, temperature %d\n",
               w.positionOutOfBand, w.positionWentBackwards, w.comprehensiveDisagrees,
               w.voltageOutOfBand, w.temperatureOutOfBand);

        TEST_RESULT("enough responses were inspected for the tallies to mean something",
                    total >= N_RATES * MIN_COMMANDS_PER_FLOOD);
        TEST_RESULT("every ping echoed back the exact payload it was sent",
                    w.pingMismatches == 0);
        TEST_RESULT("no response ever arrived one behind the request that asked for it",
                    w.pingDesyncs == 0);
        TEST_RESULT("the unique ID reported under load never differed from the quiet-bus value",
                    w.identityMismatches == 0);
        TEST_RESULT("the variable-length product description was byte-identical every time",
                    w.descriptionMismatches == 0);
        TEST_RESULT("the status word never carried an undefined bit",
                    w.statusUndefinedBits == 0);
        TEST_RESULT("the status word never reported a state the machine was not in",
                    w.statusUnexpectedFlags == 0);
        // depthOutOfRange is reported above but not asserted separately: the
        // per-rate queue assertion already requires maxDepth <= QUEUE_CAPACITY,
        // which is the same claim without a second name for it.
        TEST_RESULT("the commanded position stayed inside the move and never went backwards",
                    w.positionOutOfBand == 0 && w.positionWentBackwards == 0);
        TEST_RESULT("get comprehensive position always agreed with get position",
                    w.comprehensiveDisagrees == 0);
        TEST_RESULT("supply voltage and temperature always read a plausible value",
                    w.voltageOutOfBand == 0 && w.temperatureOutOfBand == 0);
    }

    // =====================================================================
    // 4. THE RATES REALLY WERE DIFFERENT, AND THE MACHINE IS UNHARMED. The
    //    sweep is only evidence if the four runs actually differed in load;
    //    otherwise section 2 is one experiment repeated four times. The
    //    electrical counters are checked here rather than per rate because
    //    they describe the cable rather than the protocol, and a single
    //    statement about the whole module is the honest way to put it.
    // =====================================================================
    printf("\n--- 4. the sweep was a real sweep, and nothing was damaged ---\n");
    {
        bool strictlyDecreasing = true;
        for (int i = 1; i < N_RATES; i++)
            if (results[i].commands >= results[i - 1].commands) strictlyDecreasing = false;
        printf("   commands per flood: %d, %d, %d, %d (gaps %d, %d, %d, %d ms)\n",
               results[0].commands, results[1].commands,
               results[2].commands, results[3].commands,
               FLOOD_GAPS_MS[0], FLOOD_GAPS_MS[1], FLOOD_GAPS_MS[2], FLOOD_GAPS_MS[3]);
        TEST_RESULT("a larger inter-command gap really did mean fewer commands, every time",
                    strictlyDecreasing);

        uint32_t hw = 0;
        bool allCounted = true;
        int faults = 0;
        for (int i = 0; i < N_RATES; i++) {
            if (!results[i].countersOk) allCounted = false;
            else hw += hardwareTotal(results[i].counters);
            if (results[i].fatal != 0) faults++;
        }
        printf("   electrical counters over the whole sweep: %u (budget %u); latched faults: %d\n",
               (unsigned)hw, (unsigned)ELECTRICAL_GLITCH_BUDGET, faults);
        // WEAKENED from "hw == 0", deliberately. The framing, overrun and noise
        // counters are ELECTRICAL: they describe the cable, not the protocol,
        // and tm_communication_statistics says so in as many words -- it asserts
        // them "only as deltas, never as absolute values, because a real cable
        // is entitled to the occasional glitch". The strongest figure any module
        // has actually established on hardware is zero across THIRTY clean
        // commands (tm_comms, tm_communication_statistics section 2). This sweep
        // drives thousands of frames and eight MCU reboots past that, so an
        // exact zero would be asserting something no run has ever measured.
        // A handful still catches anything systematic: a genuine framing fault
        // scales with traffic and would land in the hundreds, not at three. The
        // exact figure is printed above either way, so a creeping cable problem
        // is still visible to a human reading the log.
        //
        // The three SOFTWARE counters are a different matter and are still
        // required to be exactly zero, per rate, in section 2: those are decided
        // by packet content and are exactly reproducible.
        TEST_RESULT("the receiver's framing, overrun and noise counters stayed at the floor",
                    allCounted && hw <= ELECTRICAL_GLITCH_BUDGET);
        TEST_RESULT("no fatal error was latched at any flood rate", faults == 0);

        prepare(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   post-sweep move: %lld of %lld counts\n",
               (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after the whole sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) <= 8);
        tfhReset(m);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
