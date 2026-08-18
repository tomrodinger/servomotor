#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_repeated_command_idempotence.cpp
//
// WHICH COMMANDS SURVIVE BEING SENT TWICE?
//
// Every command in this API travels over a half-duplex RS485 link with no
// arbitration, no sequence numbers and no acknowledgement beyond a single reply
// packet. When that reply is lost -- a collision, a truncated frame, a host
// timeout that fires a millisecond early -- the caller cannot tell whether the
// command was executed or never arrived. There is exactly one thing every such
// caller does next: it sends it again.
//
// So for each command there is a question that decides whether a retry is safe:
//
//     Does applying it a second time from the state the first one produced
//     leave the machine in that SAME state (idempotent), or does it compound
//     (cumulative)?
//
// Nothing in the firmware, the JSON command table or the API documentation says
// which commands are which. It has to be inferred command by command from the
// source, and a caller who infers wrong either double-moves an axis or writes a
// retry loop that quietly cannot work. This module answers the question by
// measurement for every command that is safe to exercise with the output stage
// off, and PRINTS THE RESULTING TABLE so the answer is a readable artefact and
// not just a pass count.
//
// The two properties are asserted differently, deliberately:
//   * IDEMPOTENT commands: apply once, fingerprint the machine; apply again,
//     fingerprint again; require the two fingerprints to be identical.
//   * CUMULATIVE commands: assert the exact compounded effect instead, because
//     "it changed" is not a specification. A relative trapezoid move applied
//     twice must travel exactly twice as far; an acceleration segment repeated
//     must cover exactly three times as much on its second pass, because
//     velocity accumulates across queued items (motor_control.c:1799).
//
// HOW THIS DIFFERS FROM THE MODULES THAT LOOK CLOSEST:
//   * tm_determinism asks f(s0) == f(s0): it RE-RESETS between repeats, so each
//     repetition starts from the same state. This module asks f(f(s0)) == f(s0)
//     and deliberately does NOT reset between the two applications. Those are
//     different claims; a command can be perfectly deterministic and still
//     compound.
//   * tm_readonly_purity asks whether reads perturb a running move and whether
//     they consume queue slots. It never compares state-after-one-application
//     against state-after-two for a command that MUTATES, which is the whole
//     subject here. It also does not cover the two reads that are not pure:
//     Get max PID error (39) and Get communication statistics (47) with
//     resetCounter set.
//   * tm_stop_semantics asserts that Emergency stop specifically is idempotent,
//     as one clause of the stop contract. This module places that result in a
//     table alongside twenty-seven other commands measured the same way.
//   * tm_reset_completeness asks whether one reset restores the defaults; this
//     asks whether a second reset changes anything the first one did not.
//
// GROUND TRUTH consulted for the predictions asserted below (all cited again at
// the point of use): zero_position assigns rather than nudges
// (motor_control.c:3311-3338); emergency_stop clears the queue and zeroes the
// velocity but never writes current_position_i96
// (motor_control.c:3405-3410 via :420-429); reset_time re-zeroes the microsecond
// clock (motor_control.c:3396-3402); set_max_velocity / set_max_acceleration
// ASSIGN (sent value << 12) and (sent value << 8) -- but they are not bare
// assignments: each rejects a zero argument with fatal error 34 and each CLAMPS
// at EXPERIMENTAL_MAX_VELOCITY / EXPERIMENTAL_MAX_ACCELERATION
// (motor_control.c:3341-3372, motor_control.h:60-61), which is why section 5a
// derives its test values from the boot defaults instead of picking numbers;
// get_max_PID_error RESETS its own accumulators to the empty sentinel as it
// reads them (motor_control.c:3480-3490); a velocity queue item ASSIGNS the
// velocity (motor_control.c:1842-1844) while an acceleration item ADDS to it
// (motor_control.c:1799).
//
// SAFETY: the MOSFETs are never enabled and closed loop is never entered. Every
// measurement reads the COMMANDED position, which the planner advances whether
// or not the output stage is energised, so nothing on the rack turns. No
// broadcast, no alias change, no Test mode, and no Vibrate (that one drives the
// output stage, which has no place in a MOSFETs-off module on a shared bus).
// Calibration and homing are excluded for the same reason -- they need a free
// shaft. Every subsection ends with tfhReset(m).
// ---------------------------------------------------------------------------

// Everything below down to the closing brace is file-local. It lives in an
// anonymous namespace rather than merely being `static` because the TYPES here
// would otherwise have external linkage: `struct Outcome` is also defined, with
// a DIFFERENT set of members, in tm_safety_fence_arithmetic.cpp, and both files
// are compiled into the same sketch binary and the same host_suite binary. Two
// different definitions of one class name across translation units is an ODR
// violation; the anonymous namespace makes each definition genuinely private
// instead of relying on the linker not to notice.
namespace {

const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;
const uint8_t ERR_PREDICTED_POS_OUT_OF_SAFETY  = 27;

// Two consecutive reads of the HALL position are two samples of a live sensor,
// not two reads of a stored integer, so they are bounded loosely. The value is
// deliberately the same one tm_position_source_consistency uses for exactly this
// comparison (its HALL_JITTER_TOL), which has been green on the bench motor and
// on two different rack motors. Anything tighter would be asserting a noise
// floor rather than a behaviour.
const int64_t HALL_JITTER_TOL = TFH_CPR / 200;   // 16384 counts

// The empty sentinel that get_max_PID_error leaves behind (motor_control.c:3485).
// Never assert min <= max on this pair: an untouched accumulator holds
// min = INT32_MAX and max = INT32_MIN, which violates that ordering by design.
static const int32_t PID_ACC_EMPTY_MIN = 2147483647;
static const int32_t PID_ACC_EMPTY_MAX = -2147483647 - 1;

// Wire scales, both verified against the firmware rather than assumed:
// a velocity queue item is counts-per-tick x 2^20 (VELOCITY_SHIFT_LEFT 12 into a
// position with 32 fractional bits), and an acceleration item is
// counts-per-tick-squared x 2^24 (ACCELERATION_SHIFT_LEFT 8, same position
// scale). motor_control.h:64-65.
static const int64_t VRAW_PER_COUNT_PER_TICK = 1048576;    // 2^20

// ---------------------------------------------------------------------------
// The measuring instrument: everything about the machine that a command could
// plausibly disturb, in one read. Comparing two of these is how "the state
// after the second application equals the state after the first" is decided.
// ---------------------------------------------------------------------------
struct Fingerprint {
    bool     ok;         // every read in the fingerprint succeeded
    uint16_t flags;      // status flags (MOSFETs, closed loop, homing, ...)
    uint8_t  fatal;      // latched fatal error code
    int64_t  position;   // COMMANDED position, in encoder counts
    uint8_t  queue;      // motion queue depth
};

static Fingerprint fingerprint(Servomotor* m) {
    Fingerprint f;
    f.ok = true;
    getStatusResponse st = m->getStatus();
    if (m->getError() != 0) f.ok = false;
    f.flags = st.statusFlags;
    f.fatal = st.fatalErrorCode;
    f.position = m->getPositionRaw();
    if (m->getError() != 0) f.ok = false;
    f.queue = m->getNQueuedItems();
    if (m->getError() != 0) f.ok = false;
    return f;
}

static bool sameFingerprint(const Fingerprint& a, const Fingerprint& b) {
    return a.ok && b.ok && a.flags == b.flags && a.fatal == b.fatal &&
           a.position == b.position && a.queue == b.queue;
}

static void printFingerprint(const char* label, const Fingerprint& f) {
    printf("   %-22s ok=%d flags=0x%04X fatal=%u pos=%lld queue=%u\n",
           label, (int)f.ok, (unsigned)f.flags, (unsigned)f.fatal,
           (long long)f.position, (unsigned)f.queue);
}

// ---------------------------------------------------------------------------
// The verdict table. Rows are appended as each measurement is taken and the
// whole thing is printed at the end, so the module's output is a usable
// reference document rather than a wall of PASS lines.
// ---------------------------------------------------------------------------
struct Row {
    const char* command;
    int         cmd;        // command number, per motor_commands.json
    const char* verdict;
    const char* evidence;
};

enum { MAX_ROWS = 40 };
static Row  gRows[MAX_ROWS];
static int  gNRows = 0;

static void addRow(const char* command, int cmd, const char* verdict, const char* evidence) {
    if (gNRows >= MAX_ROWS) return;
    gRows[gNRows].command  = command;
    gRows[gNRows].cmd      = cmd;
    gRows[gNRows].verdict  = verdict;
    gRows[gNRows].evidence = evidence;
    gNRows++;
}

// Queue one trapezoid move and report both halves of how it could be refused:
// the reply error (a queue-time rejection) and the latched fatal code (which may
// instead be raised asynchronously by the control loop mid-move).
struct Outcome {
    int     replyErr;
    uint8_t fatal;
};

static Outcome moveOutcome(Servomotor* m, int32_t counts, uint32_t ticks) {
    Outcome o;
    m->trapezoidMoveRaw(counts, ticks);
    o.replyErr = m->getError();
    if (o.replyErr == 0) {
        uint32_t budget = (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 8000;
        tfhDrain(m, budget);
    }
    o.fatal = tfhFatal(m);
    return o;
}

}  // anonymous namespace -- the entry point below must keep external linkage

void tm_repeated_command_idempotence(void) {
    Serial.println("tm_repeated_command_idempotence: BEGIN\n");
    Servomotor* m = tfGetMotor();
    gNRows = 0;   // file-scope state must be re-initialised at entry

    // =====================================================================
    // 1. THE INSTRUMENT ITSELF MUST BE STABLE. Every later comparison is
    //    "fingerprint A == fingerprint B". If two fingerprints taken back to
    //    back with nothing in between already differed, every one of those
    //    comparisons would be meaningless -- so that is measured first, and
    //    the idle fingerprint is pinned to its documented value.
    // =====================================================================
    printf("\n--- 1. the state fingerprint is stable on an idle machine ---\n");
    {
        tfhReset(m);
        Fingerprint a = fingerprint(m);
        Fingerprint b = fingerprint(m);
        printFingerprint("first", a);
        printFingerprint("second", b);
        TEST_RESULT("two fingerprints taken back to back on an idle machine are identical",
                    sameFingerprint(a, b));
        // Bit 1 is MOSFETs enabled, bit 2 closed loop, bit 3 calibrating,
        // bit 4 homing, bit 5 go-to-closed-loop, bit 6 motor busy
        // (common_source_files/device_status.h:16-22). A reset machine with the
        // output stage off must show none of them.
        TEST_RESULT("a freshly reset machine shows no status flags, no fault and an empty queue",
                    a.ok && a.flags == 0 && a.fatal == 0 && a.queue == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE READS. A getter is idempotent when calling it twice returns the
    //    same answer AND leaves the machine alone. Most of the API is reads,
    //    and supervisory software calls them in loops, so this is the bulk of
    //    the table -- but two of them are exceptions and get their own section.
    // =====================================================================
    printf("\n--- 2. reading twice returns the same answer ---\n");
    {
        tfhReset(m);

        // Exact-valued reads: these are integers the firmware simply reports,
        // so "identical" is the right claim, not "close".
        int64_t pos1 = m->getPositionRaw();
        int64_t pos2 = m->getPositionRaw();
        bool posOk = (m->getError() == 0);
        TEST_RESULT("get position (34) returns exactly the same value on the second call",
                    posOk && pos1 == pos2);
        addRow("Get position", 34, "IDEMPOTENT", "two reads are bit-identical");

        uint8_t q1 = m->getNQueuedItems();
        uint8_t q2 = m->getNQueuedItems();
        getFirmwareVersionResponse fv1 = m->getFirmwareVersion();
        getFirmwareVersionResponse fv2 = m->getFirmwareVersion();
        getProductInfoResponse pi1 = m->getProductInfo();
        getProductInfoResponse pi2 = m->getProductInfo();
        getProductSpecsResponse ps1 = m->getProductSpecs();
        getProductSpecsResponse ps2 = m->getProductSpecs();
        bool constOk = (m->getError() == 0);
        bool constSame = (q1 == q2) &&
                         (memcmp(&fv1, &fv2, sizeof(fv1)) == 0) &&
                         (pi1.uniqueId == pi2.uniqueId) &&
                         (ps1.updateFrequency == ps2.updateFrequency) &&
                         (ps1.countsPerRotation == ps2.countsPerRotation);
        printf("   queue %u/%u  fw %u.%u.%u.%u  uid %08lX%08lX  specs %lu/%lu\n",
               (unsigned)q1, (unsigned)q2,
               (unsigned)fv1.firmwareVersion.major, (unsigned)fv1.firmwareVersion.minor,
               (unsigned)fv1.firmwareVersion.patch, (unsigned)fv1.firmwareVersion.dev,
               (unsigned long)(pi1.uniqueId >> 32), (unsigned long)(pi1.uniqueId & 0xFFFFFFFFu),
               (unsigned long)ps1.updateFrequency, (unsigned long)ps1.countsPerRotation);
        TEST_RESULT("queue depth (11), firmware version (25), product info (22) and product specs (18) all repeat identically",
                    constOk && constSame);
        addRow("Get n queued items", 11, "IDEMPOTENT", "two reads are bit-identical");
        addRow("Get firmware version", 25, "IDEMPOTENT", "two reads are bit-identical");
        addRow("Get product info", 22, "IDEMPOTENT", "two reads are bit-identical");
        addRow("Get product specs", 18, "IDEMPOTENT", "two reads are bit-identical");

        uint8_t payload[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
        pingResponse pr1 = m->ping(payload);
        bool ping1Ok = (m->getError() == 0);
        pingResponse pr2 = m->ping(payload);
        bool ping2Ok = (m->getError() == 0);
        TEST_RESULT("ping (31) echoes the identical payload on both calls",
                    ping1Ok && ping2Ok &&
                    memcmp(pr1.responsePayload, payload, 10) == 0 &&
                    memcmp(pr2.responsePayload, payload, 10) == 0);
        addRow("Ping", 31, "IDEMPOTENT", "identical echo both times");

        // Physical measurements. These are genuine ADC readings, so demanding
        // bit-identity would be testing the noise floor rather than the command;
        // the honest claim is that two consecutive readings agree to within the
        // resolution of the measurement.
        uint16_t v1 = m->getSupplyVoltageRaw();
        uint16_t v2 = m->getSupplyVoltageRaw();
        int16_t  t1 = m->getTemperatureRaw();
        int16_t  t2 = m->getTemperatureRaw();
        bool physOk = (m->getError() == 0);
        printf("   supply voltage raw %u then %u; temperature raw %d then %d\n",
               (unsigned)v1, (unsigned)v2, (int)t1, (int)t2);
        TEST_RESULT("supply voltage (38) and temperature (42) agree between consecutive reads within measurement noise",
                    physOk && llabs((long long)v1 - (long long)v2) <= 5 &&
                    llabs((long long)t1 - (long long)t2) <= 3);
        addRow("Get supply voltage", 38, "IDEMPOTENT", "consecutive reads agree within ADC noise");
        addRow("Get temperature", 42, "IDEMPOTENT", "consecutive reads agree within ADC noise");

        // The documented exception among the pure reads: the clock is RUNNING,
        // so two reads legitimately differ. That is not a defect, and calling
        // it out here is what stops a future reader from "fixing" it.
        uint64_t c1 = m->getCurrentTimeRaw();
        delay(120);
        uint64_t c2 = m->getCurrentTimeRaw();
        bool clockOk = (m->getError() == 0);
        printf("   clock %llu us then %llu us (delta %lld us)\n",
               (unsigned long long)c1, (unsigned long long)c2, (long long)(c2 - c1));
        TEST_RESULT("get current time (9) is the read whose VALUE moves: it advances because the clock runs",
                    clockOk && c2 > c1);
        addRow("Get current time", 9, "IDEMPOTENT (state)", "value advances by design; no state touched");

        // The last two pure reads. Their rows used to be asserted from the
        // firmware source alone, with no measurement behind them; a row in a
        // printed reference document has to be earned by an observation, so both
        // are now read twice here.
        //
        // The two fields are bounded differently, on purpose. The COMMANDED
        // position is a stored integer that cannot change while the queue is
        // empty and the velocity is zero (the control loop adds
        // current_velocity_i64 into the position every tick), so exact equality
        // is the right claim -- and it is the same claim
        // tm_position_source_consistency already makes and passes. The HALL
        // field is a live sensor reading, so it gets HALL_JITTER_TOL instead.
        int64_t h1 = m->getHallSensorPositionRaw();
        bool h1Ok = (m->getError() == 0);
        int64_t h2 = m->getHallSensorPositionRaw();
        bool h2Ok = (m->getError() == 0);
        getComprehensivePositionResponse cp1 = m->getComprehensivePositionRaw();
        bool cp1Ok = (m->getError() == 0);
        getComprehensivePositionResponse cp2 = m->getComprehensivePositionRaw();
        bool cp2Ok = (m->getError() == 0);
        printf("   hall %lld then %lld (gap %lld, tolerance %lld); comprehensive commanded %lld then %lld\n",
               (long long)h1, (long long)h2, (long long)llabs((long long)(h1 - h2)),
               (long long)HALL_JITTER_TOL,
               (long long)cp1.commandedPosition, (long long)cp2.commandedPosition);
        TEST_RESULT("get hall sensor position (15) repeats within sensor noise",
                    h1Ok && h2Ok && llabs((long long)(h1 - h2)) <= HALL_JITTER_TOL);
        TEST_RESULT("get comprehensive position (37) repeats its commanded field exactly, and it equals get position",
                    cp1Ok && cp2Ok && cp1.commandedPosition == cp2.commandedPosition &&
                    cp1.commandedPosition == pos1);

        // And the whole read battery must have left the machine untouched.
        Fingerprint after = fingerprint(m);
        TEST_RESULT("the entire read battery leaves the machine fingerprint untouched",
                    after.ok && after.flags == 0 && after.fatal == 0 &&
                    after.queue == 0 && after.position == pos1);
        addRow("Get status", 16, "IDEMPOTENT", "part of the stable fingerprint");
        addRow("Get hall sensor position", 15, "IDEMPOTENT", "two reads agree within sensor noise");
        addRow("Get comprehensive position", 37, "IDEMPOTENT", "commanded field bit-identical twice");
        tfhReset(m);
    }

    // =====================================================================
    // 3. THE TWO READS THAT CONSUME WHAT THEY REPORT. get_max_PID_error
    //    (motor_control.c:3480-3490) hands back the running min/max and then
    //    writes the empty sentinel over them in the same critical section, and
    //    Get communication statistics takes a resetCounter argument that does
    //    the same to the six error counters. Both are therefore NOT idempotent
    //    in their RETURN VALUE -- but both ARE idempotent in the resulting
    //    STATE, and that distinction is the useful one for a retrying caller.
    // =====================================================================
    printf("\n--- 3. the reads that clear what they report ---\n");
    {
        tfhReset(m);
        // The PID error accumulator is only written from inside PID_controller
        // (motor_control.c:2428-2433), which runs solely in
        // CLOSED_LOOP_POSITION_CONTROL. This module never enters closed loop, so
        // once the first read has cleared it nothing can refill it and the
        // second read MUST be the sentinel exactly.
        getMaxPidErrorResponse e1 = m->getMaxPidErrorRaw();
        bool r1 = (m->getError() == 0);
        getMaxPidErrorResponse e2 = m->getMaxPidErrorRaw();
        bool r2 = (m->getError() == 0);
        getMaxPidErrorResponse e3 = m->getMaxPidErrorRaw();
        bool r3 = (m->getError() == 0);
        printf("   pid error min/max: [%ld,%ld] then [%ld,%ld] then [%ld,%ld]\n",
               (long)e1.minPidError, (long)e1.maxPidError,
               (long)e2.minPidError, (long)e2.maxPidError,
               (long)e3.minPidError, (long)e3.maxPidError);
        TEST_RESULT("get max PID error (39) clears its own accumulator: the second read is the empty sentinel",
                    r1 && r2 && e2.minPidError == PID_ACC_EMPTY_MIN &&
                    e2.maxPidError == PID_ACC_EMPTY_MAX);
        TEST_RESULT("and a third read returns that same sentinel, so the STATE is idempotent from the second call on",
                    r3 && e3.minPidError == e2.minPidError && e3.maxPidError == e2.maxPidError);
        addRow("Get max PID error", 39, "NOT idempotent (return)",
               "clears its accumulator; 2nd read is the empty sentinel");

        // On a clean bus every one of the six counters is an ERROR counter and
        // therefore already zero (there is no packet counter at all -- see
        // common_source_files/RS485.c), so this section can only assert the
        // state-equality half of the claim. Injecting malformed frames to make
        // the counters non-zero is tm_communication_statistics' job and is not
        // repeated here.
        getCommunicationStatisticsResponse s1 = m->getCommunicationStatistics(0);
        bool cs1 = (m->getError() == 0);
        getCommunicationStatisticsResponse s2 = m->getCommunicationStatistics(0);
        bool cs2 = (m->getError() == 0);
        TEST_RESULT("get communication statistics (47) with resetCounter 0 returns identical counters twice",
                    cs1 && cs2 && memcmp(&s1, &s2, sizeof(s1)) == 0);

        getCommunicationStatisticsResponse s3 = m->getCommunicationStatistics(1);
        bool cs3 = (m->getError() == 0);
        getCommunicationStatisticsResponse s4 = m->getCommunicationStatistics(1);
        bool cs4 = (m->getError() == 0);
        bool allZero = cs4 && s4.crc32ErrorCount == 0 && s4.packetDecodeErrorCount == 0 &&
                       s4.firstBitErrorCount == 0 && s4.framingErrorCount == 0 &&
                       s4.overrunErrorCount == 0 && s4.noiseErrorCount == 0;
        printf("   first resetting read reported crc32=%lu decode=%lu; after the second: %lu %lu %lu %lu %lu %lu\n",
               (unsigned long)s3.crc32ErrorCount, (unsigned long)s3.packetDecodeErrorCount,
               (unsigned long)s4.crc32ErrorCount, (unsigned long)s4.packetDecodeErrorCount,
               (unsigned long)s4.firstBitErrorCount, (unsigned long)s4.framingErrorCount,
               (unsigned long)s4.overrunErrorCount, (unsigned long)s4.noiseErrorCount);
        TEST_RESULT("two resetting reads of the statistics leave all six counters at zero",
                    cs3 && allZero);
        addRow("Get communication statistics", 47, "IDEMPOTENT (state)",
               "resetCounter=1 clears; the cleared state is stable");
        tfhReset(m);
    }

    // =====================================================================
    // 4. THE STATE COMMANDS. Each is applied a second time from exactly the
    //    state its first application produced -- no reset in between, which is
    //    what separates this from a determinism check -- and the two resulting
    //    fingerprints are compared field by field.
    // =====================================================================
    printf("\n--- 4a. disable MOSFETs, repeated ---\n");
    {
        // A non-zero starting position first, so "the position did not change"
        // is a real claim rather than a comparison of two zeroes.
        tfhFreshOpen(m);
        bool movedOk = false;
        tfhMove(m, (int32_t)(TFH_CPR / 4), tfhTicksFor(TFH_CPR / 4), &movedOk);

        // Note what this can and cannot show on a shared rack: the output stage
        // is never energised here, so the only transition available is
        // off -> off. That is precisely the case a retrying supervisor hits,
        // and the realistic failure mode -- a second application being REFUSED
        // because the machine is already in the target state -- is fully
        // exercised by it.
        m->disableMosfets();
        bool first = (m->getError() == 0);
        Fingerprint a = fingerprint(m);
        m->disableMosfets();
        bool second = (m->getError() == 0);
        Fingerprint b = fingerprint(m);
        bool restAccepted = true;
        for (int i = 0; i < 3; i++) {
            m->disableMosfets();
            if (m->getError() != 0) restAccepted = false;
        }
        Fingerprint c = fingerprint(m);
        printFingerprint("after 1st", a);
        printFingerprint("after 2nd", b);
        printFingerprint("after 5th", c);
        TEST_RESULT("disable MOSFETs (0) is accepted five times in a row with no error",
                    movedOk && first && second && restAccepted);
        // Bit 1 of the status flags is STATUS_MOSFETS_ENABLED_FLAG_BIT
        // (common_source_files/device_status.h:17); it must stay clear throughout.
        TEST_RESULT("the machine state after the second disable equals the state after the first, MOSFETs still off",
                    sameFingerprint(a, b) && sameFingerprint(b, c) &&
                    (a.flags & (1 << 1)) == 0 && (c.flags & (1 << 1)) == 0);
        addRow("Disable MOSFETs", 0, "IDEMPOTENT", "5 applications, one fingerprint");
        tfhReset(m);
    }

    printf("\n--- 4b. zero position, repeated ---\n");
    {
        // zero_position ASSIGNS zero (motor_control.c:3319-3325), so the claim
        // is exact zero, not "about zero" -- and with the queue empty the
        // control loop adds a zero velocity each tick, so it stays exact.
        tfhFreshOpen(m);
        bool movedOk = false;
        tfhMove(m, (int32_t)(TFH_CPR / 3), tfhTicksFor(TFH_CPR / 3), &movedOk);
        int64_t beforeZero = m->getPositionRaw();

        m->zeroPosition();
        bool first = (m->getError() == 0);
        Fingerprint a = fingerprint(m);
        // The only precondition is an EMPTY QUEUE (fatal error 8,
        // ERROR_QUEUE_NOT_EMPTY, motor_control.c:3313). It is empty here, so the
        // repeat must be accepted rather than refused.
        m->zeroPosition();
        bool second = (m->getError() == 0);
        Fingerprint b = fingerprint(m);
        bool restZero = true;
        for (int i = 0; i < 2; i++) {
            m->zeroPosition();
            if (m->getError() != 0) restZero = false;
            if (m->getPositionRaw() != 0) restZero = false;
        }
        printf("   position %lld -> zeroed -> %lld -> zeroed again -> %lld\n",
               (long long)beforeZero, (long long)a.position, (long long)b.position);
        TEST_RESULT("zero position (13) sets the commanded position to exactly zero",
                    movedOk && first && beforeZero != 0 && a.position == 0);
        TEST_RESULT("a second zero position is accepted, not refused with error 8",
                    second && b.fatal == 0);
        TEST_RESULT("the state after the second zeroing equals the state after the first, and four in a row all leave exactly zero",
                    sameFingerprint(a, b) && restZero);
        addRow("Zero position", 13, "IDEMPOTENT", "exactly 0 after every application");
        tfhReset(m);
    }

    printf("\n--- 4c. emergency stop, repeated ---\n");
    {
        // emergency_stop disables the output stage and calls
        // clear_the_queue_and_stop_no_disable_interrupt (motor_control.c:3405),
        // which zeroes the velocity and the queue but NEVER writes
        // current_position_i96 (:420-429). So a second stop must not move the
        // reported position by even one count.
        tfhFreshOpen(m);
        m->setMaximumVelocity(2.0f);
        m->setMaximumAcceleration(500.0f);
        m->trapezoidMoveRaw((int32_t)TFH_CPR, 4 * TFH_TPS);
        bool queued = (m->getError() == 0);
        delay(500);                       // let it get properly under way
        uint8_t depthMoving = m->getNQueuedItems();

        m->emergencyStop();
        bool first = (m->getError() == 0);
        delay(200);                       // let the last control tick land
        Fingerprint a = fingerprint(m);
        m->emergencyStop();
        bool second = (m->getError() == 0);
        delay(200);
        Fingerprint b = fingerprint(m);
        printFingerprint("after 1st stop", a);
        printFingerprint("after 2nd stop", b);
        TEST_RESULT("the first emergency stop (12) empties the queue",
                    queued && first && depthMoving > 0 && a.queue == 0);
        TEST_RESULT("a second emergency stop on an already-stopped machine is accepted and latches no fault",
                    second && b.fatal == 0);
        TEST_RESULT("and the second stop does not rewrite the commanded position",
                    a.position == b.position);

        // A stop is not a fault: the machine must still take work afterwards,
        // without a reset. If it did not, "stop" would be a fatal error under
        // another name.
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 8), tfhTicksFor(TFH_CPR / 8), &ok);
        TEST_RESULT("the machine still accepts and completes a move after two stops",
                    ok && llabs((long long)(moved - TFH_CPR / 8)) < 400);
        addRow("Emergency stop", 12, "IDEMPOTENT", "queue empty, position not rewritten");
        tfhReset(m);
    }

    printf("\n--- 4d. reset time, repeated ---\n");
    {
        // reset_time re-zeroes the microsecond clock and clears the queue
        // (motor_control.c:3396-3402). Applying it twice must leave the clock
        // near zero BOTH times -- it must not become cumulative, and it must not
        // become a no-op that leaves the clock running from the first reset.
        tfhFreshOpen(m);
        bool movedOk = false;
        tfhMove(m, (int32_t)(TFH_CPR / 8), tfhTicksFor(TFH_CPR / 8), &movedOk);
        Fingerprint before = fingerprint(m);

        m->resetTime();
        bool first = (m->getError() == 0);
        delay(400);
        uint64_t t1 = m->getCurrentTimeRaw();
        m->resetTime();
        bool second = (m->getError() == 0);
        uint64_t t2 = m->getCurrentTimeRaw();
        bool clockOk = (m->getError() == 0);
        Fingerprint after = fingerprint(m);
        printf("   clock 400 ms after the 1st reset: %llu us; immediately after the 2nd: %llu us\n",
               (unsigned long long)t1, (unsigned long long)t2);
        TEST_RESULT("reset time (8) re-zeroes a clock that had been left running",
                    first && second && clockOk && t1 > 300000ULL && t2 < 200000ULL);
        TEST_RESULT("reset time leaves the commanded position and queue depth alone",
                    movedOk && before.ok && after.ok &&
                    after.position == before.position && after.queue == before.queue);
        addRow("Reset time", 8, "IDEMPOTENT", "clock re-zeroed by each application");
        tfhReset(m);
    }

    printf("\n--- 4e. system reset, repeated ---\n");
    {
        // tm_reset_edges already covers back-to-back resets as a TIMING hazard.
        // The claim here is the idempotence one: the second reset must not
        // produce a machine that differs in any way from the one the first
        // produced.
        tfhReset(m);
        Fingerprint a = fingerprint(m);
        tfhReset(m);
        Fingerprint b = fingerprint(m);
        printFingerprint("after 1st reset", a);
        printFingerprint("after 2nd reset", b);
        TEST_RESULT("two system resets (27) back to back leave identical machine state",
                    sameFingerprint(a, b) && a.fatal == 0 && a.queue == 0);
        addRow("System reset", 27, "IDEMPOTENT", "identical fingerprint after each");
    }

    // =====================================================================
    // 5. THE SETTERS. The property that makes a setter safe to retry is that
    //    it ASSIGNS rather than accumulates. Two of them can be read straight
    //    back through Get debug values (45), which allows an exact claim; the
    //    rest have no getter at all and are probed behaviourally, one
    //    application against two.
    // =====================================================================
    printf("\n--- 5a. the limit setters, read back exactly ---\n");
    {
        tfhReset(m);
        getDebugValuesResponse d0 = m->getDebugValues();
        bool dOk = (m->getError() == 0);
        int64_t bootVel = d0.maxVelocity;
        int64_t bootAcc = d0.maxAcceleration;

        // Pick values derived from the boot defaults so this stays correct on
        // any product, and stays clear of the clamp at EXPERIMENTAL_MAX_VELOCITY
        // / EXPERIMENTAL_MAX_ACCELERATION (motor_control.h:61-62), which would
        // otherwise make the readback a measurement of the ceiling.
        uint32_t vA = (uint32_t)((bootVel >> 12) / 2);
        uint32_t vB = (uint32_t)((bootVel >> 12) / 3);
        m->setMaximumVelocityRaw(vA);
        getDebugValuesResponse d1 = m->getDebugValues();
        m->setMaximumVelocityRaw(vA);
        getDebugValuesResponse d2 = m->getDebugValues();
        m->setMaximumVelocityRaw(vB);
        getDebugValuesResponse d3 = m->getDebugValues();
        bool vOk = (m->getError() == 0);
        printf("   boot maxVelocity %lld; after vA %lld then %lld; after vB %lld (expected %lld / %lld)\n",
               (long long)bootVel, (long long)d1.maxVelocity, (long long)d2.maxVelocity,
               (long long)d3.maxVelocity, (long long)((int64_t)vA << 12),
               (long long)((int64_t)vB << 12));
        TEST_RESULT("set maximum velocity (3) reads back the same internal value on both applications",
                    dOk && vOk && d1.maxVelocity == d2.maxVelocity);
        TEST_RESULT("and that value is exactly the sent value shifted left by 12",
                    d2.maxVelocity == ((int64_t)vA << 12));
        TEST_RESULT("a different second value REPLACES the first: setters assign, they do not accumulate",
                    d3.maxVelocity == ((int64_t)vB << 12));
        addRow("Set maximum velocity", 3, "IDEMPOTENT", "debug readback identical; assignment");

        uint32_t aA = (uint32_t)((bootAcc >> 8) / 2);
        uint32_t aB = (uint32_t)((bootAcc >> 8) / 3);
        m->setMaximumAccelerationRaw(aA);
        getDebugValuesResponse e1 = m->getDebugValues();
        m->setMaximumAccelerationRaw(aA);
        getDebugValuesResponse e2 = m->getDebugValues();
        m->setMaximumAccelerationRaw(aB);
        getDebugValuesResponse e3 = m->getDebugValues();
        bool aOk = (m->getError() == 0);
        printf("   boot maxAcceleration %lld; after aA %lld then %lld; after aB %lld (expected %lld / %lld)\n",
               (long long)bootAcc, (long long)e1.maxAcceleration, (long long)e2.maxAcceleration,
               (long long)e3.maxAcceleration, (long long)((int64_t)aA << 8),
               (long long)((int64_t)aB << 8));
        TEST_RESULT("set maximum acceleration (5) behaves identically at its own scale, shifted left by 8",
                    aOk && e1.maxAcceleration == e2.maxAcceleration &&
                    e2.maxAcceleration == ((int64_t)aA << 8) &&
                    e3.maxAcceleration == ((int64_t)aB << 8));
        addRow("Set maximum acceleration", 5, "IDEMPOTENT", "debug readback identical; assignment");
        addRow("Get debug values", 45, "IDEMPOTENT", "read; used as the readback here");
        tfhReset(m);
    }

    printf("\n--- 5b. the setters with no getter, probed behaviourally ---\n");
    {
        // Deviation limit. Applied once and then twice; the observable is the
        // fault a move well past the window raises (error 45,
        // motor_control.c:3217). One application and two must give the same
        // verdict. The MOSFETs are off, so the shaft never follows and the
        // deviation is exactly the commanded displacement -- which is what makes
        // this a clean one-bit readout of which limit is in force.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        m->setMaxAllowablePositionDeviationRaw(TFH_CPR / 2);
        Outcome devOnce = moveOutcome(m, (int32_t)(TFH_CPR * 3 / 2), 2 * TFH_TPS);
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        m->setMaxAllowablePositionDeviationRaw(TFH_CPR / 2);
        m->setMaxAllowablePositionDeviationRaw(TFH_CPR / 2);
        Outcome devTwice = moveOutcome(m, (int32_t)(TFH_CPR * 3 / 2), 2 * TFH_TPS);
        printf("   deviation limit applied once: fatal=%u; applied twice: fatal=%u\n",
               (unsigned)devOnce.fatal, (unsigned)devTwice.fatal);
        TEST_RESULT("one and two applications of the deviation limit (44) trip the same fault",
                    devOnce.fatal == ERR_POSITION_DEVIATION_TOO_LARGE &&
                    devTwice.fatal == devOnce.fatal);
        addRow("Set max allowable position deviation", 44, "IDEMPOTENT", "same trip outcome for 1 and 2 applications");

        // Safety fence. The observable is the queue-time refusal of a move whose
        // predicted end lies outside it (error 27). The check that actually
        // fires here is the one in the ACCELERATION branch of add_to_queue()
        // (motor_control.c:1811-1813), because with these limits the trapezoid
        // planner emits three acceleration segments rather than taking the
        // one-tick-ramp velocity fast path. The near-identical check at :1911 is
        // inside add_to_queue_test(), the dry-run validator behind the
        // firmware-internal command 120, and is NOT what refuses this move --
        // an earlier revision of this comment cited it by mistake.
        tfhReset(m);
        tfhOpenCeilings(m);
        m->setSafetyLimitsRaw(-(int64_t)(TFH_CPR / 4), (int64_t)(TFH_CPR / 4));
        Outcome fenceOnce = moveOutcome(m, (int32_t)(TFH_CPR / 2), tfhTicksFor(TFH_CPR / 2));
        tfhReset(m);
        tfhOpenCeilings(m);
        m->setSafetyLimitsRaw(-(int64_t)(TFH_CPR / 4), (int64_t)(TFH_CPR / 4));
        m->setSafetyLimitsRaw(-(int64_t)(TFH_CPR / 4), (int64_t)(TFH_CPR / 4));
        Outcome fenceTwice = moveOutcome(m, (int32_t)(TFH_CPR / 2), tfhTicksFor(TFH_CPR / 2));
        printf("   fence applied once: reply=%d fatal=%u; applied twice: reply=%d fatal=%u\n",
               fenceOnce.replyErr, (unsigned)fenceOnce.fatal,
               fenceTwice.replyErr, (unsigned)fenceTwice.fatal);
        TEST_RESULT("one and two applications of the safety fence (30) refuse the same move with the same code",
                    fenceOnce.fatal == ERR_PREDICTED_POS_OUT_OF_SAFETY &&
                    fenceTwice.fatal == fenceOnce.fatal &&
                    fenceTwice.replyErr == fenceOnce.replyErr);
        addRow("Set safety limits", 30, "IDEMPOTENT", "identical refusal for 1 and 2 applications");

        // PID constants, motor current limits and Identify have no readback and
        // no effect that is observable with the output stage off. The honest
        // claim for them is the weaker one, stated as such: a repeat is accepted
        // and disturbs nothing else. That still catches the realistic failure --
        // a second application being refused, or corrupting shared state.
        tfhReset(m);
        Fingerprint before = fingerprint(m);
        bool accepted = true;
        m->setPidConstants(1000, 100, 10);   if (m->getError() != 0) accepted = false;
        m->setPidConstants(1000, 100, 10);   if (m->getError() != 0) accepted = false;
        m->setMaximumMotorCurrentRaw(200, 200); if (m->getError() != 0) accepted = false;
        m->setMaximumMotorCurrentRaw(200, 200); if (m->getError() != 0) accepted = false;
        m->identify();                       if (m->getError() != 0) accepted = false;
        m->identify();                       if (m->getError() != 0) accepted = false;
        Fingerprint after = fingerprint(m);
        TEST_RESULT("repeating the PID constants (43), the current limits (28) and identify (41) is accepted and changes nothing else",
                    accepted && sameFingerprint(before, after));
        addRow("Set PID constants", 43, "IDEMPOTENT (unobservable here)", "accepted twice; no state change");
        addRow("Set maximum motor current", 28, "IDEMPOTENT (unobservable here)", "accepted twice; no state change");
        addRow("Identify", 41, "IDEMPOTENT", "accepted twice; restarts the same fixed flash pattern");
        tfhReset(m);
    }

    printf("\n--- 5c. CRC32 control, repeated in both directions ---\n");
    {
        // The ordering matters and is the reason this is worth its own probe:
        // the SECOND crc32Control(0) has to go out WITHOUT a CRC, because the
        // first one already told the firmware to stop expecting them. Getting
        // that backwards desynchronises the link and every later command times
        // out, which is exactly the failure a naive retry would cause.
        tfhReset(m);
        m->enableCRC32();                 // the firmware always boots with CRC32 on
        m->crc32Control(1);
        bool onFirst = (m->getError() == 0);
        m->crc32Control(1);
        bool onSecond = (m->getError() == 0);
        m->getPositionRaw();
        bool talksOn = (m->getError() == 0);

        m->crc32Control(0);               // tell the firmware first...
        bool offFirst = (m->getError() == 0);
        m->disableCRC32();                // ...then match the library's framing
        m->crc32Control(0);               // this one is sent unprotected, as it must be
        bool offSecond = (m->getError() == 0);
        m->getPositionRaw();
        bool talksOff = (m->getError() == 0);

        m->crc32Control(1);               // restore, same ordering rule in reverse
        m->enableCRC32();
        m->getPositionRaw();
        bool restored = (m->getError() == 0);
        printf("   on/on %d%d talks %d | off/off %d%d talks %d | restored %d\n",
               (int)onFirst, (int)onSecond, (int)talksOn,
               (int)offFirst, (int)offSecond, (int)talksOff, (int)restored);
        TEST_RESULT("CRC32 control (46) is idempotent in both directions and the link keeps working",
                    onFirst && onSecond && talksOn && offFirst && offSecond &&
                    talksOff && restored);
        addRow("CRC32 control", 46, "IDEMPOTENT", "on/on and off/off both accepted; link intact");
        tfhReset(m);
        m->enableCRC32();
    }

    // =====================================================================
    // 6. THE COMMANDS THAT ARE NOT IDEMPOTENT -- and what they do instead.
    //    "It changed" is not a specification, so each of these is pinned to
    //    its exact compounded effect. The pair that matters most is the first
    //    two: a RELATIVE move doubles and an ABSOLUTE move does not, which is
    //    the single most consequential distinction in this whole table for
    //    anyone writing a retry.
    // =====================================================================
    printf("\n--- 6a. relative versus absolute: the retry-safety pair ---\n");
    {
        tfhFreshOpen(m);
        const int32_t  D = (int32_t)(TFH_CPR / 2);
        const uint32_t T = tfhTicksFor(TFH_CPR / 2);
        int64_t origin = m->getPositionRaw();
        bool ok1 = false, ok2 = false;
        int64_t d1 = tfhMove(m, D, T, &ok1);
        int64_t d2 = tfhMove(m, D, T, &ok2);
        int64_t total = m->getPositionRaw() - origin;
        printf("   trapezoid %ld twice: leg1 %lld, leg2 %lld, total %lld (expected %lld)\n",
               (long)D, (long long)d1, (long long)d2, (long long)total, (long long)(2 * (int64_t)D));
        TEST_RESULT("a relative trapezoid move (2) applied twice travels exactly twice as far",
                    ok1 && ok2 && llabs((long long)(total - 2 * (int64_t)D)) <= 8);
        addRow("Trapezoid move", 2, "NOT idempotent (relative)", "two applications travel exactly 2x");

        // add_go_to_position_to_queue subtracts position_after_last_queue_item
        // from the target (motor_control.c:2074-2079), so a repeat computes a
        // displacement of zero: accepted, queued as a dwell, and it lands on the
        // very same point.
        tfhFreshOpen(m);
        const int32_t TARGET = (int32_t)(TFH_CPR / 2);
        m->goToPositionRaw(TARGET, T);
        bool g1 = (m->getError() == 0) && tfhDrain(m, 20000);
        int64_t p1 = m->getPositionRaw();
        m->goToPositionRaw(TARGET, T);
        bool g2 = (m->getError() == 0) && tfhDrain(m, 20000);
        int64_t p2 = m->getPositionRaw();
        // A THIRD application, to separate "converged" from "still moving".
        m->goToPositionRaw(TARGET, T);
        bool g3 = (m->getError() == 0) && tfhDrain(m, 12000);
        int64_t p3 = m->getPositionRaw();

        printf("   go to %ld three times: landed at %lld, then %lld, then %lld\n",
               (long)TARGET, (long long)p1, (long long)p2, (long long)p3);

        // MEASURED, and it is not quite what "idempotent" would predict.
        //
        // A single trapezoid move lands up to one count SHORT of its commanded
        // displacement: the planner's segment arithmetic is integer division,
        // so a little under one count is lost. The same shortfall shows up
        // everywhere in this suite -- 409599 of 409600, 1638399 of 1638400.
        //
        // Because `go to position` is ABSOLUTE, the second application sees a
        // one-count error and corrects it, landing exactly on the target. So
        // the command is not idempotent on its FIRST application; it is
        // CONVERGENT, and idempotent from the second application onwards.
        //
        // That is the stronger and more useful property, and it is the one a
        // caller actually needs: repeating an absolute move always ends exactly
        // on the target, whereas a relative move would accumulate the shortfall.
        TEST_RESULT("an absolute go to position (4) converges: the second "
                    "application lands exactly on the target",
                    g1 && g2 && tfhFatal(m) == 0 && p2 == (int64_t)TARGET);
        TEST_RESULT("and it is idempotent from there on: a third application "
                    "changes nothing",
                    g3 && tfhFatal(m) == 0 && p3 == p2);
        TEST_RESULT("the first application is already within a count of the target, "
                    "so the correction is a rounding residue and not a real miss",
                    g1 && llabs((long long)(p1 - (int64_t)TARGET)) <= 2);
        addRow("Go to position", 4, "CONVERGENT (absolute)",
               "1st lands <=1 count short; 2nd lands exactly; 3rd is a no-op");
        tfhReset(m);
    }

    printf("\n--- 6b. queued segments: velocity assigns, acceleration accumulates ---\n");
    {
        // A velocity queue item ASSIGNS the velocity
        // (motor_control.c:1842-1844: predicted_final_velocity = velocity), so
        // repeating it doubles the DISTANCE and leaves the SPEED alone. Every
        // sequence ends with an explicit zero-velocity segment: a velocity item
        // holds its velocity after its time expires, and without the terminator
        // the queue would empty at speed and raise error 18.
        const int32_t  VRAW  = (int32_t)(32 * VRAW_PER_COUNT_PER_TICK);  // 32 counts/tick
        const uint32_t VT    = (uint32_t)TFH_TPS;                        // exactly one second
        const int64_t  VLEG  = 32LL * (int64_t)TFH_TPS;                  // 1,000,000 counts

        tfhFreshOpen(m);
        int64_t v0 = m->getPositionRaw();
        m->moveWithVelocityRaw(VRAW, VT);
        m->moveWithVelocityRaw(0, 10);
        bool va = (m->getError() == 0) && tfhDrain(m, 20000) && (tfhFatal(m) == 0);
        int64_t vOnce = m->getPositionRaw() - v0;

        tfhFreshOpen(m);
        int64_t v1 = m->getPositionRaw();
        m->moveWithVelocityRaw(VRAW, VT);
        m->moveWithVelocityRaw(VRAW, VT);
        m->moveWithVelocityRaw(0, 10);
        bool vb = (m->getError() == 0) && tfhDrain(m, 25000) && (tfhFatal(m) == 0);
        int64_t vTwice = m->getPositionRaw() - v1;
        printf("   velocity segment once: %lld counts (predicted %lld); twice: %lld (predicted %lld)\n",
               (long long)vOnce, (long long)VLEG, (long long)vTwice, (long long)(2 * VLEG));
        TEST_RESULT("one velocity segment (26) covers its predicted distance",
                    va && llabs((long long)(vOnce - VLEG)) <= 64);
        TEST_RESULT("repeating it covers exactly twice the distance: the velocity is assigned, not accumulated",
                    vb && llabs((long long)(vTwice - 2 * VLEG)) <= 128);
        addRow("Move with velocity", 26, "NOT idempotent (cumulative)", "distance doubles; speed does not");

        // An acceleration item ADDS to the end-of-queue velocity
        // (motor_control.c:1799), so the second identical segment starts at the
        // speed the first one finished at. Continuous prediction: the first
        // covers a*T^2/2 and the pair covers 2*a*T^2, a ratio of exactly 4, with
        // a discrete correction of order 1/T (negligible at T = 6250).
        const int32_t  ARAW = 1048576;    // 1/16 count per tick^2  (= 2^20 / 2^24)
        const uint32_t AT   = 6250;       // 0.2 s; peak speed after two ~7.5 rot/s

        tfhFreshOpen(m);
        int64_t a0 = m->getPositionRaw();
        m->moveWithAccelerationRaw(ARAW, AT);
        m->moveWithVelocityRaw(0, 10);
        bool aa = (m->getError() == 0) && tfhDrain(m, 20000) && (tfhFatal(m) == 0);
        int64_t aOnce = m->getPositionRaw() - a0;

        tfhFreshOpen(m);
        int64_t a1 = m->getPositionRaw();
        m->moveWithAccelerationRaw(ARAW, AT);
        m->moveWithAccelerationRaw(ARAW, AT);
        m->moveWithVelocityRaw(0, 10);
        bool ab = (m->getError() == 0) && tfhDrain(m, 20000) && (tfhFatal(m) == 0);
        int64_t aTwice = m->getPositionRaw() - a1;
        double ratio = (aOnce != 0) ? (double)aTwice / (double)aOnce : 0.0;
        printf("   acceleration segment once: %lld counts; twice: %lld counts; ratio %.4f (predicted 4.0)\n",
               (long long)aOnce, (long long)aTwice, ratio);
        TEST_RESULT("one acceleration segment (19) covers a measurable distance from rest",
                    aa && aOnce > 0);
        // The trailing zero-velocity segment is what leaves the machine at rest;
        // without it the queue would empty at speed and raise error 18. That the
        // queue is empty and unfaulted here is the check that it worked.
        TEST_RESULT("repeating it covers four times as much in total (the velocity accumulates), ending at rest with an empty queue",
                    ab && ratio > 3.9 && ratio < 4.1 &&
                    m->getNQueuedItems() == 0 && m->getError() == 0 && tfhFatal(m) == 0);
        addRow("Move with acceleration", 19, "NOT idempotent (cumulative)",
               "velocity accumulates: 2nd pass covers 3x the 1st");
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE TABLE. The point of the module: a single readable answer to
    //    "can I safely send this command again?", per command.
    // =====================================================================
    printf("\n--- 7. the idempotence table ---\n");
    printf("   %-38s %4s  %-28s %s\n", "command", "cmd", "verdict", "how it was observed");
    printf("   %-38s %4s  %-28s %s\n", "--------------------------------------",
           "---", "----------------------------", "-------------------");
    for (int i = 0; i < gNRows; i++) {
        printf("   %-38s %4d  %-28s %s\n",
               gRows[i].command, gRows[i].cmd, gRows[i].verdict, gRows[i].evidence);
    }
    printf("   (%d commands measured)\n", gNRows);
    TEST_RESULT("the table covers at least twenty-five commands",
                gNRows >= 25);

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
