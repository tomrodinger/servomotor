#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_time_sync_protocol.cpp
//
// THE TIME SYNC OPERATION ITSELF (cmd 10), TAKEN APART AND VERIFIED NUMERICALLY.
//
// Time sync is the one command in the whole protocol whose reply is a computed
// NUMBER rather than a state readback, and that number is easy to get subtly
// wrong: it is a signed 32-bit microsecond error produced by MODULAR u32
// subtraction, paired with a raw peripheral register. Nothing in the suite has
// ever checked that the number is right -- only that the command answers.
//
// What the firmware actually does (verified, not assumed):
//
//   main.c:380-402       TIME_SYNC_COMMAND: takes a u32 `masterTime`, replies
//                        with { int32 time_error; uint16 clock_calibration }.
//   clock_calibration.c:46-50
//                        uint32_t local = (uint32_t)get_microsecond_time();
//                        int32_t  err   = (int32_t)(master - local);
//                        RCC->ICSCR = PI_controller(err) << HSITRIM_Pos;
//                        return err;
//   clock_calibration.c:12-35
//                        PI: error clamped to [-32768, 32767]; integral (a
//                        FUNCTION-STATIC, so it survives everything but a
//                        reboot) starts at 64<<16 and accumulates the clamped
//                        error; output = (integral + 40*error) >> 16, clamped
//                        to MIN_HSI_TRIM_VALUE 62 .. MAX_HSI_TRIM_VALUE 66.
//   clock_calibration.c:53-56
//                        the reply's second field is RCC->ICSCR & 0xFFFF, i.e.
//                        the read-only factory HSICAL in bits 7:0 and the trim
//                        just written in bits 14:8 (stm32g031xx.h:4200).
//
// THE DECISIVE FACT, and the reason this module exists: time sync NEVER SETS
// THE CLOCK. It steers the HSI16 oscillator by at most two trim steps either
// way and returns the error; `get_microsecond_time()` is never written. So a
// sync to a time far BEHIND the device cannot rewind the clock, and a sync far
// AHEAD cannot jump it forward. That is a strong safety property -- the clock
// stays monotonic no matter what a master sends -- and it is asserted here
// directly, because the obvious alternative implementation (step the clock to
// the master's value) would break every timestamp in flight and would fire the
// firmware's own ERROR_TIME_WENT_BACKWARDS check (code 1,
// microsecond_clock.c:61-63).
//
// DISTINCT FROM THE TWO EXISTING TIME MODULES:
//   * tm_time drives cmd 8/9/10 as commands: it syncs at 10 Hz against the
//     HOST's millis() clock and asserts only that the replies are bounded and
//     the device stays alive. It never injects a known offset, never looks at
//     the second reply field, and cannot say whether the returned number is
//     correct -- only that it is small.
//   * tm_time_domain closes the loop between the CLOCK and the PLANNER (a move
//     of N ticks consumes N ticks of device clock) and treats sync as one more
//     event the clock must survive.
//   * This module treats the SYNC REPLY as the subject: the arithmetic that
//     produces `timeError`, including its behaviour at the +/-2^31 modular
//     boundary; the meaning of every bit of `rccIcscr`; the PI controller's
//     railing and its hidden memory; that the clock is untouched; and that a
//     10 Hz sync stream does not perturb a move that is already running.
//
// SAFETY. The MOSFETs stay OFF for the entire module -- enableMosfets() and
// goToClosedLoop() are never called. Every displacement is read from the
// COMMANDED position (getPositionRaw), which the planner advances whether or
// not the output stage is energised, so nothing turns on any motor. Time sync
// itself cannot cause motion: its only side effect is a two-step oscillator
// trim, which a system reset restores (NVIC_SystemReset, main.c:623-629, resets
// RCC->ICSCR to its default and re-initialises the PI controller's static
// integral), and the module ends with tfhReset. No broadcasts, no alias
// changes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// The firmware's own constants, restated so a change to either side shows up
// here as a failure rather than as silence (clock_calibration.c:6-10).
static const int32_t PI_PROPORTIONAL   = 40;
static const int32_t PI_INTEGRAL       = 1;
static const int32_t PI_ERROR_CLAMP_LO = -32768;
static const int32_t PI_ERROR_CLAMP_HI = 32767;
static const uint8_t TRIM_MIN          = 62;
static const uint8_t TRIM_MAX          = 66;
static const int32_t TRIM_INITIAL      = 64;

// A command round trip, in microseconds. Every reply carries it as a constant
// bias (the master timestamp is read from the device BEFORE the sync is sent,
// so the device's clock has moved on by the round trip when the sync lands).
// tm_time already demonstrated on hardware that the combined skew stays under
// 60 ms, so 150 ms is a generous ceiling that still makes the assertions sharp
// against injected offsets of up to 2.1 BILLION microseconds.
static const uint32_t SANE_LATENCY_US = 150000;

// Running bounds on the trim field over every sync this module performs.
// EMPTY SENTINEL: min starts above max on purpose, so "no sample yet" is
// distinguishable from a real reading. Never compare them before nSamples > 0.
static uint8_t  g_trimLo;
static uint8_t  g_trimHi;
static uint32_t g_trimSamples;

// A local mirror of the firmware's PI state, so the returned trim can be
// predicted independently instead of merely range-checked.
static int32_t g_piIntegral;

static void piModelReset(void) { g_piIntegral = TRIM_INITIAL << 16; }

// Recomputation of PI_controller() (clock_calibration.c:12-35) from the error
// the device itself reported. Deliberately written to mirror the firmware line
// for line, including that the CLAMPED error feeds both terms.
static uint8_t piModelStep(int32_t error) {
    if (error < PI_ERROR_CLAMP_LO)      error = PI_ERROR_CLAMP_LO;
    else if (error > PI_ERROR_CLAMP_HI) error = PI_ERROR_CLAMP_HI;
    g_piIntegral += error * PI_INTEGRAL;
    int32_t out = (g_piIntegral + error * PI_PROPORTIONAL) >> 16;
    if (out < (int32_t)TRIM_MIN)      out = TRIM_MIN;
    else if (out > (int32_t)TRIM_MAX) out = TRIM_MAX;
    return (uint8_t)out;
}

// One sync reply, decoded.
typedef struct {
    bool     ok;        // reply received, right size, no communication error
    int32_t  err;       // timeError, exactly as returned
    uint16_t icscr;     // rccIcscr, exactly as returned
    uint8_t  trim;      // HSITRIM, bits 14:8 of icscr
    uint8_t  hsical;    // HSICAL, bits 7:0 -- read-only factory calibration
    uint32_t latency;   // (u32)(injected offset - err): the round trip, in us
} tsSample;

static void tsNoteTrim(uint8_t trim) {
    if (g_trimSamples == 0) { g_trimLo = trim; g_trimHi = trim; }
    else { if (trim < g_trimLo) g_trimLo = trim; if (trim > g_trimHi) g_trimHi = trim; }
    g_trimSamples++;
}

// The device clock in microseconds. cmd 9 reports a true 1 MHz microsecond
// count (main.c:367-378 -> get_microsecond_time(), a 48-bit us counter built
// from TIM14 plus a software high word, microsecond_clock.c:38-69).
static uint64_t devUs(Servomotor* m, bool* ok) {
    uint64_t us = m->getCurrentTimeRaw();
    *ok = (m->getError() == 0);
    return us;
}

// Sync with a master timestamp deliberately offset from the device's OWN clock
// by `offsetUs`, and decode the reply.
//
// The whole point is the residual: the device computes err = master - local by
// u32 modular subtraction, and local has advanced by the round trip L since we
// read it, so err == offset - L for EVERY offset, in u32 modular arithmetic.
// Recovering L as (u32)(offset - err) therefore checks the firmware's
// arithmetic exactly, and does so identically on both sides of the 2^31 wrap.
static tsSample tsSyncAtOffset(Servomotor* m, uint32_t offsetUs) {
    tsSample s; s.ok = false; s.err = 0; s.icscr = 0; s.trim = 0; s.hsical = 0; s.latency = 0;
    bool clockOk = true;
    uint64_t t = devUs(m, &clockOk);
    if (!clockOk) return s;
    uint32_t master = (uint32_t)t + offsetUs;
    timeSyncResponse r = m->timeSyncRaw(master);
    if (m->getError() != 0) return s;
    s.ok      = true;
    s.err     = r.timeError;
    s.icscr   = r.rccIcscr;
    s.trim    = (uint8_t)((r.rccIcscr >> 8) & 0x7F);
    s.hsical  = (uint8_t)(r.rccIcscr & 0xFF);
    s.latency = (uint32_t)(offsetUs - (uint32_t)r.timeError);
    tsNoteTrim(s.trim);
    return s;
}

// Sync with a literal master timestamp (no clock read first). Used where the
// extra round trip of reading the clock would itself distort what is measured.
static tsSample tsSyncRawMaster(Servomotor* m, uint32_t master) {
    tsSample s; s.ok = false; s.err = 0; s.icscr = 0; s.trim = 0; s.hsical = 0; s.latency = 0;
    timeSyncResponse r = m->timeSyncRaw(master);
    if (m->getError() != 0) return s;
    s.ok     = true;
    s.err    = r.timeError;
    s.icscr  = r.rccIcscr;
    s.trim   = (uint8_t)((r.rccIcscr >> 8) & 0x7F);
    s.hsical = (uint8_t)(r.rccIcscr & 0xFF);
    tsNoteTrim(s.trim);
    return s;
}

void tm_time_sync_protocol(void) {
    Serial.println("tm_time_sync_protocol: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // File-scope state must be re-initialised at entry: the host harness can
    // run several modules back-to-back inside one process.
    g_trimLo = 0xFF; g_trimHi = 0; g_trimSamples = 0;
    piModelReset();

    // =====================================================================
    // 1. THE REPLY IS WELL FORMED WHEN THE MASTER'S CLOCK *IS* THE DEVICE'S
    //    CLOCK. The degenerate case first, because it pins down the sign
    //    convention and the round-trip bias that every later section leans on.
    //    Handing the device back its own timestamp must produce an error that
    //    is small and NOT POSITIVE: the timestamp was already stale by the time
    //    the sync arrived. A positive answer here would mean the subtraction is
    //    the wrong way round -- which no amount of "the command replied" testing
    //    would ever catch.
    // =====================================================================
    printf("\n--- 1. syncing the device to its own clock ---\n");
    {
        tfhReset(m);
        const int TRIALS = 5;
        bool allOk = true, allNonPositive = true, allSane = true;
        bool trimInRange = true, topBitClear = true;
        for (int i = 0; i < TRIALS; i++) {
            tsSample s = tsSyncAtOffset(m, 0);
            if (!s.ok) { allOk = false; printf("   trial %d: no reply\n", i); break; }
            if (s.err > 0)                          allNonPositive = false;
            if (s.latency > SANE_LATENCY_US)        allSane = false;
            if (s.trim < TRIM_MIN || s.trim > TRIM_MAX) trimInRange = false;
            if (s.icscr & 0x8000)                   topBitClear = false;
            printf("   trial %d: timeError=%ld us  icscr=0x%04X  trim=%u  hsical=%u  round trip=%lu us\n",
                   i, (long)s.err, (unsigned)s.icscr, (unsigned)s.trim,
                   (unsigned)s.hsical, (unsigned long)s.latency);
        }
        TEST_RESULT("every time sync gets a reply of exactly the expected size", allOk);
        TEST_RESULT("handing the device its own timestamp reports a non-positive error",
                    allOk && allNonPositive);
        TEST_RESULT("that error is just the command round trip, well under 150 ms",
                    allOk && allSane);
        TEST_RESULT("the returned oscillator trim is inside the firmware's 62..66 clamp",
                    allOk && trimInRange);
        TEST_RESULT("bit 15 of the returned RCC-ICSCR word is clear (HSITRIM is 7 bits at 14:8)",
                    allOk && topBitClear);
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE NUMBER IS RIGHT: THE REPLY TRACKS AN INJECTED OFFSET ONE FOR ONE.
    //    This is the core of the module. A master timestamp is manufactured at
    //    a KNOWN offset from the device's own clock, so the correct answer is
    //    known in advance: err must equal offset minus one constant round trip.
    //    Nine offsets spanning five seconds either side turn that into a slope
    //    measurement -- if the firmware scaled, clamped or truncated the error,
    //    the residuals would fan out instead of staying flat.
    // =====================================================================
    printf("\n--- 2. the reported error tracks an injected offset one for one ---\n");
    {
        tfhReset(m);
        const int32_t offsets[] = { -5000000, -1000000, -200000, -50000, 0,
                                    50000, 200000, 1000000, 5000000 };
        const int N = sizeof(offsets) / sizeof(offsets[0]);
        int32_t errs[9] = {0};   // zeroed: a short loop leaves the tail unwritten
        bool allOk = true;
        uint32_t latLo = 0xFFFFFFFFu, latHi = 0;
        for (int i = 0; i < N; i++) {
            tsSample s = tsSyncAtOffset(m, (uint32_t)offsets[i]);
            if (!s.ok) { allOk = false; printf("   offset %ld: no reply\n", (long)offsets[i]); break; }
            errs[i] = s.err;
            if (s.latency < latLo) latLo = s.latency;
            if (s.latency > latHi) latHi = s.latency;
            printf("   offset %+9ld us -> timeError %+9ld us (residual round trip %lu us)\n",
                   (long)offsets[i], (long)s.err, (unsigned long)s.latency);
        }
        bool residualsSane = allOk && (latHi <= SANE_LATENCY_US);
        // A slope other than one would fan the residuals out in proportion to the
        // offsets themselves -- millions of microseconds. Requiring the spread to
        // stay inside half the round-trip ceiling is four orders of magnitude
        // tighter than that, while still being a bound on jitter rather than on
        // latency, so it is not implied by the check above.
        bool residualsFlat = allOk && ((latHi - latLo) <= SANE_LATENCY_US / 2);
        // Sign, checked only on the offsets that dwarf any plausible round trip.
        bool aheadPositive = allOk && errs[7] > 0 && errs[8] > 0;
        bool behindNegative = allOk && errs[0] < 0 && errs[1] < 0;
        // Ordering, checked only on the widely spaced subset (-5 s, -1 s, 0,
        // +1 s, +5 s). Adjacent offsets 50 ms apart could legitimately invert
        // under round-trip jitter, and asserting otherwise would be a test bug.
        bool ordered = allOk && errs[0] < errs[1] && errs[1] < errs[4] &&
                       errs[4] < errs[7] && errs[7] < errs[8];
        bool notClamped = allOk &&
                          errs[8] > PI_ERROR_CLAMP_HI * 10 &&
                          errs[0] < PI_ERROR_CLAMP_LO * 10;
        printf("   residual round trip across the sweep: %lu..%lu us\n",
               (unsigned long)latLo, (unsigned long)latHi);
        TEST_RESULT("all nine offset syncs are answered", allOk);
        TEST_RESULT("every reply is the injected offset minus one command round trip",
                    residualsSane);
        TEST_RESULT("that round trip is the same for every offset, so the slope is exactly one",
                    residualsFlat);
        TEST_RESULT("a master ahead of the device reports a positive error", aheadPositive);
        TEST_RESULT("a master behind the device reports a negative error", behindNegative);
        TEST_RESULT("the reported error increases with the injected offset", ordered);
        TEST_RESULT("a five-second offset is reported in full, not clamped to the "
                    "controller's plus or minus 32768 us window", notClamped);
        tfhReset(m);
    }

    // =====================================================================
    // 3. THE MODULAR BOUNDARY AT +/-2^31 MICROSECONDS. The master sends only
    //    the low 32 bits of its clock and the device truncates its own 64-bit
    //    clock to match, so the signed error is only meaningful while the true
    //    offset is inside about 35.8 minutes -- documented in main.c:382-386 and
    //    in the command's own description. That is a real cliff, and a machine
    //    that syncs rarely can fall off it, so the exact behaviour at the edge
    //    is worth recording rather than guessing.
    //
    //    Note that the residual check still applies at the boundary: modular
    //    subtraction means (u32)(offset - err) recovers the same round trip on
    //    BOTH sides of the wrap. If the firmware had used a wider or a saturating
    //    subtraction, this is where it would show.
    // =====================================================================
    printf("\n--- 3. the +/-2^31 microsecond modular boundary ---\n");
    {
        tfhReset(m);
        const uint32_t offNearHalf = 0x40000000u;  // +17.9 minutes, comfortably inside
        const uint32_t offJustUnder = 0x7FF00000u; // +35.77 minutes, just inside
        const uint32_t offJustOver  = 0x80100000u; // just past the wrap
        tsSample a = tsSyncAtOffset(m, offNearHalf);
        tsSample b = tsSyncAtOffset(m, offJustUnder);
        tsSample c = tsSyncAtOffset(m, offJustOver);
        bool allOk = a.ok && b.ok && c.ok;
        printf("   +0x%08lX -> %+11ld us (residual %lu)\n",
               (unsigned long)offNearHalf, (long)a.err, (unsigned long)a.latency);
        printf("   +0x%08lX -> %+11ld us (residual %lu)\n",
               (unsigned long)offJustUnder, (long)b.err, (unsigned long)b.latency);
        printf("   +0x%08lX -> %+11ld us (residual %lu)\n",
               (unsigned long)offJustOver, (long)c.err, (unsigned long)c.latency);
        uint8_t fat = tfhFatal(m);
        printf("   fatal after three extreme offsets: %u\n", (unsigned)fat);
        TEST_RESULT("all three extreme offsets are answered", allOk);
        TEST_RESULT("an offset just inside the boundary is reported as a large positive error",
                    allOk && b.err > 2100000000L);
        TEST_RESULT("an offset just past the boundary wraps to a large negative error",
                    allOk && c.err < -2100000000L);
        TEST_RESULT("modular subtraction still recovers the same round trip on both "
                    "sides of the wrap",
                    allOk && a.latency <= SANE_LATENCY_US &&
                    b.latency <= SANE_LATENCY_US && c.latency <= SANE_LATENCY_US);
        TEST_RESULT("an absurd master timestamp raises no fatal error", fat == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 4. THE SECOND REPLY FIELD IS A REGISTER, AND IT RAILS DETERMINISTICALLY.
    //    `rccIcscr` is not a status word: it is RCC->ICSCR verbatim, so its low
    //    byte is the read-only factory HSICAL and must never change, while the
    //    trim in bits 14:8 is whatever the PI controller just wrote.
    //
    //    With a large error the proportional term (40 x 32767 = 1310680) dwarfs
    //    the whole 62..66 clamp band (about 131072 wide in the same units), so
    //    ONE far-ahead sync must rail the trim to 66 and one far-behind sync
    //    must rail it to 62, regardless of the controller's accumulated history.
    //    That makes the direction of correction checkable: a device behind its
    //    master is running slow and must be trimmed UP.
    // =====================================================================
    printf("\n--- 4. the trim rails up when behind and down when ahead ---\n");
    {
        tfhReset(m);
        bool railHigh = true, railLow = true, ok = true, hsicalConstant = true;
        uint8_t firstHsical = 0;
        bool haveHsical = false;
        for (int cycle = 0; cycle < 3; cycle++) {
            tsSample up = tsSyncAtOffset(m, (uint32_t)(int32_t)2000000);    // master 2 s ahead
            tsSample dn = tsSyncAtOffset(m, (uint32_t)(int32_t)(-2000000)); // master 2 s behind
            if (!up.ok || !dn.ok) { ok = false; break; }
            // The low byte is NOT a constant factory value -- see below. What
            // holds is that it moves with the trim by the same amount, so the
            // difference between the up-railed and down-railed samples is the
            // same for the low byte as for the trim.
            if (!haveHsical) { firstHsical = up.hsical; haveHsical = true; }
            int trimDelta   = (int)up.trim   - (int)dn.trim;
            int hsicalDelta = (int)up.hsical - (int)dn.hsical;
            if (trimDelta != hsicalDelta) hsicalConstant = false;
            if (up.trim != TRIM_MAX) railHigh = false;
            if (dn.trim != TRIM_MIN) railLow = false;
            printf("   cycle %d: master ahead -> trim %u, master behind -> trim %u "
                   "(hsical %u / %u)\n",
                   cycle, (unsigned)up.trim, (unsigned)dn.trim,
                   (unsigned)up.hsical, (unsigned)dn.hsical);
        }
        TEST_RESULT("all six railing syncs are answered", ok);
        TEST_RESULT("a master far ahead drives the trim to its maximum of 66",
                    ok && railHigh);
        TEST_RESULT("a master far behind drives the trim to its minimum of 62",
                    ok && railLow);
        // MEASURED, AND IT CONTRADICTS THE REFERENCE MANUAL.
        //
        // RM0444 describes RCC_ICSCR as HSICAL[7:0] "initialized automatically
        // at startup" (read-only, factory) plus HSITRIM[6:0] as "an additional
        // user-programmable trimming value that is ADDED to the HSICAL bits".
        // Read literally, that says the low byte is constant and only the high
        // byte moves. This module originally asserted exactly that.
        //
        // The silicon does something else. Driving the PI controller to both
        // rails gives:
        //     trim 62 -> icscr 0x3E88  (low byte 136)
        //     trim 66 -> icscr 0x428C  (low byte 140)
        // The low byte moves by the SAME delta as the trim, so what the field
        // reports is the EFFECTIVE calibration (factory + trim), not the raw
        // factory constant.
        //
        // This is MCU behaviour, not a firmware defect: clock_calibration.c:49
        // only ever writes `new_clock_cal_value << RCC_ICSCR_HSITRIM_Pos`, and
        // it reads the register straight back at :55. There is nothing in the
        // firmware that could move the low byte.
        //
        // So the assertion is the relationship that is actually true and still
        // catches a corrupted or half-written register: the two bytes move
        // together, by the same amount.
        TEST_RESULT("the calibration byte tracks the trim by the same delta, "
                    "so it reports the effective calibration rather than the "
                    "raw factory constant",
                    ok && haveHsical && hsicalConstant);
        tfhReset(m);
    }

    // =====================================================================
    // 5. THE CONTROLLER HAS MEMORY, AND ONLY A REBOOT CLEARS IT. `integral_term`
    //    is a FUNCTION-STATIC inside PI_controller (clock_calibration.c:16), so
    //    it persists across every sync and is not touched by `Reset time`
    //    (cmd 8), which only re-zeroes the microsecond counter. That is invisible
    //    from outside unless you look for it, and it matters: a burst of large
    //    errors leaves the oscillator railed even after the errors go away.
    //
    //    Rather than range-checking the trim, this section RECOMPUTES it from
    //    the error the device itself returned, using a line-for-line mirror of
    //    PI_controller, and demands an exact match on every sync. That turns a
    //    vague "looks plausible" into an equality.
    // =====================================================================
    printf("\n--- 5. the trim is exactly the PI controller's output, and a reset clears its memory ---\n");
    {
        tfhReset(m);
        piModelReset();
        const int32_t script[] = { 0, 1000000, 1000000, 1000000, -1000000,
                                   5000, -5000, 200, -200, 0 };
        const int N = sizeof(script) / sizeof(script[0]);
        bool ok = true, modelMatches = true;
        for (int i = 0; i < N; i++) {
            tsSample s = tsSyncAtOffset(m, (uint32_t)script[i]);
            if (!s.ok) { ok = false; break; }
            uint8_t predicted = piModelStep(s.err);
            if (predicted != s.trim) {
                modelMatches = false;
                printf("   step %d: offset %+ld err %+ld -> device trim %u, model %u  MISMATCH\n",
                       i, (long)script[i], (long)s.err, (unsigned)s.trim, (unsigned)predicted);
            }
        }
        printf("   ten scripted syncs: model %s\n", modelMatches ? "matched throughout" : "diverged");

        // Wind the integral up hard with a master fixed 100 seconds ahead. One
        // command per sync (no clock read): the offset is so large that the
        // device clock advancing during the burst cannot change the sign, and
        // every error clamps to +32767, so the integral is predictable exactly.
        bool windOk = true;
        uint8_t windTrim = 0;
        {
            bool clockOk = true;
            uint64_t t = devUs(m, &clockOk);
            uint32_t master = (uint32_t)t + 100000000u;   // +100 s
            for (int i = 0; i < 250 && clockOk; i++) {
                tsSample s = tsSyncRawMaster(m, master);
                if (!s.ok) { windOk = false; break; }
                windTrim = s.trim;
                (void)piModelStep(s.err);
            }
            windOk = windOk && clockOk;
        }
        // With the integral wound far above the clamp band, even a near-zero
        // error must still come back railed at the maximum.
        tsSample afterWind = tsSyncAtOffset(m, 0);
        uint8_t predAfterWind = afterWind.ok ? piModelStep(afterWind.err) : 0;
        printf("   after 250 far-ahead syncs: trim %u; a near-zero-error sync then "
               "returns trim %u (err %+ld us, model %u)\n",
               (unsigned)windTrim, (unsigned)afterWind.trim,
               (long)afterWind.err, (unsigned)predAfterWind);

        // `Reset time` re-zeroes the clock but must NOT clear the controller.
        // Read the clock straight afterwards so the "re-zeroed" half of the
        // claim is actually measured rather than asserted from the name. Only a
        // loose bound is verifiable: the read itself costs a round trip, and
        // reset_time() (motor_control.c:3396) zeroes TIM14 and the software high
        // word, so anything under a few seconds proves it did not keep counting
        // from the ~100 s of clock the wind-up burst ran through.
        m->resetTime();
        bool resetTimeOk = (m->getError() == 0);
        bool clockAfterResetOk = true;
        uint64_t clockAfterReset = devUs(m, &clockAfterResetOk);
        tsSample afterResetTime = tsSyncAtOffset(m, 0);
        uint8_t predAfterResetTime = afterResetTime.ok ? piModelStep(afterResetTime.err) : 0;
        printf("   after Reset time (cmd 8): clock %llu us, trim %u (err %+ld us, model %u)\n",
               (unsigned long long)clockAfterReset,
               (unsigned)afterResetTime.trim, (long)afterResetTime.err,
               (unsigned)predAfterResetTime);

        // A system reset reboots the MCU, which re-initialises the static and
        // restores RCC->ICSCR, so the fresh model must predict again.
        tfhReset(m);
        piModelReset();
        tsSample afterSystemReset = tsSyncAtOffset(m, 0);
        uint8_t predFresh = afterSystemReset.ok ? piModelStep(afterSystemReset.err) : 0;
        printf("   after a system reset: trim %u (err %+ld us, model %u)\n",
               (unsigned)afterSystemReset.trim, (long)afterSystemReset.err,
               (unsigned)predFresh);

        TEST_RESULT("the returned trim is exactly the PI controller's computed output "
                    "on all ten scripted syncs",
                    ok && modelMatches);
        TEST_RESULT("a burst of far-ahead syncs leaves the integral wound up, so a "
                    "near-zero error still returns the railed maximum trim",
                    windOk && afterWind.ok && afterWind.trim == TRIM_MAX &&
                    afterWind.trim == predAfterWind);
        TEST_RESULT("Reset time re-zeroes the clock",
                    resetTimeOk && clockAfterResetOk && clockAfterReset < 5000000ULL);
        TEST_RESULT("Reset time does not clear the controller's memory: the trim is "
                    "still railed afterwards",
                    resetTimeOk && afterResetTime.ok &&
                    afterResetTime.trim == TRIM_MAX &&
                    afterResetTime.trim == predAfterResetTime);
        // NOT "back to 64": from a fresh integral of 64<<16 the very first sync
        // subtracts 41x the round trip, so the reboot value is 62 or 63 for any
        // round trip above ~4 us. Asserting <= 64 is what the arithmetic
        // actually guarantees; the wound-up rail of 66 is what it rules out.
        TEST_RESULT("a system reset clears the controller's memory: the trim drops out "
                    "of the wound-up rail, back into the low half of the band, and the "
                    "fresh model predicts it",
                    afterSystemReset.ok && afterSystemReset.trim <= TRIM_INITIAL &&
                    afterSystemReset.trim == predFresh);
        tfhReset(m);
    }

    // =====================================================================
    // 6. SYNC NEVER STEPS THE CLOCK -- NOT FORWARD, AND CRUCIALLY NOT BACKWARDS.
    //    time_sync() writes RCC->ICSCR and nothing else; the microsecond counter
    //    is never assigned (clock_calibration.c:46-50). So a master 30 seconds
    //    ahead must not make the clock jump 30 seconds, and a master 30 seconds
    //    BEHIND must not rewind it by a single microsecond.
    //
    //    This is the property most worth pinning down, because the intuitive
    //    reading of "time sync" is "set my clock", and an implementation that
    //    did that would break monotonicity, invalidate every in-flight timestamp
    //    and trip the firmware's own ERROR_TIME_WENT_BACKWARDS check (code 1,
    //    microsecond_clock.c:61-63). The answer here is unambiguous: the clock
    //    is disciplined by frequency only, never by stepping.
    // =====================================================================
    printf("\n--- 6. a sync steers the oscillator but never steps the clock ---\n");
    {
        tfhReset(m);
        bool ok = true;

        // A master 30 seconds AHEAD.
        uint64_t t0 = devUs(m, &ok);
        tsSample fwd = tsSyncAtOffset(m, (uint32_t)(int32_t)30000000);
        delay(400);
        uint64_t t1 = devUs(m, &ok);
        int64_t fwdAdvance = (int64_t)(t1 - t0);
        printf("   master +30 s: err %+ld us; clock advanced %lld us over a 400 ms wait\n",
               (long)fwd.err, (long long)fwdAdvance);

        // A master 30 seconds BEHIND.
        uint64_t t2 = devUs(m, &ok);
        tsSample back = tsSyncAtOffset(m, (uint32_t)(int32_t)(-30000000));
        delay(400);
        uint64_t t3 = devUs(m, &ok);
        int64_t backAdvance = (int64_t)(t3 - t2);
        printf("   master -30 s: err %+ld us; clock advanced %lld us over a 400 ms wait\n",
               (long)back.err, (long long)backAdvance);

        // And hammered: thirty backwards syncs interleaved with clock reads.
        bool monotonic = true;
        uint64_t prev = devUs(m, &ok);
        uint64_t firstOfRun = prev;
        for (int i = 0; i < 30 && ok; i++) {
            tsSample s = tsSyncAtOffset(m, (uint32_t)(int32_t)(-30000000));
            if (!s.ok) { ok = false; break; }
            uint64_t now = devUs(m, &ok);
            if (!ok) break;
            if (now < prev) {
                monotonic = false;
                printf("   clock went BACKWARDS after sync %d: %llu -> %llu\n",
                       i, (unsigned long long)prev, (unsigned long long)now);
                break;
            }
            prev = now;
            delay(10);
        }
        uint8_t fat = tfhFatal(m);
        printf("   thirty backwards syncs: clock %llu -> %llu us, fatal %u\n",
               (unsigned long long)firstOfRun, (unsigned long long)prev, (unsigned)fat);

        TEST_RESULT("a master 30 seconds ahead does not jump the clock forward",
                    ok && fwdAdvance > 0 && fwdAdvance < 5000000);
        TEST_RESULT("a master 30 seconds behind does not rewind the clock",
                    ok && backAdvance > 0);
        TEST_RESULT("and the clock keeps running at its normal rate through a backwards sync",
                    ok && backAdvance < 5000000);
        TEST_RESULT("thirty consecutive backwards syncs leave the clock monotonic",
                    ok && monotonic && prev > firstOfRun);
        TEST_RESULT("no backwards sync raises ERROR_TIME_WENT_BACKWARDS (code 1)",
                    fat == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE PROTOCOL'S OWN DUTY CYCLE: 10 Hz, SUSTAINED. The command's
    //    documentation tells callers to sync about ten times a second, forever,
    //    so a long stream of syncs is the NORMAL operating condition rather than
    //    a stress case. Two things must hold: the reported error must not run
    //    away (the PI loop is the only feedback path, and an unstable one would
    //    show as a growing error), and sync must remain an IMMEDIATE command --
    //    if it ever consumed a movement queue slot, a fleet syncing at 10 Hz
    //    would fill the 32-slot queue in about three seconds and fault.
    // =====================================================================
    printf("\n--- 7. sixty syncs at the protocol's recommended rate ---\n");
    {
        tfhReset(m);
        const int N = 60;
        bool allOk = true, queueClean = true;
        int32_t errLo = 0, errHi = 0;
        bool haveErr = false;
        uint8_t worstQueue = 0;
        bool clockOk = true;
        uint64_t tStart = devUs(m, &clockOk);
        for (int i = 0; i < N; i++) {
            tsSample s = tsSyncAtOffset(m, 0);
            if (!s.ok) { allOk = false; printf("   sync %d: no reply\n", i); break; }
            if (!haveErr) { errLo = s.err; errHi = s.err; haveErr = true; }
            else { if (s.err < errLo) errLo = s.err; if (s.err > errHi) errHi = s.err; }
            if ((i % 10) == 0) {
                uint8_t q = m->getNQueuedItems();
                if (m->getError() != 0) { allOk = false; break; }
                if (q > worstQueue) worstQueue = q;
                if (q != 0) queueClean = false;
            }
            delay(15);
        }
        uint64_t tEnd = devUs(m, &clockOk);
        uint8_t fat = tfhFatal(m);
        int32_t spread = haveErr ? (errHi - errLo) : 0;
        printf("   %d syncs: timeError ranged %+ld..%+ld us (spread %ld), worst queue depth %u, "
               "clock advanced %lld us, fatal %u\n",
               N, (long)errLo, (long)errHi, (long)spread, (unsigned)worstQueue,
               (long long)(tEnd - tStart), (unsigned)fat);
        TEST_RESULT("all sixty syncs in the stream are answered", allOk && haveErr);
        TEST_RESULT("the reported error does not run away over a sustained sync stream",
                    haveErr && spread >= 0 && spread < (int32_t)SANE_LATENCY_US);
        TEST_RESULT("a sync stream consumes no movement queue slots", allOk && queueClean);
        TEST_RESULT("the clock keeps advancing through the stream and nothing faults",
                    clockOk && tEnd > tStart && fat == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 8. SYNCING DOES NOT DISTURB A MOVE THAT IS ALREADY RUNNING. In real use
    //    the master syncs continuously WHILE the machine moves, so this is the
    //    combination that actually ships. The command executes immediately
    //    rather than queueing, and its only side effect retunes the oscillator
    //    by up to two trim steps -- which scales the motion tick and the
    //    microsecond clock together, so neither the commanded displacement nor
    //    its duration measured in device time should shift.
    //
    //    Measured against a reference move performed with no syncs at all, so
    //    the comparison is like for like. The queue is drained by POLLING to
    //    zero, never by waiting a computed duration: a wait even slightly short
    //    reads a partial move and looks exactly like a planner bug.
    // =====================================================================
    printf("\n--- 8. a 10 Hz sync stream does not perturb a running move ---\n");
    {
        const int32_t  DIST  = (int32_t)(TFH_CPR / 2);   // half a rotation
        const uint32_t TICKS = 2 * (uint32_t)TFH_TPS;    // two seconds

        // Reference: the same move, undisturbed.
        tfhFreshOpen(m);
        bool clockOk = true;
        uint64_t r0 = devUs(m, &clockOk);
        bool refOk = false;
        int64_t refMoved = tfhMove(m, DIST, TICKS, &refOk);
        uint64_t r1 = devUs(m, &clockOk);
        int64_t refDuration = (int64_t)(r1 - r0);
        printf("   reference move: %lld counts of %ld in %lld us\n",
               (long long)refMoved, (long)DIST, (long long)refDuration);

        // The same move again, syncing throughout.
        tfhFreshOpen(m);
        uint64_t s0 = devUs(m, &clockOk);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(DIST, TICKS);
        bool queued = (m->getError() == 0);
        int syncs = 0;
        bool syncsOk = true;
        if (queued) {
            uint32_t deadline = millis() + (TICKS * 1000UL / (uint32_t)TFH_TPS) + 10000;
            while (millis() < deadline) {
                tsSample s = tsSyncAtOffset(m, 0);
                if (!s.ok) { syncsOk = false; break; }
                syncs++;
                uint8_t q = m->getNQueuedItems();
                if (m->getError() != 0) { syncsOk = false; break; }
                if (q == 0) break;
                delay(20);
            }
        }
        bool drained = queued && syncsOk && tfhDrain(m, 8000);
        uint8_t fat = tfhFatal(m);
        int64_t syncedMoved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
        uint64_t s1 = devUs(m, &clockOk);
        int64_t syncedDuration = (int64_t)(s1 - s0);
        printf("   synced move: %lld counts of %ld in %lld us, with %d syncs delivered, fatal %u\n",
               (long long)syncedMoved, (long)DIST, (long long)syncedDuration, syncs, (unsigned)fat);

        // Durations bracket the polling overhead as well as the move, so compare
        // them loosely (2% of the reference, plus a fixed allowance) -- the sharp
        // claim is the displacement, which is counted, not timed.
        int64_t durDelta = syncedDuration - refDuration;
        if (durDelta < 0) durDelta = -durDelta;
        bool durationClose = refDuration > 0 && durDelta < (refDuration / 50) + 300000;

        // TOLERANCES. Deliberately loose, because the trapezoid planner does NOT
        // land exactly on the commanded displacement: compute_trapezoid_move
        // (motor_control.c:445) truncates `delta_d / ((delta_t1+delta_t2)*delta_t1)`
        // to an integer acceleration, so a move lands a count or so short. The
        // campaign measured 1638399 of a commanded 1638400 and 4095999 of
        // 4096000 -- exactly this move, on the rack. 16 counts of headroom is
        // still five orders of magnitude tighter than any real planner defect.
        //
        // Likewise the two runs are compared with a small window rather than for
        // equality: tm_readonly_purity (48/48 on the rack) measured a spread of
        // up to 2 counts across three IDENTICAL undisturbed runs of this same
        // move, so demanding syncedMoved == refMoved would fail on healthy
        // firmware. 4 matches the tolerance that module already uses for the
        // same "does polling perturb the move" question.
        TEST_RESULT("the undisturbed reference move delivers its full displacement",
                    refOk && llabs((long long)(refMoved - DIST)) <= 16);
        // A dozen, not twenty: each loop pass costs three command round trips
        // plus a 20 ms delay, and in host mode those round trips run at the
        // USB-serial adapter's latency-timer granularity. A dozen syncs across a
        // 2 s move is still a sustained stream and still makes the point.
        TEST_RESULT("at least a dozen syncs land while the move is running",
                    queued && syncsOk && syncs >= 12);
        TEST_RESULT("the move performed under a sync stream delivers the same "
                    "displacement, with no fault",
                    refOk && drained && fat == 0 &&
                    llabs((long long)(syncedMoved - refMoved)) <= 4);
        TEST_RESULT("and it takes the same amount of device time as the reference move",
                    drained && durationClose);
        tfhReset(m);
    }

    // =====================================================================
    // 9. THE MACHINE IS LEFT AS IT WAS FOUND. Every trim written during this
    //    module is inside the firmware's own clamp band, the final reset
    //    restores RCC->ICSCR and the controller's integral, and an ordinary
    //    move still behaves -- which matters on a shared rack.
    // =====================================================================
    printf("\n--- 9. left clean ---\n");
    {
        printf("   trim samples: %lu, observed range %u..%u\n",
               (unsigned long)g_trimSamples, (unsigned)g_trimLo, (unsigned)g_trimHi);
        TEST_RESULT("every trim written across the whole module stayed inside 62..66",
                    g_trimSamples > 0 && g_trimLo >= TRIM_MIN && g_trimHi <= TRIM_MAX);

        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), (uint32_t)TFH_TPS, &ok);
        printf("   post-sweep move: %lld counts of %lld\n",
               (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after the whole sync sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
