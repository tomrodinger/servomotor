#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_motion_timing_precision.cpp
//
// WHEN DOES A MOVE ACTUALLY START, AND WHEN DOES IT ACTUALLY STOP?
//
// Every motion test in this suite measures WHERE the shaft ends up. Not one
// measures WHEN. That leaves the entire time axis of the machine unguarded:
// a move could be accepted promptly and start 50 ms later, or start promptly
// and run 50 ms long, and every existing assertion would still pass, because
// the destination would be identical either way.
//
// For anything that coordinates a motor with something else -- a conveyor, a
// camera trigger, a second axis, an operator's finger on an e-stop -- the
// destination is the easy half. The hard half is that the timing is the SAME
// every time. Determinism, not accuracy on average, is the property here.
//
// HOW THIS DIFFERS FROM WHAT ALREADY EXISTS
//
//   tm_time_domain closes the loop between the clock and the planner, but only
//   coarsely: its check is "a move of N ticks consumes at least N ticks of
//   device clock and not more than N + 31250". A one-second move that took an
//   extra 900 ms would pass. It takes ONE measurement per duration and never
//   looks at the spread.
//
//   tm_determinism repeats identical moves and requires identical DISTANCES.
//   It is the position-domain twin of this module and says nothing about time.
//
//   tm_command_latency_profile times how long individual COMMANDS take on the
//   wire. That is the transport; this is the mechanism. A command can be
//   answered in a millisecond and still leave the motion it requested starting
//   late, and the two are measured with different instruments here: round-trip
//   cost is the RESOLUTION of this module (section 1), not its subject.
//
// This module measures the time axis directly, repeats every measurement ten
// or twelve times, and reports min / median / max explicitly so that the
// SPREAD -- the jitter -- is the headline number rather than an average.
//
// THE MEASUREMENT, AND WHY IT IS HONEST
//
// All timing is taken from the DEVICE's own clock (`Get current time`, command
// 9, which returns microseconds -- main.c:367..378, sourced from TIM14
// prescaled to 1 us in common_source_files/microsecond_clock.c:15). The host's
// millis() is used only to bound loops. That removes host scheduling and the
// USB stack from every number reported below.
//
// What it cannot remove is the RS485 round trip: a clock reading is a request
// and a reply, so the instant it describes is only known to within one round
// trip. Section 1 therefore MEASURES the round trip first and every later
// tolerance is expressed as a multiple of it. That is the module's resolution,
// stated up front instead of assumed.
//
// Two tricks buy precision far below that resolution:
//
//   1. A CONSTANT-VELOCITY SEGMENT IS A CLOCK. `Move with velocity` (command
//      26) queues one item whose velocity is held for exactly its duration
//      (motor_control.c:2140..2146 consumes one tick per control interrupt),
//      and the commanded position advances by exactly velocity/2^20 counts per
//      tick (add_to_queue shifts the raw parameter left by VELOCITY_SHIFT_LEFT
//      = 12, motor_control.c:1842, and motor_control.c:2191 adds it into a
//      96-bit accumulator whose low 32 bits are fractional). Choosing a raw
//      velocity of 104857600 gives EXACTLY 100 counts per tick with no
//      fractional part at all, so the commanded position is a perfect,
//      noise-free ruler over time. Sampling it twice lets the start instant be
//      back-extrapolated instead of caught in the act.
//
//   2. AN EMERGENCY STOP FREEZES THAT RULER. `Emergency stop` (command 12)
//      zeroes the velocity synchronously with interrupts masked
//      (motor_control.c:3405..3410), so the commanded position stops on an
//      exact control tick and can then be read at leisure. The final
//      displacement divided by 100 is the exact number of ticks the move ran --
//      a 32 microsecond measurement recovered from a millisecond-resolution
//      link.
//
// SAFETY: the MOSFETs stay OFF for the whole module, and the only stage that
// touches them at all is the emergency stop, which DISABLES them. Every
// measurement reads the COMMANDED position, which the planner advances whether
// or not the output stage is energised, so nothing turns on any motor at any
// point. No broadcasts, no alias changes, no test modes: safe on the shared
// 35-motor rack.
// ---------------------------------------------------------------------------

// A raw velocity of 104857600 is exactly 100 counts per control tick:
// (104857600 << 12) == 100 * 2^32, so each tick adds exactly 100.000000 counts
// to the 96-bit position accumulator. No rounding, ever. In user units this is
// 0.954 rotations/second -- far below the 15 rot/s ceiling tfhOpenCeilings sets.
static const int32_t TIMING_VEL_RAW = 104857600;
static const int64_t TIMING_CPT     = 100;      // counts per tick at that velocity
static const int64_t US_PER_TICK    = 32;       // 31250 ticks/s -> 32 us exactly

// Read the device clock in microseconds. `ok` is only ever cleared, never set,
// so a single flag can carry the health of a whole trial.
static uint64_t timingClockUs(Servomotor* m, bool* ok) {
    uint64_t t = m->getCurrentTimeRaw();
    if (m->getError() != 0) { *ok = false; return 0; }
    return t;
}

// Poll the queue depth as fast as the link allows and capture the device clock
// the instant it first reads zero. Deliberately NOT tfhDrain: that one sleeps
// 20 ms between polls and 150 ms afterwards, which is fine for "did it finish"
// but destroys any timing resolution. Resolution here is one round trip, which
// section 1 measures.
static bool timingTightDrain(Servomotor* m, uint32_t budgetMs, uint64_t* stopClock) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) {
            bool ok = true;
            uint64_t t = timingClockUs(m, &ok);
            if (!ok) return false;
            *stopClock = t;
            return true;
        }
    }
    return false;
}

static void timingSort(double* v, int n) {
    for (int i = 1; i < n; i++) {
        double key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }
}

// Print min / median / max / spread and hand the same numbers back. The middle
// value is used as the median for even n; with 10-12 samples the distinction is
// not worth the arithmetic.
static void timingReport(const char* what, const char* unit,
                         const double* raw, int n,
                         double* lo, double* med, double* hi) {
    double s[32];
    if (n > 32) n = 32;
    for (int i = 0; i < n; i++) s[i] = raw[i];
    timingSort(s, n);
    *lo = s[0];
    *med = s[n / 2];
    *hi = s[n - 1];
    printf("   %s (n=%d): min %.2f %s | median %.2f %s | max %.2f %s | SPREAD %.2f %s\n",
           what, n, *lo, unit, *med, unit, *hi, unit, *hi - *lo, unit);
}

// How many of the samples sit within `tol` of the median. An outlier-tolerant
// tightness measure: one hiccup from the host's USB stack should not be able to
// fail a claim about the firmware's determinism, but three of them should.
static int timingWithin(const double* raw, int n, double med, double tol) {
    int c = 0;
    for (int i = 0; i < n; i++) {
        double d = raw[i] - med;
        if (d < 0) d = -d;
        if (d <= tol) c++;
    }
    return c;
}

// ---------------------------------------------------------------------------
// The spread that gets ASSERTED, as opposed to the one that gets PRINTED.
//
// A raw min-to-max range over ten or twelve samples is decided entirely by its
// two most extreme members, so ONE hiccup anywhere outside the firmware -- the
// USB CDC write that every library call makes, the host's serial driver, the
// OS scheduler -- fails a claim that is supposed to be about the firmware.
// That is exactly the failure mode this module's own comments warn about, and
// it is the one thing a determinism test must not do.
//
// So: the raw min / median / max / spread are still printed in full above each
// of these assertions and nothing is hidden, but what is asserted is the spread
// after discarding the single lowest and the single highest sample. Two or more
// genuinely scattered samples still widen it, and the companion `timingWithin`
// count covers the tails, so a real loss of determinism is still caught.
// ---------------------------------------------------------------------------
static double timingTrimmedSpread(const double* raw, int n) {
    if (n < 3) return 0.0;
    double s[32];
    if (n > 32) n = 32;
    for (int i = 0; i < n; i++) s[i] = raw[i];
    timingSort(s, n);
    return s[n - 2] - s[1];
}

// An absolute worst-case ceiling that is never tighter than the link it is
// measured over. Section 1 permits a single round trip of up to 200 ms; a
// fixed 100 ms cap on a quantity that CONTAINS a round trip would then be
// self-contradictory on a slow host link. Scale it, so the claim stays sharp
// on a fast link and merely stays true on a slow one.
static double timingOutlierCap(double baseUs, double roundTripUs) {
    double scaled = 20.0 * roundTripUs;
    return (scaled > baseUs) ? scaled : baseUs;
}

void tm_motion_timing_precision(void) {
    Serial.println("tm_motion_timing_precision: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // The measured cost of one command round trip, in device-clock
    // microseconds. Section 1 fills it in; every later tolerance is scaled by
    // it so the module is equally valid on the ESP32-S3's direct RS485 link and
    // on a Mac talking through a USB-serial adapter.
    double roundTripUs = 2000.0;

    // =====================================================================
    // 1. THE INSTRUMENT. Before measuring anything about motion, measure the
    //    resolution of the measuring device: how much device time passes
    //    between two consecutive clock readings. That interval IS one command
    //    round trip, and it is the floor below which no latency here can be
    //    resolved. Stating it first is the difference between a timing test
    //    and a timing anecdote.
    // =====================================================================
    printf("\n--- 1. how much device time one command round trip costs ---\n");
    {
        tfhReset(m);
        const int N = 20;
        uint64_t stamps[N];
        bool ok = true;
        for (int i = 0; i < N; i++) stamps[i] = timingClockUs(m, &ok);

        double gaps[N - 1];
        bool allForward = true;
        for (int i = 1; i < N; i++) {
            int64_t d = (int64_t)stamps[i] - (int64_t)stamps[i - 1];
            if (d <= 0) allForward = false;
            gaps[i - 1] = (double)d;
        }
        double lo = 0, med = 0, hi = 0;
        timingReport("round trip", "us", gaps, N - 1, &lo, &med, &hi);
        if (ok && med > 0) roundTripUs = med;
        printf("   -> the resolution of every latency below is about %.0f us "
               "(%.1f control ticks)\n", roundTripUs, roundTripUs / (double)US_PER_TICK);

        TEST_RESULT("all twenty device-clock reads succeed", ok);
        TEST_RESULT("the device clock advances between every consecutive pair of reads",
                    ok && allForward);
        TEST_RESULT("the typical command round trip costs less than 50 ms",
                    ok && med < 50000.0);
        TEST_RESULT("no single round trip costs more than 200 ms",
                    ok && hi < 200000.0);
        tfhReset(m);
    }

    // =====================================================================
    // 2. START LATENCY. How long after a move command is accepted does the
    //    commanded position first begin to move?
    //
    //    Catching the first change directly would only resolve to one round
    //    trip. Instead the constant-velocity segment is used as a ruler: two
    //    position samples, each bracketed by clock readings, give both the
    //    realised speed and -- by extrapolating the straight line back to zero
    //    displacement -- the instant motion began. The long baseline between
    //    the two samples makes the slope essentially exact, so the residual
    //    error is one bracket rather than one bracket per sample.
    //
    //    Twelve trials, because a single latency number is meaningless: the
    //    claim being tested is that it is the SAME latency every time.
    // =====================================================================
    printf("\n--- 2. the latency from accepting a move to the position moving ---\n");
    {
        tfhFreshOpen(m);
        const int TRIALS = 12;
        const uint32_t DUR = (uint32_t)(2.5 * TFH_TPS);   // 2.5 s of constant velocity
        double latency[TRIALS];
        double rateErr[TRIALS];
        int good = 0;

        for (int t = 0; t < TRIALS; t++) {
            bool ok = true;
            int64_t posBefore = m->getPositionRaw();
            if (m->getError() != 0) ok = false;

            uint64_t tRef = timingClockUs(m, &ok);          // reference: before the command
            m->moveWithVelocityRaw(TIMING_VEL_RAW, DUR);
            if (m->getError() != 0) ok = false;
            // The trailing zero-velocity segment is queued IMMEDIATELY, not
            // later: a velocity segment holds its velocity after expiry, and a
            // queue that empties at speed raises ERROR_RUN_OUT_OF_QUEUE_ITEMS
            // (18). Queueing it first means no error path below can leave the
            // machine running.
            m->moveWithVelocityRaw(0, 2);
            if (m->getError() != 0) ok = false;

            uint64_t a1 = timingClockUs(m, &ok);
            int64_t  p1 = m->getPositionRaw() - posBefore;
            if (m->getError() != 0) ok = false;
            uint64_t b1 = timingClockUs(m, &ok);

            delay(1800);

            uint64_t a2 = timingClockUs(m, &ok);
            int64_t  p2 = m->getPositionRaw() - posBefore;
            if (m->getError() != 0) ok = false;
            uint64_t b2 = timingClockUs(m, &ok);

            if (!tfhDrain(m, 8000)) ok = false;
            if (tfhFatal(m) != 0) ok = false;

            double mid1 = ((double)a1 + (double)b1) / 2.0;
            double mid2 = ((double)a2 + (double)b2) / 2.0;
            double span = mid2 - mid1;
            if (ok && span > 1000.0 && p2 > p1) {
                double rate = (double)(p2 - p1) / span;          // counts per microsecond
                double startUs = mid1 - (double)p1 / rate;       // extrapolate back to zero
                latency[good] = startUs - (double)tRef;
                rateErr[good] = rate / ((double)TIMING_CPT / (double)US_PER_TICK) - 1.0;
                good++;
            }
        }

        double lo = 0, med = 0, hi = 0, rLo = 0, rMed = 0, rHi = 0;
        if (good > 0) {
            timingReport("start latency", "us", latency, good, &lo, &med, &hi);
            double rPct[TRIALS];
            for (int i = 0; i < good; i++) rPct[i] = rateErr[i] * 100.0;
            timingReport("realised speed error", "%", rPct, good, &rLo, &rMed, &rHi);
            printf("   -> median start latency is %.2f ms (%.1f control ticks), "
                   "spread %.2f ms\n", med / 1000.0, med / (double)US_PER_TICK,
                   (hi - lo) / 1000.0);
        }

        double jitterCap = 20000.0;
        if (12.0 * roundTripUs > jitterCap) jitterCap = 12.0 * roundTripUs;

        // The realised rate is (p2-p1)/(mid2-mid1), and each `mid` locates a
        // position sample only to within about half a round trip, so the
        // fractional error in the rate is bounded by two round trips over the
        // ~1.8 s baseline -- NOT by anything the firmware does. On the ESP32's
        // direct RS485 link that is under 0.2 percent and 0.5 stands; on a Mac
        // USB-serial adapter whose latency timer can make a round trip 16 ms,
        // 0.5 percent is BELOW the measurement's own resolution and a healthy
        // motor would fail it. Take whichever is larger, so the tolerance never
        // claims more precision than the instrument has.
        double rateTolPct = 0.5;
        double resolutionPct = 100.0 * 2.0 * roundTripUs / 1800000.0;
        if (resolutionPct > rateTolPct) rateTolPct = resolutionPct;

        TEST_RESULT("all twelve velocity moves are accepted and complete cleanly",
                    good == TRIALS);
        printf("   realised-speed tolerance in force: %.3f percent "
               "(0.5 percent, or the two-round-trip resolution if that is coarser)\n",
               rateTolPct);
        TEST_RESULT("the realised speed matches the commanded speed to within "
                    "the resolution of the measurement",
                    good == TRIALS && rLo > -rateTolPct && rHi < rateTolPct);
        // Resolution-limited: the extrapolated start instant is only known to
        // about one round trip, so "never before the command" is asserted with
        // exactly that tolerance rather than pretending to more precision.
        TEST_RESULT("motion never begins before the command that asked for it, "
                    "to within the one-round-trip resolution",
                    good == TRIALS && lo > -roundTripUs);
        // The typical latency is a small multiple of one round trip by
        // construction (the reply to tRef, then the move command, then the
        // planner's next tick), so the ceiling must not sit below what the link
        // itself costs -- section 1 permits a median round trip of up to 50 ms.
        double medianCap = 50000.0;
        if (6.0 * roundTripUs > medianCap) medianCap = 6.0 * roundTripUs;
        TEST_RESULT("the median start latency is positive and under 50 ms "
                    "(or six round trips, whichever is longer)",
                    good == TRIALS && med > 0.0 && med < medianCap);
        double startCap = timingOutlierCap(100000.0, roundTripUs);
        TEST_RESULT("no single start is delayed by more than 100 ms "
                    "(or twenty round trips, whichever is longer)",
                    good == TRIALS && hi < startCap);
        // Trimmed rather than raw min-to-max: see timingTrimmedSpread.
        double latSpread = timingTrimmedSpread(latency, good);
        printf("   trimmed start-latency spread %.2f ms against a cap of %.2f ms\n",
               latSpread / 1000.0, jitterCap / 1000.0);
        TEST_RESULT("the start latency is repeatable: its spread is bounded once the "
                    "single fastest and single slowest trial are discarded",
                    good == TRIALS && latSpread < jitterCap);
        int tight = (good > 0) ? timingWithin(latency, good, med, jitterCap / 2.0) : 0;
        printf("   %d of %d starts lie within %.1f ms of the median\n",
               tight, good, jitterCap / 2000.0);
        TEST_RESULT("at least ten of the twelve starts cluster tightly about the median",
                    tight >= 10);
        tfhReset(m);
    }

    // =====================================================================
    // 3. DURATION, IN THE POSITION DOMAIN, WHERE IT IS EXACT. The clock-domain
    //    measurement in section 4 is limited by the link. This one is not:
    //    because the chosen velocity advances the commanded position by
    //    exactly 100 counts per tick, a segment that ran for D ticks MUST end
    //    up exactly 100*D counts further on. The final displacement is
    //    therefore a tick counter with no measurement error whatsoever, and it
    //    is read after the move, at leisure.
    //
    //    A single count of discrepancy here would mean the planner ran the
    //    segment for the wrong number of control ticks.
    // =====================================================================
    printf("\n--- 3. a segment of D ticks runs for exactly D ticks ---\n");
    {
        const uint32_t durations[] = { 1000, 12500, 31250, 62500 };
        for (unsigned i = 0; i < sizeof(durations) / sizeof(durations[0]); i++) {
            tfhFreshOpen(m);
            uint32_t D = durations[i];
            int64_t before = m->getPositionRaw();
            bool ok = (m->getError() == 0);
            m->moveWithVelocityRaw(TIMING_VEL_RAW, D);
            if (m->getError() != 0) ok = false;
            m->moveWithVelocityRaw(0, 2);
            if (m->getError() != 0) ok = false;
            if (!tfhDrain(m, (uint32_t)((uint64_t)D * 1000ULL / TFH_TPS) + 10000)) ok = false;
            if (tfhFatal(m) != 0) ok = false;
            int64_t moved = ok ? (m->getPositionRaw() - before) : -1;
            int64_t want = (int64_t)D * TIMING_CPT;
            printf("   %lu ticks commanded -> %lld counts, expected %lld (error %lld)\n",
                   (unsigned long)D, (long long)moved, (long long)want,
                   (long long)(moved - want));
            TEST_RESULT(std::string("a velocity segment of ") +
                            std::to_string((unsigned long)D) +
                            " ticks advances the commanded position by exactly that many ticks",
                        ok && moved == want);
        }

        // And the tick count is not merely right once. Ten identical segments
        // must agree EXACTLY -- a spread of zero counts, which is a spread of
        // zero ticks. This is the sharpest determinism claim the module makes.
        tfhFreshOpen(m);
        const int REPS = 10;
        const uint32_t D = TFH_TPS;         // one second
        int64_t moved[REPS];
        bool allOk = true;
        for (int r = 0; r < REPS; r++) {
            int64_t before = m->getPositionRaw();
            if (m->getError() != 0) { allOk = false; break; }
            m->moveWithVelocityRaw(TIMING_VEL_RAW, D);
            if (m->getError() != 0) { allOk = false; break; }
            m->moveWithVelocityRaw(0, 2);
            if (m->getError() != 0) { allOk = false; break; }
            if (!tfhDrain(m, 12000)) { allOk = false; break; }
            if (tfhFatal(m) != 0) { allOk = false; break; }
            moved[r] = m->getPositionRaw() - before;
        }
        bool identical = allOk;
        for (int r = 1; r < REPS && identical; r++) if (moved[r] != moved[0]) identical = false;
        if (allOk) {
            printf("   ten one-second segments: first %lld counts, last %lld counts, identical=%d\n",
                   (long long)moved[0], (long long)moved[REPS - 1], (int)identical);
        }
        TEST_RESULT("ten identical one-second segments all complete", allOk);
        TEST_RESULT("all ten run for exactly the same number of control ticks "
                    "(a spread of zero counts)",
                    identical && moved[0] == (int64_t)D * TIMING_CPT);
        tfhReset(m);
    }

    // =====================================================================
    // 4. DURATION, IN THE CLOCK DOMAIN, WITH ITS SPREAD. Section 3 proves the
    //    planner runs the right number of ticks. This asks the complementary
    //    question: does the wall time the caller experiences match, and is the
    //    overshoot repeatable?
    //
    //    The overshoot is not firmware slop -- it is the command round trip at
    //    each end, which is why section 1 had to be measured first. What
    //    matters is that it is BOUNDED and STABLE. A machine whose moves
    //    sometimes take 20 ms longer than asked can be scheduled around; one
    //    that sometimes takes 300 ms longer cannot.
    // =====================================================================
    printf("\n--- 4. the elapsed device clock over a move, ten times ---\n");
    {
        tfhFreshOpen(m);
        const int REPS = 10;
        const uint32_t D = TFH_TPS;                  // one second
        const double commandedUs = (double)(D + 2) * (double)US_PER_TICK;
        double residual[REPS];
        int good = 0;
        for (int r = 0; r < REPS; r++) {
            bool ok = true;
            uint64_t tRef = timingClockUs(m, &ok);
            m->moveWithVelocityRaw(TIMING_VEL_RAW, D);
            if (m->getError() != 0) ok = false;
            m->moveWithVelocityRaw(0, 2);
            if (m->getError() != 0) ok = false;
            // Sleep through most of the move, then poll hard: flooding the bus
            // for the whole second would be pointless, and tm_readonly_purity
            // already established that reads do not disturb a move.
            delay((uint32_t)((uint64_t)D * 1000ULL / TFH_TPS) - 150);
            uint64_t tDone = 0;
            if (ok && timingTightDrain(m, 8000, &tDone)) {
                if (tfhFatal(m) == 0) {
                    residual[good] = (double)((int64_t)tDone - (int64_t)tRef) - commandedUs;
                    good++;
                }
            }
            delay(150);
        }

        double lo = 0, med = 0, hi = 0;
        if (good > 0) {
            timingReport("duration overshoot", "us", residual, good, &lo, &med, &hi);
            printf("   -> commanded %.1f ms, median measured %.1f ms, spread %.2f ms\n",
                   commandedUs / 1000.0, (commandedUs + med) / 1000.0, (hi - lo) / 1000.0);
        }

        double jitterCap = 20000.0;
        if (12.0 * roundTripUs > jitterCap) jitterCap = 12.0 * roundTripUs;

        double overrunCap = timingOutlierCap(150000.0, roundTripUs);
        double durSpread = timingTrimmedSpread(residual, good);

        TEST_RESULT("all ten timed moves complete without a fault", good == REPS);
        TEST_RESULT("no move ever finishes sooner than the duration it was given",
                    good == REPS && lo > -roundTripUs);
        TEST_RESULT("no move overruns its commanded duration by more than 150 ms "
                    "(or twenty round trips, whichever is longer)",
                    good == REPS && hi < overrunCap);
        // Trimmed rather than raw min-to-max: see timingTrimmedSpread.
        printf("   trimmed overshoot spread %.2f ms against a cap of %.2f ms\n",
               durSpread / 1000.0, jitterCap / 1000.0);
        TEST_RESULT("the duration overshoot is repeatable: its spread is bounded once the "
                    "single shortest and single longest run are discarded",
                    good == REPS && durSpread < jitterCap);
        tfhReset(m);
    }

    // =====================================================================
    // 5. WHEN DOES A MOVE STOP? An emergency stop zeroes the velocity with
    //    interrupts masked (motor_control.c:3405), so the commanded position
    //    freezes on an exact control tick and can be read afterwards at
    //    leisure. Dividing the frozen displacement by 100 recovers the exact
    //    number of ticks the move ran -- a 32 microsecond measurement taken
    //    over a millisecond-resolution link.
    //
    //    Two properties fall out. First, the displacement must be an exact
    //    multiple of 100: if it were not, the stop would have landed
    //    mid-tick, which the firmware's structure forbids. Second, repeating
    //    an identical "run, wait, stop" sequence ten times must run an almost
    //    identical number of ticks -- and THAT spread is end-to-end timing
    //    jitter measured in control ticks, with no extrapolation anywhere.
    // =====================================================================
    printf("\n--- 5. an emergency stop freezes the position on an exact tick ---\n");
    {
        tfhFreshOpen(m);
        const int REPS = 10;
        double ticksRun[REPS];
        double deadTime[REPS];
        int good = 0;
        bool allTickAligned = true;
        bool stayedStopped = true;
        bool noFault = true;

        for (int r = 0; r < REPS; r++) {
            bool ok = true;
            int64_t before = m->getPositionRaw();
            if (m->getError() != 0) ok = false;
            uint64_t tRef = timingClockUs(m, &ok);
            // Five seconds of runway plus a terminator, so that a failure
            // anywhere below still ends in a stopped machine rather than a
            // queue underrun.
            m->moveWithVelocityRaw(TIMING_VEL_RAW, 5 * TFH_TPS);
            if (m->getError() != 0) ok = false;
            m->moveWithVelocityRaw(0, 2);
            if (m->getError() != 0) ok = false;

            // Let it run for 400 ms of DEVICE time, not host time. A plain
            // delay(400) would fold the host scheduler's jitter into a number
            // that is supposed to characterise the firmware; gating on the
            // device's own clock leaves only one round trip of uncertainty.
            // The host sleep covers most of the interval so the tight poll
            // floods the bus only over the last few tens of milliseconds.
            delay(350);
            for (uint32_t guard = millis() + 3000; millis() < guard; ) {
                uint64_t now = timingClockUs(m, &ok);
                if (!ok) break;
                if ((int64_t)(now - tRef) >= 400000) break;
            }
            m->emergencyStop();
            if (m->getError() != 0) ok = false;
            uint64_t tAfter = timingClockUs(m, &ok);

            delay(200);
            int64_t frozen = m->getPositionRaw() - before;
            if (m->getError() != 0) ok = false;
            delay(200);
            int64_t frozenAgain = m->getPositionRaw() - before;
            if (m->getError() != 0) ok = false;
            if (frozen != frozenAgain) stayedStopped = false;
            if (tfhFatal(m) != 0) noFault = false;
            if (frozen % TIMING_CPT != 0) allTickAligned = false;

            if (ok && frozen > 0) {
                ticksRun[good] = (double)(frozen / TIMING_CPT);
                // Everything between the reference clock read and the
                // post-stop clock read that was NOT spent moving: the start
                // latency plus the stop's own detection lag. Both are
                // positive, so this must be too.
                deadTime[good] = (double)((int64_t)tAfter - (int64_t)tRef)
                                 - (double)(frozen / TIMING_CPT) * (double)US_PER_TICK;
                good++;
            }
        }

        double tLo = 0, tMed = 0, tHi = 0, dLo = 0, dMed = 0, dHi = 0;
        if (good > 0) {
            timingReport("ticks actually run", "ticks", ticksRun, good, &tLo, &tMed, &tHi);
            timingReport("dead time around the move", "us", deadTime, good, &dLo, &dMed, &dHi);
            printf("   -> the tick spread is %.0f ticks = %.2f ms of end-to-end jitter\n",
                   tHi - tLo, (tHi - tLo) * (double)US_PER_TICK / 1000.0);
        }

        // 625 ticks is 20 ms. The interval is gated on the device clock, so the
        // only unmodelled term left is the round trip of the last poll plus the
        // stop command's own transit -- a couple of round trips at most.
        double tickJitterCap = 625.0;
        if (12.0 * roundTripUs / (double)US_PER_TICK > tickJitterCap)
            tickJitterCap = 12.0 * roundTripUs / (double)US_PER_TICK;
        double deadCap = timingOutlierCap(150000.0, roundTripUs);
        double tickSpread = timingTrimmedSpread(ticksRun, good);

        TEST_RESULT("all ten emergency stops leave a stopped machine with no fault",
                    good == REPS && noFault);
        TEST_RESULT("the commanded position always freezes on an exact control tick",
                    good == REPS && allTickAligned);
        TEST_RESULT("the commanded position does not creep after the stop",
                    stayedStopped);
        TEST_RESULT("no time is unaccounted for: the dead time around a move is never negative",
                    good == REPS && dLo > -roundTripUs);
        TEST_RESULT("the dead time around a move stays under 150 ms "
                    "(or twenty round trips, whichever is longer)",
                    good == REPS && dHi < deadCap);
        // Trimmed rather than raw min-to-max: see timingTrimmedSpread. The
        // instant the stop lands is set by the last poll of a host-driven loop,
        // so one scheduler stall shows up here as a lone outlier.
        printf("   trimmed tick spread %.0f ticks against a cap of %.0f ticks\n",
               tickSpread, tickJitterCap);
        TEST_RESULT("ten identical run-then-stop sequences run a repeatable number of ticks "
                    "once the single shortest and single longest are discarded",
                    good == REPS && tickSpread < tickJitterCap);
        tfhReset(m);
    }

    // =====================================================================
    // 6. A QUEUED MOVE HAS NO START LATENCY AT ALL. Everything above measures
    //    the COMMAND path. The planner itself is a different thing: a segment
    //    already sitting in the queue starts on the very tick the one before
    //    it finishes, with no gap and no overlap.
    //
    //    That is provable exactly, in the position domain: two segments of
    //    known rate and duration must deliver exactly the sum of their two
    //    displacements. One lost tick between them would show up as 100 counts
    //    missing; one double-counted tick as 100 counts extra.
    //
    //    It is also the practical answer to the latency measured in section 2 --
    //    pre-queue the motion and the command path stops mattering.
    // =====================================================================
    printf("\n--- 6. back-to-back queued segments join with no gap ---\n");
    {
        tfhFreshOpen(m);
        const int REPS = 5;
        const uint32_t D1 = 10000, D2 = 10000;
        const int64_t want = (int64_t)D1 * TIMING_CPT + (int64_t)D2 * TIMING_CPT * 2;
        int64_t total[REPS];
        bool allOk = true;
        for (int r = 0; r < REPS; r++) {
            int64_t before = m->getPositionRaw();
            if (m->getError() != 0) { allOk = false; break; }
            m->moveWithVelocityRaw(TIMING_VEL_RAW, D1);          // 100 counts/tick
            if (m->getError() != 0) { allOk = false; break; }
            m->moveWithVelocityRaw(TIMING_VEL_RAW * 2, D2);      // 200 counts/tick
            if (m->getError() != 0) { allOk = false; break; }
            m->moveWithVelocityRaw(0, 2);
            if (m->getError() != 0) { allOk = false; break; }
            if (!tfhDrain(m, 12000)) { allOk = false; break; }
            if (tfhFatal(m) != 0) { allOk = false; break; }
            total[r] = m->getPositionRaw() - before;
        }
        bool identical = allOk;
        for (int r = 1; r < REPS && identical; r++) if (total[r] != total[0]) identical = false;
        if (allOk) {
            printf("   chained pair delivered %lld counts, expected %lld (error %lld), identical=%d\n",
                   (long long)total[0], (long long)want,
                   (long long)(total[0] - want), (int)identical);
        }
        TEST_RESULT("all five chained pairs complete", allOk);
        TEST_RESULT("a chained pair delivers exactly the sum of its two segments, "
                    "so no tick is lost or repeated at the join",
                    allOk && total[0] == want);
        TEST_RESULT("all five chained pairs deliver the identical total", identical);
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE SAME QUESTION FOR A TRAPEZOID MOVE. A trapezoid takes a
    //    completely different path through the planner: compute_trapezoid_move
    //    derives an acceleration and a ramp length, then three queued segments
    //    (or two, on the one-tick-ramp fast path) are emitted -- and they must
    //    still sum to exactly the commanded total time: delta_t1 + delta_t2 +
    //    delta_t1 with delta_t2 = total_time - 2*delta_t1
    //    (motor_control.c:2039..2041, and :2034..2035 on the fast path where
    //    total_time-1 and 1 are queued). Timing determinism has to hold for the
    //    command users actually reach for, not only for the one that makes the
    //    measurement convenient.
    // =====================================================================
    printf("\n--- 7. ten identical trapezoid moves, timed ---\n");
    {
        tfhFreshOpen(m);
        const int REPS = 10;
        const int32_t DIST = 1000000;
        const uint32_t D = TFH_TPS;                 // one second
        const double commandedUs = (double)D * (double)US_PER_TICK;
        double residual[REPS];
        int64_t moved[REPS];
        int good = 0;
        for (int r = 0; r < REPS; r++) {
            bool ok = true;
            int64_t before = m->getPositionRaw();
            if (m->getError() != 0) ok = false;
            uint64_t tRef = timingClockUs(m, &ok);
            m->trapezoidMoveRaw(DIST, D);
            if (m->getError() != 0) ok = false;
            delay((uint32_t)((uint64_t)D * 1000ULL / TFH_TPS) - 150);
            uint64_t tDone = 0;
            if (ok && timingTightDrain(m, 8000, &tDone) && tfhFatal(m) == 0) {
                residual[good] = (double)((int64_t)tDone - (int64_t)tRef) - commandedUs;
                moved[good] = m->getPositionRaw() - before;
                good++;
            }
            delay(150);
        }

        double lo = 0, med = 0, hi = 0;
        int64_t dMin = 0, dMax = 0;
        for (int r = 0; r < good; r++) {
            if (r == 0) { dMin = dMax = moved[0]; }
            else { if (moved[r] < dMin) dMin = moved[r]; if (moved[r] > dMax) dMax = moved[r]; }
        }
        if (good > 0) {
            timingReport("trapezoid duration overshoot", "us", residual, good, &lo, &med, &hi);
            printf("   trapezoid displacement %lld..%lld of %ld commanded (spread %lld counts)\n",
                   (long long)dMin, (long long)dMax, (long)DIST, (long long)(dMax - dMin));
        }

        double jitterCap = 20000.0;
        if (12.0 * roundTripUs > jitterCap) jitterCap = 12.0 * roundTripUs;

        TEST_RESULT("all ten trapezoid moves complete without a fault", good == REPS);
        // A trapezoid's acceleration is an integer division, so a handful of
        // counts of rounding is expected and legitimate.
        TEST_RESULT("every trapezoid move lands within a few counts of its request",
                    good == REPS && llabs((long long)(dMin - DIST)) <= 8
                                 && llabs((long long)(dMax - DIST)) <= 8);
        // NOT "identical". The commanded position is a 96-bit accumulator whose
        // low 32 bits are FRACTIONAL, and that fraction is not reset between
        // moves. compute_trapezoid_move divides delta_d = displacement<<24 by
        // (delta_t1+delta_t2)*delta_t1 and truncates (motor_control.c:485), so
        // each move under-delivers by 256*remainder/2^32 of a count. For this
        // exact request under the ceilings tfhOpenCeilings sets -- 1000000
        // counts in 31250 ticks, delta_t1 = 93, denominator = 2897601,
        // remainder = 1896364 -- that is 0.113 counts per move. The fractional
        // debt crosses an integer boundary roughly every ninth repetition, so
        // the READ-BACK integer displacement is 1000000 on most runs and 999999
        // on the others. A healthy motor cannot deliver ten identical integers
        // here and this assertion failed on correct firmware as written.
        //
        // What IS verifiable, and is still a real determinism claim: the
        // fractional deficit is smaller than one count, so no two runs may ever
        // differ by more than a single count. Anything larger would mean the
        // planner genuinely computed a different profile. (tm_determinism makes
        // the same claim the same way -- "within 2 counts" -- for the same
        // reason.)
        TEST_RESULT("all ten trapezoid moves deliver the same displacement to within the "
                    "one count the sub-count position accumulator can carry",
                    good == REPS && (dMax - dMin) <= 1);
        TEST_RESULT("no trapezoid move finishes sooner than its commanded duration",
                    good == REPS && lo > -roundTripUs);
        // Trimmed rather than raw min-to-max: see timingTrimmedSpread.
        double trapSpread = timingTrimmedSpread(residual, good);
        printf("   trimmed trapezoid overshoot spread %.2f ms against a cap of %.2f ms\n",
               trapSpread / 1000.0, jitterCap / 1000.0);
        TEST_RESULT("the trapezoid duration overshoot is repeatable: its spread is bounded "
                    "once the single shortest and single longest run are discarded",
                    good == REPS && trapSpread < jitterCap);
        tfhReset(m);
    }

    // =====================================================================
    // 8. Still healthy afterwards. The module leans on the emergency stop and
    //    on long constant-velocity runs; neither should leave a mark.
    // =====================================================================
    printf("\n--- 8. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   post-sweep move delivered %lld of %lld counts\n",
               (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after the timing sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
