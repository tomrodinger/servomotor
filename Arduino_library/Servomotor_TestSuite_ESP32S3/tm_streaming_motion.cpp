#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_streaming_motion.cpp
//
// CONTINUOUS MOTION BY REFILLING THE QUEUE WHILE IT DRAINS.
//
// This is how a machine actually follows a path. A CNC axis, a pick-and-place
// head, a camera slider: the controller does not hand the motor one move and
// wait for it. It puts several moves in flight, then keeps TOPPING THE QUEUE UP
// as the motor eats through it, so the shaft never comes to rest between
// segments. Everything the queue exists for is that pattern.
//
// A hundred-odd modules in this suite queue moves. NOT ONE of them streams.
// Every existing motion module falls into one of three shapes, and all three
// miss the case:
//
//   * tm_soak            -- fill the queue, DRAIN IT TO ZERO, repeat. Forty
//                           rounds of exactly the thing streaming must avoid.
//   * tm_queue_stress    -- 30 paced moves, then a deliberate overflow. It
//                           asserts the depth rose above zero and later drained
//                           to zero; it never asserts the depth stayed above
//                           zero, which is the whole property here.
//   * tm_comm_during_motion / tm_queue_ordering
//                        -- one complete plan queued UP FRONT, then observed.
//                           Nothing is added while the queue is running.
//   * tm_queue_accounting / tm_multimove_capacity
//                        -- how many slots a move costs, statically.
//
// So the suite knows the queue can be filled and knows it can be emptied, and
// has never once checked that it can be KEPT FED. The failure modes that live
// only in that regime are exactly the expensive ones: a producer/consumer race
// on n_items_in_queue, a ring index that drifts after many wraps, a move
// silently dropped when a top-up lands in the same control tick as a consume,
// or motion that hitches because the queue momentarily ran dry.
//
// GROUND TRUTH (firmware/Src/motor_control.c, firmware/Src/motor_control.h):
//
//   motor_control.h:31   #define MOVEMENT_QUEUE_SIZE 32
//   motor_control.c:1790 if(n_items_in_queue < MOVEMENT_QUEUE_SIZE) { ...enqueue... }
//   motor_control.c:1869 else fatal_error(ERROR_QUEUE_IS_FULL);          // code 17
//   motor_control.c:2014 queue_trapezoid_segments(): the ordinary path is
//                        :2039-2041 three add_to_queue() calls (ramp, coast,
//                        ramp) -> THREE slots per trapezoid move. The
//                        :2017 one-tick-ramp fast path costs two; this module
//                        deliberately stays in the three-slot regime so the
//                        refill arithmetic is safe (section 1 measures the cost
//                        race-free at slow limits, and bounds it at the stream's
//                        own limits).
//   motor_control.c:2125 handle_queued_movements(), called from the 31.25 kHz
//                        control ISR, consumes ONE TICK of the head item per
//                        call and pops it at :2135 / :2143 when it is spent.
//   motor_control.c:2151-2155  when the queue empties, the planner's
//                        end-of-queue position must equal the live position
//                        (else ERROR_POSITION_DISCREPANCY, 42) and
//                        decelerate_to_stop_active is set.
//   motor_control.c:2158-2164  in that state a NONZERO velocity is fatal:
//                        ERROR_RUN_OUT_OF_QUEUE_ITEMS (18).
//
// That last pair is why a trapezoid stream is allowed to come to rest at all: a
// trapezoid move ramps up by `acceleration` for delta_t1 ticks and back down by
// the same acceleration for delta_t1 ticks, so it ends at EXACTLY zero
// velocity. An empty queue therefore means "at rest", not "runaway", and the
// clean-shutdown assertions in section 4 are testing that contract rather than
// hoping for it.
//
// WHAT THIS MODULE ASSERTS
//   1. the per-move slot cost (exactly, at limits where the reading cannot race
//      the consumer; bounded above at the stream's own limits) and the
//      exactness premise the arithmetic rests on
//   2. a NEGATIVE CONTROL: without top-ups the queue does empty and the motion
//      does stop -- so the section-3 claims are not vacuous
//   3. three hundred moves streamed continuously: the queue never empties, the
//      ring never overfills, no fatal error occurs, the commanded position
//      never stalls for as long as one move, and the trajectory stays inside
//      the band the plan defines
//   4. a clean shutdown: stop topping up, let it drain, and it comes to rest
//      with no underrun -- and lands EXACTLY on the origin, because the plan is
//      balanced and integer division truncates symmetrically about zero
//   5. the sum property: a one-directional stream delivers the total of every
//      move queued, to within a rounding remainder of a few counts in millions
//
// WHY THE POSITION SAMPLER IS THE PRIMARY EVIDENCE. Queue depth is read once
// per loop iteration, so a momentary dip to zero between two reads could hide.
// The commanded position cannot hide one: if the queue ever ran dry the shaft
// would be commanded to stand still until the next top-up, which is a stall of
// at least one bus round trip and usually far longer. Section 2 proves the
// sampler detects exactly that, on purpose, before section 3 relies on it.
//
// SAFETY. MOSFETs stay OFF throughout -- never enableMosfets(), never
// goToClosedLoop(). Only the COMMANDED position (cmd 34) and the queue depth
// (cmd 11) are read, and the planner advances both whether or not the output
// stage is energised, so nothing turns on any motor at any point. The balanced
// stream is bounded to 0.132 rotation of commanded excursion -- comfortably
// inside even the FACTORY two-rotation deviation window, let alone the raised
// one -- the one-directional stream to 1.56 rotations, and the section-1 slot
// cost probe to 0.25 rotation. All well under two turns. No broadcasts, no
// alias changes, no test modes, no calibration: safe on the shared 35-motor
// rack. Every subsection ends with a reset.
// ---------------------------------------------------------------------------

static const uint8_t ERR_QUEUE_IS_FULL          = 17;
static const uint8_t ERR_RUN_OUT_OF_QUEUE_ITEMS = 18;

// One streamed move: 3125 ticks = 0.1 s at 31250 ticks/s. Short enough that
// three hundred of them run in half a minute, long enough that a queue holding
// nine of them buffers most of a second -- far more than any bus round trip.
static const uint32_t MOVE_TICKS = 3125;
static const uint32_t MOVE_MS    = 100;

// The balanced lap: five outward legs of differing size, then the same five
// magnitudes inward. Varying the magnitude makes every move a different
// trapezoid, so the stream is not one shape repeated. Pairing each magnitude
// with its exact negative inside the lap is what makes the net EXACTLY zero:
// the numerator is total_displacement << 24 (motor_control.c:447) and C integer
// division truncates toward zero (motor_control.c:485), so the acceleration
// computed for -d is the exact negation of the one computed for +d and the two
// moves cancel in the int96 position accumulator.
static const int      MOVES_PER_LAP = 10;
static const int32_t  LAP_MAG[5]    = { 51200, 102400, 65536, 131072, 81920 };
static const int32_t  LAP_SPAN      = 51200 + 102400 + 65536 + 131072 + 81920; // 432128
static const int      LAPS          = 30;
static const int      STREAM_MOVES  = LAPS * MOVES_PER_LAP;   // 300

// The one-directional stream used for the sum property.
static const int32_t  UNI_MAG   = 51200;
static const int      UNI_MOVES = 100;                        // 5,120,000 counts

// Refill policy. A trapezoid move costs three slots, so topping up only while
// the PROJECTED depth is at or below 24 can never push past 27 -- five slots of
// headroom under the 32-slot ring. The depth read is conservative in the safe
// direction: only this host adds items, so between the read and the write the
// real depth can only have FALLEN.
static const int SLOTS_PER_MOVE = 3;
static const int REFILL_BELOW   = 24;
static const int SAFE_CEILING   = 30;   // the depth must never be seen above this

// ---------------------------------------------------------------------------
// Watches the commanded position while something else drives the motion.
// `maxStallMs` is the longest span over which the position did NOT change --
// the direct measure of "did the motion ever hitch".
// ---------------------------------------------------------------------------
struct StreamSampler {
    bool     started;
    int64_t  lastPos;
    uint32_t lastChangeMs;
    uint32_t maxStallMs;
    int64_t  minPos;
    int64_t  maxPos;
    int64_t  travel;      // sum of |position change| across samples
    int      samples;
};

static void samplerInit(StreamSampler* s) {
    memset(s, 0, sizeof(*s));
}

static bool samplerTake(Servomotor* m, StreamSampler* s) {
    int64_t p = m->getPositionRaw();
    if (m->getError() != 0) return false;
    uint32_t now = millis();
    if (!s->started) {
        s->started = true;
        s->lastPos = p; s->lastChangeMs = now;
        s->minPos = p; s->maxPos = p;
        s->samples = 1;
        return true;
    }
    s->samples++;
    if (p != s->lastPos) {
        s->travel += llabs((long long)(p - s->lastPos));
        s->lastPos = p;
        s->lastChangeMs = now;
    }
    else {
        uint32_t stall = now - s->lastChangeMs;
        if (stall > s->maxStallMs) s->maxStallMs = stall;
    }
    if (p < s->minPos) s->minPos = p;
    if (p > s->maxPos) s->maxPos = p;
    return true;
}

struct StreamResult {
    int           queued;
    int           refillRounds;
    int           minDepth;
    int           maxDepth;
    bool          commsClean;
    int           failedAtMove;
    StreamSampler s;
};

// ---------------------------------------------------------------------------
// Drive a continuous stream: prime the queue, then keep it topped up while
// sampling the commanded position, until `totalMoves` have been queued.
// Displacements come from `pattern`, cycled. The caller drains afterwards --
// stopping the top-ups is the shutdown under test, so it is not done here.
// ---------------------------------------------------------------------------
static void runStream(Servomotor* m, const int32_t* pattern, int patternLen,
                      int totalMoves, StreamResult* r) {
    r->queued = 0;
    r->refillRounds = 0;
    r->minDepth = 255;
    r->maxDepth = 0;
    r->commsClean = true;
    r->failedAtMove = -1;
    samplerInit(&r->s);

    // Prime: get moves in flight before any of them can drain away. Nine moves
    // is 27 slots, the same ceiling the steady-state policy uses.
    while (r->queued < totalMoves) {
        m->trapezoidMoveRaw(pattern[r->queued % patternLen], MOVE_TICKS);
        if (m->getError() != 0) { r->commsClean = false; r->failedAtMove = r->queued; return; }
        r->queued++;
        if (r->queued * SLOTS_PER_MOVE > REFILL_BELOW) break;
    }

    // Steady state: read the depth, top it back up if there is room, sample.
    while (r->queued < totalMoves) {
        int depth = (int)m->getNQueuedItems();
        if (m->getError() != 0) { r->commsClean = false; r->failedAtMove = r->queued; return; }
        if (depth < r->minDepth) r->minDepth = depth;
        if (depth > r->maxDepth) r->maxDepth = depth;

        int projected = depth;
        bool toppedUp = false;
        while (projected <= REFILL_BELOW && r->queued < totalMoves) {
            m->trapezoidMoveRaw(pattern[r->queued % patternLen], MOVE_TICKS);
            if (m->getError() != 0) { r->commsClean = false; r->failedAtMove = r->queued; return; }
            r->queued++;
            projected += SLOTS_PER_MOVE;
            toppedUp = true;
        }
        if (toppedUp) r->refillRounds++;

        if (!samplerTake(m, &r->s)) { r->commsClean = false; r->failedAtMove = r->queued; return; }
        delay(5);
    }
}

// The limits every stream here runs under. maxVelocity/maxAcceleration = 62
// control ticks, comfortably above the delta_t1 <= 1 threshold, so every move
// takes the ordinary three-segment path (motor_control.c:2039-2041) and the
// refill arithmetic above is exact.
//
// 62 ticks is also only ~2 ms of wall clock, which is SHORTER than one bus round
// trip in host mode. That does not affect the stream -- the three segments are
// queued back to back inside one firmware command -- but it does mean a queue
// depth read taken right after a single move may already have lost the ramp
// segment. Section 1 measures the exact slot cost at slower limits for exactly
// that reason.
static void streamLimits(Servomotor* m) {
    m->setMaximumVelocity(4.0f);
    m->setMaximumAcceleration(2000.0f);
}

void tm_streaming_motion(void) {
    Serial.println("tm_streaming_motion: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE PREMISES THE STREAM RESTS ON. Two numbers decide whether the
    //    refill policy below is safe and whether the exactness claim in
    //    section 4 means anything: the slot cost of one move, and whether a
    //    move and its negation cancel exactly. Measure both rather than
    //    assuming them -- if either changes, everything downstream is wrong
    //    and it should fail HERE, where the cause is obvious.
    // =====================================================================
    printf("\n--- 1. slot cost and the exactness premise ---\n");
    {
        // The EXACT slot cost must be measured where the reading cannot race the
        // consumer. Under the stream's own limits the ramp segment lasts
        // delta_t1 = max_velocity/max_acceleration = 62 control ticks, i.e. about
        // 2 ms, and the depth read arrives a whole bus round trip after the move
        // was queued -- roughly 1 ms on the ESP32-S3's hardware UART, and more
        // than that in host mode on a Mac, where a USB-serial adapter's latency
        // timer dominates (see tm_command_latency_profile's caveat). The ramp
        // would then already be spent and a PERFECTLY HEALTHY machine would
        // report 2. So measure the cost the way tm_queue_accounting does (that
        // module is verified 20/20 on the rack, and states this same reason):
        // 1 rot/s over 1 rot/s^2 makes delta_t1 a whole second, which puts the
        // reading far outside any host-side jitter.
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 4 * (uint32_t)TFH_TPS);
        bool accepted = (m->getError() == 0);
        int depth = (int)m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        printf("   one trapezoid move, slow limits (race-free read) -> queue depth %d"
               " (expected %d)\n", depth, SLOTS_PER_MOVE);
        TEST_RESULT("a trapezoid move is accepted", accepted && depthOk);
        TEST_RESULT("and it costs exactly three queue slots, so nine moves fit in 27",
                    depth == SLOTS_PER_MOVE);
        tfhDrain(m, 20000);   // that probe move is 4 s long

        // The same reading under the STREAM's limits. Only the UPPER bound is
        // assertable here, for the race just described -- and the upper bound is
        // all the refill policy rests on: if a move never costs MORE than
        // SLOTS_PER_MOVE, projecting three slots per top-up can only
        // over-estimate the resulting depth, never under-estimate it. The lower
        // bound of 1 is safe because the coast segment alone is delta_t2 = 3001
        // ticks (~96 ms), far longer than any round trip.
        tfhFreshOpen(m);
        streamLimits(m);
        m->trapezoidMoveRaw(LAP_MAG[1], MOVE_TICKS);
        bool sAccepted = (m->getError() == 0);
        int sDepth = (int)m->getNQueuedItems();
        bool sDepthOk = (m->getError() == 0);
        printf("   one streamed move, stream limits (2 ms ramp) -> queue depth %d\n",
               sDepth);
        TEST_RESULT("one streamed move is accepted", sAccepted && sDepthOk);
        TEST_RESULT("and it never occupies more than three slots, which is what the"
                    " refill policy assumes",
                    sDepth >= 1 && sDepth <= SLOTS_PER_MOVE);
        tfhDrain(m, 8000);

        // The exactness premise: out and back must land on the SAME COUNT, not
        // near it. This is what lets section 4 assert a residue of exactly zero
        // after three hundred moves instead of a tolerance.
        tfhFreshOpen(m);
        streamLimits(m);
        int64_t origin = m->getPositionRaw();
        bool okOut = false, okBack = false;
        int64_t out  = tfhMove(m, LAP_MAG[3], MOVE_TICKS, &okOut);
        int64_t back = tfhMove(m, -LAP_MAG[3], MOVE_TICKS, &okBack);
        int64_t residue = m->getPositionRaw() - origin;
        printf("   out %lld, back %lld (asked %ld), residue %lld counts\n",
               (long long)out, (long long)back, (long)LAP_MAG[3], (long long)residue);
        TEST_RESULT("a single move delivers what was asked to within two counts",
                    okOut && okBack && llabs((long long)(out - LAP_MAG[3])) <= 2);
        TEST_RESULT("a move and its negation cancel EXACTLY, to the count",
                    okOut && okBack && residue == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 2. NEGATIVE CONTROL. Before claiming "the queue never emptied and the
    //    motion never stalled", show that this apparatus CAN see both. Queue
    //    three moves, then walk away: the queue must empty and the commanded
    //    position must freeze, detected by the very same sampler section 3
    //    uses. Without this, section 3 would pass just as happily against a
    //    sampler that never fires.
    //
    //    It also pins a real firmware contract: an empty queue is NOT an
    //    underrun. A chain of trapezoid moves ends at exactly zero velocity,
    //    so handle_queued_movements() finds current_velocity_i64 == 0 and
    //    error 18 is not raised (motor_control.c:2158-2164).
    // =====================================================================
    printf("\n--- 2. negative control: with no top-ups it empties and stops ---\n");
    {
        tfhFreshOpen(m);
        streamLimits(m);
        int64_t origin = m->getPositionRaw();

        for (int i = 0; i < 3; i++) m->trapezoidMoveRaw(LAP_MAG[1], MOVE_TICKS);
        bool accepted = (m->getError() == 0);

        StreamSampler s;
        samplerInit(&s);
        bool sawEmpty = false;
        bool readsOk = true;
        uint32_t t0 = millis();
        while (millis() - t0 < 2000) {
            int d = (int)m->getNQueuedItems();
            if (m->getError() != 0) { readsOk = false; break; }
            if (d == 0) sawEmpty = true;
            if (!samplerTake(m, &s)) { readsOk = false; break; }
            delay(10);
        }
        uint8_t fat = tfhFatal(m);
        int64_t moved = (fat == 0) ? (m->getPositionRaw() - origin) : 0;
        printf("   3 moves, no top-ups: sawEmpty=%d longest stall %lu ms over %d samples,"
               " moved %lld of %ld, fatal %u\n",
               (int)sawEmpty, (unsigned long)s.maxStallMs, s.samples,
               (long long)moved, (long)(3 * LAP_MAG[1]), (unsigned)fat);

        TEST_RESULT("three moves with no top-ups are accepted", accepted && readsOk);
        TEST_RESULT("left alone, the queue really does drain to empty", sawEmpty);
        TEST_RESULT("and the sampler sees the motion stall for far longer than one move",
                    s.maxStallMs > 3 * MOVE_MS);
        TEST_RESULT("an empty queue is not an underrun: trapezoid moves end at zero velocity",
                    fat == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 3. THE STREAM. Three hundred moves, thirty balanced laps, queued a few
    //    ahead and topped up as they drain. Nothing waits for anything: the
    //    only pacing is the refill policy, and the motor is never given the
    //    chance to reach an empty queue.
    //
    //    Three hundred moves is nine hundred queue slots through a
    //    thirty-two-slot ring -- twenty-eight wraps of the read and write
    //    indices, with the producer and the 31.25 kHz consumer running
    //    concurrently the whole time. That is the regime a ring-index or
    //    accounting bug lives in, and no other module enters it.
    // =====================================================================
    printf("\n--- 3. three hundred moves streamed without the queue emptying ---\n");
    int64_t streamOrigin = 0;
    StreamResult r;
    {
        tfhFreshOpen(m);
        streamLimits(m);
        streamOrigin = m->getPositionRaw();

        int32_t pattern[MOVES_PER_LAP];
        for (int k = 0; k < 5; k++) {
            pattern[k]     =  LAP_MAG[k];
            pattern[k + 5] = -LAP_MAG[k];
        }

        uint32_t t0 = millis();
        runStream(m, pattern, MOVES_PER_LAP, STREAM_MOVES, &r);
        uint32_t elapsed = millis() - t0;

        uint8_t fat = tfhFatal(m);
        int64_t commandedAbs = (int64_t)LAPS * 2 * LAP_SPAN;

        printf("   queued %d/%d in %lu ms over %d refill rounds; depth %d..%d;"
               " %d samples, longest stall %lu ms\n",
               r.queued, STREAM_MOVES, (unsigned long)elapsed, r.refillRounds,
               r.minDepth, r.maxDepth, r.s.samples, (unsigned long)r.s.maxStallMs);
        printf("   band %lld..%lld counts (plan span %ld); travelled %lld of %lld commanded;"
               " fatal %u\n",
               (long long)(r.s.minPos - streamOrigin), (long long)(r.s.maxPos - streamOrigin),
               (long)LAP_SPAN, (long long)r.s.travel, (long long)commandedAbs, (unsigned)fat);

        TEST_RESULT("all three hundred streamed moves are accepted",
                    r.commsClean && r.queued == STREAM_MOVES);
        TEST_RESULT("the stream really was refilled many times, not queued up front",
                    r.refillRounds >= 30);
        TEST_RESULT("the queue never emptied while the stream was running",
                    r.minDepth >= 1);
        TEST_RESULT("it always kept at least one whole move buffered",
                    r.minDepth >= SLOTS_PER_MOVE);
        TEST_RESULT("the top-up policy never pushed the 32-slot ring near full",
                    r.maxDepth <= SAFE_CEILING);
        TEST_RESULT("no ERROR_QUEUE_IS_FULL (17) during the stream",
                    fat != ERR_QUEUE_IS_FULL);
        TEST_RESULT("no fatal error of any kind during the stream", fat == 0);
        TEST_RESULT("the commanded motion never stalled for as long as one move",
                    r.s.maxStallMs < MOVE_MS);
        // Anti-vacuity floor only, deliberately far below expectation. One loop
        // iteration is delay(5) plus two round trips, so on a healthy link this
        // lands between 1500 and 3000 samples; 100 corresponds to an average
        // iteration of ~290 ms, i.e. a link an order of magnitude slower than
        // anything tm_command_latency_profile tolerates. Setting the floor near
        // the expected value would turn ordinary USB and scheduler jitter into a
        // test failure.
        TEST_RESULT("the position was sampled often enough for that to mean something",
                    r.s.samples >= 100);
        // The plan is a closed loop out to LAP_SPAN and back, thirty times, so
        // the trajectory must stay inside that band. A dropped or duplicated
        // move would show up as the band drifting.
        TEST_RESULT("the commanded position never overshot the top of the plan's band",
                    (r.s.maxPos - streamOrigin) <= (int64_t)LAP_SPAN + 200);
        TEST_RESULT("and never fell below the bottom of it",
                    (r.s.minPos - streamOrigin) >= -200);
        TEST_RESULT("it did traverse the whole band, so the laps really ran",
                    (r.s.maxPos - streamOrigin) >= (int64_t)LAP_SPAN - 120000);
        // `travel` is a sum of |delta| between consecutive SAMPLES, so it is
        // always a LOWER bound on the true path length, and the shortfall is a
        // sampling artefact rather than a property of the machine: an interval
        // that straddles a direction reversal measures the net, not the path.
        // The loss is roughly proportional to the sampling gap (about one
        // percent of the total per 10 ms of gap, with sixty reversals in the
        // plan), so a tight tolerance here would really be asserting a loop
        // rate. Half is the honest, host-jitter-immune bound, and it still
        // catches a stream that silently dropped a large share of its moves.
        TEST_RESULT("the distance actually travelled accounts for the moves queued",
                    r.s.travel >= commandedAbs / 2);
    }

    // =====================================================================
    // 4. CLEAN SHUTDOWN. The other half of the contract: stop topping up and
    //    the machine must coast out the work already in flight and stop --
    //    promptly, quietly, and exactly where the arithmetic says.
    //
    //    The exact-zero assertion is the sharpest thing in this module. Thirty
    //    balanced laps of ten moves means three hundred trapezoids whose
    //    displacements sum to zero, and because +d and -d produce exactly
    //    negated accelerations the int96 position accumulator must return to
    //    the very count it started from. Not "within a few counts" -- zero.
    // =====================================================================
    printf("\n--- 4. clean shutdown: stop topping up and let it drain ---\n");
    {
        uint32_t t0 = millis();
        bool drained = tfhDrain(m, 20000);
        uint32_t drainMs = millis() - t0;
        uint8_t fat = tfhFatal(m);

        int64_t rest1 = m->getPositionRaw();
        bool restOk = (m->getError() == 0);
        delay(1000);
        int64_t rest2 = m->getPositionRaw();
        restOk = restOk && (m->getError() == 0);
        int depth = (int)m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);
        int64_t residue = rest1 - streamOrigin;

        printf("   drained in %lu ms, fatal %u, resting depth %d, residue %lld counts,"
               " crept %lld counts in a further second\n",
               (unsigned long)drainMs, (unsigned)fat, depth,
               (long long)residue, (long long)(rest2 - rest1));

        TEST_RESULT("the queue drains to empty on its own once the top-ups stop", drained);
        TEST_RESULT("and it does so within the time it had buffered, not much longer",
                    drained && drainMs < 5000);
        TEST_RESULT("coming to rest raises no ERROR_RUN_OUT_OF_QUEUE_ITEMS (18)",
                    fat != ERR_RUN_OUT_OF_QUEUE_ITEMS);
        TEST_RESULT("the shutdown raises no fatal error at all", fat == 0);
        TEST_RESULT("the machine stays at rest: not one count of creep in a further second",
                    restOk && rest2 == rest1);
        TEST_RESULT("the queue stays empty at rest", depthOk && depth == 0);
        TEST_RESULT("three hundred balanced streamed moves land EXACTLY on the origin",
                    r.queued == STREAM_MOVES && residue == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 5. THE SUM PROPERTY. A balanced stream proves nothing about total
    //    distance -- its answer is zero either way. So stream a hundred moves
    //    all in the SAME direction and require the endpoint to be the sum of
    //    every displacement queued.
    //
    //    "Exactly" has a precise meaning here and it is worth stating. One
    //    move delivers acceleration * delta_t1 * (delta_t1 + delta_t2) in the
    //    int96 accumulator, and the acceleration was computed with a
    //    truncating division (motor_control.c:485), so each move is short by
    //    the division remainder -- under 0.02 of a count. That remainder is
    //    carried in the accumulator's fractional bits rather than thrown away,
    //    so a hundred moves are short by a couple of counts in five million,
    //    and NOT by a hundred times anything. That distinction is the test: a
    //    per-move LEAK would show up here as a shortfall of hundreds.
    // =====================================================================
    printf("\n--- 5. a one-directional stream delivers the sum of every move ---\n");
    {
        tfhFreshOpen(m);
        streamLimits(m);
        int64_t origin = m->getPositionRaw();

        int32_t pattern[1] = { UNI_MAG };
        StreamResult u;
        runStream(m, pattern, 1, UNI_MOVES, &u);
        bool drained = tfhDrain(m, 20000);
        uint8_t fat = tfhFatal(m);
        int64_t net = (fat == 0) ? (m->getPositionRaw() - origin) : 0;
        int64_t want = (int64_t)UNI_MOVES * UNI_MAG;

        printf("   queued %d/%d, depth %d..%d, longest stall %lu ms;"
               " delivered %lld of %lld (short by %lld), fatal %u\n",
               u.queued, UNI_MOVES, u.minDepth, u.maxDepth,
               (unsigned long)u.s.maxStallMs, (long long)net, (long long)want,
               (long long)(want - net), (unsigned)fat);

        TEST_RESULT("every one of the hundred one-directional moves is accepted",
                    u.commsClean && u.queued == UNI_MOVES && drained);
        TEST_RESULT("the queue never runs dry during the one-directional stream",
                    u.minDepth >= 1);
        TEST_RESULT("that stream does not stall either", u.s.maxStallMs < MOVE_MS);
        TEST_RESULT("no fatal error during the one-directional stream", fat == 0);
        TEST_RESULT("the total delivered is the sum of every move queued, to within 8 counts",
                    fat == 0 && llabs((long long)(net - want)) <= 8);
        TEST_RESULT("so the shortfall is a rounding remainder, not a per-move leak",
                    fat == 0 && llabs((long long)(net - want)) < UNI_MOVES / 4);
        tfhReset(m);
    }

    // =====================================================================
    // 6. STILL HEALTHY. Four hundred streamed moves and twenty-eight wraps of
    //    the ring later, an ordinary move must behave exactly as it does on a
    //    cold machine.
    // =====================================================================
    printf("\n--- 6. still healthy after all the streaming ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   post-stream move delivered %lld of %lld\n",
               (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after all the streaming",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
