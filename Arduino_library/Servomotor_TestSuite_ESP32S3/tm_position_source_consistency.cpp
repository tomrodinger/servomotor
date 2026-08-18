#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_position_source_consistency.cpp
//
// THE THREE POSITION SOURCES MUST AGREE.
//
// The motor answers three different commands with a position, and a controller
// is free to use whichever is convenient:
//
//     cmd 34  Get position               -> the COMMANDED position
//     cmd 15  Get hall sensor position   -> the MEASURED position
//     cmd 37  Get comprehensive position -> both of the above, plus the
//                                           external encoder count, in ONE reply
//
// (Command numbers verified against python_programs/servomotor/motor_commands.json.
// Note that the older tm_comprehensive_position.cpp header mislabels cmd 37 as
// "cmd 34"; the JSON and the firmware say 37.)
//
// In the firmware these are not three measurements. They are three VIEWS OF THE
// SAME TWO VARIABLES, and the comprehensive handler literally calls the other
// two handlers' getters:
//
//     main.c:452   position_reply.motor_position       = get_motor_position();
//     main.c:465   hall_position_reply.hall_sensor_pos = get_hall_position();
//     main.c:481   comprehensive.motor_position        = get_motor_position();
//     main.c:482   comprehensive.hall_sensor_position  = get_hall_position();
//     main.c:483   comprehensive.external_encoder_pos  = get_external_encoder_position();
//
//     motor_control.c:3421  get_motor_position() -> current_position_i96.position
//     motor_control.c:3432  get_hall_position()  -> hall_position_i64
//     GPIO_interrupts.c:62  get_external_encoder_position() -> step_count
//
// So any disagreement between them is a bug in the reply framing, the payload
// packing, the int64 decode on the library side, or the response routing -- it
// can never be a legitimate difference of opinion about where the shaft is.
// That is a sharper property than any single source can be tested against on
// its own, because it needs no knowledge of the true position at all.
//
// THE PROPERTY THAT MAKES IT A SAFETY TEST, not just a plumbing test:
//
//     motor_control.c:3217   position_deviation = current_position_i64 - current_hall_position_i64;
//     motor_control.c:3218   if (calibration_step == 0 && llabs(position_deviation) > max_allowable_position_deviation)
//     motor_control.c:3223       fatal_error(ERROR_POSITION_DEVIATION_TOO_LARGE);   // code 45
//
// The difference between source 1 and source 2 is EXACTLY the number the stall
// watchdog compares against its window. A controller that wants to know how
// close it is to faulting computes it from these replies. If the two fields of
// cmd 37 ever came from different instants, or if cmd 34 and cmd 15 disagreed
// with the fields of cmd 37, that computed margin would be wrong and the first
// symptom would be an unexplained error 45 in the field. This module asserts
// the identity end to end: measured deviation == commanded displacement ==
// what the watchdog trips on.
//
// HOW THIS IS DISTINCT FROM THE TWO EXISTING MODULES:
//   * tm_comprehensive_position.cpp is a UNIT-CONVERSION test. It energises the
//     MOSFETs, moves one rotation in closed loop, and reads cmd 37 in each of
//     the four position units, checking the scale factors. Its only cross-source
//     check is a single loose "these are roughly equal" at one resting point,
//     with the shaft actually following, so commanded and hall are equal there
//     by construction and the check cannot fail for an interesting reason.
//   * tm_position_frames.cpp is a REFERENCE-FRAME test: negative absolute
//     targets, sequential zeroPosition re-basing, sign preservation across
//     units, and the queue-not-empty guard. It runs closed loop too, and touches
//     cmd 37 twice as a spot check.
//   * NEITHER asserts the cross-source ALGEBRA, and neither can: with the shaft
//     following, commanded minus hall is a small servo error, so the identity is
//     drowned in PID noise.
//
//     WITH THE OUTPUT STAGE OFF, THE HALL POSITION IS CONSTANT, so the deviation
//     is exactly the commanded displacement -- a known, arbitrarily large number
//     rather than a few counts of tracking error. That turns the cross-source
//     difference into a calibrated quantity that can be predicted to the count.
//
// SAFETY:
//   The MOSFETs are never enabled. Every reading here is of the COMMANDED
//   position, which the motion planner advances tick by tick whether or not the
//   output stage is energised (motor_control.c:2191 adds current_velocity_i64
//   into current_position_i96 every control tick, unconditionally). Nothing
//   turns, on any motor, at any point -- which is what makes this safe on a
//   shared 35-motor rack with loads attached. disableMosfets() is called
//   explicitly after every reset so the state is asserted rather than assumed.
//   enableMosfets() and goToClosedLoop() appear nowhere in this file.
//
//   Section 3 deliberately trips error 45 once, which is a normal latched fatal
//   error cleared by the reset that immediately follows it. While it is latched,
//   get_status is the ONLY command that returns real data, so the polling loop
//   there stops the moment a read reports an error and never tries to read a
//   position out of a faulted machine.
//
//   Moves larger than about two rotations require the deviation window to be
//   opened first (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION is
//   ONE_REVOLUTION_MICROSTEPS * 2, motor_control.c:97), so the large-move
//   sections start from tfhFreshOpen(); the small-move sections deliberately do
//   NOT, because the default window is part of what they measure.
//
//   No broadcasts, no setDeviceAlias, no testMode. Every command is addressed to
//   one motor by unique ID via tfGetMotor().
// ---------------------------------------------------------------------------

namespace {

// error_codes.json / the autogenerated firmware error_text.h
const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;

// TWO tolerances, because two quite different things are being bounded and
// using one number for both would either be unfalsifiable or flaky.
//
//  * HALL_JITTER_TOL bounds the disagreement between two readings of the SAME
//    hall variable taken a couple of milliseconds apart (cmd 15 versus the hall
//    field of cmd 37). Nothing but sensor quantisation can get in between them,
//    so this one can be tight.
//
//  * HALL_DRIFT_TOL bounds how far the hall reading may wander across a whole
//    move, or across a reset. That is a physical claim about a shaft nobody is
//    driving, not a plumbing claim, and the value is deliberately copied from
//    tm_deviation_tracking, which asserts exactly this property (its section 1,
//    "the hall position stays put while the MOSFETs are off") at TFH_CPR / 50
//    and has been green on the bench motor and on two different rack motors.
//    Picking anything tighter here would be inventing a bound no measurement
//    supports -- and a hall reading that genuinely FOLLOWED a move would blow
//    through TFH_CPR / 50 by more than an order of magnitude anyway, since the
//    smallest displacement commanded in this file is TFH_CPR / 4.
const int64_t HALL_JITTER_TOL = TFH_CPR / 200;   // 16384 counts, same-instant reads
const int64_t HALL_DRIFT_TOL  = TFH_CPR / 50;    // 65536 counts, across a move/reset

// One reading of all three sources. Deliberately NOT atomic -- the whole point
// is that three separate bus transactions must still agree, because that is how
// a real controller uses them.
struct PscSnapshot {
    int64_t getPos;    // cmd 34 Get position
    int64_t getHall;   // cmd 15 Get hall sensor position
    int64_t compCmd;   // cmd 37 field 1, commandedPosition
    int64_t compHall;  // cmd 37 field 2, hallSensorPosition
    int32_t compExt;   // cmd 37 field 3, externalEncoderPosition
    bool    ok;        // every one of the three replies arrived intact
};

PscSnapshot pscRead(Servomotor* m) {
    PscSnapshot s;
    s.getPos = 0; s.getHall = 0; s.compCmd = 0; s.compHall = 0; s.compExt = 0;
    s.ok = false;
    s.getPos = m->getPositionRaw();
    if (m->getError() != 0) return s;
    s.getHall = m->getHallSensorPositionRaw();
    if (m->getError() != 0) return s;
    getComprehensivePositionResponse c = m->getComprehensivePositionRaw();
    if (m->getError() != 0) return s;
    s.compCmd  = c.commandedPosition;
    s.compHall = c.hallSensorPosition;
    s.compExt  = c.externalEncoderPosition;
    s.ok = true;
    return s;
}

// The quantity the stall watchdog compares against its window
// (motor_control.c:3217), computed from the single comprehensive reply.
int64_t pscDeviation(const PscSnapshot& s) { return s.compCmd - s.compHall; }

// The same quantity computed from the two INDEPENDENT getters. If the sources
// are consistent these two must match to within hall noise.
int64_t pscDeviationFromGetters(const PscSnapshot& s) { return s.getPos - s.getHall; }

// Reset to a known machine with the output stage explicitly off and modest
// ceilings suitable for sub-two-rotation moves. The deviation window is left at
// its factory default here on purpose.
void pscIdle(Servomotor* m) {
    tfhReset(m);
    m->disableMosfets();          // explicit, not assumed: nothing may turn
    delay(150);
    m->setMaximumVelocity(4.0f);      // rot/s   (tfhReset set the velocity unit)
    m->setMaximumAcceleration(2000.0f);   // rot/s^2
}

// Reset, open the ceilings (deviation window included) and confirm the output
// stage is off. Required for anything past ~2 rotations.
//
// The acceleration ceiling is pulled back down from tfhFreshOpen's 5000 to the
// same 2000 rot/s^2 the small-move sections use. That is deliberate: with a
// very high acceleration limit the planner's ramp collapses to one or two
// ticks, which is a genuinely interesting edge regime -- and one already owned
// by tm_planner_arithmetic and tm_queue_accounting. This module wants an
// ordinary three-segment trapezoid so that the thing under test is the
// agreement of the position sources, not the shape of the profile.
void pscIdleOpen(Servomotor* m) {
    tfhFreshOpen(m);
    m->setMaximumAcceleration(2000.0f);   // rot/s^2: ordinary ramp, not a 1-tick one
    m->disableMosfets();
    delay(150);
}

// Set the deviation window in shaft rotations. The setter takes its argument in
// the CURRENT position unit, so the unit is switched around the call and put
// back, exactly as tm_deviation_tracking does.
bool pscSetWindowRotations(Servomotor* m, float rotations) {
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setMaxAllowablePositionDeviation(rotations);
    bool ok = (m->getError() == 0);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    return ok;
}

}  // namespace

void tm_position_source_consistency(void) {
    Serial.println("tm_position_source_consistency: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. AT REST, THE THREE SOURCES ARE THREE VIEWS OF TWO NUMBERS.
    //
    //    The machine is stationary and the queue is empty, so the commanded
    //    position cannot change between reads (motor_control.c:2191 adds
    //    current_velocity_i64 into the position every tick, and that velocity
    //    is zero at rest). Agreement on the commanded field must therefore be
    //    EXACT, not approximate -- which is the sharpest form this assertion
    //    can take, and would catch a truncated int64, a byte-order slip, or a
    //    reply read one frame late.
    //
    //    The comprehensive reply is read both BEFORE and AFTER the two
    //    single-value reads. A response-routing bug that handed back the
    //    previous reply would show up as the two comprehensive reads
    //    disagreeing with the getter sandwiched between them.
    // =====================================================================
    printf("\n--- 1. the three sources agree exactly while the machine is at rest ---\n");
    {
        pscIdle(m);
        const int trials = 6;
        bool allRepliesOk = true;
        bool cmdExactEveryTrial = true;
        bool compStableEveryTrial = true;
        bool hallAgreesEveryTrial = true;
        int64_t worstHallGap = 0;
        int64_t firstCmd = 0, lastCmd = 0;

        for (int i = 0; i < trials; i++) {
            getComprehensivePositionResponse c1 = m->getComprehensivePositionRaw();
            if (m->getError() != 0) { allRepliesOk = false; break; }
            int64_t p = m->getPositionRaw();
            if (m->getError() != 0) { allRepliesOk = false; break; }
            int64_t h = m->getHallSensorPositionRaw();
            if (m->getError() != 0) { allRepliesOk = false; break; }
            getComprehensivePositionResponse c2 = m->getComprehensivePositionRaw();
            if (m->getError() != 0) { allRepliesOk = false; break; }

            if (c1.commandedPosition != p) cmdExactEveryTrial = false;
            if (c2.commandedPosition != p) cmdExactEveryTrial = false;
            if (c1.commandedPosition != c2.commandedPosition) compStableEveryTrial = false;

            int64_t gap1 = llabs((long long)(c1.hallSensorPosition - h));
            int64_t gap2 = llabs((long long)(c2.hallSensorPosition - h));
            if (gap1 > worstHallGap) worstHallGap = gap1;
            if (gap2 > worstHallGap) worstHallGap = gap2;
            if (gap1 > HALL_JITTER_TOL || gap2 > HALL_JITTER_TOL) hallAgreesEveryTrial = false;

            if (i == 0) firstCmd = p;
            lastCmd = p;
            printf("   trial %d: cmd34=%lld comp.cmd=%lld/%lld | cmd15=%lld comp.hall=%lld (gap %lld)\n",
                   i, (long long)p, (long long)c1.commandedPosition,
                   (long long)c2.commandedPosition, (long long)h,
                   (long long)c1.hallSensorPosition, (long long)gap1);
            delay(30);
        }
        printf("   worst hall gap across %d trials: %lld counts (tolerance %lld)\n",
               trials, (long long)worstHallGap, (long long)HALL_JITTER_TOL);

        TEST_RESULT("all three position commands answer without a communication error",
                    allRepliesOk);
        TEST_RESULT("the comprehensive reply's commanded field equals Get position exactly",
                    allRepliesOk && cmdExactEveryTrial);
        TEST_RESULT("the comprehensive reply reads the same before and after Get position",
                    allRepliesOk && compStableEveryTrial);
        TEST_RESULT("the comprehensive reply's hall field equals Get hall sensor position",
                    allRepliesOk && hallAgreesEveryTrial);
        TEST_RESULT("the commanded position does not drift at rest with the output stage off",
                    allRepliesOk && firstCmd == lastCmd);
        tfhReset(m);
    }

    // =====================================================================
    // 2. THE DIFFERENCE BETWEEN THE SOURCES IS THE COMMANDED DISPLACEMENT.
    //
    //    This is the identity the watchdog is built on. With the output stage
    //    off the rotor never follows, so hall_position_i64 stops accumulating
    //    (motor_control.c:3101 adds hall_position_delta each tick, and a
    //    stationary shaft produces a delta of zero), while the commanded
    //    position advances by the full displacement. Therefore
    //
    //        (commanded - hall) after  -  (commanded - hall) before
    //          ==  the commanded displacement
    //
    //    to within sensor noise. Both signs are exercised, because a sign slip
    //    in the subtraction would still pass a positive-only test.
    //
    //    All three displacements stay inside the FACTORY DEFAULT two-rotation
    //    window, so no fault is expected and none is provoked.
    // =====================================================================
    printf("\n--- 2. commanded minus hall equals the commanded displacement ---\n");
    {
        const int64_t dists[3] = { TFH_CPR / 4, TFH_CPR / 2, -(TFH_CPR / 2) };
        bool gettersMatchComp = true;
        int64_t worstSourceGap = 0;

        for (int i = 0; i < 3; i++) {
            pscIdle(m);
            PscSnapshot before = pscRead(m);
            int64_t devBefore = pscDeviation(before);

            m->trapezoidMoveRaw((int32_t)dists[i], (uint32_t)(2 * TFH_TPS));
            bool accepted = (m->getError() == 0);
            bool drained = accepted && tfhDrain(m, 12000);
            uint8_t fat = tfhFatal(m);

            PscSnapshot after = (drained && fat == 0) ? pscRead(m) : before;
            int64_t cmdDelta  = after.compCmd  - before.compCmd;
            int64_t hallDelta = after.compHall - before.compHall;
            int64_t devAfter  = pscDeviation(after);
            int64_t devGrowth = devAfter - devBefore;
            int64_t sourceGap = llabs((long long)(devAfter - pscDeviationFromGetters(after)));
            if (sourceGap > worstSourceGap) worstSourceGap = sourceGap;
            if (sourceGap > HALL_JITTER_TOL) gettersMatchComp = false;

            printf("   asked %lld counts: commanded moved %lld, hall moved %lld, "
                   "deviation grew %lld (fatal=%u, getter/comp gap %lld)\n",
                   (long long)dists[i], (long long)cmdDelta, (long long)hallDelta,
                   (long long)devGrowth, (unsigned)fat, (long long)sourceGap);

            char name[128];
            snprintf(name, sizeof(name),
                     "the hall position does not follow a commanded move of %ld counts",
                     (long)dists[i]);
            TEST_RESULT(name, before.ok && after.ok && drained && fat == 0 &&
                              llabs((long long)hallDelta) <= HALL_DRIFT_TOL);

            snprintf(name, sizeof(name),
                     "the commanded-minus-hall deviation grows by the %ld counts commanded",
                     (long)dists[i]);
            // Two halves, because devGrowth - cmdDelta reduces algebraically to
            // -hallDelta and on its own would re-assert the line above without
            // ever looking at what was asked for. The commanded field must ALSO
            // have delivered the requested displacement, which is what makes the
            // name true. The planner's only inexactness is the integer division
            // in compute_trapezoid_move (motor_control.c:485), worth well under
            // one count for every move in this file, so TFH_CPR/100 is ample --
            // it is the same tolerance sections 7 and 8 already use.
            TEST_RESULT(name, before.ok && after.ok && drained && fat == 0 &&
                              llabs((long long)(cmdDelta - dists[i])) <= TFH_CPR / 100 &&
                              llabs((long long)(devGrowth - cmdDelta)) <= HALL_DRIFT_TOL);
        }
        printf("   worst disagreement between the two ways of computing the deviation: %lld counts\n",
               (long long)worstSourceGap);
        TEST_RESULT("the deviation from the two separate getters matches the comprehensive reply's own difference",
                    gettersMatchComp);
        tfhReset(m);
    }

    // =====================================================================
    // 3. THE MEASURED DIFFERENCE IS THE NUMBER THE WATCHDOG TRIPS ON.
    //
    //    Sections 1 and 2 show the sources agree. This section shows the
    //    agreed-upon difference is the SAME quantity the firmware uses, by
    //    setting the window to a known value and checking both sides of it:
    //    a move that keeps the observed difference under the window completes,
    //    and a move that pushes past it raises error 45.
    //
    //    That closes the loop for a controller: the margin it computes from
    //    these replies is the real margin to a fault, not a proxy for it.
    //
    //    (tm_deviation_tracking sweeps the window and characterises the trip
    //    point. What is new here is tying the trip to a difference MEASURED
    //    over the bus from the position sources, rather than to the commanded
    //    displacement inferred from what was asked for.)
    // =====================================================================
    printf("\n--- 3. the observed difference is what error 45 fires on ---\n");
    {
        const float windowRot = 0.5f;
        const int64_t windowCounts = (int64_t)(windowRot * (float)TFH_CPR);   // 1638400

        // --- inside the window: completes, and the measured difference is under it
        pscIdle(m);
        bool setOk = pscSetWindowRotations(m, windowRot);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 3 / 10), (uint32_t)(2 * TFH_TPS));  // 0.3 rot
        bool insideAccepted = (m->getError() == 0);
        bool insideDrained = insideAccepted && tfhDrain(m, 12000);
        uint8_t insideFatal = tfhFatal(m);
        PscSnapshot inside = (insideDrained && insideFatal == 0) ? pscRead(m) : PscSnapshot();
        int64_t insideDev = inside.ok ? llabs((long long)pscDeviation(inside)) : -1;
        printf("   window %lld counts, 0.3 rot commanded: fatal=%u, measured deviation %lld\n",
               (long long)windowCounts, (unsigned)insideFatal, (long long)insideDev);
        TEST_RESULT("a move that keeps the deviation inside the window completes with no fault",
                    setOk && insideDrained && insideFatal == 0);
        TEST_RESULT("and the deviation measured from the position sources is inside the window",
                    inside.ok && insideDev >= 0 && insideDev < windowCounts);

        // --- past the window: error 45, and the last reading is not far past it
        pscIdle(m);
        pscSetWindowRotations(m, windowRot);
        int64_t base = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 2), (uint32_t)(3 * TFH_TPS));   // 2 rot
        bool tripAccepted = (m->getError() == 0);
        int64_t lastDev = 0;
        int samplesBeforeFault = 0;
        if (tripAccepted) {
            uint32_t deadline = millis() + 12000;
            while (millis() < deadline) {
                getComprehensivePositionResponse c = m->getComprehensivePositionRaw();
                if (m->getError() != 0) break;      // the latched fault refuses reads
                lastDev = c.commandedPosition - c.hallSensorPosition;
                samplesBeforeFault++;
                uint8_t q = m->getNQueuedItems();
                if (m->getError() != 0) break;
                if (q == 0) break;
                delay(15);
            }
        }
        uint8_t tripFatal = tfhFatal(m);
        printf("   window %lld counts, 2 rot commanded (from %lld): fatal=%u after %d samples, "
               "last observed deviation %lld\n",
               (long long)windowCounts, (long long)base, (unsigned)tripFatal,
               samplesBeforeFault, (long long)lastDev);
        TEST_RESULT("a move that drives the deviation past the window raises error 45",
                    tripAccepted && tripFatal == ERR_POSITION_DEVIATION_TOO_LARGE);
        // The last successful read lags the real trip by one bus round trip, so
        // it should sit at or just under the window -- never far beyond it.
        // The slack is a quarter rotation, the same figure tm_deviation_tracking
        // allows for the identical sampling lag in the opposite direction (its
        // "does not fire early" assertion). Nothing in the firmware bounds this
        // more tightly than "the main loop notices soon", so a tighter number
        // here would be a guess; a quarter rotation still fails loudly if the
        // fault only fired near the end of the 2-rotation move.
        TEST_RESULT("the deviation is never observed far past the window before the fault fires",
                    tripFatal == ERR_POSITION_DEVIATION_TOO_LARGE &&
                    llabs((long long)lastDev) <= windowCounts + TFH_CPR / 4);
        tfhReset(m);
    }

    // =====================================================================
    // 4. THE SOURCES STAY CONSISTENT WHILE A MOVE IS IN PROGRESS.
    //
    //    A moving target is where a stale or mis-framed reply actually shows
    //    up; at rest everything trivially agrees. The test is a SANDWICH: read
    //    Get position, then the comprehensive reply, then Get position again.
    //    During a forward move the commanded position is monotonically
    //    increasing, so the comprehensive reply's commanded field MUST lie
    //    between the two Get position reads that bracket it. A reply built from
    //    a stale snapshot, or one frame of lag in the response queue, fails
    //    this immediately.
    //
    //    Six rotations exceeds the default two-rotation window, so the window
    //    is opened first. Only RAW int64 values are compared here: six
    //    rotations is 19,660,800 counts, past the 2^24 limit of exact integer
    //    representation in a float, so the converted views are deliberately
    //    left to section 6 where the magnitudes are small.
    // =====================================================================
    printf("\n--- 4. the sources stay consistent throughout a move ---\n");
    {
        pscIdleOpen(m);
        PscSnapshot start = pscRead(m);
        const int32_t dist = (int32_t)(TFH_CPR * 6);
        m->trapezoidMoveRaw(dist, (uint32_t)(5 * TFH_TPS));
        bool accepted = (m->getError() == 0);

        int samples = 0;
        int sandwichViolations = 0;
        int monotonicViolations = 0;
        int64_t worstHallCross = 0;
        int64_t hallLo = start.compHall, hallHi = start.compHall;
        int64_t prevComp = start.compCmd;
        bool readsClean = accepted && start.ok;

        // The sample count is capped and the loop is paced. Every assertion below
        // is gated on readsClean, so ONE unlucky byte anywhere in the loop would
        // fail all five; an unbounded free-running loop over a five-second move
        // would be several thousand transactions of exposure for no extra
        // evidence. 200 sandwiches spread across the move is already 25x the
        // minimum the assertions ask for.
        const int MAX_SAMPLES = 200;
        if (readsClean) {
            uint32_t deadline = millis() + 20000;
            while (millis() < deadline && samples < MAX_SAMPLES) {
                int64_t p1 = m->getPositionRaw();
                if (m->getError() != 0) { readsClean = false; break; }
                getComprehensivePositionResponse c = m->getComprehensivePositionRaw();
                if (m->getError() != 0) { readsClean = false; break; }
                int64_t p2 = m->getPositionRaw();
                if (m->getError() != 0) { readsClean = false; break; }
                int64_t h = m->getHallSensorPositionRaw();
                if (m->getError() != 0) { readsClean = false; break; }

                samples++;
                if (c.commandedPosition < p1 || c.commandedPosition > p2) sandwichViolations++;
                if (c.commandedPosition < prevComp) monotonicViolations++;
                prevComp = c.commandedPosition;

                int64_t cross = llabs((long long)(c.hallSensorPosition - h));
                if (cross > worstHallCross) worstHallCross = cross;
                if (c.hallSensorPosition < hallLo) hallLo = c.hallSensorPosition;
                if (c.hallSensorPosition > hallHi) hallHi = c.hallSensorPosition;

                uint8_t q = m->getNQueuedItems();
                if (m->getError() != 0) { readsClean = false; break; }
                if (q == 0) break;
                delay(10);
            }
        }
        tfhDrain(m, 15000);
        PscSnapshot end = pscRead(m);
        int64_t hallSpread = hallHi - hallLo;
        printf("   %d in-flight samples over %ld counts: %d sandwich violations, "
               "%d backward steps, hall spread %lld, worst cmd15-vs-cmd37 hall gap %lld\n",
               samples, (long)dist, sandwichViolations, monotonicViolations,
               (long long)hallSpread, (long long)worstHallCross);
        printf("   commanded travelled %lld of %ld counts\n",
               (long long)(end.ok && start.ok ? end.compCmd - start.compCmd : 0), (long)dist);

        TEST_RESULT("a moving motor can be sampled often enough for the comparison to mean something",
                    readsClean && samples >= 8);
        TEST_RESULT("the comprehensive reply's commanded field always falls between the Get position reads bracketing it",
                    readsClean && samples >= 8 && sandwichViolations == 0);
        TEST_RESULT("the commanded position never steps backwards during a forward move",
                    readsClean && samples >= 8 && monotonicViolations == 0);
        TEST_RESULT("the two hall sources agree with each other throughout the move",
                    readsClean && samples >= 8 && worstHallCross <= HALL_JITTER_TOL);
        TEST_RESULT("the hall position stays put throughout the move with the output stage off",
                    readsClean && samples >= 8 && hallSpread <= HALL_DRIFT_TOL);
        tfhReset(m);
    }

    // =====================================================================
    // 5. RESET AND ZERO POSITION RE-BASE ALL THE SOURCES TOGETHER.
    //
    //    Two firmware paths drive both variables to zero at once, and a
    //    controller relies on them landing at the same origin:
    //
    //      * System reset is an MCU reboot (main.c:628, NVIC_SystemReset), so
    //        both statics return to their initialisers:
    //        motor_control.c:153  static int96_t current_position_i96 = {0};
    //        motor_control.c:154  static int64_t hall_position_i64 = 0;
    //      * zero_position() (motor_control.c:3311) clears the commanded
    //        position (:3319-3321) AND the hall position (:3329) inside one
    //        __disable_irq() block, so the deviation collapses to zero rather
    //        than jumping to whatever the old offset was.
    //
    //    A firmware that zeroed only one of them would leave a large standing
    //    deviation and the machine would fault on its next real move -- which
    //    is exactly the kind of bug only a cross-source test can see.
    //
    //    zeroPosition is issued only with the queue drained; on a non-empty
    //    queue it raises fatal error 8 (motor_control.c:3313).
    // =====================================================================
    printf("\n--- 5. reset and zero position re-base every source together ---\n");
    {
        pscIdle(m);
        PscSnapshot afterReset = pscRead(m);
        printf("   after reset: cmd34=%lld comp.cmd=%lld | cmd15=%lld comp.hall=%lld | deviation %lld\n",
               (long long)afterReset.getPos, (long long)afterReset.compCmd,
               (long long)afterReset.getHall, (long long)afterReset.compHall,
               (long long)pscDeviation(afterReset));
        TEST_RESULT("a system reset leaves the commanded position at exactly zero in both sources",
                    afterReset.ok && afterReset.getPos == 0 && afterReset.compCmd == 0);
        // hall_position_i64 is a zero-initialised static and the first sensor
        // read after boot returns change = 0 (hall_sensor_calculations.c:146,
        // previous_largest_sensor < 0), so nothing steps it at startup. What it
        // then accumulates over the ~2 s between the reset and this read is
        // sensor noise, which is a drift claim, not a plumbing one.
        TEST_RESULT("a system reset leaves the hall position at zero in both sources",
                    afterReset.ok &&
                    llabs((long long)afterReset.getHall) <= HALL_DRIFT_TOL &&
                    llabs((long long)afterReset.compHall) <= HALL_DRIFT_TOL);
        TEST_RESULT("a system reset leaves no standing deviation between the sources",
                    afterReset.ok && llabs((long long)pscDeviation(afterReset)) <= HALL_DRIFT_TOL);

        // Build up a large, known deviation, then collapse it with zeroPosition.
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), (uint32_t)(2 * TFH_TPS));
        bool moved = (m->getError() == 0) && tfhDrain(m, 12000) && (tfhFatal(m) == 0);
        PscSnapshot loaded = moved ? pscRead(m) : PscSnapshot();
        int64_t loadedDev = loaded.ok ? pscDeviation(loaded) : 0;

        // Guarded on `moved`: zeroPosition against a queue that did not drain
        // raises fatal error 8 (motor_control.c:3313) and would turn one upstream
        // hiccup into a cascade of unrelated-looking failures.
        bool zeroAccepted = false;
        uint8_t zeroFatal = 0xFF;
        if (moved) {
            m->zeroPosition();                   // queue is drained: no fatal 8
            zeroAccepted = (m->getError() == 0);
            delay(250);
            zeroFatal = tfhFatal(m);
        }
        PscSnapshot zeroed = (zeroAccepted && zeroFatal == 0) ? pscRead(m) : PscSnapshot();
        printf("   deviation before zero %lld -> after zero %lld (fatal=%u)\n",
               (long long)loadedDev,
               (long long)(zeroed.ok ? pscDeviation(zeroed) : 0), (unsigned)zeroFatal);
        TEST_RESULT("zero position drives the commanded position to zero in both sources",
                    zeroed.ok && zeroed.getPos == 0 && zeroed.compCmd == 0);
        TEST_RESULT("zero position collapses the deviation instead of leaving a standing offset",
                    moved && llabs((long long)loadedDev) > TFH_CPR / 4 &&
                    zeroed.ok && llabs((long long)pscDeviation(zeroed)) <= HALL_DRIFT_TOL);

        // The identity must hold again from the new origin.
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), (uint32_t)(2 * TFH_TPS));
        bool removed = (m->getError() == 0) && tfhDrain(m, 12000) && (tfhFatal(m) == 0);
        PscSnapshot regrown = removed ? pscRead(m) : PscSnapshot();
        int64_t regrownDev = regrown.ok ? pscDeviation(regrown) : 0;
        printf("   after a further quarter rotation from the new origin: deviation %lld "
               "(commanded %lld)\n",
               (long long)regrownDev, (long long)(regrown.ok ? regrown.compCmd : 0));
        TEST_RESULT("the deviation identity holds again from the origin established by zero position",
                    regrown.ok &&
                    llabs((long long)(regrownDev - regrown.compCmd)) <= HALL_DRIFT_TOL);
        tfhReset(m);
    }

    // =====================================================================
    // 6. THE CONVERTED VIEWS AGREE BECAUSE THEY SHARE ONE CONVERSION.
    //
    //    Servomotor.cpp builds getPosition() and the comprehensive reply's
    //    commandedPosition with the SAME call:
    //
    //        convertPosition((float)raw, m_positionUnit, FROM_INTERNAL)
    //
    //    so with the machine at rest, and therefore the same raw int64 behind
    //    both, the two floats must be BIT-IDENTICAL in every unit. Anything
    //    less means one of the two paths applied a different factor -- which is
    //    a real hazard, because a library user reading position through cmd 37
    //    and setting limits through cmd 34 would silently be working in two
    //    different scales.
    //
    //    The position used here is half a rotation (1,638,400 counts), well
    //    under 2^24, so (float)raw is exact and the equality is meaningful
    //    rather than an artefact of rounding. The conversion factors are
    //    CONVERSION_FACTOR_SHAFT_ROTATIONS 3276800 and
    //    CONVERSION_FACTOR_DEGREES 9102.222 (AutoGeneratedUnitConversions.h).
    // =====================================================================
    printf("\n--- 6. the converted views agree with each other and with the raw counts ---\n");
    {
        pscIdle(m);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), (uint32_t)(2 * TFH_TPS));
        bool moved = (m->getError() == 0) && tfhDrain(m, 12000) && (tfhFatal(m) == 0);

        // The exact-equality claims below are only well founded if nothing is
        // moving between the reads, so establish that first.
        int64_t stable1 = m->getPositionRaw();
        int64_t stable2 = m->getPositionRaw();
        bool atRest = moved && (m->getError() == 0) && (stable1 == stable2);
        getComprehensivePositionResponse raw = m->getComprehensivePositionRaw();
        bool rawOk = atRest && (m->getError() == 0);
        printf("   raw commanded %lld, raw hall %lld\n",
               (long long)raw.commandedPosition, (long long)raw.hallSensorPosition);
        TEST_RESULT("the commanded position is stable while the converted views are read",
                    atRest);

        struct { PositionUnit unit; const char* label; float perCount; } views[3] = {
            { PositionUnit::ENCODER_COUNTS, "encoder counts", 1.0f },
            { PositionUnit::SHAFT_ROTATIONS, "shaft rotations", 1.0f / 3276800.0f },
            { PositionUnit::DEGREES,         "degrees",         1.0f / 9102.222222222f },
        };
        bool hallViewsAgree = true;
        float worstHallViewGap = 0.0f;
        for (int i = 0; i < 3; i++) {
            m->setPositionUnit(views[i].unit);
            float single = m->getPosition();
            bool okA = (m->getError() == 0);
            getComprehensivePositionResponseConverted comp = m->getComprehensivePosition();
            bool okB = (m->getError() == 0);
            float singleHall = m->getHallSensorPosition();
            bool okC = (m->getError() == 0);

            float expected = (float)raw.commandedPosition * views[i].perCount;
            printf("   %s: getPosition=%.4f comp.commanded=%.4f (expected %.4f) | "
                   "getHall=%.4f comp.hall=%.4f\n",
                   views[i].label, (double)single, (double)comp.commandedPosition,
                   (double)expected, (double)singleHall, (double)comp.hallSensorPosition);

            char name[128];
            snprintf(name, sizeof(name),
                     "in %s, Get position and the comprehensive reply return the identical value",
                     views[i].label);
            // Identical float, and both on the correct scale for the unit.
            TEST_RESULT(name, rawOk && okA && okB &&
                              single == comp.commandedPosition &&
                              approxEqual(single, expected,
                                          std::fabs(expected) * 1e-4f + 1e-3f));

            float gap = std::fabs(singleHall - comp.hallSensorPosition);
            float tol = (float)HALL_JITTER_TOL * views[i].perCount;
            if (!okC || gap > tol) hallViewsAgree = false;
            if (gap > worstHallViewGap) worstHallViewGap = gap;
        }
        TEST_RESULT("the converted hall values from both sources agree in every unit",
                    rawOk && hallViewsAgree);

        // The identity itself must survive the conversion: the deviation read in
        // rotations is the raw count difference divided by 3276800.
        m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
        getComprehensivePositionResponseConverted rotView = m->getComprehensivePosition();
        bool rotOk = (m->getError() == 0);
        float devRot = rotView.commandedPosition - rotView.hallSensorPosition;
        float devRotExpected = (float)(raw.commandedPosition - raw.hallSensorPosition) / 3276800.0f;
        printf("   deviation in rotations: %.5f, expected from raw counts %.5f\n",
               (double)devRot, (double)devRotExpected);
        TEST_RESULT("the commanded-minus-hall deviation survives the unit conversion intact",
                    rawOk && rotOk && approxEqual(devRot, devRotExpected, 0.01f));

        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE EXTERNAL ENCODER FIELD IS A GENUINELY SEPARATE THIRD QUANTITY.
    //
    //    The comprehensive reply's third field is not another view of the same
    //    state: get_external_encoder_position() (GPIO_interrupts.c:62) returns
    //    step_count, incremented from a GPIO edge interrupt on an OPTIONAL
    //    external quadrature encoder. With no encoder attached it must not
    //    track the commanded position at all. Checking this matters because the
    //    three fields are packed adjacently in one reply, and a packing or
    //    offset error would most plausibly show up as one field echoing
    //    another. motor_commands.json also warns that the motor's position unit
    //    must NOT be applied to it, which is only sound if it is independent.
    // =====================================================================
    printf("\n--- 7. the external encoder field does not shadow the other two ---\n");
    {
        pscIdleOpen(m);
        PscSnapshot before = pscRead(m);
        const int32_t dist = (int32_t)(TFH_CPR * 4);
        m->trapezoidMoveRaw(dist, (uint32_t)(4 * TFH_TPS));
        bool moved = (m->getError() == 0) && tfhDrain(m, 15000) && (tfhFatal(m) == 0);
        PscSnapshot after = moved ? pscRead(m) : before;

        int64_t cmdDelta  = after.compCmd  - before.compCmd;
        int64_t hallDelta = after.compHall - before.compHall;
        int64_t extDelta  = (int64_t)after.compExt - (int64_t)before.compExt;
        printf("   over a %ld count move: commanded %+lld, hall %+lld, external encoder %+lld "
               "(%ld -> %ld)\n",
               (long)dist, (long long)cmdDelta, (long long)hallDelta, (long long)extDelta,
               (long)before.compExt, (long)after.compExt);

        TEST_RESULT("the commanded field of the comprehensive reply delivers the full displacement",
                    before.ok && after.ok && moved &&
                    llabs((long long)(cmdDelta - dist)) <= TFH_CPR / 100);
        TEST_RESULT("the hall field of the comprehensive reply does not follow, with the output stage off",
                    before.ok && after.ok && moved &&
                    llabs((long long)hallDelta) <= HALL_DRIFT_TOL);
        // Deliberately generous: with nothing wired to the encoder input this is
        // expected to be exactly zero, but a floating input must not fail a test
        // about field independence. A tenth of the commanded travel would still
        // be four hundred thousand counts of noise.
        TEST_RESULT("the external encoder field does not track the commanded position",
                    before.ok && after.ok && moved &&
                    llabs((long long)extDelta) < llabs((long long)cmdDelta) / 10);
        tfhReset(m);
    }

    // =====================================================================
    // 8. NOTHING ACCUMULATES BETWEEN THE SOURCES OVER REPEATED CYCLES.
    //
    //    A one-count-per-move divergence between the commanded and hall paths
    //    would be invisible in any single measurement and would eventually
    //    fault a long-running machine with error 45 for no reason. Three
    //    out-and-back cycles with no reset in between put the commanded
    //    position back where it started; if the sources are consistent, the
    //    deviation must come back to zero with it.
    //
    //    Three rotations per leg is past the default window, so the ceilings
    //    are opened once at the start and left open for the whole section --
    //    there is deliberately NO reset between cycles, because a reset would
    //    hide exactly the accumulation being looked for.
    // =====================================================================
    printf("\n--- 8. the sources do not drift apart over repeated cycles ---\n");
    {
        pscIdleOpen(m);
        PscSnapshot origin = pscRead(m);
        bool everyCycleClean = origin.ok;
        const int32_t leg = (int32_t)(TFH_CPR * 3);

        for (int cycle = 0; cycle < 2; cycle++) {
            bool legsOk = true;
            int64_t worstGap = 0;
            for (int dir = 0; dir < 2; dir++) {
                m->trapezoidMoveRaw(dir == 0 ? leg : -leg, (uint32_t)(2 * TFH_TPS));
                if (m->getError() != 0 || !tfhDrain(m, 15000) || tfhFatal(m) != 0) {
                    legsOk = false;
                    break;
                }
                PscSnapshot s = pscRead(m);
                if (!s.ok) { legsOk = false; break; }
                int64_t cmdGap  = llabs((long long)(s.compCmd - s.getPos));
                int64_t hallGap = llabs((long long)(s.compHall - s.getHall));
                int64_t devGap  = llabs((long long)(pscDeviation(s) - pscDeviationFromGetters(s)));
                if (cmdGap > worstGap) worstGap = cmdGap;
                if (hallGap > worstGap) worstGap = hallGap;
                if (devGap > worstGap) worstGap = devGap;
                if (cmdGap != 0 || hallGap > HALL_JITTER_TOL || devGap > HALL_JITTER_TOL) {
                    legsOk = false;
                }
            }
            printf("   cycle %d: legs clean=%d, worst source disagreement %lld counts\n",
                   cycle, (int)legsOk, (long long)worstGap);
            char name[128];
            snprintf(name, sizeof(name),
                     "cycle %d: all three sources agree at both ends of an out-and-back",
                     cycle);
            TEST_RESULT(name, legsOk);
            if (!legsOk) everyCycleClean = false;
        }

        PscSnapshot back = pscRead(m);
        int64_t residue = back.ok && origin.ok ? (back.compCmd - origin.compCmd) : -1;
        int64_t devNow = back.ok ? pscDeviation(back) : -1;
        printf("   after the cycles: commanded residue %lld counts, deviation %lld counts\n",
               (long long)residue, (long long)devNow);
        TEST_RESULT("the commanded position returns to its starting value after the cycles",
                    everyCycleClean && back.ok && llabs((long long)residue) <= TFH_CPR / 100);
        TEST_RESULT("and the deviation returns to zero, so the two sources did not drift apart",
                    everyCycleClean && back.ok && llabs((long long)devNow) <= HALL_DRIFT_TOL);
        tfhReset(m);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean, at rest and with the output stage off",
                tfhFatal(m) == 0);
}
