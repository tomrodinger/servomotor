#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_error_code_reachability.cpp
//
// A CENSUS OF THE FATAL-ERROR CODE SPACE: WHICH OF THE 55 DOCUMENTED CODES CAN
// A CALLER ACTUALLY PROVOKE, AND DOES EACH ONE REPORT ITSELF?
//
// `python_programs/servomotor/error_codes.json` documents 55 codes, 0 through
// 54. That file is the whole diagnostic vocabulary of the product: over RS485 a
// faulted motor can tell an operator exactly one number, so a code that is
// wrong, or a code that no real command can ever produce, is a documentation
// defect with real consequences for whoever is standing at the machine.
//
// Nothing in the suite treats the code space AS a space. Individual modules own
// individual codes (tm_deviation_tracking owns 45, tm_queue_underrun owns 18,
// tm_hall_capture owns 20/21/49, tm_safety_fence owns 25/26/27,
// tm_multimove_capacity owns 24), and tm_error_taxonomy walks eight hand-picked
// provocations. This module asks the complementary question and asks it of
// EVERY code: for each of 0..54, is it reachable at all from a caller who may
// not energise the output stage, may not spin a shaft, and may not hang the
// device -- and if so, does the firmware report THAT code and no other?
//
// HOW IT IS DISTINCT FROM tm_error_taxonomy, which is the closest module:
//   * tm_error_taxonomy starts from eight PROVOCATIONS a user might stumble
//     into and asks what each produces; where two codes are both plausible it
//     accepts either (its fence check asserts "26 or 27", its over-speed check
//     asserts "15 or 16 or 28"). This module starts from the CODE and
//     constructs, from the firmware, a provocation that can produce that code
//     and no other -- so 26 and 27 get separate, pinned constructions, as do
//     15, 16 and 28.
//   * tm_error_taxonomy covers 8 codes. This one induces 18 and, for the
//     remaining 37, records in source WHY they are out of reach and cites the
//     firmware line that raises them. The reachability table it prints is the
//     deliverable: it is the thing a future firmware change would falsify.
//   * Deliberately NOT used: test mode values 14..73, which
//     `fatal_error(test_mode - 14)` (main.c:910) turns into an arbitrary forced
//     code. Forcing a code proves only that the reporting path works; it says
//     nothing about whether any real command can reach it, which is the entire
//     question here. Every code below is reached through an ordinary command.
//
// THE 18 CODES INDUCED HERE, and the firmware check each one comes from:
//
//    8  ERROR_QUEUE_NOT_EMPTY                    motor_control.c:3314
//   13  ERROR_NOT_IN_CLOSED_LOOP                 motor_control.c:1437
//   15  ERROR_ACCEL_TOO_HIGH                     motor_control.c:1796
//   16  ERROR_VEL_TOO_HIGH                       motor_control.c:1847
//   17  ERROR_QUEUE_IS_FULL                      motor_control.c:1869
//   18  ERROR_RUN_OUT_OF_QUEUE_ITEMS             motor_control.c:2164
//   20  ERROR_CAPTURE_PAYLOAD_TOO_BIG            main.c:338
//   23  ERROR_MAX_PWM_VOLTAGE_TOO_HIGH           motor_control.c:3514
//   25  ERROR_SAFETY_LIMIT_EXCEEDED              motor_control.c:2641
//   26  ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE      motor_control.c:1832
//   27  ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE  motor_control.c:1856
//   28  ERROR_PREDICTED_VELOCITY_TOO_HIGH        motor_control.c:1803
//   34  ERROR_PARAMETER_OUT_OF_RANGE             main.c:536 / main.c:617 / main.c:710
//   45  ERROR_POSITION_DEVIATION_TOO_LARGE       motor_control.c:3223
//   46  ERROR_MOVE_TOO_FAR                       motor_control.c:2077
//   49  ERROR_CAPTURE_BAD_PARAMETERS             main.c:317
//   51  ERROR_COMMAND_SIZE_WRONG                 main.c:203 (via main.c:242)
//   53  ERROR_INVALID_TEST_MODE                  main.c:922
//
// Code 34 is induced from THREE different commands (Move with acceleration,
// Move with velocity, Set safety limits). That is deliberate: a shared code is
// only useful if every command that shares it really does report it, and a
// regression in one of the three would otherwise hide behind the other two.
//
// THE 37 CODES DELIBERATELY NOT INDUCED are enumerated in `notReachable[]`
// below, each with the firmware site that raises it and the reason it is out of
// reach for an automated, MOSFETs-off, shared-rack test. In summary they fall
// into six families: (a) seven codes with NO `fatal_error()` call site anywhere
// in the firmware -- 4, 6, 12, 31, 32, 33, 35 are reserved vocabulary that
// nothing raises; (b) three whose call site is COMMENTED OUT in favour of a
// counter -- 36, 37, 38 (RS485.c:364-374); (c) codes that need the output stage
// energised or the shaft free -- 7, 9, 10, 11, 19, 22, 39, 41, 43, 44, 47;
// (d) codes raised only by internal consistency checks no command can steer --
// 1, 29, 30, 42; (e) codes needing hardware conditions -- 14 (overvoltage),
// 40 (overheat), 2/3 (flash failure); (f) codes reachable but forbidden or
// owned elsewhere -- 5 (bus flooding), 21 (floods the shared bus), 24 (needs a
// hand-built frame; owned by tm_multimove_capacity), 50 (needs Set device
// alias), 52 (bootloader only), 54 (M23 current streaming), and 0 (only via the
// excluded test mode 14).
//
// SAFETY. The MOSFETs stay OFF for the whole module: enableMosfets(),
// goToClosedLoop(), startCalibration() and vibrate() are never called, and only
// the COMMANDED position, the queue depth and the status word are read -- the
// planner advances the commanded position whether or not the output stage is
// energised, so nothing turns, on any motor, at any point. Seventeen of the
// twenty provocations are refused at QUEUE or COMMAND validation time, before a
// single tick executes. The other three are caught by per-tick checks in the
// control loop: 25 with nothing queued and nothing commanded at all, and 18 and
// 45 after commanding 0.1 and 3 rotations respectively of a shaft that is not
// energised (with the MOSFETs off the rotor does not follow, so those two
// numbers are commanded travel, not motion). Two provocations
// deliberately occupy the queue with ZERO-velocity segments, which hold the
// queue busy while commanding no motion whatsoever. Every fault is cleared by
// the tfhReset() at the head of the next probe, and that reset doubles as the
// recovery assertion. No broadcasts, no alias writes, no test mode in 10..13
// (which disable interrupts and spin forever) and none in 14..73 (which force
// codes): the only test mode sent is 200, which is rejected, and 0, which is
// the documented clear-all. Safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// The highest code defined in python_programs/servomotor/error_codes.json.
static const uint8_t HIGHEST_DEFINED_CODE = 54;   // ERROR_STREAMING_OVERFLOW

// tfhFatal() returns this when the status read itself failed.
static const uint8_t STATUS_READ_FAILED = 0xFF;

// Command IDs, checked against python_programs/servomotor/motor_commands.json.
static const uint8_t CMD_DISABLE_MOSFETS = 0;     // expects a ZERO-byte payload

// Empty the controller's receive buffer. Needed after any provocation whose
// reply nobody consumed (Servomotor::sendCommand does not read one) or whose
// reply raced a fault; a stray byte left behind would be mis-parsed as the head
// of the next response and turn a good status read into garbage.
static void drainRx(void) {
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
}

// ===========================================================================
// THE PROVOCATIONS.
//
// Each runs on a freshly reset machine (the driver calls tfhReset first) and
// must be constructed so that exactly ONE firmware check can fire. Where the
// firmware evaluates several checks in sequence, the numbers below are chosen
// so that the earlier ones pass -- that is what makes the assertion "it reports
// code N" meaningful rather than "it reports one of a family".
// ===========================================================================

// --- 15: an acceleration above the acceleration ceiling --------------------
// add_to_queue tests `llabs(parameter << ACCELERATION_SHIFT_LEFT) >
// max_acceleration` (motor_control.c:1795-1796, shift 8 at motor_control.h:64),
// and set_max_acceleration stores its own limit through the identical shift
// (motor_control.c:3366). Both sides are raw wire values, so 2000 against a
// ceiling of 1000 is exactly a factor of two over and cannot be borderline.
static void pAccelTooHigh(Servomotor* m) {
    m->setMaximumAccelerationRaw(1000);
    m->moveWithAccelerationRaw(2000, 1000);
}

// --- 16: a velocity above the velocity ceiling -----------------------------
// The velocity branch of add_to_queue tests `llabs(velocity << 12) >
// max_velocity` (motor_control.c:1846-1847) and it is the FIRST check in that
// branch, so nothing else can fire first.
static void pVelTooHigh(Servomotor* m) {
    m->setMaximumVelocityRaw(1000);
    m->moveWithVelocityRaw(2000, 1000);
}

// --- 28: an acceleration that is legal but would end up too fast -----------
// The distinction that matters: the acceleration itself passes its ceiling, and
// only the PREDICTED end-of-segment velocity `velocity_after_last_queue_item +
// acceleration * n_time_steps` exceeds the speed limit (motor_control.c:1799-
// 1803). 1000 raw against a ceiling of 1000000 is comfortably legal as an
// acceleration; over 100 ticks it predicts 25.6 million against a speed ceiling
// of 4.096 million, so only the velocity prediction can fire.
static void pPredictedVelTooHigh(Servomotor* m) {
    m->setMaximumAccelerationRaw(1000000);
    m->setMaximumVelocityRaw(1000);
    m->moveWithAccelerationRaw(1000, 100);
}

// --- 17: one more queue item than the queue holds --------------------------
// MOVEMENT_QUEUE_SIZE is 32 (motor_control.h:31) and the 33rd item falls into
// the `else` of `if(n_items_in_queue < MOVEMENT_QUEUE_SIZE)`
// (motor_control.c:1868-1869). Each item is a ZERO-velocity segment: it costs a
// slot and commands no motion at all, which makes this the safest possible way
// to fill the queue. Two seconds per item means the queue cannot drain while
// the 33 commands are going out, and a queue that empties at zero velocity
// cannot raise error 18 either.
static void pQueueIsFull(Servomotor* m) {
    for (int i = 0; i < 33; i++) m->moveWithVelocityRaw(0, 62500);
}

// --- 27: a predicted endpoint outside the safety fence ---------------------
// The velocity branch of add_to_queue has exactly ONE safety-zone check and no
// turn-point check (motor_control.c:1852-1856), so a velocity segment that
// overshoots a fence can only ever report 27 -- which is what makes 26 and 27
// separable at all. (Do not confuse this site with the identical-looking check
// at motor_control.c:1953, which lives in add_to_queue_test, the dry-run
// validator reached only by ADD_TO_QUEUE_TEST_COMMAND, never by a real move.)
// Two rotations per second for one second predicts 2 rotations against a fence
// a twentieth of a rotation wide; the current position (zero) is inside the
// fence, so the per-tick check (25) cannot fire first.
static void pPredictedPosOutOfZone(Servomotor* m) {
    tfhOpenCeilings(m);
    m->zeroPosition();
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setSafetyLimits(-0.05f, 0.05f);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->moveWithVelocity(2.0f, 31250.0f);        // 2 rot/s for 1 s -> 2 rotations
}

// --- 26: an excursion PAST the fence that comes back inside it -------------
// The turn-point check is the subtlest thing add_to_queue does
// (motor_control.c:1819-1833): a decelerating segment applied to a moving shaft
// carries it FORWARD past the point where it ends up, so the endpoint can be
// legal while the excursion is not. The firmware solves for the turn point and
// safety-checks the position there, AFTER the endpoint check has already
// passed. Reaching 26 therefore requires an endpoint inside the fence and a
// peak outside it, which is exactly what this builds:
//
//   * a velocity segment of 2 rot/s for 2 s leaves the queue predicting
//     4 rotations = 13,107,200 counts, just inside an upper fence of 13,127,200
//   * an acceleration segment of -100 rot/s^2 for 1250 ticks reverses 2 rot/s
//     to -2 rot/s (100 rot/s^2 x 0.04 s = 4 rot/s of change), so it ends within
//     ~210 counts of where it started -- still inside the fence
//   * it turns around 625 ticks in, 65,431 counts past the endpoint by the
//     firmware's own shifted formula, which is 45,000 counts OUTSIDE the fence
//
// The peak is the only value out of bounds, so 26 is the only code that can
// fire. The fault lands within milliseconds of the second command, long before
// the commanded position has travelled anywhere near the fence, so the per-tick
// check (25) cannot pre-empt it either.
static void pTurnPointOutOfZone(Servomotor* m) {
    tfhOpenCeilings(m);
    m->zeroPosition();
    m->setSafetyLimitsRaw((int64_t)-1000000, (int64_t)13127200);
    m->moveWithVelocity(2.0f, 62500.0f);        // ends the queue at 4 rotations
    m->moveWithAcceleration(-100.0f, 1250.0f);  // overshoots 0.02 rot, returns
}

// --- 25: the CURRENT position outside a newly imposed fence ----------------
// Unlike 26 and 27, which are queue-time predictions, 25 comes from the control
// loop comparing where the machine IS against the fence, every tick
// (motor_control.c:2639-2641). Moving out of a fence cannot reach it, because
// the queue-time checks refuse the move first. Moving the FENCE reaches it in
// one tick, with nothing queued and nothing commanded. Note the limits are the
// right way round, so the inverted-limits rejection (main.c:710) does not fire.
static void pSafetyLimitExceeded(Servomotor* m) {
    m->zeroPosition();
    m->setSafetyLimitsRaw((int64_t)1000000, (int64_t)2000000);   // 0 is below both
}

// --- 34, route 1 of 3: Move with acceleration, zero duration ---------------
static void pZeroDurationAccel(Servomotor* m) { m->moveWithAccelerationRaw(1000, 0); }
// --- 34, route 2 of 3: Move with velocity, zero duration -------------------
static void pZeroDurationVelocity(Servomotor* m) { m->moveWithVelocityRaw(1000, 0); }
// --- 34, route 3 of 3: Set safety limits with the limits inverted ----------
// An inverted fence would put EVERY position out of bounds and trip the
// per-tick check within one tick, racing this command's own reply, so the
// firmware rejects it deterministically at command level instead (main.c:710).
static void pInvertedSafetyLimits(Servomotor* m) {
    m->setSafetyLimitsRaw((int64_t)1000, (int64_t)-1000);
}

// --- 46: an absolute target more than an int32 away ------------------------
// Go to position carries an int32 target but the machine's position is int64,
// so the DISPLACEMENT it implies can overflow even when the target cannot
// (motor_control.c:2072-2077). Stepping 1000 counts negative first is enough:
// the implied displacement to INT32_MAX is then 2,147,484,647, one thousand
// past the limit. The check runs before compute_trapezoid_move, so no planning
// and no motion happen.
static void pMoveTooFar(Servomotor* m) {
    tfhOpenCeilings(m);
    m->zeroPosition();
    m->trapezoidMoveRaw(-1000, 31250);
    delay(300);
    m->goToPositionRaw((int32_t)2147483647L, 31250);
}

// --- 8: an operation that needs an idle queue, issued while it is busy -----
// zero_position refuses while anything is queued (motor_control.c:3312-3314):
// the origin would shift under moves already planned against it. The queue is
// held busy by a two-second ZERO-velocity segment, so the precondition is
// violated without commanding a single count of motion.
static void pQueueNotEmpty(Servomotor* m) {
    m->moveWithVelocityRaw(0, 62500);
    delay(200);
    m->zeroPosition();
}

// --- 13: homing without having gone to closed loop -------------------------
// start_homing's first act is to check the control mode
// (motor_control.c:1435-1437), before it adds anything to the queue, so this
// refusal costs nothing and moves nothing. Open loop is the power-on mode, so
// no setup is needed to be in the wrong state.
static void pNotInClosedLoop(Servomotor* m) {
    m->homingRaw(10000, 31250);
}

// --- 20: a capture whose reply would not fit in a packet -------------------
// The reply is one RS485 packet, so the firmware multiplies out
// n_points x channels x 2 bytes and refuses anything over 65535 - 5 - 4
// (main.c:335-338). 100,000 points on 3 channels is 600,000 bytes. The check
// runs BEFORE rs485_start_the_packet, so no capture starts and no capture
// traffic reaches the shared bus.
static void pCapturePayloadTooBig(Servomotor* m) {
    uint8_t buf[8];
    uint16_t actual = 0;
    m->captureHallSensorData(1, 100000, 7, 32, 1, 1, buf, sizeof(buf), &actual);
}

// --- 49: a capture type that does not exist --------------------------------
// capture_type must be 1, 2 or 3 (main.c:314-317); it is the first of the three
// capture parameter checks, so 99 can only produce 49 and cannot be confused
// with the zero-parameter or bad-bitmask routes to the same code.
static void pCaptureBadParameters(Servomotor* m) {
    uint8_t buf[8];
    uint16_t actual = 0;
    m->captureHallSensorData(99, 10, 7, 32, 1, 1, buf, sizeof(buf), &actual);
}

// --- 23: a motor current above what the PWM stage can express --------------
// set_max_motor_current bounds its argument by MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY
// (motor_control.c:3512-3514), which is 390 on the M17 (motor_control.c:57).
// 65535 is over every product's value, so this provocation is product-neutral.
// The check precedes recompute_pid_parameters_and_set_pwm_voltage, so no PWM
// setting is actually changed -- and with the output stage off there is nothing
// for a current setting to do in any case.
static void pMaxPwmVoltageTooHigh(Servomotor* m) {
    m->setMaximumMotorCurrentRaw(65535, 100);
}

// --- 51: a command carrying a payload it does not take ---------------------
// Disable MOSFETs (cmd 0) is declared with a zero-byte payload, and the handler
// opens with check_payload_size_is_zero (main.c:242 -> main.c:200-204). The
// library's typed methods cannot express this, so the frame goes out through
// sendCommand(), which sends but does not read the reply -- hence the drain.
// The refusal happens before disable_mosfets() runs, which is harmless either
// way since the output stage is already off.
static void pCommandSizeWrong(Servomotor* m) {
    const uint8_t junk[4] = { 0, 0, 0, 0 };
    m->sendCommand(CMD_DISABLE_MOSFETS, junk, sizeof(junk));
    delay(200);
    drainRx();
}

// --- 53: a test mode number that is not defined ----------------------------
// The test-mode dispatcher ends in a catch-all (main.c:921-922). 200 is chosen
// deliberately: it is far above the LED modes 10..13, which never return, and
// far above the forced-error range 14..73, which this module refuses to use.
static void pInvalidTestMode(Servomotor* m) {
    m->testMode(200);
}

// --- 45: commanding past the position-deviation window ---------------------
// One of only two provocations here that actually executes. With the output
// stage off the rotor never follows, so the deviation equals the commanded
// travel and the default two-rotation window (motor_control.c:97) trips on the
// way to three rotations (motor_control.c:3217-3223). Note this probe must NOT
// open the ceilings -- the default window is the thing being used.
static void pDeviationTooLarge(Servomotor* m) {
    m->setMaximumVelocity(4.0f);
    m->setMaximumAcceleration(2000.0f);
    m->trapezoidMoveRaw((int32_t)(TFH_CPR * 3), (uint32_t)(2 * TFH_TPS));
}

// --- 18: letting the queue run dry at a non-zero velocity ------------------
// The other executing provocation. A velocity segment HOLDS its velocity when
// its time expires; if nothing follows it, the executor finds the queue empty
// with a non-zero velocity and faults (motor_control.c:2158-2164). Omitting the
// terminating zero-velocity segment is therefore the whole provocation. The
// commanded travel is a tenth of a rotation, well inside the default deviation
// window, so 45 cannot pre-empt it.
static void pRunOutOfQueueItems(Servomotor* m) {
    m->moveWithVelocity(0.5f, 6250.0f);         // 0.5 rot/s for 0.2 s, no stop
}

// ===========================================================================
// The probe table.
// ===========================================================================
struct Probe {
    uint8_t     expected;      // the code error_codes.json documents
    const char* enumName;
    const char* what;          // plain-english description of the provocation
    void      (*run)(Servomotor*);
    uint16_t    settleMs;      // time for the fault to appear and latch
};

static const Probe probes[] = {
    { 15, "ERROR_ACCEL_TOO_HIGH",
          "an acceleration of twice the acceleration ceiling",          pAccelTooHigh,            400 },
    { 16, "ERROR_VEL_TOO_HIGH",
          "a velocity of twice the speed ceiling",                      pVelTooHigh,              400 },
    { 28, "ERROR_PREDICTED_VELOCITY_TOO_HIGH",
          "a legal acceleration held long enough to end up too fast",   pPredictedVelTooHigh,     400 },
    { 17, "ERROR_QUEUE_IS_FULL",
          "a 33rd item queued into a 32-slot queue",                    pQueueIsFull,             600 },
    { 27, "ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE",
          "a velocity move whose endpoint is outside the fence",        pPredictedPosOutOfZone,   500 },
    { 26, "ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE",
          "a reversal that overshoots the fence but returns inside it", pTurnPointOutOfZone,      600 },
    { 25, "ERROR_SAFETY_LIMIT_EXCEEDED",
          "a fence imposed that excludes where the machine already is", pSafetyLimitExceeded,     500 },
    { 34, "ERROR_PARAMETER_OUT_OF_RANGE",
          "Move with acceleration over zero time",                      pZeroDurationAccel,       400 },
    { 34, "ERROR_PARAMETER_OUT_OF_RANGE",
          "Move with velocity over zero time",                          pZeroDurationVelocity,    400 },
    { 34, "ERROR_PARAMETER_OUT_OF_RANGE",
          "Set safety limits with the two limits inverted",             pInvertedSafetyLimits,    400 },
    { 46, "ERROR_MOVE_TOO_FAR",
          "an absolute target more than an int32 of travel away",       pMoveTooFar,              600 },
    {  8, "ERROR_QUEUE_NOT_EMPTY",
          "Zero position issued while the queue is busy",               pQueueNotEmpty,           500 },
    { 13, "ERROR_NOT_IN_CLOSED_LOOP",
          "Homing issued while still in open loop",                     pNotInClosedLoop,         400 },
    { 20, "ERROR_CAPTURE_PAYLOAD_TOO_BIG",
          "a hall capture whose reply would not fit in one packet",     pCapturePayloadTooBig,    600 },
    { 49, "ERROR_CAPTURE_BAD_PARAMETERS",
          "a hall capture with a capture type that does not exist",     pCaptureBadParameters,    600 },
    { 23, "ERROR_MAX_PWM_VOLTAGE_TOO_HIGH",
          "a motor current above what the PWM stage can express",       pMaxPwmVoltageTooHigh,    400 },
    { 51, "ERROR_COMMAND_SIZE_WRONG",
          "Disable MOSFETs sent with a four-byte payload",              pCommandSizeWrong,        400 },
    { 53, "ERROR_INVALID_TEST_MODE",
          "a test mode number that is not defined",                     pInvalidTestMode,         400 },
    { 45, "ERROR_POSITION_DEVIATION_TOO_LARGE",
          "three rotations commanded into a two-rotation window",       pDeviationTooLarge,      6000 },
    { 18, "ERROR_RUN_OUT_OF_QUEUE_ITEMS",
          "a velocity segment with no terminating stop",                pRunOutOfQueueItems,     2500 },
};
static const int N_PROBES = (int)(sizeof(probes) / sizeof(probes[0]));

// ===========================================================================
// The complement: every code this module does NOT induce, with the firmware
// site that raises it and the reason it is out of reach. Together with
// probes[] this must account for all 55 documented codes, and section 4
// asserts exactly that -- so a code added to error_codes.json without being
// classified here will fail the module rather than slip through.
// ===========================================================================
struct Unreachable {
    uint8_t     code;
    const char* enumName;
    const char* why;
};

static const Unreachable notReachable[] = {
    {  0, "ERROR_NONE",                    "not an error; only test mode 14 makes it a fault state" },
    {  1, "ERROR_TIME_WENT_BACKWARDS",     "internal clock consistency check, microsecond_clock.c:62" },
    {  2, "ERROR_FLASH_UNLOCK_FAIL",       "settings.c:48; needs a flash write (alias) plus a hw failure" },
    {  3, "ERROR_FLASH_WRITE_FAIL",        "settings.c:86/112; needs a flash write plus a hw failure" },
    {  4, "ERROR_TOO_MANY_BYTES",          "reserved: no fatal_error() call site in the firmware" },
    {  5, "ERROR_COMMAND_OVERFLOW",        "RS485.c:396; needs flooding that would disturb the shared bus" },
    {  6, "ERROR_COMMAND_TOO_LONG",        "reserved: no fatal_error() call site in the firmware" },
    {  7, "ERROR_NOT_IN_OPEN_LOOP",        "motor_control.c:587; needs closed loop, i.e. MOSFETs on" },
    {  9, "ERROR_HALL_SENSOR",             "motor_control.c:704 is COMMENTED OUT: no live call site, and its context is calibration" },
    { 10, "ERROR_CALIBRATION_OVERFLOW",    "motor_control.c:740; inside calibration" },
    { 11, "ERROR_NOT_ENOUGH_MINIMA_OR_MAXIMA", "motor_control.c:881; inside calibration" },
    { 12, "ERROR_VIBRATION_FOUR_STEP",     "reserved: no fatal_error() call site in the firmware" },
    { 14, "ERROR_OVERVOLTAGE",             "GPIO_interrupts.c:45; hardware comparator or test mode 74" },
    { 19, "ERROR_MOTOR_BUSY",              "motor_control.c:1779; motor_busy needs calibration/homing/closed loop" },
    { 21, "ERROR_CAPTURE_OVERFLOW",        "motor_control.c:1379; needs capture traffic flooding the shared bus" },
    { 22, "ERROR_CURRENT_SENSOR_FAILED",   "motor_control.c:3563; only checked while energising the output stage" },
    { 24, "ERROR_MULTIMOVE_MORE_THAN_32_MOVES", "main.c:661; needs a hand-built frame, owned by tm_multimove_capacity" },
    { 29, "ERROR_DEBUG1",                  "ADC.c:195; internal buffer-size sanity checks" },
    { 30, "ERROR_CONTROL_LOOP_TOOK_TOO_LONG", "motor_control.c:3195; a profiler overrun, not commandable" },
    { 31, "ERROR_INDEX_OUT_OF_RANGE",      "reserved: no fatal_error() call site in the firmware" },
    { 32, "ERROR_CANT_PULSE_WHEN_INTERVALS_ACTIVE", "reserved: no fatal_error() call site in the firmware" },
    { 33, "ERROR_INVALID_RUN_MODE",        "reserved: no fatal_error() call site in the firmware" },
    { 35, "ERROR_DISABLE_MOSFETS_FIRST",   "reserved: no fatal_error() call site in the firmware" },
    { 36, "ERROR_FRAMING",                 "RS485.c:364; the fatal_error() call is commented out, a counter runs" },
    { 37, "ERROR_OVERRUN",                 "RS485.c:369; the fatal_error() call is commented out, a counter runs" },
    { 38, "ERROR_NOISE",                   "RS485.c:374; the fatal_error() call is commented out, a counter runs" },
    { 39, "ERROR_GO_TO_CLOSED_LOOP_FAILED", "motor_control.c:1167; inside go-to-closed-loop" },
    { 40, "ERROR_OVERHEAT",                "temperature.c:75; needs a genuinely hot driver IC" },
    { 41, "ERROR_TEST_MODE_ACTIVE",        "motor_control.c:596; needs a motor test mode driving the output stage" },
    { 42, "ERROR_POSITION_DISCREPANCY",    "motor_control.c:2153; internal planner-vs-executor cross-check" },
    { 43, "ERROR_OVERCURRENT",             "no fatal_error() call site; hardware protection" },
    { 44, "ERROR_PWM_TOO_HIGH",            "motor_control.c:3056; only evaluated with the output stage on" },
    { 47, "ERROR_HALL_POSITION_DELTA_TOO_LARGE", "motor_control.c:3149; needs the shaft to physically jump" },
    { 48, "ERROR_INVALID_FIRST_BYTE",      "a receive counter (first_bit_error_count), never a fatal error" },
    { 50, "ERROR_BAD_ALIAS",               "main.c:556; needs Set device alias, forbidden on the shared rack" },
    { 52, "ERROR_INVALID_FLASH_PAGE",      "bootloader_STM32G031/Src/main.c:207; bootloader only" },
    { 54, "ERROR_STREAMING_OVERFLOW",      "current_streaming.c:220; M23 current streaming only" },
};
static const int N_UNREACHABLE = (int)(sizeof(notReachable) / sizeof(notReachable[0]));

void tm_error_code_reachability(void) {
    Serial.println("tm_error_code_reachability: BEGIN\n");
    Servomotor* m = tfGetMotor();

    uint8_t observed[N_PROBES];
    for (int i = 0; i < N_PROBES; i++) observed[i] = 0;

    // =====================================================================
    // 1. THE BASELINE. Everything below is a difference from this state, so
    //    it is worth one measurement rather than an assumption: a machine
    //    that arrives already faulted would make every probe below report a
    //    stale code and pass for the wrong reason.
    // =====================================================================
    printf("\n--- 1. a freshly reset machine carries no code ---\n");
    {
        tfhReset(m);
        uint8_t code = tfhFatal(m);
        uint8_t queued = m->getNQueuedItems();
        printf("   after reset: fatal=%u queue=%u\n", (unsigned)code, (unsigned)queued);
        TEST_RESULT("a freshly reset machine reports fatal error code 0", code == 0);
        TEST_RESULT("and its motion queue is empty", queued == 0);
    }

    // =====================================================================
    // 2. THE SWEEP. One provocation per row, each constructed so that only
    //    one firmware check can fire. The reset at the head of each row is
    //    also the RECOVERY check for the row before it: if any fault
    //    survived a reset, the next probe would start dirty and `recovered`
    //    would record it.
    // =====================================================================
    printf("\n--- 2. inducing each reachable code in turn ---\n");
    {
        bool recovered = true;
        bool allFaulted = true;

        for (int i = 0; i < N_PROBES; i++) {
            tfhReset(m);
            if (tfhFatal(m) != 0) {
                recovered = false;
                printf("   !! the reset before probe %d did not clear the previous fault\n", i);
            }

            probes[i].run(m);
            delay(probes[i].settleMs);
            drainRx();
            uint8_t code = tfhFatal(m);
            observed[i] = code;
            if (code == 0 || code == STATUS_READ_FAILED) allFaulted = false;

            printf("   %-2u %-44s -> got %u\n",
                   (unsigned)probes[i].expected, probes[i].what, (unsigned)code);
            TEST_RESULT(std::string(probes[i].what) + " raises " +
                            probes[i].enumName + " (" +
                            std::to_string((unsigned)probes[i].expected) + ")",
                        code == probes[i].expected);
        }

        // A final reset so the last probe's recovery is checked too.
        tfhReset(m);
        if (tfhFatal(m) != 0) recovered = false;

        // Every provocation must actually fault. This catches the failure mode
        // that a per-code assertion cannot: a firmware that quietly permits an
        // illegal request would report 0, and 0 is a code, so without this the
        // reader has to notice it in twenty separate lines.
        TEST_RESULT("every provocation faults rather than being quietly permitted",
                    allFaulted);
        TEST_RESULT("a system reset clears every one of the induced faults", recovered);

        // The codes must also DISCRIMINATE. Three rows share code 34 by design;
        // the other seventeen are distinct, so the sweep must yield exactly 18
        // different values. A firmware that answered one catch-all code for
        // everything would pass many of the rows above by luck but not this.
        int distinct = 0;
        for (int i = 0; i < N_PROBES; i++) {
            bool seenBefore = false;
            for (int j = 0; j < i; j++) if (observed[j] == observed[i]) seenBefore = true;
            if (!seenBefore) distinct++;
        }
        printf("   %d provocations produced %d distinct codes (expected 18)\n",
               N_PROBES, distinct);
        TEST_RESULT("the twenty provocations produce eighteen distinct codes, not one catch-all",
                    distinct == 18);
    }

    // =====================================================================
    // 3. THE NEGATIVE CONTROLS. Half of section 2 would be satisfied by a
    //    firmware that refused everything, so each family of refusal is
    //    paired with the nearest LEGAL request. These are the assertions
    //    that make the refusals meaningful rather than merely present.
    // =====================================================================
    printf("\n--- 3. the legal counterpart of each refusal is accepted ---\n");
    {
        // (a) An acceleration EXACTLY at the ceiling. The check is
        //     `llabs(...) > max_acceleration`, so the boundary value must pass;
        //     if it were `>=` the whole ceiling would be off by one unit. The
        //     chain is closed with an explicit zero-velocity segment, because a
        //     queue that empties at speed raises error 18 (which is probe 20's
        //     subject, not this one's).
        tfhReset(m);
        m->setMaximumAccelerationRaw(1000);
        m->moveWithAccelerationRaw(1000, 31250);
        bool atCeilingOk = (m->getError() == 0);
        delay(300);
        m->moveWithVelocityRaw(0, 1);
        bool drained = tfhDrain(m, 8000);
        uint8_t f1 = tfhFatal(m);
        printf("   acceleration exactly at the ceiling: accepted=%d drained=%d fatal=%u\n",
               (int)atCeilingOk, (int)drained, (unsigned)f1);
        TEST_RESULT("an acceleration exactly at the ceiling is accepted, not refused",
                    atCeilingOk && drained && f1 == 0);
        tfhReset(m);

        // (b) A small, well-formed hall capture. Without this, codes 20 and 49
        //     would be satisfied by a capture command that never worked at all.
        uint8_t buf[64];
        uint16_t actual = 0;
        m->captureHallSensorData(1, 8, 7, 64, 1, 1, buf, sizeof(buf), &actual);
        bool captureOk = (m->getError() == 0);
        delay(300);
        drainRx();
        uint8_t f2 = tfhFatal(m);
        printf("   a legal 8-point hall capture: accepted=%d bytes=%u fatal=%u\n",
               (int)captureOk, (unsigned)actual, (unsigned)f2);
        TEST_RESULT("a small well-formed hall capture is accepted and returns data",
                    captureOk && f2 == 0 && actual > 0);
        tfhReset(m);

        // (c) A motor current inside the PWM stage's range.
        m->setMaximumMotorCurrentRaw(100, 100);
        bool currentOk = (m->getError() == 0);
        delay(200);
        uint8_t f3 = tfhFatal(m);
        printf("   a motor current of 100 (limit 390 on the M17): accepted=%d fatal=%u\n",
               (int)currentOk, (unsigned)f3);
        TEST_RESULT("a motor current inside the PWM stage's range is accepted",
                    currentOk && f3 == 0);
        tfhReset(m);

        // (d) Test mode 0, the documented clear-all. It is the one test-mode
        //     value this module sends other than the rejected 200, and it must
        //     NOT be refused -- it is how a caller gets out of a test mode.
        m->testMode(0);
        bool clearOk = (m->getError() == 0);
        delay(200);
        uint8_t f4 = tfhFatal(m);
        printf("   test mode 0 (clear all): accepted=%d fatal=%u\n",
               (int)clearOk, (unsigned)f4);
        TEST_RESULT("test mode 0, the documented clear-all, is accepted",
                    clearOk && f4 == 0);
        tfhReset(m);

        // (e) A move that stays INSIDE a fence, using the same fence that
        //     refused the move in probe 5. A fence that rejected everything
        //     would satisfy 25, 26 and 27 while being useless.
        tfhOpenCeilings(m);
        m->zeroPosition();
        m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
        m->setSafetyLimits(-0.05f, 0.05f);
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), TFH_TPS);   // 0.01 rotation
        bool insideOk = (m->getError() == 0) && tfhDrain(m, 8000);
        uint8_t f5 = tfhFatal(m);
        int64_t moved = (insideOk && f5 == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   0.01 rotation inside a 0.1-rotation fence: accepted=%d fatal=%u moved=%lld\n",
               (int)insideOk, (unsigned)f5, (long long)moved);
        TEST_RESULT("a move that stays inside the fence is allowed through",
                    insideOk && f5 == 0 &&
                    llabs((long long)(moved - TFH_CPR / 100)) <= TFH_CPR / 1000 + 4);
        tfhReset(m);
    }

    // =====================================================================
    // 4. THE REACHABILITY TABLE. The deliverable of the module: every code in
    //    error_codes.json, classified as INDUCED (with the value the machine
    //    actually reported) or as out of reach with the reason. The two
    //    assertions guard the audit itself -- a code that is in neither list,
    //    or in both, means the census has drifted from the vocabulary and the
    //    table below can no longer be trusted.
    // =====================================================================
    printf("\n--- 4. reachability of every documented code ---\n");
    {
        int classifiedOnce = 0;
        int misclassified = 0;

        printf("   code  %-44s  outcome\n", "enum");
        printf("   ----  %-44s  -------------------------------------------\n",
               "--------------------------------------------");

        for (int code = 0; code <= (int)HIGHEST_DEFINED_CODE; code++) {
            int inProbes = 0, probeIdx = -1;
            for (int i = 0; i < N_PROBES; i++) {
                if ((int)probes[i].expected == code) { inProbes++; if (probeIdx < 0) probeIdx = i; }
            }
            int inUnreachable = 0, unrIdx = -1;
            for (int i = 0; i < N_UNREACHABLE; i++) {
                if ((int)notReachable[i].code == code) { inUnreachable++; unrIdx = i; }
            }

            if ((inProbes > 0) && (inUnreachable == 0)) {
                classifiedOnce++;
                // Report the code the machine actually produced for every route.
                for (int i = 0; i < N_PROBES; i++) {
                    if ((int)probes[i].expected != code) continue;
                    printf("   %4d  %-44s  INDUCED, reported %u  (%s)\n",
                           code, probes[i].enumName, (unsigned)observed[i], probes[i].what);
                }
            }
            else if ((inUnreachable == 1) && (inProbes == 0)) {
                classifiedOnce++;
                printf("   %4d  %-44s  not safely reachable: %s\n",
                       code, notReachable[unrIdx].enumName, notReachable[unrIdx].why);
            }
            else {
                misclassified++;
                printf("   %4d  %-44s  ** UNCLASSIFIED (%d probe, %d unreachable) **\n",
                       code, "?", inProbes, inUnreachable);
            }
        }

        printf("   %d of %d codes classified exactly once; %d induced, %d out of reach\n",
               classifiedOnce, (int)HIGHEST_DEFINED_CODE + 1,
               (int)HIGHEST_DEFINED_CODE + 1 - N_UNREACHABLE, N_UNREACHABLE);
        TEST_RESULT("every documented code is classified exactly once, as induced or out of reach",
                    classifiedOnce == (int)HIGHEST_DEFINED_CODE + 1 && misclassified == 0);
        TEST_RESULT("eighteen of the fifty-five documented codes are reachable without the MOSFETs",
                    (int)HIGHEST_DEFINED_CODE + 1 - N_UNREACHABLE == 18);
    }

    // =====================================================================
    // 5. THE AFTERMATH. Twenty deliberate faults in a row is the harshest
    //    thing this module does to the machine, so the last word is that it
    //    is still an ordinary working motor and is handed back clean -- it is
    //    shared with 34 others.
    // =====================================================================
    printf("\n--- 5. the machine is ordinary again afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   a quarter-rotation move after the sweep: ok=%d moved=%lld of %lld\n",
               (int)ok, (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move works after twenty deliberate faults",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);

        tfhReset(m);
        TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
    }
}
