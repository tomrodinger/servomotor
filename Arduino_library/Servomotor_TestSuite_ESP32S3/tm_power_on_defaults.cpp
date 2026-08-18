#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_power_on_defaults.cpp
//
// WHAT, EXACTLY, IS THE POWER-ON STATE?
//
// `tm_reset_completeness` proves that a reset RESTORES the defaults: it cripples
// every setting, resets, and shows the crippling is gone. That is a RELATIVE
// claim -- it would still pass in full if a firmware change moved the default
// speed limit from 9.33 to 3 rotations/second, because the "after" would simply
// be compared against a new "before".
//
// This module asks the complementary, ABSOLUTE question:
//
//     what ARE the defaults, numerically, and are they the ones the firmware
//     source says they should be?
//
// Every number asserted here is a firmware constant that a well-meaning change
// could move without anybody noticing, because almost none of them has a getter
// and none is checked anywhere else in the suite:
//
//   MOSFETs off, open loop           mosfets.c is_mosfets_enabled(),
//                                    motor_control.c:170 (OPEN_LOOP_POSITION_CONTROL)
//   every status flag clear          device_status.h bits 0..6
//   no fatal error latched           get status (cmd 16) errorCode
//   commanded position zero          motor_control.c:153 current_position_i96 = {0}
//   motion queue empty               motor_control.c:118-120
//   clock restarts near zero         main.c:623-628 NVIC_SystemReset()
//   CRC32 REQUIRED                   RS485.c:49  static uint8_t crc32_enabled = 1
//   deviation window = 2 rotations   motor_control.c:97
//                                    DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION
//                                    (ONE_REVOLUTION_MICROSTEPS * 2)
//   velocity ceiling = MAX_VELOCITY  motor_control.c:164, motor_control.h:47-51
//                                    (MAX_RPM 560 -> 9.333 rotations/second)
//   accel ceiling = MAX_ACCELERATION motor_control.c:163, motor_control.h:53-55
//                                    (12000 rotations/second^2)
//   PID error window empty           motor_control.c:182-183 (INT32_MAX/INT32_MIN)
//
// HOW THE TWO CEILINGS ARE PINNED DOWN. Each is nailed from BOTH ends:
//
//   1. Independently DERIVED from the firmware's own formula, using the
//      counts-per-rotation and update rate the device itself reports (cmd 18),
//      and compared against what `Get debug values` (cmd 45) reads back. That
//      catches a changed rating.
//   2. BRACKETED BEHAVIOURALLY TO A SINGLE COUNT. `add_to_queue()` compares the
//      request against the ceiling with a strict `>` (motor_control.c:1846 for
//      velocity, :1795 for acceleration), and the wire parameter is shifted left
//      by VELOCITY_SHIFT_LEFT 12 / ACCELERATION_SHIFT_LEFT 8 (motor_control.h:64-65).
//      So the largest accepted parameter is exactly `ceiling >> shift`, and
//      `+1` must be refused. Asserting both sides means the enforced ceiling and
//      the reported ceiling cannot drift apart unnoticed.
//
// WHY THE BOUNDARY MOVES ARE SENT AS MULTIMOVE (cmd 29). A velocity segment
// HOLDS its velocity after its duration expires, and an acceleration segment
// leaves the velocity it built up; if the queue empties at speed the firmware
// raises ERROR_RUN_OUT_OF_QUEUE_ITEMS (motor_control.c:2164). At the boot
// acceleration ceiling the predicted-velocity check (:1802) limits a single
// segment to about 24 control ticks -- 0.8 ms -- far too short to get a second
// command onto the wire in time. Packing the segment and its zero-velocity stop
// into ONE multimove makes the pair atomic, so the underrun cannot happen.
//
// SAFETY. The MOSFETs are never enabled and the closed loop is never entered;
// everything measured is the COMMANDED position, which the planner advances
// whether or not the output stage is energised, so nothing turns on any motor
// at any point. The largest commanded excursion is 2.15 rotations, chosen to
// straddle the default deviation window; the deviation fault it raises is
// deliberate and is cleared by the reset that follows it. The one deliberately
// malformed frame (section 4) is a CRC-less EXTENDED-ADDRESSED packet: every
// other motor on the bus discards it at the unique-ID comparison
// (RS485.c:236-243) before anything else looks at it. No broadcasts, no alias
// changes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_ACCEL_TOO_HIGH               = 15;
static const uint8_t ERR_VEL_TOO_HIGH                 = 16;
static const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;

// motor_control.h:64-65
static const int32_t VELOCITY_SHIFT_LEFT     = 12;
static const int32_t ACCELERATION_SHIFT_LEFT = 8;

// The rated maxima the boot defaults are built from (motor_control.h:47, :53).
static const uint64_t RATED_MAX_RPM             = 560;
static const uint64_t RATED_MAX_ACCEL_ROT_S2    = 12000;

// The experimental headroom a user may deliberately ask for (motor_control.h:61-62).
static const int64_t EXPERIMENTAL_VELOCITY_FACTOR     = 2;
static const int64_t EXPERIMENTAL_ACCELERATION_FACTOR = 4;

static const int32_t I32MAX =  2147483647;
static const int32_t I32MIN = -2147483647 - 1;

// ---------------------------------------------------------------------------
// The firmware's own derivation of the boot velocity ceiling, replayed exactly,
// integer truncation and all (motor_control.h:42, :49-51):
//     MICROSTEPS_PER_ROTATION      = countsPerRotation << 32
//     MAX_MICROSTEPS_PER_SECOND    = MAX_RPM * MICROSTEPS_PER_ROTATION / 60
//     MAX_MICROSTEPS_PER_TIME_STEP = MAX_MICROSTEPS_PER_SECOND / updateFrequency
// The division order matters: doing it any other way changes the last few units
// and the comparison against the readback would fail for no real reason.
// ---------------------------------------------------------------------------
static int64_t derivedMaxVelocity(uint32_t countsPerRotation, uint32_t updateFrequency) {
    uint64_t microstepsPerRotation = (uint64_t)countsPerRotation << 32;
    uint64_t perSecond = RATED_MAX_RPM * microstepsPerRotation / 60ULL;
    return (int64_t)(perSecond / (uint64_t)updateFrequency);
}

// Likewise for acceleration (motor_control.h:54-55). The firmware divides by the
// update rate FIRST, because 12000 * MICROSTEPS_PER_ROTATION would overflow 64
// bits; replaying it in the other order gives a different truncation.
static int64_t derivedMaxAcceleration(uint32_t countsPerRotation, uint32_t updateFrequency) {
    uint64_t microstepsPerRotation = (uint64_t)countsPerRotation << 32;
    uint64_t perTick = microstepsPerRotation / (uint64_t)updateFrequency;
    return (int64_t)((perTick * RATED_MAX_ACCEL_ROT_S2) / (uint64_t)updateFrequency);
}

// Internal velocity is in units of 2^-32 encoder counts per control tick, and
// the planner simply adds it to a 96-bit position once per tick. So the exact
// displacement of a velocity segment is (velocity * ticks) >> 32, computed here
// in two halves to stay inside int64 without needing 128-bit arithmetic.
static int64_t countsForVelocitySegment(int64_t velocityInternal, uint32_t ticks) {
    int64_t whole = (velocityInternal >> 32) * (int64_t)ticks;
    int64_t frac  = (int64_t)((((uint64_t)velocityInternal & 0xFFFFFFFFULL) *
                               (uint64_t)ticks) >> 32);
    return whole + frac;
}

// Convert an internal velocity to rotations/second, for printing and for the
// human-scale assertion that the ceiling really is the rated 560 rpm.
static double velocityToRotPerSec(int64_t velocityInternal, uint32_t cpr, uint32_t freq) {
    return (double)velocityInternal * (double)freq /
           ((double)cpr * 4294967296.0);
}

static double accelerationToRotPerSec2(int64_t accelInternal, uint32_t cpr, uint32_t freq) {
    return (double)accelInternal * (double)freq * (double)freq /
           ((double)cpr * 4294967296.0);
}

// ---------------------------------------------------------------------------
// Command a move and watch the commanded position until the queue drains or a
// fault stops the reads. Returns how far the commanded position had travelled
// when the watching stopped; `code` receives the latched fatal error (0 if the
// move simply completed). Same shape as tm_deviation_tracking's tripPoint --
// the sampled distance lags the real one because it is polled over the bus, so
// it is only ever used as an approximate readout, never as a tight bound.
// ---------------------------------------------------------------------------
static int64_t runAndWatch(Servomotor* m, int32_t distance, uint32_t ticks, uint8_t* code) {
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw(distance, ticks);
    if (m->getError() != 0) { *code = 0xFD; return 0; }

    int64_t furthest = 0;
    uint32_t deadline = millis() + (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 8000;
    while (millis() < deadline) {
        int64_t p = m->getPositionRaw();
        if (m->getError() != 0) break;              // a latched fault refuses reads
        furthest = p - before;
        if (m->getNQueuedItems() == 0 && m->getError() == 0) break;
        delay(15);
    }
    *code = tfhFatal(m);
    return furthest;
}

void tm_power_on_defaults(void) {
    Serial.println("tm_power_on_defaults: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 0. THE GROUND TRUTH THE DERIVATIONS REST ON. Both ceiling derivations
    //    below are built from the counts-per-rotation and the control-loop
    //    rate. Rather than hard-code them, ask the device (Get product specs,
    //    cmd 18 -- main.c:512-525 returns get_update_frequency() and
    //    ONE_REVOLUTION_MICROSTEPS) and check the answer agrees with the
    //    constants the rest of this suite assumes. If it did not, every
    //    number below would be derived from the wrong machine.
    // =====================================================================
    printf("\n--- 0. the machine constants the derivations use ---\n");
    tfhReset(m);
    getProductSpecsResponse specs = m->getProductSpecs();
    bool specsOk = (m->getError() == 0);
    uint32_t cpr  = specsOk ? specs.countsPerRotation : (uint32_t)TFH_CPR;
    uint32_t freq = specsOk ? specs.updateFrequency   : (uint32_t)TFH_TPS;
    printf("   countsPerRotation=%lu  updateFrequency=%lu Hz\n",
           (unsigned long)cpr, (unsigned long)freq);
    TEST_RESULT("the device reports the counts-per-rotation and control rate "
                "this module's derivations assume",
                specsOk && cpr == (uint32_t)TFH_CPR && freq == (uint32_t)TFH_TPS);

    const int64_t derivedVel = derivedMaxVelocity(cpr, freq);
    const int64_t derivedAcc = derivedMaxAcceleration(cpr, freq);

    // =====================================================================
    // 1. NOTHING IS RUNNING AND NOTHING IS FAULTED. The status word is the
    //    one place a caller can see the whole machine at a glance, so every
    //    bit of it is nailed down individually rather than as one lump: a
    //    future flag that came up set would otherwise hide inside a
    //    "flags == 0" that had been relaxed.
    // =====================================================================
    printf("\n--- 1. every status flag is clear and no fault is latched ---\n");
    {
        tfhReset(m);
        getStatusResponse st = m->getStatus();
        bool ok = (m->getError() == 0);
        uint16_t f = ok ? st.statusFlags : 0xFFFF;
        getFirmwareVersionResponse fw = m->getFirmwareVersion();
        bool fwOk = (m->getError() == 0);

        printf("   statusFlags=0x%04X fatalErrorCode=%u  firmware %u.%u.%u.%u inBootloader=%u\n",
               (unsigned)f, (unsigned)(ok ? st.fatalErrorCode : 0xFF),
               (unsigned)fw.firmwareVersion.major, (unsigned)fw.firmwareVersion.minor,
               (unsigned)fw.firmwareVersion.patch, (unsigned)fw.firmwareVersion.dev,
               (unsigned)fw.inBootloader);

        TEST_RESULT("get status is answerable on a freshly reset machine", ok);
        TEST_RESULT("no fatal error is latched at power-on",
                    ok && st.fatalErrorCode == 0);
        TEST_RESULT("the whole status word reads zero at power-on", ok && f == 0);
        // Bit 1 -- device_status.h STATUS_MOSFETS_ENABLED_FLAG_BIT. The single
        // most important default: a machine that energised its output stage on
        // boot would move the instant it was powered.
        TEST_RESULT("the MOSFETs are off at power-on", ok && (f & (1u << 1)) == 0);
        // Bit 2 -- motor_control.c:170, motor_control_mode = OPEN_LOOP_POSITION_CONTROL.
        TEST_RESULT("the motor boots in open loop, not closed loop",
                    ok && (f & (1u << 2)) == 0);
        // Bits 3, 4 and 5 -- calibration, homing and go-to-closed-loop are all
        // long-running procedures; none may be in progress on a cold machine.
        TEST_RESULT("no calibration, homing or go-to-closed-loop is in progress at power-on",
                    ok && (f & ((1u << 3) | (1u << 4) | (1u << 5))) == 0);
        // Bit 6 -- motor_busy. add_to_queue() refuses everything while it is set
        // (motor_control.c:1779), so a stuck busy bit would make the machine
        // look dead.
        TEST_RESULT("the motor is not busy at power-on", ok && (f & (1u << 6)) == 0);
        // Bit 0 plus the independent report from Get firmware version
        // (main.c:604 hard-codes in_bootloader = 0 in the application image).
        TEST_RESULT("the device is running the application, not the bootloader",
                    ok && fwOk && (f & 1u) == 0 && fw.inBootloader == 0);
    }

    // =====================================================================
    // 2. THE DIRECTLY READABLE STATE: ORIGIN, QUEUE, CLOCK AND OUTPUT. These
    //    need no behavioural proxy, so they are asserted exactly rather than
    //    approximately -- position to the count, queue to the item.
    // =====================================================================
    printf("\n--- 2. origin, queue, clock and output stage ---\n");
    {
        tfhReset(m);
        int64_t pos = m->getPositionRaw();
        bool posOk = (m->getError() == 0);
        getComprehensivePositionResponse comp = m->getComprehensivePositionRaw();
        bool compOk = (m->getError() == 0);
        uint8_t depth = m->getNQueuedItems();
        bool depthOk = (m->getError() == 0);

        uint64_t t0 = m->getCurrentTimeRaw();
        bool clockOk = (m->getError() == 0);
        delay(500);
        uint64_t t1 = m->getCurrentTimeRaw();
        clockOk = clockOk && (m->getError() == 0);
        int64_t elapsedUs = (int64_t)(t1 - t0);

        getDebugValuesResponse dbg = m->getDebugValues();
        bool dbgOk = (m->getError() == 0);
        getDebugValuesResponse dbg2 = m->getDebugValues();
        dbgOk = dbgOk && (m->getError() == 0);

        printf("   position=%lld comprehensive.commanded=%lld queue=%u\n",
               (long long)pos, (long long)comp.commandedPosition, (unsigned)depth);
        printf("   clock t0=%llu us, +500 ms later t1=%llu us (delta %lld us), pwm=%u/%u\n",
               (unsigned long long)t0, (unsigned long long)t1,
               (long long)elapsedUs,
               (unsigned)dbg.motorPwmVoltage, (unsigned)dbg2.motorPwmVoltage);

        TEST_RESULT("the commanded position is exactly zero at power-on",
                    posOk && pos == 0);
        // Two independent commands (34 and 37) must agree about the origin.
        TEST_RESULT("get position and get comprehensive position both read zero",
                    posOk && compOk && comp.commandedPosition == 0 &&
                    comp.commandedPosition == pos);
        TEST_RESULT("the motion queue is empty at power-on", depthOk && depth == 0);
        // A system reset is a real MCU reset (main.c:628 NVIC_SystemReset), so
        // the microsecond clock restarts. tfhReset waits 1.5 s, so a few
        // seconds is the whole plausible range.
        TEST_RESULT("the device clock restarts near zero",
                    clockOk && t0 < 5000000ULL);
        TEST_RESULT("and it then advances at about real time",
                    clockOk && elapsedUs > 400000 && elapsedUs < 700000);
        // WHAT THE PWM READOUT ACTUALLY IS AT BOOT -- and it is NOT zero.
        // This assertion originally read `dbg.motorPwmVoltage == 0` and would
        // have FAILED on healthy firmware. In open loop (the boot mode), with
        // nothing moving and no calibration running, the M17 branch of
        // motor_phase_calculations() parks the PWM magnitude at HALF the
        // ceiling (motor_control.c:2706-2718):
        //     else { motor_pwm_voltage = max_motor_pwm_voltage >> 1; }
        // and max_motor_pwm_voltage boots at DEFAULT_MAX_MOTOR_PWM_VOLTAGE
        // (motor_control.h:129-134), so a production M17 reports 100 here. The
        // M1/M2 and M23 branches park identically. It is harmless because the
        // MOSFETs are off: the number is what the commutation stage WOULD
        // drive, not what reaches the coils.
        //
        // tm_response_field_integrity section 9b already pins the exact
        // idle == moving >> 1 relation, and it needs a move to do so. Without
        // commanding motion, all that is verifiable here is that the park value
        // is present and steady -- so that is what is asserted. A firmware
        // change that dropped the idle park to zero, or made it wander, is
        // still caught.
        TEST_RESULT("the PWM readout holds the open-loop idle park value "
                    "(half the boot ceiling), which is not zero",
                    dbgOk && dbg.motorPwmVoltage > 0);
        TEST_RESULT("and that idle park value is steady across two reads",
                    dbgOk && dbg2.motorPwmVoltage == dbg.motorPwmVoltage);
    }

    // =====================================================================
    // 3. THE PID ERROR WINDOW STARTS EMPTY. min_PID_error and max_PID_error
    //    are seeded with INT32_MAX and INT32_MIN (motor_control.c:182-183),
    //    the standard running-min/running-max initialisation. On an idle
    //    machine that means the reported bounds are INVERTED -- min above max
    //    -- which is correct but looks absurd, and the naive `min <= max`
    //    invariant is FALSE. It is asserted here as a documented default so a
    //    future change of seed is caught, and so nobody rediscovers it the
    //    hard way (tm_diagnostics_readouts' first draft did).
    // =====================================================================
    printf("\n--- 3. the PID error window is the empty sentinel ---\n");
    {
        tfhReset(m);
        getMaxPidErrorResponse w = m->getMaxPidErrorRaw();
        bool ok = (m->getError() == 0);
        bool sentinel = (w.minPidError == I32MAX) && (w.maxPidError == I32MIN);
        printf("   min=%ld max=%ld (%s)\n", (long)w.minPidError, (long)w.maxPidError,
               sentinel ? "empty sentinel" : "populated");

        TEST_RESULT("get max PID error is answerable at power-on", ok);
        TEST_RESULT("the PID error window holds the empty sentinel at power-on, "
                    "with the minimum above the maximum",
                    ok && sentinel);

        // get_max_PID_error() re-seeds the accumulators on every read
        // (motor_control.c:3483-3485), and with the closed loop never entered
        // nothing repopulates them, so the sentinel must persist.
        bool stillEmpty = ok;
        for (int i = 0; i < 10 && stillEmpty; i++) {
            getMaxPidErrorResponse r = m->getMaxPidErrorRaw();
            if (m->getError() != 0) { stillEmpty = false; break; }
            if (!((r.minPidError == I32MAX) && (r.maxPidError == I32MIN))) stillEmpty = false;
        }
        TEST_RESULT("it is still the empty sentinel after ten reads, confirming the "
                    "closed loop never ran", stillEmpty);
    }

    // =====================================================================
    // 4. CRC32 IS REQUIRED FROM POWER-ON. RS485.c:49 seeds crc32_enabled to 1,
    //    so a cold device answers protected frames and ignores unprotected
    //    ones. This is the default with the loudest failure mode: a caller who
    //    assumes the opposite gets total silence rather than an error, and the
    //    link looks like a wiring fault.
    //
    //    The probe changes only the LIBRARY side (enableCRC32/disableCRC32);
    //    the device's own setting (CRC32 control, cmd 46) is never touched, so
    //    the machine is left exactly as it booted.
    // =====================================================================
    printf("\n--- 4. an unprotected frame is ignored at power-on ---\n");
    {
        tfhReset(m);
        m->enableCRC32();
        m->getStatus();
        int errProtected = m->getError();

        m->disableCRC32();                  // stop appending the CRC
        m->getStatus();                     // the device must not answer this
        int errUnprotected = m->getError();
        // What the device actually does with that frame: the library's size
        // byte stops counting the four CRC bytes (Communication.cpp:169-172),
        // so the device reads the whole, short packet and then rejects it at
        // the length check in rs485_get_next_packet() (RS485.c:254-258,
        // packet_size < bytes_parsed + command + crc32_size). It bumps
        // packet_decode_error_count and answers nothing; no frame is left
        // half-parsed. The silence and the drain below are therefore
        // belt-and-braces rather than a requirement -- they also cover a
        // firmware that chose to answer with an error frame instead of staying
        // silent, whose bytes would otherwise be read as the head of the next
        // response.
        delay(300);
        for (uint32_t t = millis(); Serial1.available() && (millis() - t) < 500; ) Serial1.read();

        m->enableCRC32();                   // put it back
        m->getStatus();
        int errRestored = m->getError();

        printf("   protected=%d unprotected=%d restored=%d\n",
               errProtected, errUnprotected, errRestored);
        TEST_RESULT("a CRC-protected frame is answered at power-on", errProtected == 0);
        TEST_RESULT("an unprotected frame is refused at power-on, so CRC32 is "
                    "required by default", errUnprotected != 0);
        TEST_RESULT("re-attaching the CRC restores the link", errRestored == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 5. THE DEFAULT SPEED CEILING. max_velocity boots at MAX_VELOCITY
    //    (motor_control.c:164), which motor_control.h:47-51 builds from the
    //    rated 560 rpm. Checked three ways: against the firmware's own
    //    derivation, in human units, and by finding the exact parameter at
    //    which add_to_queue() flips from accepting to refusing.
    // =====================================================================
    printf("\n--- 5. the default speed ceiling ---\n");
    int64_t bootVel = 0;
    {
        tfhReset(m);
        getDebugValuesResponse dbg = m->getDebugValues();
        bool ok = (m->getError() == 0);
        bootVel = ok ? dbg.maxVelocity : 0;
        double rotPerSec = velocityToRotPerSec(bootVel, cpr, freq);
        printf("   boot maxVelocity=%lld (derived %lld) = %.4f rot/s = %.1f rpm\n",
               (long long)bootVel, (long long)derivedVel, rotPerSec, rotPerSec * 60.0);

        TEST_RESULT("the reported boot speed ceiling is exactly the firmware's "
                    "derived MAX_VELOCITY", ok && bootVel == derivedVel);
        TEST_RESULT("that ceiling is the rated 560 rpm (9.333 rotations/second)",
                    ok && rotPerSec > 9.30 && rotPerSec < 9.37);

        // The largest wire parameter that survives `llabs(value << 12) > max_velocity`
        // (motor_control.c:1846), and the one count past it.
        int64_t atCeiling = bootVel >> VELOCITY_SHIFT_LEFT;
        bool fitsInt32 = (atCeiling > 0) && (atCeiling < 2147483647LL);
        const uint32_t segTicks = 1000;     // 32 ms, about 0.3 rotations

        // Accepted side. Sent as ONE multimove so the segment and its
        // zero-velocity stop are atomic: a velocity segment holds its velocity,
        // and a queue that empties at speed raises error 18.
        int64_t moved = 0;
        bool accepted = false, completed = false;
        if (fitsInt32) {
            multimoveList_t mv[2];
            mv[0].value = (int32_t)atCeiling;  mv[0].timeSteps = segTicks;
            mv[1].value = 0;                   mv[1].timeSteps = 1;
            int64_t before = m->getPositionRaw();
            m->multimoveRaw(2, 0x3u, mv);      // bits 0 and 1 set: both velocity moves
            accepted = (m->getError() == 0);
            if (accepted) {
                completed = tfhDrain(m, 8000) && (tfhFatal(m) == 0);
                if (completed) moved = m->getPositionRaw() - before;
            }
        }
        int64_t predicted = countsForVelocitySegment(atCeiling << VELOCITY_SHIFT_LEFT, segTicks);
        printf("   parameter %lld accepted=%d moved=%lld (predicted %lld)\n",
               (long long)atCeiling, (int)accepted, (long long)moved, (long long)predicted);
        TEST_RESULT("a velocity segment exactly at the boot ceiling is accepted",
                    fitsInt32 && accepted && completed);
        // One control tick of this segment is about 979 counts, so a window of
        // 64 counts still catches a single missing or extra tick -- the thing
        // actually worth catching -- without being hostage to the last count of
        // fixed-point rounding in either the firmware's 96-bit accumulator or
        // the prediction above. The campaign has already burned a debugging
        // cycle on a one-count prediction mismatch of exactly this shape
        // ("a velocity chain covering 0.5 rot measured 1638399 of 1638400").
        TEST_RESULT("and it delivers the displacement that ceiling predicts, to "
                    "well inside one control tick",
                    completed && llabs((long long)(moved - predicted)) <= 64);

        // Refused side. A refused move never runs, so no stop segment is needed.
        tfhReset(m);
        m->moveWithVelocityRaw((int32_t)(atCeiling + 1), segTicks);
        uint8_t fat = tfhFatal(m);
        printf("   parameter %lld (one count higher) -> fatal=%u\n",
               (long long)(atCeiling + 1), (unsigned)fat);
        TEST_RESULT("one count above the boot ceiling is refused with "
                    "ERROR_VEL_TOO_HIGH (16)", fat == ERR_VEL_TOO_HIGH);

        // A user may deliberately exceed the rating, but only up to
        // EXPERIMENTAL_MAX_VELOCITY = 2 x MAX_VELOCITY (motor_control.h:61),
        // clamped at motor_control.c:3349-3351. Asking for the largest possible
        // value therefore reveals the factor exactly.
        tfhReset(m);
        m->setMaximumVelocityRaw(0xFFFFFFFFu);
        bool setOk = (m->getError() == 0);
        getDebugValuesResponse after = m->getDebugValues();
        bool afterOk = (m->getError() == 0);
        printf("   asking for the maximum possible ceiling gives %lld (boot x %.2f)\n",
               (long long)after.maxVelocity,
               bootVel ? (double)after.maxVelocity / (double)bootVel : 0.0);
        TEST_RESULT("the settable speed ceiling clamps at exactly twice the boot default",
                    setOk && afterOk &&
                    after.maxVelocity == bootVel * EXPERIMENTAL_VELOCITY_FACTOR);
        tfhReset(m);
    }

    // =====================================================================
    // 6. THE DEFAULT ACCELERATION CEILING. Same shape as section 5, against
    //    max_acceleration (motor_control.c:163 = MAX_ACCELERATION,
    //    motor_control.h:53-55, rated 12000 rotations/second^2) and the
    //    strict `>` at motor_control.c:1795.
    // =====================================================================
    printf("\n--- 6. the default acceleration ceiling ---\n");
    int64_t bootAcc = 0;
    {
        tfhReset(m);
        getDebugValuesResponse dbg = m->getDebugValues();
        bool ok = (m->getError() == 0);
        bootAcc = ok ? dbg.maxAcceleration : 0;
        double rotPerSec2 = accelerationToRotPerSec2(bootAcc, cpr, freq);
        printf("   boot maxAcceleration=%lld (derived %lld) = %.1f rot/s^2\n",
               (long long)bootAcc, (long long)derivedAcc, rotPerSec2);

        TEST_RESULT("the reported boot acceleration ceiling is exactly the firmware's "
                    "derived MAX_ACCELERATION", ok && bootAcc == derivedAcc);
        TEST_RESULT("that ceiling is the rated 12000 rotations/second squared",
                    ok && rotPerSec2 > 11900.0 && rotPerSec2 < 12100.0);

        int64_t atCeiling = bootAcc >> ACCELERATION_SHIFT_LEFT;
        bool fitsInt32 = (atCeiling > 0) && (atCeiling < 2147483647LL);

        // At the ceiling, the predicted-velocity check (motor_control.c:1802)
        // allows only about max_velocity / max_acceleration ticks -- roughly 24
        // -- before the segment would exceed the speed ceiling too. 20 leaves
        // headroom so the verdict is about acceleration and nothing else.
        const uint32_t accelTicks = 20;
        bool accepted = false, completed = false;
        if (fitsInt32) {
            multimoveList_t mv[2];
            mv[0].value = (int32_t)atCeiling;  mv[0].timeSteps = accelTicks;
            mv[1].value = 0;                   mv[1].timeSteps = 1;
            m->multimoveRaw(2, 0x2u, mv);      // bit 0 clear: acceleration; bit 1 set: velocity stop
            accepted = (m->getError() == 0);
            if (accepted) completed = tfhDrain(m, 8000) && (tfhFatal(m) == 0);
        }
        printf("   parameter %lld accepted=%d completed=%d\n",
               (long long)atCeiling, (int)accepted, (int)completed);
        TEST_RESULT("an acceleration segment exactly at the boot ceiling is accepted",
                    fitsInt32 && accepted && completed);

        tfhReset(m);
        m->moveWithAccelerationRaw((int32_t)(atCeiling + 1), accelTicks);
        uint8_t fat = tfhFatal(m);
        printf("   parameter %lld (one count higher) -> fatal=%u\n",
               (long long)(atCeiling + 1), (unsigned)fat);
        TEST_RESULT("one count above the boot ceiling is refused with "
                    "ERROR_ACCEL_TOO_HIGH (15)", fat == ERR_ACCEL_TOO_HIGH);

        // EXPERIMENTAL_MAX_ACCELERATION = 4 x MAX_ACCELERATION
        // (motor_control.h:62), clamped at motor_control.c:3368-3370. Note the
        // factor differs from velocity's -- an asymmetry worth pinning down.
        tfhReset(m);
        m->setMaximumAccelerationRaw(0xFFFFFFFFu);
        bool setOk = (m->getError() == 0);
        getDebugValuesResponse after = m->getDebugValues();
        bool afterOk = (m->getError() == 0);
        printf("   asking for the maximum possible ceiling gives %lld (boot x %.2f)\n",
               (long long)after.maxAcceleration,
               bootAcc ? (double)after.maxAcceleration / (double)bootAcc : 0.0);
        TEST_RESULT("the settable acceleration ceiling clamps at exactly four times "
                    "the boot default",
                    setOk && afterOk &&
                    after.maxAcceleration == bootAcc * EXPERIMENTAL_ACCELERATION_FACTOR);
        tfhReset(m);
    }

    // =====================================================================
    // 7. THE DEFAULT DEVIATION WINDOW IS TWO ROTATIONS. There is no getter for
    //    max_allowable_position_deviation, so it is probed behaviourally, the
    //    way tm_deviation_tracking does: with the MOSFETs off the shaft never
    //    follows, the hall reading stays put, and the deviation
    //    (motor_control.c:3218) is therefore exactly the commanded
    //    displacement. That turns error 45 into a ruler.
    //
    //    The bracket is deliberately tight -- 1.85 rotations must complete and
    //    2.15 must trip -- which excludes one, two-and-a-half and four
    //    rotations, not merely "somewhere around two". The 0.15-rotation margin
    //    on each side is roughly 490,000 counts, far larger than any resting
    //    hall offset, and that premise is measured rather than assumed.
    // =====================================================================
    printf("\n--- 7. the default deviation window is two rotations ---\n");
    {
        tfhReset(m);
        int64_t restingHall = m->getHallSensorPositionRaw();
        bool hallOk = (m->getError() == 0);
        printf("   resting hall position = %lld counts (%.4f rotations)\n",
               (long long)restingHall, (double)restingHall / (double)TFH_CPR);
        // The deviation is commanded MINUS hall, so a large resting hall offset
        // would shift the trip point and blunt the bracket below.
        TEST_RESULT("the resting hall position is near zero, which is what makes "
                    "the bracket below sharp",
                    hallOk && llabs((long long)restingHall) < TFH_CPR / 20);

        // Inside the window: 1.85 rotations, at the FACTORY limits (no
        // tfhFreshOpen here -- opening the ceilings would also open the very
        // window under test).
        int32_t inside = (int32_t)(TFH_CPR * 185 / 100);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(inside, (uint32_t)TFH_TPS);
        bool insideAccepted = (m->getError() == 0);
        bool insideDone = insideAccepted && tfhDrain(m, 12000) && (tfhFatal(m) == 0);
        int64_t insideMoved = insideDone ? (m->getPositionRaw() - before) : 0;

        // Outside the window: 2.15 rotations must trip.
        tfhReset(m);
        uint8_t code = 0;
        int32_t outside = (int32_t)(TFH_CPR * 215 / 100);
        int64_t reached = runAndWatch(m, outside, (uint32_t)TFH_TPS, &code);

        printf("   1.85 rot: completed=%d moved=%lld of %ld\n",
               (int)insideDone, (long long)insideMoved, (long)inside);
        printf("   2.15 rot: reached about %lld counts, fatal=%u\n",
               (long long)reached, (unsigned)code);

        TEST_RESULT("1.85 rotations is inside the default window and completes cleanly",
                    insideDone);
        TEST_RESULT("and it delivers its full displacement",
                    insideDone && llabs((long long)(insideMoved - inside)) <= inside / 100 + 4);
        TEST_RESULT("2.15 rotations is outside the default window and trips "
                    "ERROR_POSITION_DEVIATION_TOO_LARGE (45)",
                    code == ERR_POSITION_DEVIATION_TOO_LARGE);
        tfhReset(m);
    }

    // =====================================================================
    // 8. THE DEFAULT STATE IS REPRODUCIBLE. Everything above was measured
    //    once. A default that came up right only sometimes -- because it was
    //    seeded from uninitialised memory, or from a sensor read that had not
    //    settled -- would pass every assertion so far and still be a serious
    //    defect. Three fresh boots must produce the SAME snapshot, compared
    //    against each other rather than against a hard-coded expectation.
    // =====================================================================
    printf("\n--- 8. three fresh boots give an identical snapshot ---\n");
    {
        uint16_t flags[3]; uint8_t fatal[3]; int64_t pos[3];
        uint8_t depth[3]; int64_t vel[3]; int64_t acc[3];
        bool allRead = true;

        for (int i = 0; i < 3; i++) {
            tfhReset(m);
            getStatusResponse st = m->getStatus();
            if (m->getError() != 0) { allRead = false; break; }
            flags[i] = st.statusFlags;
            fatal[i] = st.fatalErrorCode;
            pos[i]   = m->getPositionRaw();
            if (m->getError() != 0) { allRead = false; break; }
            depth[i] = m->getNQueuedItems();
            if (m->getError() != 0) { allRead = false; break; }
            getDebugValuesResponse d = m->getDebugValues();
            if (m->getError() != 0) { allRead = false; break; }
            vel[i] = d.maxVelocity;
            acc[i] = d.maxAcceleration;
            printf("   boot %d: flags=0x%04X fatal=%u pos=%lld queue=%u vel=%lld acc=%lld\n",
                   i + 1, (unsigned)flags[i], (unsigned)fatal[i], (long long)pos[i],
                   (unsigned)depth[i], (long long)vel[i], (long long)acc[i]);
        }

        bool sameState = allRead &&
                         flags[0] == flags[1] && flags[1] == flags[2] &&
                         fatal[0] == fatal[1] && fatal[1] == fatal[2] &&
                         pos[0]   == pos[1]   && pos[1]   == pos[2]   &&
                         depth[0] == depth[1] && depth[1] == depth[2];
        bool sameLimits = allRead &&
                          vel[0] == vel[1] && vel[1] == vel[2] && vel[0] == bootVel &&
                          acc[0] == acc[1] && acc[1] == acc[2] && acc[0] == bootAcc;

        TEST_RESULT("three consecutive resets give an identical status, position "
                    "and queue snapshot", sameState);
        TEST_RESULT("and identical speed and acceleration ceilings, matching the "
                    "ones measured earlier", sameLimits);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
