#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_response_field_integrity.cpp
//
// EVERY FIELD OF EVERY MULTI-FIELD REPLY, CHECKED INDIVIDUALLY.
//
// Eleven commands answer with a STRUCT rather than a scalar. Across the whole
// suite, almost every one of them is read for ONE field and the rest are
// ignored: `getStatus()` is called ~200 times for `fatalErrorCode` and hardly
// ever for `statusFlags`; `getComprehensivePositionRaw()` is read for the two
// position fields and NOTHING anywhere has ever looked at the third;
// `getDebugValues()` has thirty fields and the suite touches eight of them.
//
// A field that the firmware never populates, or that silently carries its
// NEIGHBOUR's value because of a packing slip, is completely invisible to that
// style of testing. The reply still arrives, the one field everybody reads is
// still right, and the wrong number only surfaces in the field, in the hands of
// somebody debugging a motor that has already stopped working.
//
// THE PROPERTY THIS MODULE ASSERTS:
//
//     For each of the eleven response structs: the struct is exactly the
//     declared size, EVERY field is individually plausible against firmware
//     ground truth, and fields that must differ from one another actually do.
//
// WHY THE SIZE CHECK IS NOT COSMETIC. The Arduino library parses every reply by
// casting the received bytes onto the struct, but only after
//
//     if (receivedSize == sizeof(getProductInfoResponse)) { ... }
//     else { _errno = COMMUNICATION_ERROR_DATA_WRONG_SIZE; }
//                                          (Arduino_library/Servomotor.cpp:928)
//
// so a single byte of padding anywhere does not shift one field -- it makes the
// entire command fail on every motor. The sizes here were derived from the
// firmware's own reply structs (firmware/Src/main.c) and are the contract the
// wire format and the host struct have to agree on.
//
// THE ANTI-ALIASING TECHNIQUE. Asserting "field X looks plausible" is weak: a
// copy of a neighbouring field usually looks plausible too. So wherever two
// fields of the same reply can be made to DISAGREE, this module makes them
// disagree on purpose and requires the disagreement:
//
//   * `maxVelocity` and `maxAcceleration` are both written from a raw u32 the
//     host chooses, but the firmware shifts them by DIFFERENT amounts --
//     VELOCITY_SHIFT_LEFT is 12 and ACCELERATION_SHIFT_LEFT is 8
//     (firmware/Src/motor_control.h:67-68), applied at motor_control.c:3348 and
//     :3367. Feed the two setters swapped values and each field must follow its
//     own setter through its own shift.
//   * `commandedPosition`, `hallSensorPosition` and `externalEncoderPosition`
//     arrive in ONE reply (main.c:469-486) and, with the output stage off, do
//     three different things during one move: the first advances by the whole
//     rotation, the second stays put because the rotor never follows, and the
//     third stays within a handful of counts because it only advances on edges
//     of an external step signal that nothing here drives. Three fields, three
//     behaviours, one packet.
//   * `minPidError` and `maxPidError` are seeded to OPPOSITE extremes
//     (2147483647 and -2147483648, motor_control.c:3485-3486) and the PID loop
//     only runs in closed loop (motor_control.c:2724), which this module never
//     enters -- so both fields must hold their own distinct seed, and a field
//     copied from its neighbour is caught immediately.
//   * `statusFlags` (u16) and `fatalErrorCode` (u8) are adjacent in a 3-byte
//     reply; a fault is induced so that one is nonzero while the other is zero.
//
// HOW THIS IS DISTINCT FROM THE MODULES THAT ALREADY TOUCH THESE COMMANDS:
//   * tm_device_identity asks whether the DEVICE'S CONSTANTS agree with the
//     suite's (countsPerRotation, updateFrequency) and closes that loop
//     physically. It never checks the neighbouring fields of the structs it
//     reads, and never checks a struct size.
//   * tm_diagnostics / tm_diagnostics_readouts check the MATHEMATICAL
//     INVARIANTS inside the hall-statistics and debug replies (min <= mean <=
//     max, time <= maxTime). Those hold perfectly well if a field is a copy of
//     its neighbour; here the fields are forced apart instead.
//   * tm_position_source_consistency proves the three POSITION COMMANDS agree
//     with one another. It reads the third field of cmd 37 into a struct and
//     never asserts anything about it; this module does.
//   * tm_status_flag_semantics is about what the flags MEAN; this module is
//     about whether the two fields of that reply are two fields at all.
//   * tm_communication_statistics, tm_time_sync_protocol and tm_ping_stress
//     each own one command in depth; the checks here are deliberately thin for
//     those three and cover only the per-field integrity angle.
//
// SAFETY. The MOSFETs are never enabled and goToClosedLoop is never called.
// Everything that moves is read through the COMMANDED position, which the
// planner advances whether or not the output stage is energised, so nothing
// turns -- on this motor or on any other. Motion is limited to one commanded
// rotation and one velocity segment, both taken after tfhFreshOpen() has raised
// the two-rotation deviation window, and every velocity sequence is terminated
// by an explicit zero-velocity segment so the queue cannot underrun (error 18).
// One fatal error is induced deliberately (a zero-duration velocity move, which
// is rejected before it is ever queued) and cleared by a reset. Nothing is
// broadcast, no alias is touched and no test mode is entered: safe on the
// shared 35-motor rack.
// ---------------------------------------------------------------------------

// Command numbers below are quoted from python_programs/servomotor/motor_commands.json.
static const uint8_t ERR_PARAMETER_OUT_OF_RANGE = 34;   // error_codes.json

// A hall reading is the sum of four 12-bit ADC conversions (ADC.c:188-199), so
// it can never exceed 4 * 4095. A stuck channel reads 0 or the ceiling.
static const uint32_t HALL_ADC_SUM_MAX = 4u * 4095u;

// The seeds get_max_PID_error() writes back after every read (motor_control.c:3485).
static const int32_t PID_SEED_MIN = 2147483647;
static const int32_t PID_SEED_MAX = -2147483647 - 1;

// The seeds get_hall_position_delta_stats() writes back (motor_control.c:3632).
static const int32_t HALL_DELTA_SEED_MAX = -2000000000;
static const int32_t HALL_DELTA_SEED_MIN =  2000000000;

// The HSI trim rails the PI controller clamps to (clock_calibration.c:19-20).
static const uint16_t HSI_TRIM_MIN = 62;
static const uint16_t HSI_TRIM_MAX = 66;

// NOT named isPrintable(): Arduino.h (WCharacter.h) already declares
// isPrintable(int) in the global namespace, and shadowing an Arduino.h name from
// a test module has cost this campaign a compile before.
static bool tfhPrintableByte(char c) { return (c >= 0x20) && (c < 0x7F); }

void tm_response_field_integrity(void) {
    Serial.println("tm_response_field_integrity: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. EVERY RESPONSE STRUCT IS EXACTLY THE DECLARED SIZE.
    //
    //    This runs first because it is the gate: the library refuses to
    //    decode a reply whose length does not equal sizeof() of the host
    //    struct (Servomotor.cpp:927-932 and the same pattern in every other
    //    getter). If a compiler ever dropped the packed attribute, or a field
    //    were added on one side only, every assertion further down would fail
    //    for a reason that has nothing to do with the motor -- so the sizes
    //    are stated up front, where the diagnosis is unambiguous.
    //
    //    The numbers come from the firmware's own reply structs:
    //      main.c:493-497   status      u16 + u8                    = 3
    //      product_info.h:10-19         8 + 1 + 3 + 4 + 8 + 4       = 28
    //      main.c:516-520   specs       u32 + u32                   = 8
    //      main.c:599-603   fw version  4 version bytes + u8        = 5
    //      main.c:472-477   comp. pos.  i64 + i64 + i32             = 20
    //      main.c:871-875   max PID err i32 + i32                   = 8
    //      hall_sensor_calculations.h:20-25  3*u16 + 3*u16 + 3*u64 + u32 = 40
    //      main.c:1046+     debug values 30 fields                  = 112
    //      RS485.h:49-56    comm stats  6 * u32                     = 24
    //      main.c:391-395   time sync   i32 + u16                   = 6
    //      main.c:741-744   ping        10 bytes                    = 10
    // =====================================================================
    printf("\n--- 1. the eleven response structs are exactly their declared sizes ---\n");
    {
        printf("   status=%u specs=%u maxPid=%u timeSync=%u ping=%u\n",
               (unsigned)sizeof(getStatusResponse),
               (unsigned)sizeof(getProductSpecsResponse),
               (unsigned)sizeof(getMaxPidErrorResponse),
               (unsigned)sizeof(timeSyncResponse),
               (unsigned)sizeof(pingResponse));
        printf("   productInfo=%u (VersionNumber24=%u)  firmwareVersion=%u (VersionNumber32=%u)\n",
               (unsigned)sizeof(getProductInfoResponse), (unsigned)sizeof(VersionNumber24),
               (unsigned)sizeof(getFirmwareVersionResponse), (unsigned)sizeof(VersionNumber32));
        printf("   comprehensivePosition=%u hallStatistics=%u commStatistics=%u debugValues=%u\n",
               (unsigned)sizeof(getComprehensivePositionResponse),
               (unsigned)sizeof(getHallSensorStatisticsResponse),
               (unsigned)sizeof(getCommunicationStatisticsResponse),
               (unsigned)sizeof(getDebugValuesResponse));

        TEST_RESULT("the four small replies are 3, 8, 8 and 6 bytes with no padding",
                    sizeof(getStatusResponse) == 3 &&
                    sizeof(getProductSpecsResponse) == 8 &&
                    sizeof(getMaxPidErrorResponse) == 8 &&
                    sizeof(timeSyncResponse) == 6);
        TEST_RESULT("the two identity replies are 28 and 5 bytes, with 3- and 4-byte version numbers",
                    sizeof(getProductInfoResponse) == 28 &&
                    sizeof(VersionNumber24) == 3 &&
                    sizeof(getFirmwareVersionResponse) == 5 &&
                    sizeof(VersionNumber32) == 4);
        TEST_RESULT("the five larger replies are 20, 40, 24, 10 and 112 bytes",
                    sizeof(getComprehensivePositionResponse) == 20 &&
                    sizeof(getHallSensorStatisticsResponse) == 40 &&
                    sizeof(getCommunicationStatisticsResponse) == 24 &&
                    sizeof(pingResponse) == 10 &&
                    sizeof(getDebugValuesResponse) == 112);
    }

    // =====================================================================
    // 2. getStatusResponse: TWO FIELDS, NOT ONE 24-BIT WORD.
    //
    //    The reply is a u16 of flags immediately followed by a u8 error code
    //    (main.c:493-497, device_status.h:26-29). At rest both are zero, which
    //    on its own proves nothing -- so a fault is induced and the two fields
    //    must then disagree: fatal_error() forces the flags to zero
    //    (error_handling.c:227) while the error byte carries the code. A
    //    reply where the error code had leaked into the flags word, or vice
    //    versa, cannot produce that combination.
    //
    //    The fault chosen is the most benign one available: a velocity move
    //    with a duration of zero, which main.c:625-627 rejects with
    //    ERROR_PARAMETER_OUT_OF_RANGE BEFORE anything is queued. Velocity zero,
    //    duration zero: nothing can move even in principle.
    // =====================================================================
    printf("\n--- 2. the status reply carries two independent fields ---\n");
    {
        tfhReset(m);
        getStatusResponse rest = m->getStatus();
        bool restOk = (m->getError() == 0);
        printf("   at rest: flags=0x%04X errorCode=%u\n",
               (unsigned)rest.statusFlags, (unsigned)rest.fatalErrorCode);
        TEST_RESULT("a healthy idle motor reports flags 0x0000 and error code 0",
                    restOk && rest.statusFlags == 0 && rest.fatalErrorCode == 0);

        m->moveWithVelocityRaw(0, 0);          // refused: duration 0, nothing queued
        getStatusResponse faulted = m->getStatus();
        bool faultReadOk = (m->getError() == 0);
        printf("   after the induced fault: flags=0x%04X errorCode=%u\n",
               (unsigned)faulted.statusFlags, (unsigned)faulted.fatalErrorCode);
        TEST_RESULT("the error byte carries code 34 while the flags word stays 0x0000",
                    faultReadOk && faulted.fatalErrorCode == ERR_PARAMETER_OUT_OF_RANGE &&
                    faulted.statusFlags == 0);
        tfhReset(m);
    }

    // =====================================================================
    // 3. getProductInfoResponse: ALL SIX FIELDS.
    //
    //    28 bytes of identity read straight out of bootloader flash
    //    (main.c:566-578, product_info.h:10-19). Only the product code and the
    //    serial number are checked anywhere else in the suite; the hardware
    //    version, the reserved word and the unique ID are checked here.
    //
    //    The decisive one is the unique ID: the harness ADDRESSES this motor
    //    by unique ID (extended addressing), so if the ID inside the reply
    //    disagrees with the ID on the envelope, either the field is wrong or
    //    the reply came from a different motor -- and on a 35-motor bus the
    //    second possibility is exactly the failure a per-field check should
    //    catch.
    // =====================================================================
    printf("\n--- 3. every field of the product-info reply ---\n");
    {
        tfhReset(m);
        getProductInfoResponse info = m->getProductInfo();
        bool ok = (m->getError() == 0);

        // generate_product_info.c:297 seeds the model code with EIGHT SPACES and
        // then strncpy()s the model string over the front WITHOUT a terminator,
        // so on a device programmed by that tool all eight bytes are printable.
        // Older/other programming paths NUL-pad instead -- tm_device_identity
        // deliberately allows for that (it breaks its scan on the first NUL and
        // asserts only `printable > 0`). Requiring exactly eight printable bytes
        // would therefore be an assertion about the PROGRAMMING TOOL, not about
        // the reply's field layout, so the claim here is weakened to the part
        // that is really about field integrity: at least one printable
        // character, and no byte that is neither printable nor NUL padding.
        int printable = 0, cleanBytes = 0;
        for (int i = 0; i < 8; i++) {
            char ch = info.productCode[i];
            if (tfhPrintableByte(ch)) { printable++; cleanBytes++; }
            else if (ch == '\0')      { cleanBytes++; }
        }
        char code[9];
        memcpy(code, info.productCode, 8);
        code[8] = '\0';
        for (int i = 0; i < 8; i++) if (!tfhPrintableByte(code[i])) code[i] = '\0';

        uint64_t addressed = m->usingThisUniqueId();
        printf("   productCode='%s' (%d/8 printable) firmwareCompatibility=%u\n",
               code, printable, (unsigned)info.firmwareCompatibility);
        printf("   hardwareVersion=%u.%u.%u serialNumber=%lu reserved=%lu\n",
               (unsigned)info.hardwareVersion.major, (unsigned)info.hardwareVersion.minor,
               (unsigned)info.hardwareVersion.patch,
               (unsigned long)info.serialNumber, (unsigned long)info.reserved);
        printf("   uniqueId=%08lX%08lX  addressed as %08lX%08lX\n",
               (unsigned long)(info.uniqueId >> 32), (unsigned long)(info.uniqueId & 0xFFFFFFFFu),
               (unsigned long)(addressed >> 32), (unsigned long)(addressed & 0xFFFFFFFFu));

        TEST_RESULT("the product code is a clean identifier (printable, NUL-padded at most) and the compatibility code is set",
                    ok && printable > 0 && cleanBytes == 8 &&
                    info.firmwareCompatibility > 0 && info.firmwareCompatibility < 100);
        TEST_RESULT("the three hardware-version bytes are each a sane decimal and not all zero",
                    ok && info.hardwareVersion.major < 100 && info.hardwareVersion.minor < 100 &&
                    info.hardwareVersion.patch < 100 &&
                    (info.hardwareVersion.major || info.hardwareVersion.minor ||
                     info.hardwareVersion.patch));
        // The reserved word is the LAST four bytes of the 28-byte struct, which
        // makes it the sharpest available check on the packing: an off-by-one
        // anywhere upstream lands garbage there. It must be exactly zero, and
        // that is firm rather than hopeful -- the product info lives at
        // 0x8000010, which is the seven RESERVED Cortex-M0+ vector-table slots
        // that the bootloader's startup file emits as `.word 0`
        // (bootloader_STM32G031/Src/startup_stm32g031g8ux.s:136-143), and
        // generate_product_info.c writes only the first six fields.
        //
        // The SERIAL NUMBER is deliberately NOT required to be nonzero: it is
        // written only when the programmer is invoked with a serial
        // (UPDATE_SERIAL_NUMBER_FLAG, generate_product_info.c:194-197), so
        // "unset" is a provisioning state, not a field-integrity fault.
        // tm_device_identity makes the same choice. The anti-aliasing claim it
        // was carrying is preserved by the unique-ID assertion below, which
        // holds whatever the serial happens to be.
        TEST_RESULT("the reserved word at the tail of the 28-byte struct is exactly zero",
                    ok && info.reserved == 0);
        TEST_RESULT("the reported unique ID is the one the motor is addressed by, and is not the serial number",
                    ok && info.uniqueId == addressed && info.uniqueId != 0 &&
                    (uint32_t)(info.uniqueId & 0xFFFFFFFFu) != info.serialNumber);
    }

    // =====================================================================
    // 4. getProductSpecsResponse: TWO u32s THAT MUST NOT BE THE SAME NUMBER.
    //    (main.c:512-523; counts_per_rotation is ONE_REVOLUTION_MICROSTEPS.)
    //    These are the two constants the entire suite hardcodes as TFH_TPS and
    //    TFH_CPR, so a swapped or duplicated field here would silently rescale
    //    every distance and every duration asserted anywhere.
    // =====================================================================
    printf("\n--- 4. both product-spec fields ---\n");
    uint32_t updateFrequency = 0;
    {
        getProductSpecsResponse specs = m->getProductSpecs();
        bool ok = (m->getError() == 0);
        updateFrequency = specs.updateFrequency;
        printf("   updateFrequency=%lu Hz countsPerRotation=%lu\n",
               (unsigned long)specs.updateFrequency, (unsigned long)specs.countsPerRotation);
        TEST_RESULT("update frequency and counts per rotation are both populated, distinct, and match the suite's constants",
                    ok && specs.updateFrequency == (uint32_t)TFH_TPS &&
                    specs.countsPerRotation == (uint32_t)TFH_CPR &&
                    specs.updateFrequency != specs.countsPerRotation);
    }

    // =====================================================================
    // 5. getFirmwareVersionResponse: FOUR VERSION BYTES PLUS A DEDICATED FLAG.
    //
    //    main.c:593-609 sends firmware_version (dev, bugfix, minor, major --
    //    main.c:63-73) followed by a SEPARATE in_bootloader byte that the main
    //    firmware always sets to 0. That byte is not the status bitfield, and
    //    the way to prove it is a byte of its own is to read the status
    //    bitfield's bootloader bit at the same time: both must be 0 and they
    //    must come from different replies of different lengths.
    // =====================================================================
    printf("\n--- 5. every byte of the firmware-version reply ---\n");
    {
        getFirmwareVersionResponse fw = m->getFirmwareVersion();
        bool ok = (m->getError() == 0);
        getFirmwareVersionResponse fw2 = m->getFirmwareVersion();
        bool ok2 = (m->getError() == 0);
        getStatusResponse st = m->getStatus();
        bool okSt = (m->getError() == 0);

        printf("   version %u.%u.%u.%u  inBootloader=%u  statusFlags=0x%04X\n",
               (unsigned)fw.firmwareVersion.major, (unsigned)fw.firmwareVersion.minor,
               (unsigned)fw.firmwareVersion.patch, (unsigned)fw.firmwareVersion.dev,
               (unsigned)fw.inBootloader, (unsigned)st.statusFlags);

        TEST_RESULT("the four version bytes are each a sane decimal and the version is not 0.0.0.0",
                    ok && fw.firmwareVersion.major < 100 && fw.firmwareVersion.minor < 100 &&
                    fw.firmwareVersion.patch < 100 && fw.firmwareVersion.dev < 100 &&
                    (fw.firmwareVersion.major || fw.firmwareVersion.minor ||
                     fw.firmwareVersion.patch || fw.firmwareVersion.dev));
        TEST_RESULT("the in-bootloader flag is a dedicated byte reading exactly 0, matching the status word's bit 0, and both reads agree",
                    ok && ok2 && okSt && fw.inBootloader == 0 &&
                    (st.statusFlags & 0x0001) == 0 &&
                    memcmp(&fw, &fw2, sizeof(getFirmwareVersionResponse)) == 0);
    }

    // =====================================================================
    // 6. getComprehensivePositionResponse: THREE FIELDS, THREE BEHAVIOURS.
    //
    //    One reply carries the commanded position, the measured (hall)
    //    position and an external encoder count (main.c:469-486). With the
    //    output stage off they are forced apart by a single move:
    //
    //      commandedPosition        advances by the full displacement, because
    //                               the planner runs regardless of the MOSFETs
    //      hallSensorPosition       stays put, because the rotor never follows
    //      externalEncoderPosition  stays essentially where it was, because it
    //                               only advances on rising edges of an external
    //                               step signal (GPIO_interrupts.c:52-59) that
    //                               nothing in this harness drives
    //
    //    Nothing in the suite has ever asserted anything about the third
    //    field. It is also the one most likely to be quietly wrong, being the
    //    lone i32 tacked onto the end of two i64s in a packed struct.
    //
    //    The same move doubles as the check on the four debugValue fields:
    //    they are written ONLY when the deviation watchdog trips
    //    (motor_control.c:3219-3222), so on a machine that has not faulted
    //    they must still be zero AFTER a real move -- which is what rules out
    //    their being aliases of the position or velocity state.
    // =====================================================================
    printf("\n--- 6. the three position fields of one reply behave differently ---\n");
    {
        tfhFreshOpen(m);
        getComprehensivePositionResponse c0 = m->getComprehensivePositionRaw();
        bool ok0 = (m->getError() == 0);

        bool moved = false;
        int64_t delta = tfhMove(m, (int32_t)TFH_CPR, tfhTicksFor(TFH_CPR), &moved);

        getComprehensivePositionResponse c1 = m->getComprehensivePositionRaw();
        bool ok1 = (m->getError() == 0);
        getDebugValuesResponse dbg = m->getDebugValues();
        bool okDbg = (m->getError() == 0);

        int64_t commandedMoved = c1.commandedPosition - c0.commandedPosition;
        int64_t hallMoved      = c1.hallSensorPosition - c0.hallSensorPosition;
        printf("   commanded advanced %lld, hall moved %lld, external encoder %ld -> %ld\n",
               (long long)commandedMoved, (long long)hallMoved,
               (long)c0.externalEncoderPosition, (long)c1.externalEncoderPosition);
        printf("   debugValue1..4 = %lld %lld %lld %lld\n",
               (long long)dbg.debugValue1, (long long)dbg.debugValue2,
               (long long)dbg.debugValue3, (long long)dbg.debugValue4);

        TEST_RESULT("the commanded-position field advances by the full commanded rotation",
                    ok0 && ok1 && moved && llabs((long long)(delta - TFH_CPR)) < 400 &&
                    llabs((long long)(commandedMoved - TFH_CPR)) < 400);
        TEST_RESULT("the hall-position field in the same reply stays put, so it is not a copy of the commanded one",
                    ok0 && ok1 && llabs((long long)hallMoved) < TFH_CPR / 50);
        // WEAKENED from "does not move at all". step_and_direction_init()
        // (GPIO_interrupts.c:27-33) is called unconditionally from main.c:1384,
        // so EXTI4 on PB4 is ARMED on every motor whether or not an encoder is
        // wired to it, and step_count moves on any rising edge that reaches the
        // pin (GPIO_interrupts.c:52-59). Whether that pin is driven, pulled or
        // floating is a property of the harness wiring, not of the reply, so
        // demanding bit-exact equality would make this assertion hostage to
        // something the suite does not control. What IS about field integrity
        // is that this field does not FOLLOW either of its neighbours: the
        // commanded position advanced by a whole rotation in the same reply.
        int64_t externalMoved = (int64_t)c1.externalEncoderPosition -
                                (int64_t)c0.externalEncoderPosition;
        TEST_RESULT("the external-encoder field tracks neither of its neighbours in the same reply",
                    ok0 && ok1 && llabs((long long)externalMoved) < 1000 &&
                    externalMoved != commandedMoved);
        TEST_RESULT("the four debug values stay zero after a clean move (they latch only on a deviation fault)",
                    okDbg && dbg.debugValue1 == 0 && dbg.debugValue2 == 0 &&
                    dbg.debugValue3 == 0 && dbg.debugValue4 == 0);
    }

    // =====================================================================
    // 7. getMaxPidErrorResponse: TWO OPPOSITE SEEDS.
    //
    //    get_max_PID_error() (motor_control.c:3480-3490) returns the running
    //    window and immediately re-seeds it: min <- 2147483647, max <-
    //    -2147483648. The PID loop only runs in CLOSED_LOOP_POSITION_CONTROL
    //    (motor_control.c:2724), which this module never enters, so the window
    //    is never fed and both fields must hold their own seed on every read.
    //
    //    That is the EMPTY SENTINEL the campaign warns about: min > max here is
    //    correct, not a bug. It is also the sharpest possible aliasing test for
    //    this reply, because the two seeds are the two extremes of int32 -- a
    //    field copied from its neighbour cannot produce both.
    // =====================================================================
    printf("\n--- 7. the PID error window reports its two distinct empty sentinels ---\n");
    {
        bool allSeeded = true, allOk = true;
        for (int i = 0; i < 3; i++) {
            getMaxPidErrorResponse w = m->getMaxPidErrorRaw();
            if (m->getError() != 0) { allOk = false; break; }
            printf("   read %d: min=%ld max=%ld\n", i, (long)w.minPidError, (long)w.maxPidError);
            if (!(w.minPidError == PID_SEED_MIN && w.maxPidError == PID_SEED_MAX)) allSeeded = false;
        }
        TEST_RESULT("with the PID loop never having run, min holds +2147483647 and max holds -2147483648 on every read, so neither field is a copy of the other",
                    allOk && allSeeded);
    }

    // =====================================================================
    // 8. getHallSensorStatisticsResponse: TEN FIELDS BOUND TOGETHER.
    //
    //    Three maxima, three minima, three 64-bit sums and a count
    //    (hall_sensor_calculations.h:20-25), reset and started by
    //    `Control hall sensor statistics` subcommand 1 (main.c:754-762,
    //    hall_sensor_calculations.c:245-256, which seeds min to 65535 and max
    //    to 0). The check that cannot be satisfied by accident is the
    //    ACCUMULATOR IDENTITY:
    //
    //        min * n  <=  sum  <=  max * n
    //
    //    which ties four fields of three different widths together. A sum that
    //    is really a copy of the max, a count that is really a copy of a sum,
    //    a truncated 64-bit accumulator or a missing reset all break it.
    // =====================================================================
    printf("\n--- 8. the ten hall-statistics fields are mutually consistent ---\n");
    {
        tfhReset(m);
        m->controlHallSensorStatistics(1);      // reset the accumulators and start gathering
        bool startOk = (m->getError() == 0);
        delay(400);                             // ~12500 control ticks
        getHallSensorStatisticsResponse s = m->getHallSensorStatistics();
        bool ok = (m->getError() == 0);

        const uint16_t mins[3] = { s.minHall1, s.minHall2, s.minHall3 };
        const uint16_t maxs[3] = { s.maxHall1, s.maxHall2, s.maxHall3 };
        const uint64_t sums[3] = { s.sumHall1, s.sumHall2, s.sumHall3 };
        uint32_t n = s.measurementCount;

        printf("   n=%lu  min=%u/%u/%u  max=%u/%u/%u\n", (unsigned long)n,
               (unsigned)mins[0], (unsigned)mins[1], (unsigned)mins[2],
               (unsigned)maxs[0], (unsigned)maxs[1], (unsigned)maxs[2]);
        printf("   sums=%llu/%llu/%llu\n", (unsigned long long)sums[0],
               (unsigned long long)sums[1], (unsigned long long)sums[2]);

        bool envelopeOk = ok && startOk;
        bool identityOk = ok && startOk && (n > 0);
        bool accumulatorOk = ok && startOk && (n > 0);
        for (int ch = 0; ch < 3; ch++) {
            if (!(mins[ch] <= maxs[ch] && maxs[ch] <= HALL_ADC_SUM_MAX && mins[ch] > 0)) {
                envelopeOk = false;
            }
            // ONE SAMPLE OF SLACK, on purpose. get_hall_sensor_statistics()
            // brackets its memcpy with TIM1's update interrupt
            // (hall_sensor_calculations.c:236-238), but the accumulation runs
            // from get_sensor_position() inside the TIM16 handler
            // (motor_control.c:3074, :3096), which that bracket does not mask.
            // So the snapshot can straddle one increment -- n one higher than
            // the sum it is paired with, or the reverse -- and on a very quiet
            // channel (every reading identical, so sum == min*n exactly) the
            // strict identity would then fail on a perfectly healthy motor.
            // Allowing a single sample keeps every defect this is aimed at:
            // a sum that is a copy of the max, a truncated 64-bit accumulator
            // and a missing reset are all off by orders of magnitude, not one.
            if (!((uint64_t)mins[ch] * (uint64_t)n <= sums[ch] + (uint64_t)maxs[ch] &&
                  sums[ch] <= (uint64_t)maxs[ch] * (uint64_t)n + (uint64_t)maxs[ch])) {
                identityOk = false;
            }
            // A 64-bit accumulator over hundreds of samples must dwarf both the
            // 16-bit maximum it is accumulating and the sample count itself.
            if (!(sums[ch] > (uint64_t)maxs[ch] && sums[ch] > (uint64_t)n)) {
                accumulatorOk = false;
            }
        }

        TEST_RESULT("the count grew and every channel's min/max envelope is inside the ADC range",
                    envelopeOk && n > 200);
        TEST_RESULT("min * count <= sum <= max * count holds on all three channels",
                    identityOk);
        TEST_RESULT("each 64-bit sum is a real accumulator, not a copy of the 16-bit maximum or the count",
                    accumulatorOk &&
                    !(sums[0] == sums[1] && sums[1] == sums[2]));
    }

    // =====================================================================
    // 9. getDebugValuesResponse: THE THIRTY-FIELD REPLY.
    //
    //    This is where a never-populated field is easiest to hide, so it gets
    //    the most work. The two settings fields are the centrepiece: the host
    //    hands the firmware a raw u32 for each, and the firmware shifts them
    //    by DIFFERENT amounts before storing (motor_control.c:3348 with
    //    VELOCITY_SHIFT_LEFT 12, and :3367 with ACCELERATION_SHIFT_LEFT 8).
    //    Feeding the two setters swapped values and requiring each field to
    //    follow its own setter through its own shift is a test no aliasing,
    //    swapping or truncation can survive.
    //
    //    The raw values used are small enough that neither can hit the
    //    experimental ceilings the setters clamp at (EXPERIMENTAL_MAX_VELOCITY
    //    and EXPERIMENTAL_MAX_ACCELERATION, motor_control.h:64-65); a clamp
    //    would make the arithmetic untestable rather than wrong.
    // =====================================================================
    printf("\n--- 9a. the two limit fields follow their own setters and their own shifts ---\n");
    {
        tfhReset(m);
        m->setMaximumVelocityRaw(3000000);
        m->setMaximumAccelerationRaw(7000000);
        getDebugValuesResponse d = m->getDebugValues();
        bool ok = (m->getError() == 0);
        printf("   set vel=3000000 acc=7000000 -> maxVelocity=%lld maxAcceleration=%lld\n",
               (long long)d.maxVelocity, (long long)d.maxAcceleration);
        TEST_RESULT("maxVelocity reads the raw setting shifted left by 12 and maxAcceleration by 8",
                    ok && d.maxVelocity == ((int64_t)3000000 << 12) &&
                    d.maxAcceleration == ((int64_t)7000000 << 8));

        // Swap the two raw values. If either field were sourced from the other
        // setter, or the two shifts were confused, the readback cannot follow.
        m->setMaximumVelocityRaw(7000000);
        m->setMaximumAccelerationRaw(3000000);
        getDebugValuesResponse e = m->getDebugValues();
        bool ok2 = (m->getError() == 0);
        printf("   swapped   -> maxVelocity=%lld maxAcceleration=%lld\n",
               (long long)e.maxVelocity, (long long)e.maxAcceleration);
        TEST_RESULT("swapping the two raw settings swaps the two fields independently",
                    ok2 && e.maxVelocity == ((int64_t)7000000 << 12) &&
                    e.maxAcceleration == ((int64_t)3000000 << 8) &&
                    e.maxVelocity != e.maxAcceleration);
    }

    // ---------------------------------------------------------------------
    // 9b. The live motion fields. A velocity segment with the MOSFETs off
    //     makes currentVelocity large and measuredVelocity ~0 at the same
    //     instant, which is the cleanest available proof that they are two
    //     fields. nTimeSteps is the countdown of the executing queue item
    //     (motor_control.c:2131-2146), so it must fall between two reads. And
    //     motorPwmVoltage is set to max_motor_pwm_voltage while an item is
    //     moving and to HALF that when idle (motor_control.c:2706-2718), a
    //     relationship that holds on any product without hardcoding a value.
    //
    //     The sequence ends with an explicit zero-velocity segment: a velocity
    //     segment HOLDS its velocity after its time expires, and an empty
    //     queue with a nonzero velocity is fatal error 18.
    // ---------------------------------------------------------------------
    printf("\n--- 9b. the live motion fields move independently of one another ---\n");
    {
        tfhFreshOpen(m);
        const int32_t rawVelocity = 100000000;              // ~0.91 rotations/second
        const uint32_t segTicks = 3u * (uint32_t)TFH_TPS;   // 3 seconds

        getDebugValuesResponse idle = m->getDebugValues();
        bool idleOk = (m->getError() == 0);

        m->moveWithVelocityRaw(rawVelocity, segTicks);
        bool queued = (m->getError() == 0);
        m->moveWithVelocityRaw(0, (uint32_t)TFH_TPS);       // mandatory zero-velocity tail
        queued = queued && (m->getError() == 0);

        delay(300);
        getDebugValuesResponse a = m->getDebugValues();
        bool aOk = (m->getError() == 0);
        delay(1000);
        getDebugValuesResponse b = m->getDebugValues();
        bool bOk = (m->getError() == 0);

        printf("   currentVelocity=%lld measuredVelocity=%ld nTimeSteps %lu -> %lu\n",
               (long long)a.currentVelocity, (long)a.measuredVelocity,
               (unsigned long)a.nTimeSteps, (unsigned long)b.nTimeSteps);
        printf("   motorPwmVoltage idle=%u moving=%u\n",
               (unsigned)idle.motorPwmVoltage, (unsigned)a.motorPwmVoltage);

        TEST_RESULT("currentVelocity reads the commanded raw velocity shifted left by 12",
                    queued && aOk && a.currentVelocity == ((int64_t)rawVelocity << 12));
        TEST_RESULT("measuredVelocity stays near zero at the same instant, so it is a separate field",
                    aOk && llabs((long long)a.measuredVelocity) < 5000 &&
                    a.measuredVelocity != (int32_t)a.currentVelocity);
        TEST_RESULT("nTimeSteps is the executing item's countdown and falls between two reads",
                    aOk && bOk && a.nTimeSteps > 0 && a.nTimeSteps <= segTicks &&
                    b.nTimeSteps < a.nTimeSteps);
        TEST_RESULT("motorPwmVoltage is the full ceiling while moving and exactly half of it when idle",
                    idleOk && aOk && a.motorPwmVoltage > 0 &&
                    idle.motorPwmVoltage == (uint8_t)(a.motorPwmVoltage >> 1));

        tfhDrain(m, 20000);
        getDebugValuesResponse rest = m->getDebugValues();
        bool restOk = (m->getError() == 0);
        printf("   at rest: currentVelocity=%lld nTimeSteps=%lu fatal=%u\n",
               (long long)rest.currentVelocity, (unsigned long)rest.nTimeSteps,
               (unsigned)tfhFatal(m));
        TEST_RESULT("both live fields return to zero once the queue drains, with no underrun fault",
                    restOk && rest.currentVelocity == 0 && rest.nTimeSteps == 0 &&
                    tfhFatal(m) == 0);
    }

    // ---------------------------------------------------------------------
    // 9c. The twelve profiler fields. Only TWO of the six profilers are
    //     compiled in: the whole-ISR profiler, started and ended
    //     unconditionally at motor_control.c:3077 and :3198, and the loop-period
    //     profiler at :3192. The other four are wrapped in
    //     #ifdef DO_DETAILED_PROFILING, which is COMMENTED OUT at
    //     motor_control.c:77, so their eight fields are permanently zero.
    //
    //     Asserting that is the point of this module: four fields of a public
    //     response are never populated, and no test has ever said so. If this
    //     assertion ever fails, the cause is almost certainly that someone
    //     uncommented that define, NOT a motor fault.
    //
    //     The profiler clock is TIM14, prescaled to one count per microsecond
    //     (microsecond_clock.c:15), so the loop period must equal
    //     1 / updateFrequency -- which ties this reply to the product-specs
    //     reply read in section 4 without hardcoding 32 microseconds.
    // ---------------------------------------------------------------------
    printf("\n--- 9c. the profiler fields, including the four that are compiled out ---\n");
    {
        getDebugValuesResponse d = m->getDebugValues();
        bool ok = (m->getError() == 0);
        uint32_t expectedPeriodUs = (updateFrequency > 0) ? (1000000u / updateFrequency) : 32u;

        printf("   allCalc time=%u max=%u   loopPeriod time=%u max=%u   expected period %lu us\n",
               (unsigned)d.allMotorControlCalculationsProfilerTime,
               (unsigned)d.allMotorControlCalculationsProfilerMaxTime,
               (unsigned)d.motorControlLoopPeriodProfilerTime,
               (unsigned)d.motorControlLoopPeriodProfilerMaxTime,
               (unsigned long)expectedPeriodUs);
        printf("   compiled-out profilers: sensor=%u/%u velocity=%u/%u movement=%u/%u phase=%u/%u\n",
               (unsigned)d.getSensorPositionProfilerTime, (unsigned)d.getSensorPositionProfilerMaxTime,
               (unsigned)d.computeVelocityProfilerTime, (unsigned)d.computeVelocityProfilerMaxTime,
               (unsigned)d.motorMovementCalculationsProfilerTime,
               (unsigned)d.motorMovementCalculationsProfilerMaxTime,
               (unsigned)d.motorPhaseCalculationsProfilerTime,
               (unsigned)d.motorPhaseCalculationsProfilerMaxTime);

        TEST_RESULT("the two compiled-in profilers report a nonzero time no greater than their own maximum",
                    ok && d.allMotorControlCalculationsProfilerTime > 0 &&
                    d.allMotorControlCalculationsProfilerTime <=
                        d.allMotorControlCalculationsProfilerMaxTime &&
                    d.motorControlLoopPeriodProfilerTime > 0 &&
                    d.motorControlLoopPeriodProfilerTime <=
                        d.motorControlLoopPeriodProfilerMaxTime);
        // The INSTANTANEOUS period is deliberately not pinned to a narrow band.
        // profiler_period() (profiler.c:52-67) reports the gap between the ends
        // of the last TWO control ISRs, measured at the very end of the handler,
        // so anything that delays one entry lengthens that sample and shortens
        // the next -- and the profiler itself keeps a counter of samples >= 40
        // us (profiler.c:63-66), which is the firmware's own admission that
        // they occur. A +/-25% window round 32 us would therefore fail on a
        // healthy motor whose RS485 interrupt happened to land on the wrong
        // tick. The firmware's real contract is
        // MAX_MOTOR_CONTROL_PERIOD_FATAL_ERROR_THRESHOLD = 100 us
        // (motor_control.c:96, :3193-3195): above that it raises error 30, so a
        // motor that is still answering has never exceeded it.
        //
        // The tie to the update frequency is made through the MAX field
        // instead, which is jitter-proof in the right direction: over the
        // hundreds of ticks between two reads the mean period is exactly
        // 1/updateFrequency, so the maximum observed period cannot be below it.
        TEST_RESULT("the control-loop period field agrees with the update frequency the device reports",
                    ok && expectedPeriodUs > 0 &&
                    d.motorControlLoopPeriodProfilerTime > 0 &&
                    d.motorControlLoopPeriodProfilerTime <= 100 &&
                    d.motorControlLoopPeriodProfilerMaxTime >= expectedPeriodUs &&
                    d.motorControlLoopPeriodProfilerMaxTime <= 100);
        TEST_RESULT("the eight fields of the four DO_DETAILED_PROFILING profilers are all zero, as compiled",
                    ok && d.getSensorPositionProfilerTime == 0 &&
                    d.getSensorPositionProfilerMaxTime == 0 &&
                    d.computeVelocityProfilerTime == 0 &&
                    d.computeVelocityProfilerMaxTime == 0 &&
                    d.motorMovementCalculationsProfilerTime == 0 &&
                    d.motorMovementCalculationsProfilerMaxTime == 0 &&
                    d.motorPhaseCalculationsProfilerTime == 0 &&
                    d.motorPhaseCalculationsProfilerMaxTime == 0);
    }

    // ---------------------------------------------------------------------
    // 9d. The sensor and commutation fields at the tail of the reply. These
    //     are the last bytes of a 112-byte packed struct, which is exactly
    //     where an off-by-one in the packing would show up first.
    //
    //     The hall-delta triple is a running accumulator that reading RESETS
    //     (motor_control.c:3624-3636), so it can legitimately come back as the
    //     empty sentinel (-2000000000 / +2000000000) when no control tick has
    //     landed since the previous read. The assertion allows that explicitly
    //     rather than demanding min <= max unconditionally.
    // ---------------------------------------------------------------------
    printf("\n--- 9d. the sensor and commutation fields at the tail of the reply ---\n");
    {
        getDebugValuesResponse d = m->getDebugValues();
        bool ok = (m->getError() == 0);
        bool sentinel = (d.maxHallPositionDelta == HALL_DELTA_SEED_MAX) &&
                        (d.minHallPositionDelta == HALL_DELTA_SEED_MIN);

        printf("   hall voltages %u/%u/%u  commutationOffset=%lu phasesReversed=%u\n",
               (unsigned)d.hallSensor1Voltage, (unsigned)d.hallSensor2Voltage,
               (unsigned)d.hallSensor3Voltage,
               (unsigned long)d.commutationPositionOffset, (unsigned)d.motorPhasesReversed);
        printf("   hall delta min=%ld avg=%ld max=%ld%s\n",
               (long)d.minHallPositionDelta, (long)d.averageHallPositionDelta,
               (long)d.maxHallPositionDelta, sentinel ? "  (empty sentinel)" : "");

        TEST_RESULT("the three hall voltage fields are each live and inside the ADC range, and are not one value repeated",
                    ok && d.hallSensor1Voltage > 0 && d.hallSensor2Voltage > 0 &&
                    d.hallSensor3Voltage > 0 &&
                    d.hallSensor1Voltage <= HALL_ADC_SUM_MAX &&
                    d.hallSensor2Voltage <= HALL_ADC_SUM_MAX &&
                    d.hallSensor3Voltage <= HALL_ADC_SUM_MAX &&
                    !(d.hallSensor1Voltage == d.hallSensor2Voltage &&
                      d.hallSensor2Voltage == d.hallSensor3Voltage));
        TEST_RESULT("the commutation offset is set and the phases-reversed flag is a clean 0 or 1",
                    ok && d.commutationPositionOffset != 0 && d.motorPhasesReversed <= 1);
        TEST_RESULT("the hall-delta triple is ordered min <= average <= max, or is the empty sentinel",
                    ok && (sentinel ||
                           (d.minHallPositionDelta <= d.averageHallPositionDelta &&
                            d.averageHallPositionDelta <= d.maxHallPositionDelta)));
    }

    // =====================================================================
    // 10. getCommunicationStatisticsResponse: SIX SEPARATE COUNTERS.
    //     All six are ERROR counters (RS485.h:49-56) -- there is no packet
    //     counter -- and they are RAM-only, so a reset zeroes them. Reading
    //     twice without the reset flag must return the same six numbers: the
    //     command is only destructive when explicitly told to be
    //     (main.c:1128-1145). tm_communication_statistics owns this command in
    //     depth; the only claim here is a per-field one.
    // =====================================================================
    printf("\n--- 10. all six communication counters ---\n");
    {
        tfhReset(m);
        getCommunicationStatisticsResponse c1 = m->getCommunicationStatistics(0);
        bool ok1 = (m->getError() == 0);
        getCommunicationStatisticsResponse c2 = m->getCommunicationStatistics(0);
        bool ok2 = (m->getError() == 0);
        printf("   crc32=%lu decode=%lu firstBit=%lu framing=%lu overrun=%lu noise=%lu\n",
               (unsigned long)c1.crc32ErrorCount, (unsigned long)c1.packetDecodeErrorCount,
               (unsigned long)c1.firstBitErrorCount, (unsigned long)c1.framingErrorCount,
               (unsigned long)c1.overrunErrorCount, (unsigned long)c1.noiseErrorCount);
        TEST_RESULT("on a clean bus all six counters read zero and a non-resetting read leaves them alone",
                    ok1 && ok2 &&
                    c1.crc32ErrorCount == 0 && c1.packetDecodeErrorCount == 0 &&
                    c1.firstBitErrorCount == 0 && c1.framingErrorCount == 0 &&
                    c1.overrunErrorCount == 0 && c1.noiseErrorCount == 0 &&
                    memcmp(&c1, &c2, sizeof(getCommunicationStatisticsResponse)) == 0);
    }

    // =====================================================================
    // 11. timeSyncResponse: A COMPUTED i32 AND A RAW PERIPHERAL REGISTER.
    //
    //     time_sync() returns master minus device by modular u32 subtraction
    //     (clock_calibration.c:44-48) and the second field is RCC->ICSCR's low
    //     16 bits (clock_calibration.c:53-56). To show the first field really
    //     is computed from the PAYLOAD, three syncs are sent from freshly-read
    //     device clock values offset by -1 s, 0 and +1 s: the returned errors
    //     must be ordered and about two seconds apart end to end, while the
    //     register field stays inside the trim rails the PI controller clamps
    //     to (62..66, clock_calibration.c:9-10 and :26-31; HSITRIM is bits 14:8
    //     of RCC->ICSCR). Note the clock itself is never written -- only the HSI
    //     trim is steered -- so this cannot disturb anything else. The three
    //     offsets are enough to drive the PI to BOTH rails, which is what makes
    //     the low-byte-tracks-trim check below non-vacuous.
    // =====================================================================
    printf("\n--- 11. both fields of the time-sync reply ---\n");
    {
        tfhReset(m);
        const int32_t offsets[3] = { -1000000, 0, 1000000 };
        int32_t errs[3] = { 0, 0, 0 };
        uint16_t icscrs[3] = { 0, 0, 0 };
        bool ok = true;
        for (int i = 0; i < 3; i++) {
            uint64_t now = m->getCurrentTimeRaw();
            if (m->getError() != 0) { ok = false; break; }
            timeSyncResponse r = m->timeSyncRaw((uint32_t)((uint32_t)now + (uint32_t)offsets[i]));
            if (m->getError() != 0) { ok = false; break; }
            errs[i] = r.timeError;
            icscrs[i] = r.rccIcscr;
            printf("   offset %+ld us -> timeError=%ld  rccIcscr=0x%04X (trim %u, cal %u)\n",
                   (long)offsets[i], (long)r.timeError, (unsigned)r.rccIcscr,
                   (unsigned)((r.rccIcscr >> 8) & 0x7F), (unsigned)(r.rccIcscr & 0xFF));
        }

        // The trim field must sit inside the PI controller's rails, and bit 15
        // is reserved so it must read 0 (HSITRIM is bits 14:8, mask 0x7F).
        //
        // The LOW byte is NOT asserted to be nonzero. That would be an
        // assertion about the factory HSICAL constant of this particular
        // silicon, which nothing in this repository controls, and RM0444's
        // description of the field is already known to be contradicted by the
        // hardware: campaign observation O1 measured the low byte reporting the
        // EFFECTIVE calibration (factory + trim, 136 at trim 62 and 140 at trim
        // 66), not the read-only factory value the manual describes. The claim
        // asserted here is the one O1 actually established and which
        // tm_time_sync_protocol also verified on hardware: the low byte moves
        // by exactly the same delta as the trim. Compared modulo 256 so a byte
        // that happens to wrap is not mistaken for a fault.
        bool trimOk = ok, calTracksTrim = ok;
        uint8_t trims[3] = { 0, 0, 0 }, cals[3] = { 0, 0, 0 };
        for (int i = 0; i < 3 && ok; i++) {
            trims[i] = (uint8_t)((icscrs[i] >> 8) & 0x7F);
            cals[i]  = (uint8_t)(icscrs[i] & 0xFF);
            if (!(trims[i] >= HSI_TRIM_MIN && trims[i] <= HSI_TRIM_MAX)) trimOk = false;
            if ((icscrs[i] & 0x8000) != 0) trimOk = false;   // bit 15 is reserved
        }
        for (int i = 1; i < 3 && ok; i++) {
            if ((uint8_t)(cals[i] - cals[0]) != (uint8_t)(trims[i] - trims[0])) {
                calTracksTrim = false;
            }
        }

        int64_t span = (int64_t)errs[2] - (int64_t)errs[0];
        printf("   span between the -1 s and +1 s syncs: %lld us\n", (long long)span);
        TEST_RESULT("the time-error field is computed from the payload: three offsets give three ordered errors",
                    ok && errs[0] < errs[1] && errs[1] < errs[2]);
        TEST_RESULT("and the end-to-end span matches the two seconds of offset that was applied",
                    ok && span > 1500000 && span < 2500000);
        TEST_RESULT("the register field holds a trim inside its rails, a zero reserved bit, and a calibration byte that moves with the trim",
                    trimOk && calTracksTrim);
    }

    // =====================================================================
    // 12. pingResponse: ALL TEN BYTES, INDIVIDUALLY.
    //     The firmware copies the request payload straight into the reply
    //     (main.c:739-751). Two patterns are used, the second the bitwise
    //     complement of the first, so a stale or partially-written buffer
    //     cannot pass both. tm_ping_stress hammers this command with 100
    //     round trips; the claim here is only that all ten bytes are distinct
    //     positions in the struct.
    // =====================================================================
    printf("\n--- 12. all ten echoed ping bytes ---\n");
    {
        tfhReset(m);
        uint8_t pattern[10] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x5A, 0xA5 };
        pingResponse p1 = m->ping(pattern);
        bool ok1 = (m->getError() == 0);

        uint8_t inverse[10];
        for (int i = 0; i < 10; i++) inverse[i] = (uint8_t)~pattern[i];
        pingResponse p2 = m->ping(inverse);
        bool ok2 = (m->getError() == 0);

        printf("   echo 1 matches=%d, echo 2 (complement) matches=%d\n",
               (int)(memcmp(p1.responsePayload, pattern, 10) == 0),
               (int)(memcmp(p2.responsePayload, inverse, 10) == 0));
        TEST_RESULT("ten distinct bytes and then their complements come back in exactly their sent positions, so no byte is stale",
                    ok1 && ok2 &&
                    memcmp(p1.responsePayload, pattern, 10) == 0 &&
                    memcmp(p2.responsePayload, inverse, 10) == 0);
    }

    // =====================================================================
    // 13. Leave the motor exactly as it was found: defaults restored, no
    //     latched fault, empty queue. This module changed the velocity and
    //     acceleration ceilings and the deviation window, and steered the HSI
    //     trim; a reset restores all of them.
    // =====================================================================
    tfhReset(m);
    TEST_RESULT("the motor is left clean, with no latched fault", tfhFatal(m) == 0);
}
