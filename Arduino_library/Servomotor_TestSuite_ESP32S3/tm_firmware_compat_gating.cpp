#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_firmware_compat_gating.cpp
//
// THE VERSION AND COMPATIBILITY FIELDS AS A CONTRACT, NOT AS DECORATION.
//
// Two read-only commands describe which software and which hardware this motor
// is:
//
//   Get product info (cmd 22)      -> productCode[8], firmwareCompatibility,
//                                     hardwareVersion{major,minor,patch}, ...
//   Get firmware version (cmd 25)  -> firmwareVersion{major,minor,patch,dev},
//                                     inBootloader
//
// Three existing modules already read those fields. This one is deliberately
// NOT a fourth reading of them:
//
//   * tm_product_info          reads each field once and range-checks it.
//   * tm_version_info_consistency reads them 3-5 times and checks they do not
//                              change between reads.
//   * tm_device_identity       checks the reported constants (countsPerRotation,
//                              updateFrequency) are physically true, and that
//                              identity survives a reset and a fault.
//
// All three ask "is the field plausible and stable?". None of them asks the
// question that actually matters to a caller:
//
//     CAN I TRUST THESE NUMBERS ENOUGH TO CHANGE MY BEHAVIOUR BASED ON THEM?
//
// That question has two halves, and this module is built around them.
//
// HALF ONE -- the compatibility code MEANS something specific. It is not a
// label. It is the key the bootloader compares before it will burn a firmware
// page:
//
//     bootloader_STM32G031/Src/main.c:184  memcmp(payload, model_code, 8)
//     bootloader_STM32G031/Src/main.c:189  payload[8] != firmware_compatibility_code
//                                          -> "Firmware compatibility code mismatch"
//
// and it is exactly the 9 header bytes bin2firmware.c:129-130 writes at the
// head of every .firmware file, and the `sccN` token in the release filename
// (servomotor_M17_fw0.15.12.0_scc3_hw1.5.firmware). It also selects which
// driver source is compiled in at all (firmware/Src/motor_control.h:8,
// common_source_files/gpio_M17.c:85). The (model, hardware version) ->
// compatibility code mapping is recorded in firmware/VERSIONS. So the code the
// device reports is checkable against a table, not merely "in range", and a
// caller can build the correct upgrade filename for a motor it has never seen
// from nothing but these two replies. That is asserted here.
//
// HALF TWO -- and this is the useful part -- the four-part firmware version
// must be a RELIABLE GATE. The August 2026 campaign is running one test binary
// against two firmwares at once (the 35-motor rack on 0.15.12.0, the ESP32-S3
// bench motor on 0.15.9.0), and several documented behaviours differ between
// them. Every module written so far hardcodes ONE side of that split and simply
// fails on the other. This module instead READS the version, PREDICTS from it
// which behaviour it should see, and then RUNS the probe and checks the
// prediction. That makes the version field itself the thing under test: if the
// reported version did not correspond to the code actually running, the
// predictions would miss.
//
// WHICH CAMPAIGN FINDING APPLIES TO WHICH VERSION
//
//   behaviour                                              applies to
//   ----------------------------------------------------   -------------------
//   planner divide-by-zero: a move whose ramp truncates     fw <  0.15.12.0
//     to zero ticks is ACCEPTED, raises no error and
//     never moves (motor_control.c:456-484 is the fix)
//   int64->int32 truncation of the computed acceleration:   fw <  0.15.12.0
//     an over-ambitious move silently becomes a different
//     move (motor_control.c:510-515 saturates instead)
//   over-ambitious move refused with ERROR_ACCEL_TOO_HIGH   fw >= 0.15.12.0
//     (15) rather than silently accepted
//   two-slot fast path for a one-tick ramp                  fw >= 0.15.12.0
//     (motor_control.c:2017, delta_t1 <= 1 && total >= 3)
//   zero-duration move refused with                         fw >= 0.15.9.0
//     ERROR_PARAMETER_OUT_OF_RANGE (34)
//     (motor_control.c:2051-2053)
//   F1: lowering max_velocity mid-move faults the move      all versions tested
//     with ERROR_VEL_TOO_HIGH (16) (motor_control.c:2187)
//   F2: a faulted motor answers only get_status             all versions tested
//     (common_source_files/error_handling.c:169)
//   O1: RCC_ICSCR's low byte reports the EFFECTIVE          all versions tested
//     calibration, not the factory constant
//
// The three gated rows above are what sections 7's probes exercise; the
// ungated rows are used as CONTROLS, because a gate that predicts a difference
// everywhere is not a gate.
//
// SAFETY. MOSFETs stay OFF throughout -- they are never enabled and closed loop
// is never entered. Every measurement reads the COMMANDED position, which the
// planner advances whether or not the output stage is energised, so nothing
// turns on any motor. The largest commanded displacement is 3 rotations, used
// once and deliberately, to trip the two-rotation deviation watchdog and prove
// the bootloader flag survives a fault; every other move stays under half a
// rotation. No firmware upgrade is ever sent (the upgrade contract is asserted
// on the header bytes a caller would build, never by burning anything). No
// broadcasts, no alias writes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_ACCEL_TOO_HIGH        = 15;  // error_text.h:22
static const uint8_t ERR_PARAMETER_OUT_OF_RANGE = 34; // error_text.h:41
static const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45; // error_text.h:52

// ---------------------------------------------------------------------------
// The whole point of a four-part version is that it ORDERS. Pack it into one
// comparable key, most significant part first, dev last -- which is the byte
// order the firmware's own struct uses (main.c:63-73: {development, bugfix,
// minor, major}) and the order the library's VersionNumber32 mirrors.
// ---------------------------------------------------------------------------
static uint32_t packVersion(uint8_t major, uint8_t minor, uint8_t patch, uint8_t dev) {
    return ((uint32_t)major << 24) | ((uint32_t)minor << 16) |
           ((uint32_t)patch << 8) | (uint32_t)dev;
}

// The two gate boundaries this module reasons about, from the release history:
//   fa0b80b "Firmware 0.15.12.0: fix three ways a move could silently do the
//            wrong thing"       -> the planner fixes
//   3806596 "Firmware 0.15.9.0: fatal-error reply-race fix, validation
//            hardening"          -> the zero-duration guard
static const uint32_t FW_PLANNER_FIXES = 0x000F0C00u;  // 0.15.12.0
static const uint32_t FW_ZERO_TIME_GUARD = 0x000F0900u; // 0.15.9.0

// ---------------------------------------------------------------------------
// firmware/VERSIONS maps a (product, hardware version) pair to its software
// compatibility code. Active rows are uncommented there; retired rows are
// commented out but were really shipped, so they are accepted here too.
//
// M1 (V26), M2 (V4) and M4 (V1) are deliberately absent: their hardware version
// strings are not three decimal numbers, so they cannot be represented in the
// three bytes product_info_struct carries (generate_product_info.c
// decompose_hardware_string rejects the leading 'V'). A motor reporting one of
// those would legitimately fail the table lookup below, which is the honest
// answer rather than a silently widened check.
// ---------------------------------------------------------------------------
struct VersionsRow {
    const char* model;
    uint8_t hwMajor;
    uint8_t hwMinor;
    uint8_t hwPatch;
    uint8_t scc;
};
static const VersionsRow VERSIONS_TABLE[] = {
    { "M17", 1,  5, 0, 3 },   // active   -- firmware/VERSIONS line 9
    { "M17", 1,  0, 0, 3 },   // retired  -- firmware/VERSIONS line 8
    { "M23", 0, 10, 0, 1 },   // active   -- firmware/VERSIONS line 11
    { "M3",  1,  3, 0, 3 },   // retired
    { "M3",  1,  2, 0, 3 },   // retired
    { "M3",  1,  1, 0, 3 },   // retired (V11)
};

// Copy the 8-byte product code into a NUL-terminated model name, dropping the
// trailing space padding bin2firmware.c:151/166 writes.
static void trimModel(const char src[8], char out[9]) {
    int n = 8;
    for (int i = 0; i < 8; i++) out[i] = src[i];
    out[8] = '\0';
    while (n > 0 && (out[n - 1] == ' ' || out[n - 1] == '\0')) { out[n - 1] = '\0'; n--; }
}

// Look up the compatibility code firmware/VERSIONS records for this device.
// Returns -1 if the (model, hardware version) pair is not a known row.
static int expectedSccFor(const char* model, uint8_t maj, uint8_t min, uint8_t pat) {
    for (unsigned i = 0; i < sizeof(VERSIONS_TABLE) / sizeof(VERSIONS_TABLE[0]); i++) {
        const VersionsRow& r = VERSIONS_TABLE[i];
        if (strcmp(r.model, model) == 0 && r.hwMajor == maj &&
            r.hwMinor == min && r.hwPatch == pat) {
            return (int)r.scc;
        }
    }
    return -1;
}

// WEAKENED DELIBERATELY (adversarial review): the table above is a SNAPSHOT of
// firmware/VERSIONS taken while this module was written, and requiring an exact
// (model, hardware version) row would FAIL on a perfectly healthy motor whose
// board revision was never written into that file, or was added to it after
// this snapshot. Every row VERSIONS carries for a given product agrees on the
// compatibility code anyway (M17 -> 3 for both 1.0 and 1.5, M3 -> 3 for all
// three rows, M23 -> 1), so the code is predictable from the MODEL alone. That
// keeps the claim that actually matters -- the reported code is the one
// VERSIONS records for this product -- without inventing a hardware-revision
// whitelist the firmware never promised. Returns -1 if the model is unknown or
// (were the file ever to change) its rows disagree.
static int sccForModel(const char* model) {
    int found = -1;
    for (unsigned i = 0; i < sizeof(VERSIONS_TABLE) / sizeof(VERSIONS_TABLE[0]); i++) {
        const VersionsRow& r = VERSIONS_TABLE[i];
        if (strcmp(r.model, model) != 0) continue;
        if (found < 0) found = (int)r.scc;
        else if (found != (int)r.scc) return -1;
    }
    return found;
}

// ---------------------------------------------------------------------------
// A scanf-free parser for the release filename, used by section 3.
//
// The obvious way to write that round trip is sscanf with a "%15[^_]" scanset.
// No other module in this suite uses scanf at all, and the ESP32 build's newlib
// can be built without the scanset conversion, in which case the parse returns
// zero fields and the assertion fails on firmware that is entirely correct.
// These two helpers use nothing but pointer arithmetic and strncmp, so the
// round trip tests the NAMING, which is the point, rather than the C library.
// ---------------------------------------------------------------------------
static bool scanLiteral(const char** p, const char* lit) {
    size_t n = strlen(lit);
    if (strncmp(*p, lit, n) != 0) return false;
    *p += n;
    return true;
}

static bool scanUint(const char** p, unsigned* out) {
    const char* s = *p;
    if (*s < '0' || *s > '9') return false;
    unsigned v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10u + (unsigned)(*s - '0'); s++; }
    *p = s;
    *out = v;
    return true;
}

// One probe: command a move, report exactly what happened. Deliberately does
// NOT assume the move is legal -- that is the thing being predicted.
struct Probe {
    bool    accepted;   // the command itself did not answer with an error
    uint8_t queued;     // queue depth read immediately after acceptance
    uint8_t fatal;      // latched fatal code afterwards (0 = none)
    int64_t moved;      // commanded-position delta (0 if unreadable)
};

static Probe probeMove(Servomotor* m, int32_t counts, uint32_t ticks, uint32_t drainMs) {
    Probe p;
    p.accepted = false; p.queued = 0; p.fatal = 0; p.moved = 0;
    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) return p;
    m->trapezoidMoveRaw(counts, ticks);
    p.accepted = (m->getError() == 0);
    if (p.accepted) {
        p.queued = m->getNQueuedItems();
        if (m->getError() != 0) p.queued = 0xFF;
        tfhDrain(m, drainMs);
    } else {
        delay(300);
    }
    p.fatal = tfhFatal(m);
    if (p.fatal == 0) {
        int64_t after = m->getPositionRaw();
        if (m->getError() == 0) p.moved = after - before;
    }
    return p;
}

void tm_firmware_compat_gating(void) {
    Serial.println("tm_firmware_compat_gating: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE FOUR-PART VERSION READS AND DECODES. Everything downstream is
    //    arithmetic on these four bytes, so they are established first and
    //    the packed key is computed once and reused for every prediction.
    // =====================================================================
    printf("\n--- 1. the firmware version decodes into four sane parts ---\n");
    uint32_t fwKey = 0;
    getFirmwareVersionResponse fwRef;
    memset(&fwRef, 0, sizeof(fwRef));
    {
        tfhReset(m);
        fwRef = m->getFirmwareVersion();
        bool ok = (m->getError() == 0);
        const VersionNumber32 v = fwRef.firmwareVersion;
        fwKey = packVersion(v.major, v.minor, v.patch, v.dev);
        printf("   firmware %u.%u.%u.%u  (packed key 0x%08lX)  inBootloader=%u\n",
               (unsigned)v.major, (unsigned)v.minor, (unsigned)v.patch,
               (unsigned)v.dev, (unsigned long)fwKey, (unsigned)fwRef.inBootloader);

        TEST_RESULT("the firmware version command answers", ok);
        TEST_RESULT("the version has at least one non-zero part", ok && fwKey != 0);
        // The parts are decimal-coded for humans (0.15.12.0), so each should be
        // a small number; a byte-order mistake anywhere would show up as a part
        // in the hundreds.
        TEST_RESULT("every part of the version is a sane decimal-coded number",
                    ok && v.major < 100 && v.minor < 100 && v.patch < 100 && v.dev < 100);
        // 0.15.9.0 is the OLDEST firmware anywhere in this fleet (the ESP32-S3
        // bench motor). Anything older than that and the predictions below have
        // no basis, so this is a precondition rather than a style check.
        TEST_RESULT("the version is at least 0.15.9.0, the oldest firmware in this fleet",
                    ok && fwKey >= FW_ZERO_TIME_GUARD);
        TEST_RESULT("the bootloader flag is a boolean",
                    ok && (fwRef.inBootloader == 0 || fwRef.inBootloader == 1));
    }

    // =====================================================================
    // 2. THE COMPATIBILITY CODE MATCHES WHAT firmware/VERSIONS RECORDS FOR
    //    THIS HARDWARE. "In range for a uint8" is not a check -- the code is
    //    one specific number per (product, hardware version) pair, and the
    //    pair is reported by the same command, so the two can be checked
    //    against each other rather than merely accepted.
    // =====================================================================
    printf("\n--- 2. the compatibility code matches this hardware's row in firmware/VERSIONS ---\n");
    getProductInfoResponse infoRef;
    memset(&infoRef, 0, sizeof(infoRef));
    char model[9];
    model[0] = '\0';
    {
        infoRef = m->getProductInfo();
        bool ok = (m->getError() == 0);
        trimModel(infoRef.productCode, model);

        // The model name must sit at the HEAD of the 8 bytes with nothing but
        // padding after it: the bootloader compares all 8 bytes.
        bool leftAligned = ok && model[0] != '\0';
        for (int i = 0; i < (int)strlen(model); i++) {
            if (model[i] == ' ') leftAligned = false;   // no interior spaces
        }

        // exactScc is the row for this precise board revision if firmware/VERSIONS
        // happens to list it; wantScc is the product-wide value, which is what is
        // actually asserted (see the comment on sccForModel).
        int exactScc = expectedSccFor(model, infoRef.hardwareVersion.major,
                                      infoRef.hardwareVersion.minor,
                                      infoRef.hardwareVersion.patch);
        int wantScc = sccForModel(model);
        printf("   model '%s'  hardware %u.%u.%u  compatibility code %u  "
               "(firmware/VERSIONS: this product %d, this exact revision %d)\n",
               model, (unsigned)infoRef.hardwareVersion.major,
               (unsigned)infoRef.hardwareVersion.minor,
               (unsigned)infoRef.hardwareVersion.patch,
               (unsigned)infoRef.firmwareCompatibility, wantScc, exactScc);

        TEST_RESULT("the product info command answers", ok);
        TEST_RESULT("the model name sits at the head of the product code with no interior spaces",
                    leftAligned);
        TEST_RESULT("the reported model is a product listed in firmware/VERSIONS",
                    ok && wantScc >= 0);
        TEST_RESULT("the compatibility code is the one firmware/VERSIONS records for this product",
                    ok && wantScc >= 0 && (int)infoRef.firmwareCompatibility == wantScc);
        // Vacuously true for a board revision the table does not list, which is a
        // legitimate state for a healthy motor -- so this tightens the check only
        // where firmware/VERSIONS really does pin the answer.
        TEST_RESULT("where firmware/VERSIONS lists this exact board revision, the code matches that row",
                    ok && (exactScc < 0 || (int)infoRef.firmwareCompatibility == exactScc));

        // Unchanged after real work -- not another back-to-back read, but a
        // re-read on the far side of a completed move, which is when a field
        // backed by a mutable copy rather than by flash would drift.
        tfhFreshOpen(m);
        bool moveOk = false;
        tfhMove(m, (int32_t)(TFH_CPR / 8), TFH_TPS, &moveOk);
        tfhReset(m);
        getProductInfoResponse again = m->getProductInfo();
        bool ok2 = (m->getError() == 0);
        TEST_RESULT("the compatibility code and hardware version are unchanged after real work",
                    ok && ok2 && moveOk &&
                    again.firmwareCompatibility == infoRef.firmwareCompatibility &&
                    again.hardwareVersion.major == infoRef.hardwareVersion.major &&
                    again.hardwareVersion.minor == infoRef.hardwareVersion.minor &&
                    again.hardwareVersion.patch == infoRef.hardwareVersion.patch);
    }

    // =====================================================================
    // 3. WHAT THE COMPATIBILITY CODE IS FOR: it is the upgrade key. The
    //    bootloader refuses any firmware page whose 8-byte model code or
    //    following compatibility byte differ from the device's own
    //    (bootloader_STM32G031/Src/main.c:184 and :189). So a caller must be
    //    able to derive the right image for this device from these replies
    //    alone -- which is exactly what upgrade_firmware.py:120-121 reads out
    //    of the file and compares. Nothing is burned here; the assertions are
    //    on the header a caller would have to construct.
    // =====================================================================
    printf("\n--- 3. the compatibility code is the firmware-upgrade key ---\n");
    {
        // The padding really must be spaces: bin2firmware.c:151 starts the
        // model code as eight spaces and copies the name over the front, and
        // the bootloader memcmp's all eight bytes. NUL padding would build an
        // image no device in this fleet could ever accept.
        int nameLen = (int)strlen(model);
        bool spacePadded = (nameLen > 0) && (nameLen <= 8);
        for (int i = nameLen; i < 8; i++) {
            if (infoRef.productCode[i] != ' ') spacePadded = false;
        }
        TEST_RESULT("the product code is padded with spaces, which is what the bootloader compares",
                    spacePadded);

        // A zero compatibility code would match an image whose header byte was
        // simply never filled in. Every current product carries a non-zero one.
        TEST_RESULT("the compatibility code is non-zero, so an image with an unset header cannot match",
                    infoRef.firmwareCompatibility != 0);

        // Build the release filename this device needs, purely from its own two
        // replies, and parse it back. Round-tripping proves the fields carry
        // everything the upgrade path needs and nothing is lost in the naming.
        const VersionNumber32 v = fwRef.firmwareVersion;
        char fname[96];
        snprintf(fname, sizeof(fname),
                 "servomotor_%s_fw%u.%u.%u.%u_scc%u_hw%u.%u.firmware",
                 model, (unsigned)v.major, (unsigned)v.minor,
                 (unsigned)v.patch, (unsigned)v.dev,
                 (unsigned)infoRef.firmwareCompatibility,
                 (unsigned)infoRef.hardwareVersion.major,
                 (unsigned)infoRef.hardwareVersion.minor);
        printf("   this device's own release filename: %s\n", fname);
        TEST_RESULT("a release filename for this device can be derived from its own answers",
                    strlen(fname) > 20 && strstr(fname, ".firmware") != NULL);

        // Parsed with the local helpers rather than sscanf: see the comment on
        // scanLiteral/scanUint. The grammar checked is identical, and the trailing
        // *p == '\0' means nothing may be left over.
        char backModel[16];
        backModel[0] = '\0';
        unsigned bMaj = 0, bMin = 0, bPat = 0, bDev = 0, bScc = 0, bHwMaj = 0, bHwMin = 0;
        const char* p = fname;
        bool parsed = scanLiteral(&p, "servomotor_");
        if (parsed) {
            const char* start = p;
            while (*p != '\0' && *p != '_') p++;
            size_t n = (size_t)(p - start);
            if (n == 0 || n >= sizeof(backModel)) {
                parsed = false;
            } else {
                memcpy(backModel, start, n);
                backModel[n] = '\0';
            }
        }
        parsed = parsed &&
                 scanLiteral(&p, "_fw")  && scanUint(&p, &bMaj)   && scanLiteral(&p, ".") &&
                 scanUint(&p, &bMin)     && scanLiteral(&p, ".")  && scanUint(&p, &bPat)  &&
                 scanLiteral(&p, ".")    && scanUint(&p, &bDev)   &&
                 scanLiteral(&p, "_scc") && scanUint(&p, &bScc)   &&
                 scanLiteral(&p, "_hw")  && scanUint(&p, &bHwMaj) && scanLiteral(&p, ".") &&
                 scanUint(&p, &bHwMin)   && scanLiteral(&p, ".firmware") && (*p == '\0');
        bool roundTrip = parsed &&
                         (strcmp(backModel, model) == 0) &&
                         (bMaj == v.major) && (bMin == v.minor) &&
                         (bPat == v.patch) && (bDev == v.dev) &&
                         (bScc == infoRef.firmwareCompatibility) &&
                         (bHwMaj == infoRef.hardwareVersion.major) &&
                         (bHwMin == infoRef.hardwareVersion.minor);
        TEST_RESULT("parsing that filename back yields the same model, version and compatibility code",
                    roundTrip);
    }

    // =====================================================================
    // 4. THE BOOTLOADER FLAG IS 0 IN NORMAL OPERATION, UNDER EVERY CONDITION
    //    A CALLER MIGHT READ IT. main.c:604 hard-codes in_bootloader = 0 in
    //    the main firmware, so a 1 can only mean the bootloader image
    //    answered instead. That makes the byte a "which image is talking"
    //    discriminator, and its value is only worth anything if it is 0 at
    //    every moment the main firmware is genuinely in charge -- including
    //    the moment right after a reset, when the bootloader window has only
    //    just closed, and after a fault.
    // =====================================================================
    printf("\n--- 4. the bootloader flag stays 0 while the main firmware is in charge ---\n");
    {
        tfhReset(m);
        getFirmwareVersionResponse first = m->getFirmwareVersion();  // first command after reset
        bool okFirst = (m->getError() == 0);
        TEST_RESULT("the bootloader flag is 0 on the very first command after a reset",
                    okFirst && first.inBootloader == 0);

        delay(1000);
        getFirmwareVersionResponse settled = m->getFirmwareVersion();
        bool okSettled = (m->getError() == 0);
        TEST_RESULT("the bootloader flag is still 0 once the device has settled",
                    okSettled && settled.inBootloader == 0);

        // In flight: queue a two-second move and read the flag while it runs.
        tfhFreshOpen(m);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 2 * TFH_TPS);
        bool queued = (m->getError() == 0);
        delay(400);
        getFirmwareVersionResponse moving = m->getFirmwareVersion();
        bool okMoving = (m->getError() == 0);
        bool stillBusy = (m->getNQueuedItems() > 0);
        tfhDrain(m, 12000);
        printf("   read during motion: queue was %sbusy, inBootloader=%u\n",
               stillBusy ? "" : "NOT ", (unsigned)moving.inBootloader);
        TEST_RESULT("the bootloader flag is 0 while a move is in flight",
                    queued && okMoving && moving.inBootloader == 0);

        // After a fault: trip the deviation watchdog, which is version
        // independent (3 rotations against the factory two-rotation window),
        // then recover and re-read.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 3), 2 * TFH_TPS);
        uint32_t deadline = millis() + 9000;
        while (millis() < deadline) {
            if (m->getNQueuedItems() == 0 || m->getError() != 0) break;
            delay(25);
        }
        uint8_t faultCode = tfhFatal(m);
        tfhReset(m);
        getFirmwareVersionResponse afterFault = m->getFirmwareVersion();
        bool okAfter = (m->getError() == 0);
        printf("   deliberate fault code %u, then inBootloader=%u\n",
               (unsigned)faultCode, (unsigned)afterFault.inBootloader);
        TEST_RESULT("the machine really was faulted in between",
                    faultCode == ERR_POSITION_DEVIATION_TOO_LARGE);
        TEST_RESULT("the bootloader flag is 0 again after a fault and a recovery",
                    okAfter && afterFault.inBootloader == 0);
    }

    // =====================================================================
    // 5. DETECTION IS RELIABLE. A gate is only as good as the read that feeds
    //    it, and a caller gating on the version reads it exactly once, with no
    //    setup, at whatever moment it happens to start. So: answerable as the
    //    very first thing after a reset, identical over many reads, and
    //    identical when read in the middle of motion rather than at rest --
    //    and the reading must not perturb the motion it is read during.
    // =====================================================================
    printf("\n--- 5. the version can be detected reliably, at any moment ---\n");
    {
        tfhReset(m);
        getFirmwareVersionResponse cold = m->getFirmwareVersion();
        bool okCold = (m->getError() == 0);
        TEST_RESULT("the version is readable as the first command after a reset, with no setup",
                    okCold && packVersion(cold.firmwareVersion.major, cold.firmwareVersion.minor,
                                          cold.firmwareVersion.patch, cold.firmwareVersion.dev) == fwKey);

        bool identical = okCold;
        for (int i = 0; i < 12 && identical; i++) {
            getFirmwareVersionResponse r = m->getFirmwareVersion();
            if (m->getError() != 0 ||
                packVersion(r.firmwareVersion.major, r.firmwareVersion.minor,
                            r.firmwareVersion.patch, r.firmwareVersion.dev) != fwKey ||
                r.inBootloader != 0) {
                identical = false;
                printf("   read %d disagreed with the reference version\n", i);
            }
        }
        TEST_RESULT("twelve consecutive reads return an identical version", identical);

        // Read it hard during a move, then check the move still landed where it
        // was asked to. Gating code runs alongside real work; the read must be
        // free.
        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        int32_t dist = (int32_t)(TFH_CPR / 4);
        m->trapezoidMoveRaw(dist, 2 * TFH_TPS);
        bool queued = (m->getError() == 0);
        bool sameDuringMotion = queued;
        for (int i = 0; i < 15 && sameDuringMotion; i++) {
            getFirmwareVersionResponse r = m->getFirmwareVersion();
            if (m->getError() != 0 ||
                packVersion(r.firmwareVersion.major, r.firmwareVersion.minor,
                            r.firmwareVersion.patch, r.firmwareVersion.dev) != fwKey) {
                sameDuringMotion = false;
            }
            delay(20);
        }
        bool drained = queued && tfhDrain(m, 15000);
        int64_t moved = (drained && tfhFatal(m) == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   fifteen version reads during a move; it delivered %lld of %ld counts\n",
               (long long)moved, (long)dist);
        TEST_RESULT("the version read in the middle of motion is the same one read at rest",
                    sameDuringMotion);
        TEST_RESULT("reading the version repeatedly during a move does not disturb it",
                    drained && llabs((long long)(moved - dist)) <= dist / 200 + 4);
    }

    // =====================================================================
    // 6. THE COMPARATOR ITSELF IS CORRECT. Every prediction in section 7 is a
    //    ">= boundary" test on the packed key, so a comparator that ordered
    //    the parts wrongly would make every gate decision silently wrong while
    //    still "passing". Checked here as pure arithmetic, on the exact
    //    version pairs this fleet actually contains.
    // =====================================================================
    printf("\n--- 6. the version comparator orders the four parts correctly ---\n");
    {
        // The two firmwares in this fleet differ ONLY in the patch field, so a
        // comparator that stopped at major.minor would call them equal.
        TEST_RESULT("0.15.9.0 sorts before 0.15.12.0",
                    packVersion(0, 15, 9, 0) < packVersion(0, 15, 12, 0));
        TEST_RESULT("the development field is the least significant part",
                    packVersion(0, 15, 12, 0) < packVersion(0, 15, 12, 1));
        TEST_RESULT("a minor bump outranks any patch and development value",
                    packVersion(0, 15, 255, 255) < packVersion(0, 16, 0, 0));
        TEST_RESULT("a major bump outranks any minor value",
                    packVersion(0, 255, 255, 255) < packVersion(1, 0, 0, 0));
        TEST_RESULT("the gate is inclusive at its own boundary",
                    packVersion(0, 15, 12, 0) >= FW_PLANNER_FIXES &&
                    packVersion(0, 15, 11, 255) < FW_PLANNER_FIXES);
    }

    // =====================================================================
    // 7. THE PAYOFF: THE VERSION PREDICTS BEHAVIOUR. Each probe's expected
    //    outcome is computed from fwKey BEFORE the probe runs, then compared
    //    against what the machine actually does. Passing means the version
    //    field is a trustworthy thing to gate on; failing means the number the
    //    device reports does not describe the code it is running, which is a
    //    far more serious defect than any single one of these behaviours.
    // =====================================================================
    printf("\n--- 7. the reported version predicts what the machine will do ---\n");
    {
        const bool hasPlannerFixes  = (fwKey >= FW_PLANNER_FIXES);
        const bool hasZeroTimeGuard = (fwKey >= FW_ZERO_TIME_GUARD);
        printf("   gate verdict from the version alone: plannerFixes=%d zeroTimeGuard=%d\n",
               (int)hasPlannerFixes, (int)hasZeroTimeGuard);

        // ---- Probe A: one rotation commanded in a single control tick. ----
        // Left on the FACTORY limits (9.3333 rot/s, 12000 rot/s^2 -> a 24-tick
        // ramp), so nothing here depends on a setter. With total_time = 1 the
        // ramp halves to zero either way; >= 0.15.12.0 then floors it at one
        // tick, computes an enormous acceleration, saturates it instead of
        // truncating it, and add_to_queue refuses the move outright
        // (ERROR_ACCEL_TOO_HIGH). Older firmware divides by zero, gets
        // acceleration 0, and accepts a move that never happens.
        tfhReset(m);
        Probe a = probeMove(m, (int32_t)TFH_CPR, 1, 6000);
        tfhReset(m);
        uint8_t predictedA = hasPlannerFixes ? ERR_ACCEL_TOO_HIGH : 0;
        printf("   A: one rotation in one tick -> accepted=%d fatal=%u moved=%lld "
               "(predicted fatal %u)\n",
               (int)a.accepted, (unsigned)a.fatal, (long long)a.moved, (unsigned)predictedA);
        TEST_RESULT("a rotation commanded in a single tick behaves exactly as the version predicts",
                    a.fatal == predictedA);
        // True on BOTH sides, for different reasons: refused before queueing on
        // the new firmware, silently inert on the old one. Either way the shaft
        // must never be told it moved a rotation it did not move.
        TEST_RESULT("that command never delivered the rotation it was asked for, on either firmware",
                    a.moved == 0);

        // ---- Probe B (control): a zero-duration move. ----
        // Guarded since 0.15.9.0, so BOTH firmwares in this fleet must refuse
        // it identically. A gate that predicted a difference here would be
        // over-fitting the version to the split.
        tfhReset(m);
        Probe b = probeMove(m, (int32_t)(TFH_CPR / 4), 0, 4000);
        uint8_t bFatal = b.fatal;
        tfhReset(m);
        bool recovered = (tfhFatal(m) == 0);
        uint8_t predictedB = hasZeroTimeGuard ? ERR_PARAMETER_OUT_OF_RANGE : 0;
        printf("   B: zero-duration move -> accepted=%d fatal=%u (predicted fatal %u)\n",
               (int)b.accepted, (unsigned)bFatal, (unsigned)predictedB);
        TEST_RESULT("a zero-duration move is refused exactly as the version predicts",
                    bFatal == predictedB);
        TEST_RESULT("and the machine is recoverable afterwards", recovered);

        // ---- Probe C: a ramp shorter than one control tick. ----
        // A 0.3 rot/s ceiling against the UNTOUCHED 12000 rot/s^2 factory
        // acceleration makes delta_t1 truncate to zero -- this is the firmware's
        // own worked example of the fault (motor_control.c:449 computes it,
        // :462-465 names exactly this pair of limits). Only the velocity is set,
        // so the acceleration stays at its reset default and nothing here rests
        // on two setters agreeing. >= 0.15.12.0 floors the ramp at one tick and
        // takes the two-segment fast path (motor_control.c:2017), so the move
        // happens and costs 2 queue slots. Older firmware divides by zero:
        // accepted, no error, no motion, and no fast path to take.
        tfhReset(m);
        m->setMaximumVelocity(0.3f);
        int32_t cDist = (int32_t)(TFH_CPR / 8);          // 0.125 rot, far inside
        Probe c = probeMove(m, cDist, (uint32_t)(4 * TFH_TPS), 20000);
        tfhReset(m);
        int64_t predictedCMoved = hasPlannerFixes ? (int64_t)cDist : 0;
        printf("   C: sub-tick ramp -> accepted=%d fatal=%u slots=%u moved=%lld "
               "(predicted moved %lld, slots %s)\n",
               (int)c.accepted, (unsigned)c.fatal, (unsigned)c.queued,
               (long long)c.moved, (long long)predictedCMoved,
               hasPlannerFixes ? "2" : "fewer than 2");
        TEST_RESULT("a move whose ramp is shorter than one tick behaves as the version predicts",
                    c.accepted && c.fatal == 0 &&
                    llabs((long long)(c.moved - predictedCMoved)) <= cDist / 200 + 4);
        // The two-slot fast path exists only on the fixed firmware, and both of
        // its segments are long, so the depth read back over the bus is a solid
        // 2. Before the fix the same command emits three acceleration segments
        // of which the two ramps have ZERO time steps, and add_to_queue drops
        // zero-length items outright (motor_control.c:1782-1789, present since
        // 0.15.9.0), leaving a single coast segment.
        TEST_RESULT("the queue slot cost of that move matches the version's prediction",
                    hasPlannerFixes ? (c.queued == 2) : (c.queued < 2));

        // ---- Probe D (control): an entirely ordinary move. ----
        // The gate must be narrow. A quarter turn over a second uses a normal
        // multi-tick ramp on every firmware here and must behave identically.
        tfhFreshOpen(m);
        bool okD = false;
        int64_t dMoved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &okD);
        printf("   D (control): ordinary quarter turn moved %lld of %lld\n",
               (long long)dMoved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move delivers its full displacement regardless of version",
                    okD && llabs((long long)(dMoved - TFH_CPR / 4)) <= TFH_CPR / 800 + 4);

        // Finally: every prediction above was made against the version read at
        // the top of this module. If the device had rebooted into a different
        // image somewhere in between, all of it would be meaningless.
        tfhReset(m);
        getFirmwareVersionResponse fwEnd = m->getFirmwareVersion();
        bool okEnd = (m->getError() == 0);
        uint32_t endKey = packVersion(fwEnd.firmwareVersion.major, fwEnd.firmwareVersion.minor,
                                      fwEnd.firmwareVersion.patch, fwEnd.firmwareVersion.dev);
        TEST_RESULT("the version every prediction was made from is the version still running",
                    okEnd && endKey == fwKey && fwEnd.inBootloader == 0);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
