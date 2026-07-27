#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_version_info_consistency.cpp
//
// Version / identity cross-consistency probe. No motion, no test-mode, no raw
// bus injection. Everything is read-only and unique-ID addressed (tfGetMotor),
// so it is collision-safe on the single-motor bench AND the 39-motor rack.
//
// What it checks, and the GROUND TRUTH each assertion encodes:
//
//   Get firmware version (cmd 55, GET_FIRMWARE_VERSION_COMMAND):
//     Firmware reply (firmware/Src/main.c ~L593) copies the global
//     `firmware_version` struct then sets in_bootloader = 0 (running main
//     firmware, never the bootloader). The struct byte order is
//     {dev, patch(bugfix), minor, major} == VersionNumber32
//     (Arduino_library/DataTypes.h). Current bench firmware is 0.15.9.0
//     (main.c: MAJOR=0 MINOR=15 BUGFIX=9 DEVELOPMENT=0).
//       * not all-zero (a real version is present)
//       * inBootloader == 0 (main firmware answered, not the bootloader)
//       * stable across 5 consecutive reads (identical every time)
//       * major.minor >= 0.15 on this bench (assert >=, not ==, so a future
//         bump doesn't break the test)
//
//   Get product info (cmd 41, GET_PRODUCT_INFO_COMMAND):
//     Firmware reply (main.c ~L566) memcpy's product_info_struct from flash
//     (common_source_files/product_info.h): model_code[8], u8
//     firmware_compatibility_code, u8 hw bugfix/minor/major, u32 serial_number,
//     u64 unique_id, u32 not_used. Library maps this to getProductInfoResponse
//     { char productCode[8]; u8 firmwareCompatibility; VersionNumber24
//     hardwareVersion{patch,minor,major}; u32 serialNumber; u64 uniqueId;
//     u32 reserved }.
//     Assertions mirror python_programs/test_get_product_info.py
//     validate_product_info():
//       * firmwareCompatibility identical across 3 reads
//       * serialNumber stable across reads and 0 < serial < 2^32
//       * uniqueId stable across reads AND == the addressed unique ID
//         (product info comes from the SAME device we extended-addressed)
//       * productCode is 8 printable ASCII chars (0x20..0x7E, no ws)
//       * hardware version patch/minor/major each in 0..99 (plausible; we do
//         NOT hard-code M17 1.5 so the module is product-agnostic)
//       * reserved (not_used) == 0
//
// SAFETY: read-only identity/version queries only. No queue, no motion, no
// fatal-error injection, so no reset-after-fatal dance is needed between steps.
// The module still opens and closes on a clean systemReset + drain.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;  // wait after a plain reset
static const int N_FW_READS            = 5;     // firmware-version stability reads
static const int N_INFO_READS          = 3;     // product-info stability reads

// Drain any bytes sitting in the RS485 receive buffer (bounded so a stuck line
// can never hang the whole test slot). Reads only -- never writes the bus.
static void drainSerial() {
    uint32_t t0 = millis();
    while (Serial1.available()) {
        Serial1.read();
        if (millis() - t0 > 2000) break;
    }
}

// System reset + settle + drain. Leaves the device clean and responsive.
static void cleanReset(Servomotor* motor) {
    motor->systemReset();            // unique-ID addressed; no response expected
    delay(NORMAL_RESET_DELAY_MS);    // nothing answers during reset; don't check errors
    drainSerial();
}

// True iff c is a printable, non-whitespace ASCII character (space excluded to
// match the python test's exclusion of \r\n\t\v\f; space 0x20 is printable but
// a padded model_code should not contain interior spaces -- we allow 0x20 only
// via the range test below, matching string.printable which includes space).
static bool isPrintableAscii(char c) {
    unsigned char u = (unsigned char)c;
    return u >= 0x20 && u <= 0x7E;  // printable range incl. space, excl. control/DEL
}

void tm_version_info_consistency(void) {
    Serial.println("tm_version_info_consistency: BEGIN\n");

    Servomotor* motor = tfGetMotor();
    const uint64_t addressedUid = motor->usingThisUniqueId();

    // ---- Clean, known starting state. ----
    cleanReset(motor);

    // =======================================================================
    // Get firmware version: read N_FW_READS times, check sanity + stability.
    // =======================================================================
    getFirmwareVersionResponse fw[N_FW_READS];
    bool fwCommsOk = true;
    for (int i = 0; i < N_FW_READS; i++) {
        fw[i] = motor->getFirmwareVersion();
        if (motor->getError() != 0) {
            printf("  getFirmwareVersion read %d comms error %d\n", i, motor->getError());
            fwCommsOk = false;
        }
    }
    TEST_RESULT("Get firmware version: all 5 reads succeeded at comms layer", fwCommsOk);

    const VersionNumber32 v0 = fw[0].firmwareVersion;
    printf("Firmware version: %u.%u.%u.%u  inBootloader=%u\n",
           (unsigned)v0.major, (unsigned)v0.minor, (unsigned)v0.patch,
           (unsigned)v0.dev, (unsigned)fw[0].inBootloader);

    // Not all-zero: a real firmware version must be present.
    bool notAllZero = (v0.major != 0) || (v0.minor != 0) ||
                      (v0.patch != 0) || (v0.dev != 0);
    TEST_RESULT("Firmware version is not all-zero", fwCommsOk && notAllZero);

    // Running the main firmware, not the bootloader (firmware hard-sets 0).
    // GROUND TRUTH: main.c GET_FIRMWARE_VERSION sets in_bootloader = 0.
    TEST_RESULT("Firmware reports inBootloader == 0 (main firmware)",
                fwCommsOk && fw[0].inBootloader == 0);

    // Stable across all 5 reads: identical major.minor.patch.dev and
    // inBootloader every time.
    bool fwStable = true;
    for (int i = 1; i < N_FW_READS; i++) {
        if (fw[i].firmwareVersion.major != v0.major ||
            fw[i].firmwareVersion.minor != v0.minor ||
            fw[i].firmwareVersion.patch != v0.patch ||
            fw[i].firmwareVersion.dev   != v0.dev   ||
            fw[i].inBootloader          != fw[0].inBootloader) {
            fwStable = false;
            printf("  firmware version read %d differs from read 0\n", i);
        }
    }
    TEST_RESULT("Firmware version identical across 5 reads", fwCommsOk && fwStable);

    // major.minor >= 0.15 on this bench (>=, not ==; a future bump is fine).
    // GROUND TRUTH: main.c MAJOR=0 MINOR=15 -> current 0.15.x.y.
    bool fwAtLeast_0_15 = (v0.major > 0) || (v0.major == 0 && v0.minor >= 15);
    TEST_RESULT("Firmware major.minor >= 0.15", fwCommsOk && fwAtLeast_0_15);

    // =======================================================================
    // Get product info: read N_INFO_READS times, check identity + stability.
    // =======================================================================
    getProductInfoResponse pi[N_INFO_READS];
    bool piCommsOk = true;
    for (int i = 0; i < N_INFO_READS; i++) {
        pi[i] = motor->getProductInfo();
        if (motor->getError() != 0) {
            printf("  getProductInfo read %d comms error %d\n", i, motor->getError());
            piCommsOk = false;
        }
    }
    TEST_RESULT("Get product info: all 3 reads succeeded at comms layer", piCommsOk);

    const getProductInfoResponse& p0 = pi[0];
    printf("Product info: fwCompat=%u  hwVer=%u.%u.%u  serial=%lu  uid=0x%016llX  reserved=%lu\n",
           (unsigned)p0.firmwareCompatibility,
           (unsigned)p0.hardwareVersion.major, (unsigned)p0.hardwareVersion.minor,
           (unsigned)p0.hardwareVersion.patch,
           (unsigned long)p0.serialNumber,
           (unsigned long long)p0.uniqueId,
           (unsigned long)p0.reserved);

    // firmwareCompatibility identical across the 3 reads.
    bool compatStable = true;
    for (int i = 1; i < N_INFO_READS; i++) {
        if (pi[i].firmwareCompatibility != p0.firmwareCompatibility) compatStable = false;
    }
    TEST_RESULT("firmwareCompatibility identical across 3 reads",
                piCommsOk && compatStable);

    // serialNumber stable across reads AND in (0, 2^32). u32 so upper bound is
    // structural; the meaningful check is nonzero. (test_get_product_info.py L112)
    bool serialStable = true;
    for (int i = 1; i < N_INFO_READS; i++) {
        if (pi[i].serialNumber != p0.serialNumber) serialStable = false;
    }
    TEST_RESULT("serialNumber stable across 3 reads", piCommsOk && serialStable);
    TEST_RESULT("serialNumber is nonzero", piCommsOk && p0.serialNumber != 0);

    // uniqueId stable across reads AND == the unique ID we addressed the device
    // by. This proves the product info came from the very device we targeted.
    // (test_get_product_info.py L116: unique_id == expected_unique_id)
    bool uidStable = true;
    for (int i = 1; i < N_INFO_READS; i++) {
        if (pi[i].uniqueId != p0.uniqueId) uidStable = false;
    }
    TEST_RESULT("uniqueId stable across 3 reads", piCommsOk && uidStable);
    TEST_RESULT("uniqueId matches the addressed unique ID",
                piCommsOk && p0.uniqueId == addressedUid);

    // productCode: 8 printable ASCII chars. (test_get_product_info.py L91-94)
    bool codePrintable = true;
    for (int i = 0; i < 8; i++) {
        if (!isPrintableAscii(p0.productCode[i])) codePrintable = false;
    }
    {
        char safeCode[9];
        for (int i = 0; i < 8; i++)
            safeCode[i] = isPrintableAscii(p0.productCode[i]) ? p0.productCode[i] : '?';
        safeCode[8] = '\0';
        printf("productCode = \"%s\"\n", safeCode);
    }
    TEST_RESULT("productCode is 8 printable ASCII characters",
                piCommsOk && codePrintable);

    // Hardware version patch/minor/major each plausible (0..99). Product-
    // agnostic: we do NOT hard-code M17 1.5. (test_get_product_info.py L100-108)
    bool hwPlausible =
        p0.hardwareVersion.patch <= 99 &&
        p0.hardwareVersion.minor <= 99 &&
        p0.hardwareVersion.major <= 99;
    TEST_RESULT("Hardware version fields all in 0..99", piCommsOk && hwPlausible);

    // reserved (not_used) must be 0. (test_get_product_info.py L118-120)
    TEST_RESULT("reserved field is zero", piCommsOk && p0.reserved == 0);

    // ---- Leave the device clean. ----
    cleanReset(motor);

    Serial.println("tm_version_info_consistency: END\n");
}
