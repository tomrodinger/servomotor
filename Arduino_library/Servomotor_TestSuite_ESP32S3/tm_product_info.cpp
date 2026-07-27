#include "tf_framework.h"

// Product-info group test. Mirrors the Python per-command tests:
//   - test_get_product_specs.py        (cmd 18 Get product specs)
//   - test_get_product_info.py         (cmd 22 Get product info)
//   - test_get_product_description.py  (cmd 24 Get product description) -- BUG-25
//   - test_get_firmware_version.py     (cmd 25 Get firmware version)
//
// The motor is addressed by unique-ID (extended addressing) via the wrapper, so
// all commands are called WITHOUT a uniqueId argument.

// Plausible bounds for the control-loop update frequency (Hz). Firmware runs in
// the tens of kHz (M17 is 31250 Hz); bounds are wide enough that any sane value
// passes and any wildly wrong value fails.
static const uint32_t MIN_UPDATE_FREQUENCY_HZ = 1000;
static const uint32_t MAX_UPDATE_FREQUENCY_HZ = 200000;
// The shaft_rotations conversion factor the host library uses (3,276,800). The
// firmware must report exactly this or every position conversion is silently
// wrong. This is the value for M3/M17/M23.
static const uint32_t EXPECTED_COUNTS_PER_ROTATION = 3276800;

static bool isPrintableAscii(char c) {
    return (c >= 0x20) && (c < 0x7F);
}

void tm_product_info(void) {
    Servomotor* motor = tfGetMotor();

    // Start from a clean, known state.
    motor->systemReset();
    delay(1500);

    char buf[128];

    // ========================================================================
    // Command 18: Get product specs
    // ========================================================================
    printf("\n--- Get product specs (cmd 18) ---\n");
    getProductSpecsResponse specs = motor->getProductSpecs();
    int specsErr = motor->getError();
    printf("getProductSpecs error=%d updateFrequency=%u countsPerRotation=%u\n",
           specsErr, specs.updateFrequency, specs.countsPerRotation);

    TEST_RESULT("Product specs no error", specsErr == 0);

    TEST_RESULT("Update frequency plausible",
                specs.updateFrequency >= MIN_UPDATE_FREQUENCY_HZ &&
                specs.updateFrequency <= MAX_UPDATE_FREQUENCY_HZ);

    TEST_RESULT("Counts per rotation matches shaft_rotations factor",
                specs.countsPerRotation == EXPECTED_COUNTS_PER_ROTATION);

    // Values are static: a second read must be identical.
    getProductSpecsResponse specs2 = motor->getProductSpecs();
    TEST_RESULT("Product specs static across reads",
                specs2.updateFrequency == specs.updateFrequency &&
                specs2.countsPerRotation == specs.countsPerRotation);

    // ========================================================================
    // Command 22: Get product info
    // ========================================================================
    printf("\n--- Get product info (cmd 22) ---\n");
    getProductInfoResponse info = motor->getProductInfo();
    int infoErr = motor->getError();
    printf("getProductInfo error=%d\n", infoErr);
    TEST_RESULT("Product info no error", infoErr == 0);

    // productCode: 8 printable ASCII chars (string8, not NUL-terminated).
    bool codePrintable = true;
    for (int i = 0; i < 8; i++) {
        if (!isPrintableAscii(info.productCode[i])) {
            codePrintable = false;
            break;
        }
    }
    // Print the code safely (it may not be NUL-terminated).
    char codeStr[9];
    memcpy(codeStr, info.productCode, 8);
    codeStr[8] = '\0';
    printf("  productCode = \"%s\"  firmwareCompatibility=%u\n",
           codeStr, info.firmwareCompatibility);
    TEST_RESULT("Product code is 8 printable ASCII chars", codePrintable);

    // firmwareCompatibility: u8, always in range by type; assert anyway.
    TEST_RESULT("Firmware compatibility in range",
                info.firmwareCompatibility <= 255);

    // hardwareVersion: each of patch/minor/major in [0,99].
    printf("  hardwareVersion = %u.%u.%u (major.minor.patch)\n",
           info.hardwareVersion.major, info.hardwareVersion.minor,
           info.hardwareVersion.patch);
    bool hwOk = info.hardwareVersion.patch <= 99 &&
                info.hardwareVersion.minor <= 99 &&
                info.hardwareVersion.major <= 99;
    TEST_RESULT("Hardware version fields in [0,99]", hwOk);

    // serialNumber: u32, nonzero.
    printf("  serialNumber = %u\n", info.serialNumber);
    TEST_RESULT("Serial number is nonzero", info.serialNumber != 0);

    // uniqueId: must match the unique ID we are addressing by.
    uint64_t addressedUid = motor->usingThisUniqueId();
    snprintf(buf, sizeof(buf), "  uniqueId (info) = 0x%016llX  addressed = 0x%016llX\n",
             (unsigned long long)info.uniqueId, (unsigned long long)addressedUid);
    printf("%s", buf);
    TEST_RESULT("Unique ID matches addressed unique ID",
                info.uniqueId == addressedUid);

    // reserved: must be zero.
    printf("  reserved = %u\n", info.reserved);
    TEST_RESULT("Reserved field is zero", info.reserved == 0);

    // Static: a second read must be byte-identical.
    getProductInfoResponse info2 = motor->getProductInfo();
    TEST_RESULT("Product info static across reads",
                memcmp(&info2, &info, sizeof(getProductInfoResponse)) == 0);

    // ========================================================================
    // Command 24: Get product description (variable-length string, caller buffer)
    // The firmware replies with a NUL-terminated "Servomotor" string. The new
    // caller-owned-buffer API copies it into descStr, NUL-terminates it, and
    // reports the string length in descLen (excluding the terminator).
    // ========================================================================
    printf("\n--- Get product description (cmd 24) ---\n");
    char descStr[64];
    uint16_t descLen = 0;
    motor->getProductDescription(descStr, sizeof(descStr), &descLen);
    int descErr = motor->getError();
    printf("  error=%d description=\"%s\" (len = %u)\n",
           descErr, descStr, (unsigned)descLen);

    TEST_RESULT("Product description no error", descErr == 0);
    TEST_RESULT("Product description non-empty", descLen > 0);

    bool descPrintable = descLen > 0;
    for (uint16_t i = 0; i < descLen; i++) {
        if (!isPrintableAscii(descStr[i])) {
            descPrintable = false;
            break;
        }
    }
    TEST_RESULT("Product description printable ASCII", descPrintable);
    TEST_RESULT("Product description is 'Servomotor'",
                strcmp(descStr, "Servomotor") == 0);
    TEST_RESULT("Product description length matches strlen",
                descLen == (uint16_t)strlen(descStr));

    // ========================================================================
    // Command 25: Get firmware version
    // ========================================================================
    printf("\n--- Get firmware version (cmd 25) ---\n");
    getFirmwareVersionResponse fw = motor->getFirmwareVersion();
    int fwErr = motor->getError();
    printf("  error=%d version=%u.%u.%u.%u (major.minor.patch.dev) inBootloader=%u\n",
           fwErr, fw.firmwareVersion.major, fw.firmwareVersion.minor,
           fw.firmwareVersion.patch, fw.firmwareVersion.dev, fw.inBootloader);

    TEST_RESULT("Firmware version no error", fwErr == 0);

    // Version components are u8 by type; the meaningful check is the bootloader
    // flag and that we are in main firmware (not bootloader) after a clean boot.
    TEST_RESULT("inBootloader flag is 0 or 1",
                fw.inBootloader == 0 || fw.inBootloader == 1);
    TEST_RESULT("Running main firmware (inBootloader == false)",
                fw.inBootloader == 0);

    // A real firmware has at least one nonzero version component.
    bool versionNonzero = (fw.firmwareVersion.major | fw.firmwareVersion.minor |
                           fw.firmwareVersion.patch | fw.firmwareVersion.dev) != 0;
    TEST_RESULT("Firmware version is not all zero", versionNonzero);

    // Static: a second read must be identical.
    getFirmwareVersionResponse fw2 = motor->getFirmwareVersion();
    TEST_RESULT("Firmware version static across reads",
                memcmp(&fw2, &fw, sizeof(getFirmwareVersionResponse)) == 0);

    // Leave the device in a clean, reset state.
    motor->systemReset();
    delay(1500);
}
