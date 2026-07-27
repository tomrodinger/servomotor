#include "tf_framework.h"

// ============================================================================
// tm_varlen_bugs
//
// Mirrors the two Python "variable-length response" per-command tests:
//   - test_capture_hall_sensor_data.py  (cmd  7)
//   - test_read_multipurpose_buffer.py  (cmd 35)
//
// Both firmware commands reply with a VARIABLE-LENGTH payload. The Arduino
// library now delivers such payloads into a CALLER-OWNED buffer
//   void captureHallSensorData(<inputs>, uint8_t* buffer, uint16_t bufferSize, uint16_t* actualSize);
//   void readMultipurposeBuffer(uint8_t* buffer, uint16_t bufferSize, uint16_t* actualSize);
// and reports the byte count in *actualSize (this fixed BUG-26 / BUG-27, which
// used to force these replies through a fixed 1-byte struct).
//
// What this test does:
//   * cmd 7: exercise the invalid-parameter cases the Python test uses — each
//     must raise fatal error 49 (ERROR_CAPTURE_BAD_PARAMETERS) and recover on
//     reset; also verify the new caller-buffer error contract (actualSize == 0
//     on the error path). We NEVER issue a *valid* capture: a real capture
//     streams and, per the Python test's docstring, can pin the device in
//     "still capturing" until a power cycle. The variable-length RETRIEVAL path
//     is exercised safely by readMultipurposeBuffer below (identical mechanism).
//   * cmd 35: read the multipurpose buffer into a caller buffer — empty returns
//     a single data_type=0 byte; after a closed-loop PID-debug capture it
//     returns data_type=4 plus the debug bytes (this is what the old 1-byte
//     struct could not deliver, and now does).
//
// SAFETY: never test_mode(0) or 10..13 (firmware while(1) hangs); cleanup is
// always via systemReset().
// ============================================================================

static const uint8_t  ERROR_CAPTURE_BAD_PARAMETERS = 49;   // firmware error_text.h
static const uint16_t CLOSED_LOOP_BIT              = 1 << 2; // status flag bit 2
static const uint8_t  TEST_MODE_PID_DEBUG          = 3;      // READ_PID_DEBUG_DATA_TEST_MODE (SAFE)
static const uint8_t  MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA = 4;

struct CaptureCase {
    const char* label;
    uint8_t  captureType;
    uint32_t nPointsToRead;
    uint8_t  channelsBitmask;
    uint16_t timeStepsPerSample;
    uint16_t nSamplesToSum;
    uint16_t divisionFactor;
};

void tm_varlen_bugs(void) {
    Serial.println("tm_varlen_bugs: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ------------------------------------------------------------------
    // cmd 7 : Capture hall sensor data — invalid-parameter cases.
    // Mirror test_capture_hall_sensor_data.py: a "would-be-valid" baseline with
    // exactly ONE bad field per case. Each invalid call must fatal_error(49);
    // getStatus() then latches 49, systemReset() clears it. The new API also
    // takes a caller buffer; on the error path *actualSize must be 0.
    // (No valid capture is ever issued — see header.)
    // ------------------------------------------------------------------
    CaptureCase cases[] = {
        {"capture_type=0 (out of range 1..3)",         0, 10, 1, 1000, 1, 1},
        {"capture_type=4 (out of range 1..3)",         4, 10, 1, 1000, 1, 1},
        {"division_factor=0 (must be non-zero)",       1, 10, 1, 1000, 1, 0},
        {"channels_bitmask=0 (no channels selected)",  1, 10, 0, 1000, 1, 1},
        {"channels_bitmask=0b1000 (bit outside 0..2)", 1, 10, 0b1000, 1000, 1, 1},
    };
    const int nCases = sizeof(cases) / sizeof(cases[0]);

    uint8_t  capBuf[64];      // caller-owned buffer (unused on the error path)
    uint16_t capLen = 0xFFFF; // must be set to 0 by the error path

    for (int i = 0; i < nCases; i++) {
        CaptureCase& c = cases[i];
        char buf[200];

        // Clean state before this case.
        motor->systemReset();
        delay(1500);
        getStatusResponse pre = motor->getStatus();
        snprintf(buf, sizeof(buf), "[%s] pre fatalErrorCode=%u", c.label, pre.fatalErrorCode);
        Serial.println(buf);
        TEST_RESULT((std::string("Clean state before: ") + c.label).c_str(),
                    pre.fatalErrorCode == 0);

        // Invalid capture must fatal-error the device (NOT return success).
        capLen = 0xFFFF;
        motor->captureHallSensorData(c.captureType, c.nPointsToRead, c.channelsBitmask,
                                     c.timeStepsPerSample, c.nSamplesToSum, c.divisionFactor,
                                     capBuf, sizeof(capBuf), &capLen);
        delay(100);
        getStatusResponse st = motor->getStatus();
        snprintf(buf, sizeof(buf), "[%s] post-call fatalErrorCode=%u (expect %u), capLen=%u",
                 c.label, st.fatalErrorCode, ERROR_CAPTURE_BAD_PARAMETERS, capLen);
        Serial.println(buf);
        TEST_RESULT((std::string("Capture invalid raises fatal 49: ") + c.label).c_str(),
                    st.fatalErrorCode == ERROR_CAPTURE_BAD_PARAMETERS);
        TEST_RESULT((std::string("Capture error path sets actualSize=0: ") + c.label).c_str(),
                    capLen == 0);

        // Reset must clear the latched fatal error.
        motor->systemReset();
        delay(1500);
        getStatusResponse post = motor->getStatus();
        TEST_RESULT((std::string("Reset clears fatal after: ") + c.label).c_str(),
                    post.fatalErrorCode == 0);
    }

    // ------------------------------------------------------------------
    // cmd 35 : Read multipurpose buffer (variable-length blob, caller buffer)
    // ------------------------------------------------------------------
    motor->systemReset();
    delay(1500);

    uint8_t  mpBuf[128];
    uint16_t mpLen = 0;

    // Empty buffer: firmware >= 0.15.4.0 replies with a single data_type=0 byte.
    motor->readMultipurposeBuffer(mpBuf, sizeof(mpBuf), &mpLen);
    int emptyErr = motor->getError();
    printf("Empty buffer read: err=%d len=%u data_type=%u\n",
           emptyErr, (unsigned)mpLen, mpLen > 0 ? mpBuf[0] : 255);
    TEST_RESULT("Empty multipurpose buffer read: no error", emptyErr == 0);
    TEST_RESULT("Empty multipurpose buffer returns data_type 0",
                mpLen >= 1 && mpBuf[0] == 0);
    getStatusResponse afterEmpty = motor->getStatus();
    TEST_RESULT("No fatal error after empty buffer read", afterEmpty.fatalErrorCode == 0);

    // Populate the buffer via closed-loop PID debug capture (test_mode 3), the
    // exact populator the Python test uses. Needs a calibrated motor.
    motor->enableMosfets();
    delay(300);          // let the enable-alignment transient settle
    motor->goToClosedLoop();

    bool inClosedLoop = false;
    for (int i = 0; i < 60; i++) {          // up to ~6 s
        getStatusResponse s = motor->getStatus();
        if (s.statusFlags & CLOSED_LOOP_BIT) { inClosedLoop = true; break; }
        delay(100);
    }
    printf("closed-loop engaged: %s\n", inClosedLoop ? "yes" : "no");
    TEST_RESULT("Closed loop engaged (motor calibrated)", inClosedLoop);

    // Assert test_mode 3 so a PID iteration writes its debug struct.
    // (SAFE value: never 0, never 10..13.)
    motor->testMode(TEST_MODE_PID_DEBUG);
    delay(50);           // one PID iteration is ~32 us; plenty

    // The populated reply is [data_type=4][PID debug bytes] — more than one byte.
    // The caller-owned buffer now retrieves the whole thing (BUG-27 fix).
    mpLen = 0;
    motor->readMultipurposeBuffer(mpBuf, sizeof(mpBuf), &mpLen);
    int popErr = motor->getError();
    printf("Populated read: err=%d len=%u data_type=%u (want data_type=%u, len>1)\n",
           popErr, (unsigned)mpLen, mpLen > 0 ? mpBuf[0] : 255,
           MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA);
    TEST_RESULT("Populated multipurpose buffer read: no error", popErr == 0);
    TEST_RESULT("Populated buffer returns PID-debug data_type 4",
                mpLen >= 1 && mpBuf[0] == MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA);
    TEST_RESULT("Populated buffer is variable-length (more than one byte)",
                mpLen > 1);

    // Cleanup: systemReset (NOT test_mode(0) — that hangs the firmware). This
    // clears test_mode, exits closed loop, disables the MOSFETs and clears the
    // buffer. Motor ends at rest.
    motor->systemReset();
    delay(1500);

    getStatusResponse afterCleanup = motor->getStatus();
    TEST_RESULT("No fatal error after cleanup", afterCleanup.fatalErrorCode == 0);

    mpLen = 0;
    motor->readMultipurposeBuffer(mpBuf, sizeof(mpBuf), &mpLen);
    int emptyAgainErr = motor->getError();
    printf("Empty buffer read after cleanup: err=%d len=%u data_type=%u\n",
           emptyAgainErr, (unsigned)mpLen, mpLen > 0 ? mpBuf[0] : 255);
    TEST_RESULT("Empty multipurpose buffer again after cleanup",
                emptyAgainErr == 0 && mpLen >= 1 && mpBuf[0] == 0);
}
