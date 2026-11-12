#include "tf_motion_helpers.h"

// Firmware 0.17.0.0 acceptance. One M17, selected by UID by the harness.
// Only +/-1 degree reference changes; 100/100 internal current limits. PID
// extrema are tested with outputs disabled, then reset before enabling motion.
// Never run this module against the separate adapter-testing hub.
static bool immediateReady(Servomotor* m) {
    tfhReset(m);
    m->setMaximumMotorCurrentRaw(100, 100);
    if (m->getError()) return false;
    m->goToClosedLoop();
    if (m->getError()) return false;
    delay(250);
    m->zeroPosition();
    return m->getError() == 0 && tfhFatal(m) == 0;
}

static void immediateFault(Servomotor* m, int expected) {
    const int actual = m->getError();
    TEST_RESULT("immediate rejection returns expected fatal code", actual == expected);
    const auto status = m->getStatus();
    TEST_RESULT("fatal rejection disables output stage", m->getError() == 0 &&
                status.fatalErrorCode == expected && (status.statusFlags & 2) == 0);
}

void tm_immediate_position(void) {
    Servomotor* m = tfGetMotor();
    const uint64_t uid = m->usingThisUniqueId();
    const auto info = m->getProductInfo();
    const bool correctMotor = m->getError() == 0 && m->isUsingExtendedAddressing() &&
        uid > 255 && info.uniqueId == uid && std::memcmp(info.productCode, "M17     ", 8) == 0;
    TEST_RESULT("immediate test requires selected M17 unique ID", correctMotor);
    if (!correctMotor) return;
    const auto fw = m->getFirmwareVersion();
    const auto v = fw.firmwareVersion;
    const bool compatible = m->getError() == 0 && !fw.inBootloader && v.major == 0 &&
        v.minor == 17 && v.patch == 0 && (v.dev == 0 || v.dev >= 3);
    TEST_RESULT("compact immediate-position firmware supported", compatible);
    if (!compatible) return;

    tfhReset(m);
    const auto defaults = m->getPidConstants();
    TEST_RESULT("PID defaults readable", m->getError() == 0);
    const uint32_t gains[][3] = {{0,0,0}, {1,256,65536},
        {0x7fffffffU,0x80000000U,0xffffffffU}, {0xffffffffU,0xffffffffU,0xffffffffU}};
    for (const auto& g : gains) {
        m->setPidConstants(g[0], g[1], g[2]);
        TEST_RESULT("PID extrema setter accepted with outputs off", m->getError() == 0);
        const auto p = m->getPidConstants(uid);
        TEST_RESULT("PID extrema full unsigned readback", m->getError() == 0 &&
                    p.kP == g[0] && p.kI == g[1] && p.kD == g[2]);
        const auto s = m->getStatus();
        TEST_RESULT("PID readback preserves disabled state", m->getError() == 0 && s.statusFlags == 0 && s.fatalErrorCode == 0);
    }
    tfhReset(m); // Never energize with the extrema gains.
    const auto restored = m->getPidConstants();
    TEST_RESULT("reset restores PID defaults", m->getError() == 0 &&
                restored.kP == defaults.kP && restored.kI == defaults.kI && restored.kD == defaults.kD);

    const int64_t step = TFH_CPR / 360;
    bool ready = immediateReady(m);
    TEST_RESULT("closed-loop preparation succeeds", ready);
    if (ready) {
        for (bool crcOn : {true, false}) {
            m->crc32Control(crcOn ? 1 : 0);
            TEST_RESULT("device CRC transition accepted", m->getError() == 0);
            if (m->getError()) { ready = false; break; }
            if (crcOn) m->enableCRC32(); else m->disableCRC32();
            for (int i = 0; i < 256; ++i) {
                const int64_t target = (i % 3 - 1) * step;
                int64_t measured;
                switch (i % 4) {
                    case 0:
                        m->setImmediatePositionTargetRaw(target);
                        if (m->getError()) { ready = false; measured = 0; break; }
                        measured = m->getHallSensorPositionRaw(); break;
                    case 1: measured = m->setImmediatePositionTargetAndGetPositionRaw(target); break;
                    case 2:
                        m->setImmediatePositionTargetRaw(uid, target);
                        if (m->getError()) { ready = false; measured = 0; break; }
                        measured = m->getHallSensorPositionRaw(); break;
                    default: measured = m->setImmediatePositionTargetAndGetPositionRaw(uid, target); break;
                }
                if (!ready || m->getError()) { TEST_RESULT("immediate stress transport succeeds", false); ready = false; break; }
                const int64_t commanded = m->getPositionRaw();
                TEST_RESULT("reference applied exactly without queue entries", m->getError() == 0 &&
                            commanded == target && m->getNQueuedItems() == 0 && m->getError() == 0);
                TEST_RESULT("cached measured position stays within small motion envelope", measured >= -3*step && measured <= 3*step);
            }
            if (!ready) break;
        }
        if (ready) {
            m->crc32Control(1);
            TEST_RESULT("CRC enabled before followup", m->getError() == 0);
            if (m->getError()) ready = false; else m->enableCRC32();
        }
        if (ready) {
            m->setImmediatePositionTargetRaw(step);
            const int holdCommandError = m->getError();
            delay(500);
            auto p = m->getComprehensivePositionRaw();
            const int holdReadError = m->getError();
            printf("HOLD target=%lld commanded=%lld hall=%lld write_error=%d read_error=%d\n",
                   (long long)step, (long long)p.commandedPosition, (long long)p.hallSensorPosition,
                   holdCommandError, holdReadError);
            const auto drive = m->getDebugValues();
            printf("HOLD_DRIVE current_limit=100 pwm=%u measured_velocity=%ld error=%d\n",
                   drive.motorPwmVoltage, (long)drive.measuredVelocity, m->getError());
            for (int sample = 0; sample < 5; ++sample) {
                const int64_t hall = m->getHallSensorPositionRaw();
                printf("HOLD_SAMPLE t_ms=%d hall=%lld error=%d\n", 500 + sample*100, (long long)hall, m->getError());
                delay(100);
            }
            TEST_RESULT("target holds after updates stop", holdCommandError == 0 && holdReadError == 0 && p.commandedPosition == step &&
                        p.hallSensorPosition > step-step/2 && p.hallSensorPosition < step+step/2);
            m->setPositionUnit(PositionUnit::DEGREES);
            const float measured = m->setImmediatePositionTargetAndGetPosition(uid, -1.0f);
            TEST_RESULT("converted combined UID method returns measured degrees", m->getError() == 0 && std::fabs(measured) < 3.0f);
            m->setImmediatePositionTarget(1.0f);
            TEST_RESULT("converted standalone method accepted", m->getError() == 0);
            m->setImmediatePositionTarget(uid, -1.0f);
            TEST_RESULT("converted standalone UID method accepted", m->getError() == 0);
            const float configuredMeasured = m->setImmediatePositionTargetAndGetPosition(1.0f);
            TEST_RESULT("converted configured-address combined method", m->getError() == 0 && std::fabs(configuredMeasured) < 3.0f);
            m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
            m->goToPositionRaw(0, TFH_TPS / 2);
            TEST_RESULT("legacy trajectory follows immediate baseline", m->getError() == 0 && tfhDrain(m, 2500) && m->getPositionRaw() == 0 && m->getError() == 0);
            const auto after = m->getPidConstants();
            TEST_RESULT("retargeting preserves PID gains", m->getError() == 0 && after.kP == defaults.kP && after.kI == defaults.kI && after.kD == defaults.kD);
        }
    }

    // A failed stress/CRC exchange must not lead to further motion probes.
    // Reset with the current link setting, then verify cleanup and stop.
    if (!ready) {
        tfhReset(m);
        const auto stopped = m->getStatus();
        TEST_RESULT("failed stress cleanup disabled outputs", m->getError() == 0 &&
                    stopped.statusFlags == 0 && stopped.fatalErrorCode == 0);
        return;
    }

    // Check the converted-input guard on the actual MCU and Arduino math
    // implementation, with every public overload. Rejections must not send a
    // target, change the reference, or put the motor into a fatal state.
    const float invalidTargets[] = {NAN, INFINITY, -INFINITY, 0x1p63f, -0x1.000002p63f};
    for (float invalid : invalidTargets) {
        for (bool combined : {false, true}) for (bool explicitUid : {false, true}) {
            const int64_t before = m->getPositionRaw();
            if (combined) {
                if (explicitUid) m->setImmediatePositionTargetAndGetPosition(uid, invalid);
                else m->setImmediatePositionTargetAndGetPosition(invalid);
            } else {
                if (explicitUid) m->setImmediatePositionTarget(uid, invalid);
                else m->setImmediatePositionTarget(invalid);
            }
            TEST_RESULT("invalid converted target rejected locally", m->getError() == COMMUNICATION_ERROR_PARAMETER_OUT_OF_RANGE);
            const auto status = m->getStatus();
            TEST_RESULT("invalid input preserves healthy closed-loop state", m->getError() == 0 && status.statusFlags == 6 && status.fatalErrorCode == 0);
            TEST_RESULT("invalid input preserves the entire reference", m->getPositionRaw() == before && m->getError() == 0);
        }
    }

    // Both methods must enforce each guard. Every fatal case starts afresh.
    for (bool combined : {false, true}) {
        tfhReset(m);
        if (combined) m->setImmediatePositionTargetAndGetPositionRaw(0); else m->setImmediatePositionTargetRaw(0);
        immediateFault(m, 13);
        for (int error : {55, 8, 27, 45, 19}) {
            if (!immediateReady(m)) { TEST_RESULT("guard preparation", false); break; }
            int64_t target = 0;
            if (error == 55) m->disableMosfets();
            if (error == 8) m->moveWithVelocityRaw(0, TFH_TPS);
            if (error == 27) { m->setSafetyLimitsRaw(-step, step); target = 3*step; }
            if (error == 45) { m->setMaxAllowablePositionDeviationRaw(step); target = 3*step; }
            if (error == 19) m->testMode(3);
            TEST_RESULT("guard setup accepted", m->getError() == 0);
            if (combined) m->setImmediatePositionTargetAndGetPositionRaw(uid, target); else m->setImmediatePositionTargetRaw(uid, target);
            immediateFault(m, error);
        }
    }
    // The generated methods always send eight bytes. Exercise firmware's
    // size guard through the real Arduino transport as well, in both framings.
    for (uint8_t command : {uint8_t(49), uint8_t(50)}) for (bool crcOn : {true, false}) {
        if (!immediateReady(m)) { TEST_RESULT("payload guard preparation", false); break; }
        if (!crcOn) {
            m->crc32Control(0);
            TEST_RESULT("payload guard CRC transition", m->getError() == 0);
            if (m->getError()) break;
            m->disableCRC32();
        }
        Communication wire(Serial1, 230400);
        if (!crcOn) wire.disableCRC32();
        const uint8_t malformed[7] = {};
        wire.sendCommandByUniqueId(uid, command, malformed, sizeof(malformed));
        uint8_t response[8];
        uint16_t received = 0;
        const int16_t error = wire.getResponse(response, sizeof(response), received);
        TEST_RESULT("malformed target payload rejected with fatal 51", error == 51);
        const auto status = m->getStatus();
        TEST_RESULT("malformed target disables motor", m->getError() == 0 && status.fatalErrorCode == 51 && (status.statusFlags & 2) == 0);
    }
    tfhReset(m);
    const auto end = m->getStatus();
    TEST_RESULT("cleanup leaves motor disabled and healthy", m->getError() == 0 && end.statusFlags == 0 && end.fatalErrorCode == 0);
    TEST_RESULT("cleanup empties trajectory queue", m->getNQueuedItems() == 0 && m->getError() == 0);
    const auto finalGains = m->getPidConstants();
    TEST_RESULT("cleanup restores PID defaults", m->getError() == 0 && finalGains.kP == defaults.kP && finalGains.kI == defaults.kI && finalGains.kD == defaults.kD);
}
