// Real-motor integration test of generated Arduino command-50 methods.
// Accepts only the established rack adapter and one explicit unique ID.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include "Servomotor.h"

ConsoleSerial Serial;
HardwareSerial Serial1;
static unsigned checks, transactions;
static void require(bool condition, const char* message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
static void reset(Servomotor& motor) {
    motor.systemReset();
    motor.enableCRC32();
    delay(1500);
}
static void prepare(Servomotor& motor) {
    reset(motor);
    // Match the established Python rack-test limit: 50 internal units,
    // approximately 256 mA by the library conversion (not 50 mA).
    motor.setMaximumMotorCurrentRaw(50, 50);
    require(motor.getError() == 0, "current limits failed");
    motor.goToClosedLoop();
    require(motor.getError() == 0, "closed-loop entry failed");
    delay(200);
    motor.zeroPosition();
    require(motor.getError() == 0, "zero position failed");
}

int main(int argc, char** argv) {
    if (argc != 3 || std::strcmp(argv[1], "/dev/cu.usbserial-113210") != 0 ||
        std::strlen(argv[2]) != 16 || std::strspn(argv[2], "0123456789abcdefABCDEF") != 16) return 2;
    uint64_t uid = std::strtoull(argv[2], nullptr, 16);
    if (uid <= 255) return 2;
    Serial1 = HardwareSerial(argv[1]);
    if (!Serial1.begin(230400)) return 2;
    Servomotor motor(255, Serial1);
    motor.useUniqueId(uid);
    bool passed = false, compatible = false;
    try {
        auto info = motor.getProductInfo();
        require(motor.getError() == 0 && info.uniqueId == uid, "unique-ID mismatch");
        require(std::memcmp(info.productCode, "M17     ", 8) == 0 && info.firmwareCompatibility == 3, "model mismatch");
        auto fw = motor.getFirmwareVersion();
        require(motor.getError() == 0 && !fw.inBootloader && fw.firmwareVersion.major == 0 &&
                fw.firmwareVersion.minor == 17 && fw.firmwareVersion.patch == 0 && fw.firmwareVersion.dev == 0,
                "firmware must be 0.17.0.0");
        compatible = true;
        auto specs = motor.getProductSpecs();
        require(motor.getError() == 0 && specs.countsPerRotation > 0, "product specs failed");
        const int64_t step = specs.countsPerRotation / 360;
        prepare(motor);
        motor.setPositionUnit(PositionUnit::ENCODER_COUNTS);
        auto gains = motor.getPidConstants();
        for (int crc = 1; crc >= 0; --crc) {
            motor.crc32Control(crc);
            if (crc) motor.enableCRC32(); else motor.disableCRC32();
            require(motor.getError() == 0, "CRC mode change failed");
            for (int i = 0; i < 1000; ++i) {
                int64_t target = (i & 1) ? -step : step;
                int64_t measured = (i & 2) ? motor.setImmediatePositionTargetAndGetPositionRaw(uid, target)
                                         : motor.setImmediatePositionTargetAndGetPositionRaw(target);
                ++transactions;
                require(motor.getError() == 0, "raw command 50 failed");
                require(std::llabs(measured) < 3 * step, "measurement outside test range");
                if (i % 100 == 0) {
                    require(motor.getPositionRaw() == target && motor.getError() == 0, "reference not exact");
                    auto state = motor.getStatus();
                    require(motor.getError() == 0 && state.statusFlags == 6 && !state.fatalErrorCode,
                            "output/mode state changed");
                }
            }
            motor.setPositionUnit(PositionUnit::DEGREES);
            for (int i = 0; i < 8; ++i) {
                float target = (i & 1) ? -1.0f : 1.0f;
                float measured = (i & 2) ? motor.setImmediatePositionTargetAndGetPosition(uid, target)
                                         : motor.setImmediatePositionTargetAndGetPosition(target);
                ++transactions;
                require(motor.getError() == 0 && std::isfinite(measured), "converted command 50 failed");
                delay(150);
                measured = motor.setImmediatePositionTargetAndGetPosition(target);
                ++transactions;
                std::printf("TRACK target=%f measured=%f error=%d\n", target, measured, motor.getError());
                require(motor.getError() == 0 && std::fabs(measured - target) < 0.5f, "converted tracking failed");
            }
            motor.setPositionUnit(PositionUnit::ENCODER_COUNTS);
        }
        motor.crc32Control(1); motor.enableCRC32();
        auto after = motor.getPidConstants();
        require(motor.getError() == 0 && gains.kP == after.kP && gains.kI == after.kI && gains.kD == after.kD,
                "PID constants changed");
        auto debug = motor.getDebugValues();
        require(motor.getError() == 0 && debug.allMotorControlCalculationsProfilerMaxTime < 32, "ISR period exceeded");
        motor.emergencyStop();
        motor.setImmediatePositionTargetAndGetPositionRaw(0);
        require(motor.getError() == 55, "disabled output guard missing");
        const int64_t edges[] = {INT64_MIN, INT64_MAX, -(INT64_C(1) << 53) - 1, (INT64_C(1) << 53) + 1};
        for (auto target : edges) {
            prepare(motor);
            motor.setImmediatePositionTargetAndGetPositionRaw(target);
            require(motor.getError() == 45, "full-width unsafe reference not rejected");
            auto state = motor.getStatus();
            require(motor.getError() == 0 && state.statusFlags == 0 && state.fatalErrorCode == 45,
                    "unsafe reference did not disable outputs");
        }
        passed = true;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
    }
    if (compatible) {
        try {
            reset(motor);
            auto state = motor.getStatus();
            require(motor.getError() == 0 && state.statusFlags == 0 && state.fatalErrorCode == 0,
                    "cleanup status failed");
            require(motor.getNQueuedItems() == 0 && motor.getError() == 0, "cleanup queue failed");
        } catch (const std::exception& error) {
            std::fprintf(stderr, "Cleanup failed: %s\n", error.what()); passed = false;
        }
    }
    std::printf("%s: %u checks; %u command-50 transactions; raw/converted UID overloads, both CRC modes, i64 guards, cleanup\n",
                passed ? "PASSED" : "FAILED", checks, transactions);
    return passed ? 0 : 1;
}
