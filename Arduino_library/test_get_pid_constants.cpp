// Native hardware test of the generated Arduino bindings, addressed by unique ID.
// Build/run instructions are in README.md. Resets the motor, writes raw gains
// only with outputs disabled, then restores startup defaults. No motion.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include "Servomotor.h"

ConsoleSerial Serial;
HardwareSerial Serial1;

static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

static void reset(Servomotor& motor) {
    motor.systemReset();
    delay(1500);
}

static void checkGains(Servomotor& motor, uint64_t uid,
                       uint32_t p, uint32_t i, uint32_t d) {
    // Both public generated overloads must return the full raw u32 values.
    getPidConstantsResponse a = motor.getPidConstants();
    require(motor.getError() == 0, "getPidConstants failed");
    require(a.kP == p && a.kI == i && a.kD == d, "configured-address readback differs");
    getPidConstantsResponse b = motor.getPidConstants(uid);
    require(motor.getError() == 0, "getPidConstants(uniqueId) failed");
    require(b.kP == p && b.kI == i && b.kD == d, "unique-ID readback differs");
}

int main(int argc, char** argv) {
    if (argc != 3 || std::strlen(argv[2]) != 16 ||
        std::strspn(argv[2], "0123456789abcdefABCDEF") != 16) {
        std::fprintf(stderr, "Usage: %s <serial-port> <16-hex-unique-id>\n", argv[0]);
        return 2;
    }
    uint64_t uid = std::strtoull(argv[2], nullptr, 16);
    if (uid <= 255) return 2;
    Serial1 = HardwareSerial(argv[1]);
    if (!Serial1.begin(230400)) return 2;
    Servomotor motor(255, Serial1);
    motor.useUniqueId(uid);
    bool passed = false;
    try {
        getProductInfoResponse info = motor.getProductInfo();
        require(motor.getError() == 0 && info.uniqueId == uid, "motor identity mismatch");
        require(std::memcmp(info.productCode, "M17     ", 8) == 0, "test expects M17 defaults");
        reset(motor);
        checkGains(motor, uid, 2000, 5, 175000);
        const uint32_t cases[][3] = {
            {0, 0, 0}, {1, 256, 65536}, {2000, 5, 175001},
            {0x7fffffff, 0x80000000, 0xffffffff},
            {0xffffffff, 0xffffffff, 0xffffffff},
            {0x01234567, 0x89abcdef, 0xfedcba98}, {2000, 5, 175000}
        };
        for (const auto& gains : cases) {
            motor.setPidConstants(gains[0], gains[1], gains[2]);
            require(motor.getError() == 0, "setter failed");
            checkGains(motor, uid, gains[0], gains[1], gains[2]);
            getStatusResponse status = motor.getStatus();
            require(motor.getError() == 0 && status.statusFlags == 0 &&
                    status.fatalErrorCode == 0, "readback changed output/status");
            require(motor.getNQueuedItems() == 0 && motor.getError() == 0,
                    "readback changed queue");
        }
        passed = true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
    }
    try {
        reset(motor);
        checkGains(motor, uid, 2000, 5, 175000);
        getStatusResponse status = motor.getStatus();
        require(motor.getError() == 0 && status.statusFlags == 0 &&
                status.fatalErrorCode == 0, "cleanup failed");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Cleanup: %s\n", e.what());
        passed = false;
    }
    std::printf("%016llX %s\n", (unsigned long long)uid, passed ? "PASSED" : "FAILED");
    return passed ? 0 : 1;
}
