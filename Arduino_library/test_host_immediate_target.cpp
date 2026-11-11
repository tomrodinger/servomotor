// Host-only check of the generated command-50 methods. Communication is a
// payload-level stand-in: no serial port or motor is opened. Real C framing
// and CRCs are checked by python_programs/test_host_immediate_target.py.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <type_traits>
#include "Servomotor.h"

ConsoleSerial Serial;
HardwareSerial Serial1;

static int64_t target, measured;
static uint64_t expectedAddress;
static bool expectedExtended;
static unsigned sends, receives;
static uint16_t responseSize = 8;
static int16_t responseError;

Communication::Communication(HardwareSerial& port, uint32_t baud, int8_t rx, int8_t tx)
    : _serial(port), _crc32Enabled(true), _baud(baud), _rxPin(rx), _txPin(tx) {}
void Communication::openSerialPort() {}
void Communication::flush() {}
void Communication::enableCRC32() { _crc32Enabled = true; }
void Communication::disableCRC32() { _crc32Enabled = false; }
bool Communication::isCRC32Enabled() const { return _crc32Enabled; }

static void command(bool extended, uint64_t address, uint8_t id,
                    const uint8_t* payload, uint16_t size) {
    assert(extended == expectedExtended && address == expectedAddress);
    assert(id == 50 && size == 8);
    for (unsigned i = 0; i < 8; ++i) {
        assert(payload[i] == static_cast<uint8_t>(static_cast<uint64_t>(target) >> (8 * i)));
    }
    ++sends;
}

void Communication::sendCommand(uint8_t alias, uint8_t id,
                                const uint8_t* payload, uint16_t size) {
    command(false, alias, id, payload, size);
}
void Communication::sendCommandByUniqueId(uint64_t uid, uint8_t id,
                                          const uint8_t* payload, uint16_t size) {
    command(true, uid, id, payload, size);
}
int16_t Communication::getResponse(uint8_t* buffer, uint16_t size, uint16_t& received) {
    ++receives;
    received = responseSize;
    if (responseError) return responseError;
    if (responseSize > size) return COMMUNICATION_ERROR_BUFFER_TOO_SMALL;
    for (unsigned i = 0; i < responseSize; ++i) {
        buffer[i] = static_cast<uint8_t>(static_cast<uint64_t>(measured) >> (8 * i));
    }
    return 0;
}

int main() {
    static_assert(sizeof(setImmediatePositionTargetAndGetPositionResponse) == 8, "compact response");
    static_assert(std::is_same<decltype(std::declval<Servomotor>().setImmediatePositionTargetAndGetPositionRaw(int64_t{})), int64_t>::value, "exact raw result");
    static_assert(std::is_same<decltype(std::declval<Servomotor>().setImmediatePositionTargetAndGetPosition(float{})), float>::value, "converted result");
    Servomotor motor('X', Serial1);
    const uint64_t uid = UINT64_C(0x04245C407D26CCD0);
    const int64_t edges[] = {INT64_MIN, INT64_MIN + 1, -(INT64_C(1) << 53) - 1,
                            (INT64_C(1) << 53) + 1, INT64_MAX - 1, INT64_MAX};
    for (int mode = 0; mode < 3; ++mode) {
        expectedExtended = mode != 0;
        expectedAddress = expectedExtended ? uid : 'X';
        if (mode == 2) motor.useUniqueId(uid);
        for (auto value : edges) {
            target = value;
            measured = value == INT64_MIN ? INT64_MAX : value - 1;
            sends = receives = 0;
            int64_t result = mode == 1 ? motor.setImmediatePositionTargetAndGetPositionRaw(uid, target)
                                       : motor.setImmediatePositionTargetAndGetPositionRaw(target);
            assert(result == measured && motor.getError() == 0);
            assert(sends == 1 && receives == 1);
        }
    }
    motor.useAlias('X');
    motor.setPositionUnit(PositionUnit::DEGREES);
    target = 819200; measured = -819200;
    expectedExtended = false; expectedAddress = 'X';
    assert(std::fabs(motor.setImmediatePositionTargetAndGetPosition(90.0f) + 90.0f) < 0.0001f && motor.getError() == 0);
    expectedExtended = true; expectedAddress = uid;
    assert(std::fabs(motor.setImmediatePositionTargetAndGetPosition(uid, 90.0f) + 90.0f) < 0.0001f && motor.getError() == 0);
    expectedExtended = false; expectedAddress = 'X';
    for (uint16_t size : {uint16_t(7), uint16_t(24)}) {
        responseSize = size; sends = receives = 0;
        assert(motor.setImmediatePositionTargetAndGetPositionRaw(target) == 0);
        assert(motor.getError() != 0 && sends == 1 && receives == 1);
    }
    responseSize = 8; responseError = 55; sends = receives = 0;
    assert(motor.setImmediatePositionTargetAndGetPositionRaw(target) == 0);
    assert(motor.getError() == 55 && sends == 1 && receives == 1);
    Serial.println("PASSED: generated Arduino command 50, exact i64, units, addressing, errors and no retries");
}
