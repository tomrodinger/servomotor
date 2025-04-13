#include "Servomotor.h"
#include "test_framework.h"

// External declarations to access the global variables from ArduinoEmulator.cpp
extern char g_motorAlias;
extern uint64_t g_uniqueId;
extern bool g_useUniqueId;

void setup() {
    Serial.begin(115200);  // For debug output
    Serial.println("test_emergency_stop: BEGIN\n");

    // Create a Servomotor instance using the wrapper function
    Servomotor* motor = Servomotor_TestModeConvenienceWrapper();
    
    // Reset system to get to a known state
    motor->systemReset();
    delay(1500);  // Wait for system to reset

    // Check 1: Verify MOSFETs are disabled after system reset
    getStatusResponse reset_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after reset: 0x%02X\n", reset_status.statusFlags);
    TEST_RESULT("MOSFETs Disabled After Reset", (reset_status.statusFlags & 0x02) == 0);

    // Set units before moving
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // Test 2: Enable mosfets and verify
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(100);  // Wait for status to update
    getStatusResponse enabled_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    TEST_RESULT("MOSFETs Successfully Enabled", (enabled_status.statusFlags & 0x02) == 0x02);

    // Test 3: Check position is zero before starting movement
    float initial_position = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Initial position before movement: %.2f rotations\n", initial_position);
    TEST_RESULT("Position Is Zero Before Movement", approxEqual(initial_position, 0.0f, 0.05f));

    // Test 4: Start motor spinning
    const float target_velocity = 2.0f;  // rotations per second
    const float motion_duration = 5.0f;  // seconds
    const float spin_time = 1.0f;        // seconds to let motor spin before emergency stop
    
    printf("Starting motor motion at %.1f rotations/sec...\n", target_velocity);
    motor->moveWithVelocity(target_velocity, motion_duration);
    checkMotorError(*motor, "moveWithVelocity");
    
    // Let motor spin for 1 second
    delay(1000);  // 1 second = 1000 ms

    // Get position before emergency stop
    float pos_before_stop = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Position before stop: %.2f rotations\n", pos_before_stop);
    
    // Check position is approximately correct (should be around 2 rotations after 1 second)
    float expected_position = target_velocity * spin_time;
    TEST_RESULT("Position Before Stop Is Correct",
                approxEqual(pos_before_stop, expected_position, 0.5f));

    // Test 5: Emergency stop
    printf("Executing emergency stop...\n");
    motor->emergencyStop();
    checkMotorError(*motor, "emergencyStop");
    delay(100);  // Wait for status to update

    // Get position immediately after emergency stop
    float pos_after_stop = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Position immediately after stop: %.2f rotations\n", pos_after_stop);
    
    // Check position hasn't changed much after emergency stop
    TEST_RESULT("Position Unchanged Immediately After Stop",
                approxEqual(pos_before_stop, pos_after_stop, 0.1f));

    // Verify mosfets are disabled
    getStatusResponse emergency_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after emergency stop: 0x%02X\n", emergency_status.statusFlags);
    TEST_RESULT("Emergency Stop Disables MOSFETs", (emergency_status.statusFlags & 0x02) == 0);

    // Verify queue is empty
    uint8_t items_in_queue = motor->getNQueuedItems();
    checkMotorError(*motor, "getNQueuedItems");
    TEST_RESULT("Emergency Stop Clears Queue", items_in_queue == 0);
    
    // Wait additional time to verify motor doesn't continue moving
    delay(500);  // Wait 0.5 seconds
    float pos_after_delay = motor->getPosition();
    checkMotorError(*motor, "getPosition");
    printf("Position after 0.5s delay: %.2f rotations\n", pos_after_delay);
    
    // Check position hasn't changed after delay
    TEST_RESULT("Motor Doesn't Move After Emergency Stop",
                approxEqual(pos_after_stop, pos_after_delay, 0.05f));

    // Print test results
    TestRunner::printResults();

    // Clean up
    delete motor;

    // Exit with appropriate status
    exit(TestRunner::allTestsPassed() ? 0 : 1);
}

void loop() {
    // Not used in this test
}
