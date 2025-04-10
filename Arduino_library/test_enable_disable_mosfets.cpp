#include "Servomotor.h"
#include "test_framework.h"
#include <iomanip>

void setup() {
    Serial.begin(115200);  // For debug output
    Serial.println("test_enable_disable_mosfets: BEGIN\n");

    // Create a Servomotor instance using the wrapper
    Servomotor* motor = Servomotor_TestModeConvenienceWrapper();

    // Reset system to get to a known state
    motor->systemReset();
    delay(1500);  // Wait for system to reset

    // Test 1: Check initial status (mosfets should be disabled after reset)
    StatusResponse initial_status = motor->getStatus();
    printf("Initial status flags: 0x%02X\n", initial_status.statusFlags);

    // Test 2: Enable mosfets and verify status change
    motor->enableMosfets();
    delay(100);  // Wait for status to update
    StatusResponse enabled_status = motor->getStatus();
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    
    // Test 3: Disable mosfets and verify status change
    motor->disableMosfets();
    delay(100);  // Wait for status to update
    StatusResponse disabled_status = motor->getStatus();
    printf("Status after disable: 0x%02X\n", disabled_status.statusFlags);

    // Print test results
    printf("\nStatus flag changes:\n");
    printf("Initial  -> Enable : 0x%02X -> 0x%02X (changed bits: 0x%02X)\n", 
           initial_status.statusFlags, enabled_status.statusFlags,
           initial_status.statusFlags ^ enabled_status.statusFlags);
    printf("Enable   -> Disable: 0x%02X -> 0x%02X (changed bits: 0x%02X)\n",
           enabled_status.statusFlags, disabled_status.statusFlags,
           enabled_status.statusFlags ^ disabled_status.statusFlags);

    // The bit that changes between enable and disable is likely our MOSFET status bit
    uint8_t mosfet_status_bit = enabled_status.statusFlags ^ disabled_status.statusFlags;
    bool enabled_bit_is_1 = (enabled_status.statusFlags & mosfet_status_bit) != 0;

    // Test results based on status changes
    TEST_RESULT("Status Changes on Enable", 
                initial_status.statusFlags != enabled_status.statusFlags);
    TEST_RESULT("Status Changes on Disable", 
                enabled_status.statusFlags != disabled_status.statusFlags);
    TEST_RESULT("Status Returns to Initial", 
                initial_status.statusFlags == disabled_status.statusFlags);

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
