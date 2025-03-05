#include "Servomotor.h"
#include "test_framework.h"

void setup() {
    Serial.begin(115200);  // For debug output
    Serial.println("test_move_with_velocity: BEGIN\n");

    // Create a Servomotor instance
    Servomotor motor('X', Serial1);  // Initialize with alias 'X' and Serial1 port

    // Reset system to get to a known state
    motor.systemReset();
    delay(1500);  // Wait for system to reset
    // Note: We don't check for errors after systemReset because the motor resets and doesn't respond

    // Enable MOSFETs
    motor.enableMosfets();
    checkMotorError(motor, "enableMosfets");
    
    delay(100);  // Wait for status to update
    StatusResponse enabled_status = motor.getStatus();
    checkMotorError(motor, "getStatus");
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    TEST_RESULT("MOSFETs Successfully Enabled", (enabled_status.statusFlags & 0x02) == 0x02);

    // Test 1: Move with velocity in ROTATIONS_PER_SECOND
    printf("\n=== Test 1: Move with velocity in ROTATIONS_PER_SECOND ===\n");
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test1 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test1);
    TEST_RESULT("Initial Position Is Zero", approxEqual(initial_pos_test1, 0.0f, 0.05f));
    
    // Move with velocity (2 rotations per second for 1 second)
    const float velocity_test1 = 2.0f;  // rotations per second
    const float duration_test1 = 1.0f;  // seconds
    printf("Moving at %.1f rotations/sec for %.1f seconds...\n", velocity_test1, duration_test1);
    motor.moveWithVelocity(velocity_test1, duration_test1);
    checkMotorError(motor, "moveWithVelocity");
    
    // Queue a second move with velocity 0 to stop the motor properly
    const float stop_velocity = 0.0f;  // rotations per second
    const float stop_duration = 0.1f;  // seconds
    printf("Stopping with velocity 0 for %.1f seconds...\n", stop_duration);
    motor.moveWithVelocity(stop_velocity, stop_duration);
    checkMotorError(motor, "moveWithVelocity (stop)");
    
    // Wait for move to complete (plus a little extra)
    delay(1200);  // 1.2 seconds
    
    // Get final position
    float final_pos_test1 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f rotations\n", final_pos_test1);
    
    // Expected position: initial + (velocity * duration)
    float expected_pos_test1 = initial_pos_test1 + (velocity_test1 * duration_test1);
    printf("Expected position: %.2f rotations\n", expected_pos_test1);
    TEST_RESULT("Final Position Is Correct (ROTATIONS_PER_SECOND)", approxEqual(final_pos_test1, expected_pos_test1, 0.1f));
    
    // Test 2: Move with velocity in DEGREES_PER_SECOND
    printf("\n=== Test 2: Move with velocity in DEGREES_PER_SECOND ===\n");
    motor.setPositionUnit(PositionUnit::DEGREES);
    motor.setVelocityUnit(VelocityUnit::DEGREES_PER_SECOND);
    motor.setTimeUnit(TimeUnit::SECONDS);
        
    // Get initial position
    float initial_pos_test2 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f degrees\n", initial_pos_test2);
    // We don't expect this to be zero anymore, since we've accumulated position from test 1
    
    // Move with velocity (720 degrees per second for 1 second = 2 rotations)
    const float velocity_test2 = 720.0f;  // degrees per second
    const float duration_test2 = 1.0f;    // seconds
    printf("Moving at %.1f degrees/sec for %.1f seconds...\n", velocity_test2, duration_test2);
    motor.moveWithVelocity(velocity_test2, duration_test2);
    checkMotorError(motor, "moveWithVelocity");
    
    // Queue a second move with velocity 0 to stop the motor properly
    const float stop_velocity2 = 0.0f;  // degrees per second
    const float stop_duration2 = 0.1f;  // seconds
    printf("Stopping with velocity 0 for %.1f seconds...\n", stop_duration2);
    motor.moveWithVelocity(stop_velocity2, stop_duration2);
    checkMotorError(motor, "moveWithVelocity (stop)");
    
    // Wait for move to complete (plus a little extra)
    delay(1200);  // 1.2 seconds
    
    // Get final position
    float final_pos_test2 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f degrees\n", final_pos_test2);
    
    // Expected position: initial + (velocity * duration)
    float expected_pos_test2 = initial_pos_test2 + (velocity_test2 * duration_test2);
    printf("Expected position: %.2f degrees\n", expected_pos_test2);
    TEST_RESULT("Final Position Is Correct (DEGREES_PER_SECOND)", approxEqual(final_pos_test2, expected_pos_test2, 10.0f));
    
    // Test 3: Move with velocity in ENCODER_COUNTS
    printf("\n=== Test 3: Move with velocity in ENCODER_COUNTS ===\n");
    motor.setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor.setVelocityUnit(VelocityUnit::COUNTS_PER_SECOND);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test3 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f counts\n", initial_pos_test3);
    // We don't expect this to be zero anymore, since we've accumulated position from tests 1 and 2
    
    // Move with velocity (6553600 counts per second for 1 second = 2 rotations)
    const float velocity_test3 = 6553600.0f;  // counts per second (2 * 3276800)
    const float duration_test3 = 1.0f;        // seconds
    printf("Moving at %.1f counts/sec for %.1f seconds...\n", velocity_test3, duration_test3);
    motor.moveWithVelocity(velocity_test3, duration_test3);
    checkMotorError(motor, "moveWithVelocity");
    
    // Queue a second move with velocity 0 to stop the motor properly
    const float stop_velocity3 = 0.0f;  // counts per second
    const float stop_duration3 = 0.1f;  // seconds
    printf("Stopping with velocity 0 for %.1f seconds...\n", stop_duration3);
    motor.moveWithVelocity(stop_velocity3, stop_duration3);
    checkMotorError(motor, "moveWithVelocity (stop)");
    
    // Wait for move to complete (plus a little extra)
    delay(1200);  // 1.2 seconds
    
    // Get final position
    float final_pos_test3 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f counts\n", final_pos_test3);
    
    // Expected position: initial + (velocity * duration)
    float expected_pos_test3 = initial_pos_test3 + (velocity_test3 * duration_test3);
    printf("Expected position: %.2f counts\n", expected_pos_test3);
    // Use a larger tolerance for the counts test
    TEST_RESULT("Final Position Is Correct (COUNTS_PER_SECOND)", approxEqual(final_pos_test3, expected_pos_test3, 100000.0f));
    
    // Test 4: Get position in raw encoder counts
    printf("\n=== Test 4: Get position in raw encoder counts ===\n");
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor.setTimeUnit(TimeUnit::SECONDS);

    // Move with velocity (2 rotations per second for 1 second)
    const float velocity_test4 = 2.0f;  // rotations per second
    const float duration_test4 = 1.0f;  // seconds
    printf("Moving at %.1f rotations/sec for %.1f seconds...\n", velocity_test4, duration_test4);
    motor.moveWithVelocity(velocity_test4, duration_test4);
    checkMotorError(motor, "moveWithVelocity");
    
    // Queue a second move with velocity 0 to stop the motor properly
    const float stop_velocity4 = 0.0f;  // rotations per second
    const float stop_duration4 = 0.1f;  // seconds
    printf("Stopping with velocity 0 for %.1f seconds...\n", stop_duration4);
    motor.moveWithVelocity(stop_velocity4, stop_duration4);
    checkMotorError(motor, "moveWithVelocity (stop)");
    
    // Wait for move to complete (plus a little extra)
    delay(1200);  // 1.2 seconds
    
    // Get position in rotations
    float pos_in_rotations = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Position in rotations: %.2f rotations\n", pos_in_rotations);
    
    // Get position in raw encoder counts
    getHallSensorPositionResponse raw_pos = motor.getPositionRaw();
    checkMotorError(motor, "getPositionRaw");
    printf("Position in raw encoder counts: %lld\n", raw_pos.hallSensorPosition);
    
    // The raw position will include all the accumulated movements from previous tests
    // Calculate the expected raw position based on the accumulated movements:
    // Test 1: 2 rotations
    // Test 2: 2 rotations (720 degrees = 2 rotations)
    // Test 3: 2 rotations (6553600 counts = 2 rotations)
    // Test 4: 2 rotations
    // Total: 8 rotations * 3276800 counts/rotation = 26214400 counts
    const int64_t ONE_ROTATION_COUNTS = 3276800;
    const int64_t TOTAL_ROTATIONS = 8; // 2 + 2 + 2 + 2 rotations
    int64_t expected_raw_pos = TOTAL_ROTATIONS * ONE_ROTATION_COUNTS;
    printf("Expected raw position: %lld counts\n", expected_raw_pos);
    
    // We're checking if the position is approximately equal to the expected value
    // Use a reasonable tolerance for this test
    TEST_RESULT("Raw Position Is Correct", abs(raw_pos.hallSensorPosition - expected_raw_pos) < 1000);
    
    // Test 5: Move with negative velocity
    printf("\n=== Test 5: Move with negative velocity ===\n");
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test5 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test5);
    
    // Move with negative velocity (-2 rotations per second for 1 second)
    const float velocity_test5 = -2.0f;  // rotations per second
    const float duration_test5 = 1.0f;   // seconds
    printf("Moving at %.1f rotations/sec for %.1f seconds...\n", velocity_test5, duration_test5);
    motor.moveWithVelocity(velocity_test5, duration_test5);
    checkMotorError(motor, "moveWithVelocity");
    
    // Queue a second move with velocity 0 to stop the motor properly
    const float stop_velocity5 = 0.0f;  // rotations per second
    const float stop_duration5 = 0.1f;  // seconds
    printf("Stopping with velocity 0 for %.1f seconds...\n", stop_duration5);
    motor.moveWithVelocity(stop_velocity5, stop_duration5);
    checkMotorError(motor, "moveWithVelocity (stop)");
    
    // Wait for move to complete (plus a little extra)
    delay(1200);  // 1.2 seconds
    
    // Get final position
    float final_pos_test5 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f rotations\n", final_pos_test5);
    
    // Expected position: initial + (velocity * duration)
    float expected_pos_test5 = initial_pos_test5 + (velocity_test5 * duration_test5);
    printf("Expected position: %.2f rotations\n", expected_pos_test5);
    TEST_RESULT("Negative Velocity Movement Is Correct", approxEqual(final_pos_test5, expected_pos_test5, 0.1f));
    
    // Print test results
    TestRunner::printResults();
    
    // Exit with appropriate status
    exit(TestRunner::allTestsPassed() ? 0 : 1);
}

void loop() {
    // Not used in this test
}