#include "Servomotor.h"
#include "test_framework.h"

void setup() {
    Serial.begin(115200);  // For debug output
    Serial.println("test_move_with_acceleration: BEGIN\n");

    // Create a ServoMotor instance
    ServoMotor motor('X', Serial1);  // Initialize with alias 'X' and Serial1 port

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

    // Test 1: Move with acceleration in ROTATIONS_PER_SECOND_SQUARED
    printf("\n=== Test 1: Move with acceleration in ROTATIONS_PER_SECOND_SQUARED ===\n");
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor.setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test1 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test1);
    TEST_RESULT("Initial Position Is Zero", approxEqual(initial_pos_test1, 0.0f, 0.05f));
    
    // Move with acceleration (2 rotations per second squared for 1 second)
    const float accel_test1 = 2.0f;  // rotations per second squared
    const float duration_test1 = 1.0f;  // seconds
    printf("Moving with acceleration %.1f rotations/sec² for %.1f seconds...\n", accel_test1, duration_test1);
    motor.moveWithAcceleration(accel_test1, duration_test1);
    checkMotorError(motor, "moveWithAcceleration");
    
    // Queue a second move with negative acceleration to stop the motor properly
    // To stop, we need to apply the reverse acceleration for the same duration
    const float stop_accel = -accel_test1;  // negative of the initial acceleration
    const float stop_duration = duration_test1;  // same duration as acceleration
    printf("Stopping with acceleration %.1f rotations/sec² for %.1f seconds...\n", stop_accel, stop_duration);
    motor.moveWithAcceleration(stop_accel, stop_duration);
    checkMotorError(motor, "moveWithAcceleration (stop)");
    
    // Wait for both moves to complete (plus a little extra)
    delay((int)((duration_test1 + stop_duration) * 1000 + 200));  // Convert to ms and add 200ms buffer
    
    // Get final position
    float final_pos_test1 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f rotations\n", final_pos_test1);
    
    // Expected position calculation:
    // For constant acceleration a over time t, distance = 0.5 * a * t^2
    // For acceleration phase: d1 = 0.5 * a * t^2
    // For deceleration phase: d2 = v*t - 0.5 * a * t^2 (where v = a*t from first phase)
    // Total distance = d1 + d2 = 0.5 * a * t^2 + a*t*t - 0.5 * a * t^2 = a * t^2
    float expected_pos_test1 = initial_pos_test1 + (accel_test1 * duration_test1 * duration_test1);
    printf("Expected position: %.2f rotations\n", expected_pos_test1);
    TEST_RESULT("Final Position Is Correct (ROTATIONS_PER_SECOND_SQUARED)", 
                approxEqual(final_pos_test1, expected_pos_test1, 0.1f));
    
    // Test 2: Move with acceleration in DEGREES_PER_SECOND_SQUARED
    printf("\n=== Test 2: Move with acceleration in DEGREES_PER_SECOND_SQUARED ===\n");
    motor.setPositionUnit(PositionUnit::DEGREES);
    motor.setVelocityUnit(VelocityUnit::DEGREES_PER_SECOND);
    motor.setAccelerationUnit(AccelerationUnit::DEGREES_PER_SECOND_SQUARED);
    motor.setTimeUnit(TimeUnit::SECONDS);
        
    // Get initial position
    float initial_pos_test2 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f degrees\n", initial_pos_test2);
    // We don't expect this to be zero anymore, since we've accumulated position from test 1
    
    // Move with acceleration (720 degrees per second squared for 1 second)
    const float accel_test2 = 720.0f;  // degrees per second squared
    const float duration_test2 = 1.0f;  // seconds
    printf("Moving with acceleration %.1f degrees/sec² for %.1f seconds...\n", accel_test2, duration_test2);
    motor.moveWithAcceleration(accel_test2, duration_test2);
    checkMotorError(motor, "moveWithAcceleration");
    
    // Queue a second move with negative acceleration to stop the motor properly
    const float stop_accel2 = -accel_test2;  // negative of the initial acceleration
    const float stop_duration2 = duration_test2;  // same duration as acceleration
    printf("Stopping with acceleration %.1f degrees/sec² for %.1f seconds...\n", stop_accel2, stop_duration2);
    motor.moveWithAcceleration(stop_accel2, stop_duration2);
    checkMotorError(motor, "moveWithAcceleration (stop)");
    
    // Wait for both moves to complete (plus a little extra)
    delay((int)((duration_test2 + stop_duration2) * 1000 + 200));  // Convert to ms and add 200ms buffer
    
    // Get final position
    float final_pos_test2 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f degrees\n", final_pos_test2);
    
    // Expected position calculation (same formula as Test 1)
    float expected_pos_test2 = initial_pos_test2 + (accel_test2 * duration_test2 * duration_test2);
    printf("Expected position: %.2f degrees\n", expected_pos_test2);
    TEST_RESULT("Final Position Is Correct (DEGREES_PER_SECOND_SQUARED)", 
                approxEqual(final_pos_test2, expected_pos_test2, 10.0f));
    
    // Test 3: Move with acceleration in COUNTS_PER_SECOND_SQUARED
    printf("\n=== Test 3: Move with acceleration in COUNTS_PER_SECOND_SQUARED ===\n");
    motor.setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor.setVelocityUnit(VelocityUnit::COUNTS_PER_SECOND);
    motor.setAccelerationUnit(AccelerationUnit::COUNTS_PER_SECOND_SQUARED);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test3 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f counts\n", initial_pos_test3);
    // We don't expect this to be zero anymore, since we've accumulated position from tests 1 and 2
    
    // Move with acceleration (6553600 counts per second squared for 1 second)
    // 6553600 counts = 2 rotations (3276800 counts per rotation)
    const float accel_test3 = 6553600.0f;  // counts per second squared
    const float duration_test3 = 1.0f;     // seconds
    printf("Moving with acceleration %.1f counts/sec² for %.1f seconds...\n", accel_test3, duration_test3);
    motor.moveWithAcceleration(accel_test3, duration_test3);
    checkMotorError(motor, "moveWithAcceleration");
    
    // Queue a second move with negative acceleration to stop the motor properly
    const float stop_accel3 = -accel_test3;  // negative of the initial acceleration
    const float stop_duration3 = duration_test3;  // same duration as acceleration
    printf("Stopping with acceleration %.1f counts/sec² for %.1f seconds...\n", stop_accel3, stop_duration3);
    motor.moveWithAcceleration(stop_accel3, stop_duration3);
    checkMotorError(motor, "moveWithAcceleration (stop)");
    
    // Wait for both moves to complete (plus a little extra)
    delay((int)((duration_test3 + stop_duration3) * 1000 + 200));  // Convert to ms and add 200ms buffer
    
    // Get final position
    float final_pos_test3 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f counts\n", final_pos_test3);
    
    // Expected position calculation (same formula as Test 1)
    float expected_pos_test3 = initial_pos_test3 + (accel_test3 * duration_test3 * duration_test3);
    printf("Expected position: %.2f counts\n", expected_pos_test3);
    // Use a larger tolerance for the counts test
    TEST_RESULT("Final Position Is Correct (COUNTS_PER_SECOND_SQUARED)", 
                approxEqual(final_pos_test3, expected_pos_test3, 100000.0f));
    
    // Test 4: Move with acceleration in RADIANS_PER_SECOND_SQUARED
    printf("\n=== Test 4: Move with acceleration in RADIANS_PER_SECOND_SQUARED ===\n");
    motor.setPositionUnit(PositionUnit::RADIANS);
    motor.setVelocityUnit(VelocityUnit::RADIANS_PER_SECOND);
    motor.setAccelerationUnit(AccelerationUnit::RADIANS_PER_SECOND_SQUARED);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test4 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f radians\n", initial_pos_test4);
    
    // Move with acceleration (12.56 radians per second squared for 1 second)
    // 12.56 radians = 2π radians = 1 rotation
    const float accel_test4 = 12.56f;  // radians per second squared (approx 2π)
    const float duration_test4 = 1.0f;  // seconds
    printf("Moving with acceleration %.2f radians/sec² for %.1f seconds...\n", accel_test4, duration_test4);
    motor.moveWithAcceleration(accel_test4, duration_test4);
    checkMotorError(motor, "moveWithAcceleration");
    
    // Queue a second move with negative acceleration to stop the motor properly
    const float stop_accel4 = -accel_test4;  // negative of the initial acceleration
    const float stop_duration4 = duration_test4;  // same duration as acceleration
    printf("Stopping with acceleration %.2f radians/sec² for %.1f seconds...\n", stop_accel4, stop_duration4);
    motor.moveWithAcceleration(stop_accel4, stop_duration4);
    checkMotorError(motor, "moveWithAcceleration (stop)");
    
    // Wait for both moves to complete (plus a little extra)
    delay((int)((duration_test4 + stop_duration4) * 1000 + 200));  // Convert to ms and add 200ms buffer
    
    // Get final position
    float final_pos_test4 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f radians\n", final_pos_test4);
    
    // Expected position calculation (same formula as Test 1)
    float expected_pos_test4 = initial_pos_test4 + (accel_test4 * duration_test4 * duration_test4);
    printf("Expected position: %.2f radians\n", expected_pos_test4);
    TEST_RESULT("Final Position Is Correct (RADIANS_PER_SECOND_SQUARED)", 
                approxEqual(final_pos_test4, expected_pos_test4, 0.2f));
    
    // Test 5: Move with acceleration in COUNTS_PER_TIMESTEP_SQUARED
    printf("\n=== Test 5: Move with acceleration in COUNTS_PER_TIMESTEP_SQUARED ===\n");
    motor.setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor.setVelocityUnit(VelocityUnit::COUNTS_PER_TIMESTEP);
    motor.setAccelerationUnit(AccelerationUnit::COUNTS_PER_TIMESTEP_SQUARED);
    motor.setTimeUnit(TimeUnit::TIMESTEPS);
    
    // Get initial position
    float initial_pos_test5 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f counts\n", initial_pos_test5);
    
    // Move with acceleration (0.0067 counts per timestep squared for 31250 timesteps)
    // This is equivalent to 2.0 rotations/sec² for 1.0 seconds
    // Calculation: 2.0 rot/sec² * 3276800 counts/rot = 6553600 counts/sec²
    // 6553600 counts/sec² / (31250 timesteps/sec)² = 0.0067 counts/timestep²
    const float accel_test5 = 0.0067f;  // counts per timestep squared
    const float duration_test5 = 31250.0f;  // timesteps (1.0 seconds)
    printf("Moving with acceleration %.6f counts/timestep² for %.1f timesteps...\n", accel_test5, duration_test5);
    motor.moveWithAcceleration(accel_test5, duration_test5);
    checkMotorError(motor, "moveWithAcceleration");
    
    // Queue a second move with negative acceleration to stop the motor properly
    const float stop_accel5 = -accel_test5;  // negative of the initial acceleration
    const float stop_duration5 = duration_test5;  // same duration as acceleration
    printf("Stopping with acceleration %.6f counts/timestep² for %.1f timesteps...\n", stop_accel5, stop_duration5);
    motor.moveWithAcceleration(stop_accel5, stop_duration5);
    checkMotorError(motor, "moveWithAcceleration (stop)");
    
    // Wait for both moves to complete (plus a little extra)
    delay((int)((duration_test5 + stop_duration5) / 31250.0f * 1000 + 200));  // Convert timesteps to ms and add 200ms buffer
    
    // Get final position
    float final_pos_test5 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f counts\n", final_pos_test5);
    
    // Expected position calculation (same formula as Test 1)
    float expected_pos_test5 = initial_pos_test5 + (accel_test5 * duration_test5 * duration_test5);
    printf("Expected position: %.2f counts\n", expected_pos_test5);
    TEST_RESULT("Final Position Is Correct (COUNTS_PER_TIMESTEP_SQUARED)",
                approxEqual(final_pos_test5, expected_pos_test5, 100.0f));
    
    // Test 6: Move with negative acceleration
    printf("\n=== Test 6: Move with negative acceleration ===\n");
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor.setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Get initial position
    float initial_pos_test6 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Initial position: %.2f rotations\n", initial_pos_test6);
    
    // Move with negative acceleration (-2 rotations per second squared for 1 second)
    const float accel_test6 = -2.0f;  // rotations per second squared
    const float duration_test6 = 1.0f;  // seconds
    printf("Moving with acceleration %.1f rotations/sec² for %.1f seconds...\n", accel_test6, duration_test6);
    motor.moveWithAcceleration(accel_test6, duration_test6);
    checkMotorError(motor, "moveWithAcceleration");
    
    // Queue a second move with positive acceleration to stop the motor properly
    const float stop_accel6 = -accel_test6;  // opposite of the initial acceleration
    const float stop_duration6 = duration_test6;  // same duration as acceleration
    printf("Stopping with acceleration %.1f rotations/sec² for %.1f seconds...\n", stop_accel6, stop_duration6);
    motor.moveWithAcceleration(stop_accel6, stop_duration6);
    checkMotorError(motor, "moveWithAcceleration (stop)");
    
    // Wait for both moves to complete (plus a little extra)
    delay((int)((duration_test6 + stop_duration6) * 1000 + 200));  // Convert to ms and add 200ms buffer
    
    // Get final position
    float final_pos_test6 = motor.getPosition();
    checkMotorError(motor, "getPosition");
    printf("Final position: %.2f rotations\n", final_pos_test6);
    
    // Expected position calculation (same formula as Test 1)
    float expected_pos_test6 = initial_pos_test6 + (accel_test6 * duration_test6 * duration_test6);
    printf("Expected position: %.2f rotations\n", expected_pos_test6);
    TEST_RESULT("Negative Acceleration Movement Is Correct",
                approxEqual(final_pos_test6, expected_pos_test6, 0.1f));

    // Note: We've already effectively tested the requirement that all moves must end with zero velocity
    // in all the previous tests by applying the reverse acceleration for the same duration.
    
    // Print test results
    TestRunner::printResults();
    
    // Exit with appropriate status
    exit(TestRunner::allTestsPassed() ? 0 : 1);
}

void loop() {
    // Not used in this test
}