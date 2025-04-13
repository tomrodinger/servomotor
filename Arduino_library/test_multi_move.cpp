#include "Servomotor.h"
#include "test_framework.h"
#include <stdio.h>
#include <math.h>

// Servomotor instance will be created in setup() using the wrapper

// Helper function to calculate and perform delay for multiMove sequences
template<typename T>
void multiMoveDelay(uint8_t multiMoveCount, const T* multiMoveList) {
    // Calculate total duration of the sequence
    float totalDuration = 0.0f;
    for (int i = 0; i < multiMoveCount; i++) {
        totalDuration += multiMoveList[i].duration;
    }
    
    // Add a buffer of 5% to ensure the sequence completes
    float waitTime = totalDuration * 1.05f;
    printf("Waiting for %.2f seconds (sequence duration: %.2f seconds + 5%% buffer)\n", waitTime, totalDuration);
    
    // Wait for the sequence to complete
    delay(static_cast<int>(waitTime * 1000));
}

// Helper function to calculate and perform delay for multiMoveRaw sequences with internal time units
void multiMoveDelayInternalTimeInput(uint8_t multiMoveCount, const multimoveList_t* multiMoveList) {
    // Calculate total duration of the sequence in internal time units
    float totalDurationInternalUnits = 0.0f;
    for (int i = 0; i < multiMoveCount; i++) {
        totalDurationInternalUnits += multiMoveList[i].timeSteps;
    }
    
    // Convert from internal time units to seconds (CONVERSION_FACTOR_SECONDS = 31250.0f)
    float totalDuration = totalDurationInternalUnits / 31250.0f;
    
    // Add a buffer of 5% to ensure the sequence completes
    float waitTime = totalDuration * 1.05f;
    printf("Waiting for %.2f seconds (sequence duration: %.2f seconds + 5%% buffer)\n", waitTime, totalDuration);
    
    // Wait for the sequence to complete
    delay(static_cast<int>(waitTime * 1000));
}


// Test multiMove with different velocity units
void testMultiMoveVelocityUnits(Servomotor* motor) {
    printf("\n=== Testing multiMove with different velocity units ===\n");
    
    // Test with ROTATIONS_PER_SECOND
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of moves
    const uint8_t multiMoveCount = 2;
    uint32_t multiMoveTypes = 0b11;  // Both are velocity moves
    
    // Create move list with user units
    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, 1.0f},    // Velocity move: 2 rot/sec for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with ROTATIONS_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (ROTATIONS_PER_SECOND)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (ROTATIONS_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with ROTATIONS_PER_SECOND", approxEqual(end_pos, 2.0f));
    
    // Test with RPM
    motor->setVelocityUnit(VelocityUnit::RPM);
    checkMotorError(*motor, "setVelocityUnit(RPM)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with RPM units (120 RPM = 2 RPS)
    multimoveListConverted_t multiMoveListRPM[multiMoveCount] = {
        {120.0f, 1.0f},  // Velocity move: 120 RPM for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 RPM for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with RPM...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRPM);
    checkMotorError(*motor, "multiMove (RPM)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveListRPM);
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RPM)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with RPM", approxEqual(end_pos, 2.0f));
    
    // Test with DEGREES_PER_SECOND
    motor->setVelocityUnit(VelocityUnit::DEGREES_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(DEGREES_PER_SECOND)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with degrees/sec units (720 deg/sec = 2 RPS)
    multimoveListConverted_t multiMoveListDegrees[multiMoveCount] = {
        {720.0f, 1.0f},  // Velocity move: 720 deg/sec for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 deg/sec for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with DEGREES_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListDegrees);
    checkMotorError(*motor, "multiMove (DEGREES_PER_SECOND)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveListDegrees);
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (DEGREES_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with DEGREES_PER_SECOND", approxEqual(end_pos, 2.0f));
    
    // Test with RADIANS_PER_SECOND
    motor->setVelocityUnit(VelocityUnit::RADIANS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(RADIANS_PER_SECOND)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with radians/sec units (4π rad/sec = 2 RPS)
    multimoveListConverted_t multiMoveListRadians[multiMoveCount] = {
        {4.0f * M_PI, 1.0f},  // Velocity move: 4π rad/sec for 1 sec
        {0.0f, 0.1f}          // Velocity move: 0 rad/sec for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with RADIANS_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRadians);
    checkMotorError(*motor, "multiMove (RADIANS_PER_SECOND)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveListRadians);
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RADIANS_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with RADIANS_PER_SECOND", approxEqual(end_pos, 2.0f));
}

// Test multiMove with different acceleration units
void testMultiMoveAccelerationUnits(Servomotor* motor) {
    printf("\n=== Testing multiMove with different acceleration units ===\n");
    
    // Test with ROTATIONS_PER_SECOND_SQUARED
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(ROTATIONS_PER_SECOND_SQUARED)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of moves
    const uint8_t multiMoveCount = 3;
    uint32_t multiMoveTypes = 0b001;  // First is velocity move, others are acceleration moves
    
    // Create move list with user units
    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {0.0f, 0.1f},     // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {2.0f, 2.0f},     // Acceleration move: 2 rot/sec² for 2 sec
        {-2.0f, 2.0f}     // Acceleration move: -2 rot/sec² for 2 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with ROTATIONS_PER_SECOND_SQUARED...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (ROTATIONS_PER_SECOND_SQUARED)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (ROTATIONS_PER_SECOND_SQUARED)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 8 rotations
    // The acceleration moves work differently than expected:
    // Starting from rest (0 velocity), accelerating at 2 rot/sec² for 2 sec results in:
    // Final velocity = 2 rot/sec² * 2 sec = 4 rot/sec
    // Displacement = 0.5 * 2 rot/sec² * 2² sec² = 4 rotations
    // Then decelerating at -2 rot/sec² for 2 sec from 4 rot/sec results in:
    // Final velocity = 4 rot/sec - 2 rot/sec² * 2 sec = 0 rot/sec
    // Displacement = 4 rot/sec * 2 sec - 0.5 * 2 rot/sec² * 2² sec² = 8 - 4 = 4 rotations
    // Total displacement = 4 + 4 = 8 rotations
    TEST_RESULT("multiMove with ROTATIONS_PER_SECOND_SQUARED", approxEqual(end_pos, 8.0f));
    
    // Test with RPM_PER_SECOND
    motor->setAccelerationUnit(AccelerationUnit::RPM_PER_SECOND);
    checkMotorError(*motor, "setAccelerationUnit(RPM_PER_SECOND)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with RPM/sec units (120 RPM/sec = 2 RPS²)
    multimoveListConverted_t multiMoveListRPM[multiMoveCount] = {
        {0.0f, 0.1f},      // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {120.0f, 2.0f},    // Acceleration move: 120 RPM/sec for 2 sec
        {-120.0f, 2.0f}    // Acceleration move: -120 RPM/sec for 2 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with RPM_PER_SECOND...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRPM);
    checkMotorError(*motor, "multiMove (RPM_PER_SECOND)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveListRPM);
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RPM_PER_SECOND)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 8 rotations
    TEST_RESULT("multiMove with RPM_PER_SECOND", approxEqual(end_pos, 8.0f));
    
    // Test with DEGREES_PER_SECOND_SQUARED
    motor->setAccelerationUnit(AccelerationUnit::DEGREES_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(DEGREES_PER_SECOND_SQUARED)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with degrees/sec² units (720 deg/sec² = 2 RPS²)
    multimoveListConverted_t multiMoveListDegrees[multiMoveCount] = {
        {0.0f, 0.1f},      // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {720.0f, 2.0f},    // Acceleration move: 720 deg/sec² for 2 sec
        {-720.0f, 2.0f}    // Acceleration move: -720 deg/sec² for 2 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with DEGREES_PER_SECOND_SQUARED...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListDegrees);
    checkMotorError(*motor, "multiMove (DEGREES_PER_SECOND_SQUARED)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveListDegrees);
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (DEGREES_PER_SECOND_SQUARED)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 8 rotations
    TEST_RESULT("multiMove with DEGREES_PER_SECOND_SQUARED", approxEqual(end_pos, 8.0f));
    
    // Test with RADIANS_PER_SECOND_SQUARED
    motor->setAccelerationUnit(AccelerationUnit::RADIANS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(RADIANS_PER_SECOND_SQUARED)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with radians/sec² units (4π rad/sec² = 2 RPS²)
    multimoveListConverted_t multiMoveListRadians[multiMoveCount] = {
        {0.0f, 0.1f},           // Velocity move: 0 rot/sec for 0.1 sec (start from rest)
        {4.0f * M_PI, 2.0f},    // Acceleration move: 4π rad/sec² for 2 sec
        {-4.0f * M_PI, 2.0f}    // Acceleration move: -4π rad/sec² for 2 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with RADIANS_PER_SECOND_SQUARED...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListRadians);
    checkMotorError(*motor, "multiMove (RADIANS_PER_SECOND_SQUARED)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveListRadians);
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (RADIANS_PER_SECOND_SQUARED)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 8 rotations
    TEST_RESULT("multiMove with RADIANS_PER_SECOND_SQUARED", approxEqual(end_pos, 8.0f));
}

// Test multiMove with different time units
void testMultiMoveTimeUnits(Servomotor* motor) {
    printf("\n=== Testing multiMove with different time units ===\n");
    
    // Test with SECONDS
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of moves
    const uint8_t multiMoveCount = 2;
    uint32_t multiMoveTypes = 0b11;  // Both are velocity moves
    const float multiMoveTime0 = 1.0;
    const float multiMoveTime1 = 0.1;
    // Create move list with user units
    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, multiMoveTime0},    // Velocity move: 2 rot/sec for 1 sec
        {0.0f, multiMoveTime1}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with SECONDS...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (SECONDS)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (SECONDS)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with SECONDS", approxEqual(end_pos, 2.0f));
    
    // Test with MILLISECONDS
    motor->setTimeUnit(TimeUnit::MILLISECONDS);
    checkMotorError(*motor, "setTimeUnit(MILLISECONDS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with milliseconds units
    multimoveListConverted_t multiMoveListMS[multiMoveCount] = {
        {2.0f, multiMoveTime0 * 1000.0f},  // Velocity move: 2 rot/sec for 1000 ms
        {0.0f, multiMoveTime1 * 100.0f}    // Velocity move: 0 rot/sec for 100 ms (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with MILLISECONDS...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListMS);
    checkMotorError(*motor, "multiMove (MILLISECONDS)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList); // Note: We are using the array containing the time in units of seconds
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (MILLISECONDS)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with MILLISECONDS", approxEqual(end_pos, 2.0f));
    
    // Test with MINUTES
    motor->setTimeUnit(TimeUnit::MINUTES);
    checkMotorError(*motor, "setTimeUnit(MINUTES)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create move list with minutes units
    multimoveListConverted_t multiMoveListMin[multiMoveCount] = {
        {2.0f, multiMoveTime0 / 60.0f},  // Velocity move: 2 rot/sec for 1/60 minute (1 sec)
        {0.0f, multiMoveTime1 / 60.0f}   // Velocity move: 0 rot/sec for 0.1/60 minute (0.1 sec)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing with MINUTES...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveListMin);
    checkMotorError(*motor, "multiMove (MINUTES)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList); // Note: We are using the array containing the time in units of seconds
    
    // Get final position
    end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (MINUTES)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMove with MINUTES", approxEqual(end_pos, 2.0f));
}

// Test multiMoveRaw with internal units
void testMultiMoveRaw(Servomotor* motor) {
    printf("\n=== Testing multiMoveRaw with internal units ===\n");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of moves
    const uint8_t multiMoveCount = 2;
    uint32_t multiMoveTypes = 0b11;  // Both are velocity moves
    
    // Create move list with internal units
    // For velocity: CONVERSION_FACTOR_ROTATIONS_PER_SECOND = 109951162.777600005f
    // For time: CONVERSION_FACTOR_SECONDS = 31250.000000000f
    multimoveList_t multiMoveList[multiMoveCount] = {
        {219902325, 31250},  // Velocity move: 2 rot/sec for 1 sec in internal units
        {0, 3125}            // Velocity move: 0 rot/sec for 0.1 sec in internal units
    };
    
    // Execute the multi-move sequence with raw units
    printf("Testing multiMoveRaw...\n");
    motor->multimoveRaw(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMoveRaw");
    
    // Wait for the sequence to complete
    multiMoveDelayInternalTimeInput(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Raw)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position: 2 rotations
    TEST_RESULT("multiMoveRaw", approxEqual(end_pos, 2.0f));
}

// Test complex motion sequence with mixed velocity and acceleration moves
void testComplexMotionSequence(Servomotor* motor) {
    printf("\n=== Testing complex motion sequence with mixed moves ===\n");
    
    // Set units
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(ROTATIONS_PER_SECOND_SQUARED)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of moves
    const uint8_t multiMoveCount = 4;
    uint32_t multiMoveTypes = 0b1001;  // 1st and 4th are velocity moves, 2nd and 3rd are acceleration moves
    
    // Create move list with user units
    multimoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, 1.0f},    // Velocity move: 2 rot/sec for 1 sec
        {2.0f, 1.0f},    // Acceleration move: 2 rot/sec² for 1 sec
        {-2.0f, 1.0f},   // Acceleration move: -2 rot/sec² for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing complex motion sequence...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (Complex)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Complex)");
    printf("Position after move: %.2f rotations\n", end_pos);
    
    // Expected position calculation:
    // 1. Velocity move: 2 rot/sec * 1 sec = 2 rotations
    // 2. Acceleration move: Starting velocity 2 rot/sec, accelerating at 2 rot/sec²
    //    Displacement = (2 rot/sec * 1 sec) + (0.5 * 2 rot/sec² * 1 sec²) = 2 + 1 = 3 rotations
    //    Final velocity = 2 + (2 * 1) = 4 rot/sec
    // 3. Acceleration move: Starting velocity 4 rot/sec, decelerating at 2 rot/sec²
    //    Displacement = (4 rot/sec * 1 sec) - (0.5 * 2 rot/sec² * 1 sec²) = 4 - 1 = 3 rotations
    //    Final velocity = 4 - (2 * 1) = 2 rot/sec
    // 4. Velocity move: 0 rot/sec for 0.1 sec (stop)
    //    Displacement = 0 rotations
    // Total: 2 + 3 + 3 + 0 = 8 rotations
    
    // Expected position: 8 rotations
    TEST_RESULT("Complex motion sequence", approxEqual(end_pos, 8.0f));
}

// Test maximum number of moves (32) with velocity moves
void testMaximumNumberOfVelocityMoves(Servomotor* motor) {
    printf("\n=== Testing maximum number of velocity moves (32) ===\n");
    
    // Set units
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    checkMotorError(*motor, "setVelocityUnit(ROTATIONS_PER_SECOND)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of 32 moves (maximum)
    const uint8_t multiMoveCount = 32;
    uint32_t multiMoveTypes = 0xFFFFFFFF;  // All velocity moves
    
    // Create move list with user units
    multimoveListConverted_t multiMoveList[multiMoveCount];
    
    // Fill the move list with alternating positive and negative velocity moves
    // to make the motor wiggle back and forth
    for (int i = 0; i < multiMoveCount - 1; i++) {
        // Alternate between positive and negative velocities
        float velocity = (i % 2 == 0) ? 1.0f : -1.0f;
        
        // Each move lasts 0.1 seconds
        multiMoveList[i].value = velocity;
        multiMoveList[i].duration = 0.1f;
    }
    
    // Last move is always a stop
    multiMoveList[multiMoveCount - 1].value = 0.0f;
    multiMoveList[multiMoveCount - 1].duration = 0.1f;
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing maximum number of velocity moves...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (Max Velocity Moves)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Max Velocity Moves)");
    printf("Position after velocity moves: %.2f rotations\n", end_pos);
    
    // We're not checking the exact position here, just that the command executed without errors
    TEST_RESULT("Maximum number of velocity moves", motor->getError() == 0);
}

// Test maximum number of moves (32) with acceleration moves
void testMaximumNumberOfAccelerationMoves(Servomotor* motor) {
    printf("\n=== Testing maximum number of acceleration moves (32) ===\n");
    
    // Set units
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    checkMotorError(*motor, "setAccelerationUnit(ROTATIONS_PER_SECOND_SQUARED)");
    motor->setTimeUnit(TimeUnit::SECONDS);
    checkMotorError(*motor, "setTimeUnit(SECONDS)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    checkMotorError(*motor, "setPositionUnit(SHAFT_ROTATIONS)");
    
    // Reset position
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    
    // Create a sequence of 32 moves (maximum)
    const uint8_t multiMoveCount = 32;
    uint32_t multiMoveTypes = 0x00000000;  // All acceleration moves
    
    // Create move list with user units
    multimoveListConverted_t multiMoveList[multiMoveCount];
    
    // Fill the move list with a wiggle pattern
    // One wiggle consists of 4 acceleration moves:
    // 1. Positive acceleration: increases velocity from 0 to positive
    // 2. Negative acceleration: decreases velocity from positive to 0
    // 3. Negative acceleration: decreases velocity from 0 to negative
    // 4. Positive acceleration: increases velocity from negative to 0
    
    // We can fit 8 complete wiggles (32 moves)
    for (int wiggle = 0; wiggle < 8; wiggle++) {
        int baseIndex = wiggle * 4;
        multiMoveList[baseIndex + 0].value = 5.0f;
        multiMoveList[baseIndex + 0].duration = 0.2f;
        multiMoveList[baseIndex + 1].value = -5.0f;
        multiMoveList[baseIndex + 1].duration = 0.2f;
        multiMoveList[baseIndex + 2].value = -5.0f;
        multiMoveList[baseIndex + 2].duration = 0.2f;
        multiMoveList[baseIndex + 3].value = 5.0f;
        multiMoveList[baseIndex + 3].duration = 0.2f;
    }
    
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Testing maximum number of acceleration moves...\n");
    motor->multimove(multiMoveCount, multiMoveTypes, multiMoveList);
    checkMotorError(*motor, "multiMove (Max Acceleration Moves)");
    
    // Wait for the sequence to complete
    multiMoveDelay(multiMoveCount, multiMoveList);
    
    // Get final position
    float end_pos = motor->getPosition();
    checkMotorError(*motor, "getPosition (Max Acceleration Moves)");
    printf("Position after acceleration moves: %.2f rotations\n", end_pos);
    
    // We're not checking the exact position here, just that the command executed without errors
    TEST_RESULT("Maximum number of acceleration moves", motor->getError() == 0);
}

void setup() {
    // Create a Servomotor instance using the wrapper
    Servomotor* motor = Servomotor_TestModeConvenienceWrapper();
    Serial.begin(115200);  // Debugging port at 115200 baud
    
    printf("\n\n=== Starting multiMove Tests ===\n\n");
    
    // Reset system to get to a known state
    printf("Resetting system...\n");
    motor->systemReset();
    delay(1500);  // Wait for system to reset
    // Note: We don't check for errors after systemReset because the motor resets and doesn't respond
    
    // Configure motor units
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor->setTimeUnit(TimeUnit::SECONDS);
    
    // Enable mosfets before moving
    printf("Enabling MOSFETs...\n");
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    
    delay(100);  // Wait for status to update
    getStatusResponse enabled_status = motor->getStatus();
    checkMotorError(*motor, "getStatus");
    printf("Status after enable: 0x%02X\n", enabled_status.statusFlags);
    TEST_RESULT("MOSFETs Successfully Enabled", (enabled_status.statusFlags & 0x02) == 0x02);
    
    // Run tests
    testMultiMoveVelocityUnits(motor);
    testMultiMoveAccelerationUnits(motor);
    testMultiMoveTimeUnits(motor);
    testMultiMoveRaw(motor);
    testComplexMotionSequence(motor);
    testMaximumNumberOfVelocityMoves(motor);
    testMaximumNumberOfAccelerationMoves(motor);
    
    printf("\n=== All tests completed ===\n");
    
    // Print test results
    TestRunner::printResults();

    // Clean up
    delete motor;
    
    // Exit with appropriate status
    exit(TestRunner::allTestsPassed() ? 0 : 1);
}

void loop() {
    // Not used
}