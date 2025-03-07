#include "Servomotor.h"


// Create Servomotor instance on the stack
// This is safe because ArduinoEmulator.cpp ensures Serial1 is initialized
// before setup() is called, and exits if initialization fails
Servomotor motor('X', Serial1);

void setup() {
    Serial.begin(115200);  // Debugging port at 115200 baud
    
    // Configure motor units
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor.setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Enable mosfets before moving
    motor.enableMosfets();
    
    // Get initial position
    float start_pos = motor.getPosition();
    printf("Position before move: %.2f rotations\n", start_pos);
    
    // Create a sequence of moves
    const uint8_t multiMoveCount = 4;
    uint32_t multiMoveTypes = 0b1001;  // 1001 in binary: 1st and 4th are velocity moves, 2nd and 3rd are acceleration moves
    
    // Create move list with user units
    multiMoveListConverted_t multiMoveList[multiMoveCount] = {
        {2.0f, 1.0f},    // Velocity move: 2 rot/sec for 1 sec
        {2.0f, 1.0f},    // Acceleration move: 2 rot/sec² for 1 sec
        {-2.0f, 1.0f},   // Acceleration move: -2 rot/sec² for 1 sec
        {0.0f, 0.1f}     // Velocity move: 0 rot/sec for 0.1 sec (stop)
    };
    
    // Execute the multi-move sequence with automatic unit conversion
    printf("Executing multi-move sequence...\n");
    
    motor.multiMove(multiMoveCount, multiMoveTypes, multiMoveList);
    
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
    
    // Get final position
    float end_pos = motor.getPosition();
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
    printf("Expected position: %.2f rotations\n", start_pos + 8.0f);
    
    // Exit after move is complete
    exit(0);
}

void loop() {
    // Not used
}