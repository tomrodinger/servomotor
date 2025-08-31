import servomotor
import time

# Connect to the servomotor
motor = servomotor.M3(port='/dev/ttyUSB0')

# Enable the mosfets (power on the motor)
motor.enable_mosfets()

# Wait for motor to be ready
time.sleep(0.5)

# Perform a trapezoid move
# Move 10000 steps with acceleration 1000, velocity 5000
motor.trapezoid_move(
    position=10000,  # Target position in steps
    velocity=5000,   # Maximum velocity in steps/second
    acceleration=1000 # Acceleration in steps/second^2
)

# Wait for move to complete
motor.wait_for_move_complete()

# Move back to starting position
motor.trapezoid_move(
    position=0,
    velocity=5000,
    acceleration=1000
)

# Wait and disable mosfets
motor.wait_for_move_complete()
motor.disable_mosfets()

# Close connection
motor.close()