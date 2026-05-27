#!/usr/bin/env python3
"""
Simple trapezoid move test for MicroPython on ESP32-S3
Tests basic motor communication and movement
"""

import servomotor
import time

# Configuration
ALIAS = ord('X')  # Motor alias
HALF_ROTATION = 0.5  # Half a rotation
MOVE_TIME = 0.5  # Time for the move in seconds
VERBOSE = 2  # Verbose output level

print("=" * 50)
print("ESP32-S3 MicroPython Motor Test")
print("=" * 50)

try:
    # Step 1: Open serial port
    print("\n1. Opening serial port...")
    servomotor.open_serial_port()
    print("   Serial port opened successfully")
    
    # Step 2: Create motor instance
    print("\n2. Creating motor instance...")
    m = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations", verbose=VERBOSE)
    print("   Motor instance created")
    
    # Step 3: Reset the motor
    print("\n3. Resetting motor...")
    response = m.system_reset(verbose=VERBOSE)
    print(f"   Reset response: {response}")
    
    # Step 4: Wait 1 second
    print("\n4. Waiting 1 second...")
    time.sleep(1.0)
    print("   Done waiting")
    
    # Step 5: Enable MOSFETs
    print("\n5. Enabling MOSFETs...")
    m.enable_mosfets(verbose=VERBOSE)
    print("   MOSFETs enabled")
    
    # Step 6: Perform trapezoid move (half rotation over 0.5 seconds)
    print(f"\n6. Moving {HALF_ROTATION} rotations over {MOVE_TIME} seconds...")
    m.trapezoid_move(HALF_ROTATION, MOVE_TIME, verbose=VERBOSE)
    print("   Move command sent")
    
    # Step 7: Wait for move to complete (0.6 seconds)
    print("\n7. Waiting 0.6 seconds for move to complete...")
    time.sleep(0.6)
    print("   Move should be complete")
    
    # Step 8: Disable MOSFETs
    print("\n8. Disabling MOSFETs...")
    m.disable_mosfets(verbose=VERBOSE)
    print("   MOSFETs disabled")
    
    # Step 9: Wait 0.05 seconds
    print("\n9. Waiting 0.05 seconds...")
    time.sleep(0.05)
    print("   Done waiting")
    
    # Step 10: Close serial port
    print("\n10. Closing serial port...")
    servomotor.close_serial_port()
    print("    Serial port closed")
    
    print("\n" + "=" * 50)
    print("Test completed successfully!")
    print("=" * 50)
    
except Exception as e:
    print("\n" + "=" * 50)
    print("ERROR during test!")
    print("=" * 50)
    print(f"Exception: {e}")
    
    # Print detailed traceback
    import sys
    sys.print_exception(e)
    
    # Try to close serial port
    try:
        servomotor.close_serial_port()
        print("\nSerial port closed after error")
    except:
        pass
    
    print("\nTest failed!")
    raise SystemExit(1)