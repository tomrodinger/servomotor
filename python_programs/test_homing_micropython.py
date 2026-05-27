#!/usr/bin/env python3
"""
Homing test for MicroPython on ESP32-S3
Tests homing command and position reading
"""

import servomotor
import time

# Configuration
ALIAS = ord('X')
VERBOSE = 2

print("=" * 50)
print("ESP32-S3 Homing Test")
print("=" * 50)

try:
    # Step 1: Open serial port
    print("\n1. Opening serial port...")
    servomotor.open_serial_port()
    print("   Serial port opened")
    
    # Step 2: Create motor instance
    print("\n2. Creating motor instance...")
    m = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations", verbose=VERBOSE)
    print("   Motor instance created")
    
    # Step 3: Reset the motor
    print("\n3. Resetting motor...")
    response = m.system_reset(verbose=VERBOSE)
    print(f"   Reset complete")
    
    # Step 4: Wait 1 second for device to reset (CRITICAL - device needs time to reset)
    print("\n4. Waiting 1 second for device to reset...")
    time.sleep(1.0)
    print("   Done waiting")
    
    # Step 5: Enable MOSFETs
    print("\n5. Enabling MOSFETs...")
    m.enable_mosfets(verbose=VERBOSE)
    print("   MOSFETs enabled")
    
    # Step 6: Wait 0.3 seconds
    print("\n6. Waiting 0.3 seconds...")
    time.sleep(0.3)
    print("   Done waiting")
    
    # Step 7: Set PID constants
    print("\n7. Setting PID constants...")
    m.set_pid_constants(3000, 2, 175000, verbose=VERBOSE)
    print("   PID constants set")
    
    # Step 8: Go to closed loop mode (required before homing)
    print("\n8. Going to closed loop mode...")
    m.go_to_closed_loop(verbose=VERBOSE)
    print("   Closed loop mode enabled")
    
    # Step 9: Wait 0.2 seconds
    print("\n9. Waiting 0.2 seconds...")
    time.sleep(0.2)
    print("   Done waiting")
    
    # Step 10: Read initial position
    print("\n10. Reading initial position...")
    commanded1, sensed1, external1 = m.get_comprehensive_position(verbose=VERBOSE)
    print(f"    Initial position:")
    print(f"      Commanded: {commanded1}")
    print(f"      Sensed: {sensed1}")
    print(f"      External: {external1}")
    
    # Step 11: Set current to 50
    print("\n11. Setting motor current to 50...")
    m.set_maximum_motor_current(50, 50, verbose=VERBOSE)
    print("    Current set to 50")
    
    # Step 12: Run homing command (direction 1, max time 3 seconds)
    print("\n12. Running homing command (direction=1, max_time=3s)...")
    m.homing(1, 3, verbose=VERBOSE)
    print("    Homing command sent")
    
    # Step 13: Wait for homing to complete
    print("\n13. Waiting for homing to complete...")
    max_wait = 5.0  # Maximum 5 seconds
    start_time = time.time()
    homing_complete = False
    
    while (time.time() - start_time) < max_wait:
        status_flags, error_code = m.get_status(verbose=VERBOSE)
        # Bit 4 indicates homing in progress
        if (status_flags & (1 << 4)) == 0:
            homing_complete = True
            print("   Homing complete!")
            break
        time.sleep(0.1)
    
    if not homing_complete:
        print("    WARNING: Homing did not complete within timeout")
    
    # Step 14: Read final position
    print("\n14. Reading final position...")
    commanded2, sensed2, external2 = m.get_comprehensive_position(verbose=VERBOSE)
    print(f"    Final position:")
    print(f"      Commanded: {commanded2}")
    print(f"      Sensed: {sensed2}")
    print(f"      External: {external2}")
    
    # Step 15: Calculate distance moved
    print("\n15. Calculating distance moved...")
    commanded_distance = commanded2 - commanded1
    sensed_distance = sensed2 - sensed1
    external_distance = external2 - external1
    
    print(f"    Distance moved:")
    print(f"      Commanded: {commanded_distance} rotations")
    print(f"      Sensed: {sensed_distance} rotations")
    print(f"      External: {external_distance} rotations")
    
    # Step 16: Disable MOSFETs
    print("\n16. Disabling MOSFETs...")
    m.disable_mosfets(verbose=VERBOSE)
    print("    MOSFETs disabled")
    
    # Step 17: Wait 0.05 seconds
    print("\n17. Waiting 0.05 seconds...")
    time.sleep(0.05)
    print("    Done waiting")
    
    # Step 18: Close serial port
    print("\n18. Closing serial port...")
    servomotor.close_serial_port()
    print("    Serial port closed")
    
    print("\n" + "=" * 50)
    print("Test completed")
    print("=" * 50)
    
except Exception as e:
    print("\n" + "=" * 50)
    print("ERROR during test")
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
    
    raise SystemExit(1)