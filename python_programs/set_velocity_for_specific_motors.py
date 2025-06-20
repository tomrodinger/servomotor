#!/usr/bin/env python3

import time
import servomotor

RUN_TIME = 20.0

# Motor configurations - (unique_id, alias, velocity)
MOTOR_CONFIGS = [
    ("1C1299E12A574E11", 9, 251.37),
    ("0485862C15439E76", 3, 758.70),
    ("4E6DF10C1A7363D9", 12, 457.92),
    ("22561F930FC5C2EE", 18, 979.89),
    ("6AB18E0B0C273C19", 15, 208.15),
    ("68BEABE716C9EEA1", 10, 468.48),
    ("1B3AAE4D1E45272D", 8, 508.82),
    ("2D0754F9716E3BA0", 5, 740.57)
]

MOTOR_CONFIGS = [
    ("23C6EBDA224305E1", 15, 1085.34 ),
    ("1A81BC5365D6FC59", 11, 239.29  ),
    ("44F625536977949A", 24, 473.91  ),
    ("5C0F719F0F0880B4", 21, -1301.35),
    ("142D97F0650801F1", 13, 453.33  ),
    ("6E086E6B2548D53E", 7 , 662.21  ),
    ("3E87F47B43F1B807", 17, -1304.77),
    ("653F822D1A15AB73", 6 , -410.98 ),
]

MOTOR_CONFIGS = [
#    ("3E87F47B43F1B807", 17,  808.68 ),
#    ("4284A747771802FF", 18,  -1207.02 ),
#    ("0A29A21244978F1E", 10,  -621.30 ),
#    ("3A48E4A822981957", 19,  79.76 ),
#    ("63BDE0A9538877A0", 9 , -149.31 ),
#    ("0C3908303ABBB36E", 5 , -1526.61 ),
#    ("44F625536977949A", 24,  -936.56 ),
    ("034B7B4B59DF0C2C", 4 , -478.25 ),
]

def main():
    # Initialize communication with broadcast address (255)
    motor255 = servomotor.M3(255, time_unit="seconds", position_unit="degrees", 
                            velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                            current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")
    servomotor.open_serial_port()

    # Initialize motor instance for individual control
    motor = servomotor.M3(255, time_unit="seconds", position_unit="degrees", 
                         velocity_unit="degrees_per_second", acceleration_unit="degrees_per_second_squared", 
                         current_unit="milliamps", voltage_unit="volts", temperature_unit="celsius")

    try:
        # Reset the system
        print("Resetting the system...")
        motor255.system_reset()
        time.sleep(1.5)

        # Enable and set velocities for each motor
        for unique_id, alias, velocity in MOTOR_CONFIGS:
            print(f"Moving device {unique_id} with alias {alias} (0x{alias:02x}) at {velocity:.2f} degrees/sec for 20 seconds...")
            motor.use_this_alias_or_unique_id(alias)
            motor.enable_mosfets()
            motor.set_maximum_motor_current(390, 390)  # Set to same max current as stress test
            motor.move_with_velocity(velocity, RUN_TIME)
            motor.move_with_velocity(0, 0.01)  # Stop over 0.01 seconds

        # Wait for movements to complete
        time.sleep(RUN_TIME + 0.5)

    finally:
        # Disable all MOSFETs using broadcast
        motor255.disable_mosfets()
        time.sleep(0.2)

if __name__ == "__main__":
    main()
