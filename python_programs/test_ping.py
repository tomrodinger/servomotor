#!/usr/bin/env python3
import servomotor
import time

N_PINGS = 1000


def generate_random_10_byte_string():
    import random
    return bytes([random.randint(0, 255) for i in range(10)])


motorX = servomotor.M3("X", motor_type="M3", time_unit="seconds", position_unit="degrees", velocity_unit="degrees/s", acceleration_unit="degree/s^2", current_unit="mA", voltage_unit="V", temperature_unit="C", verbose=True)

servomotor.open_serial_port()
motorX.system_reset()
time.sleep(1.2)

for i in range(N_PINGS):
    print("Ping", i)
    random_10_byte_string = generate_random_10_byte_string()
    response = motorX.ping(random_10_byte_string)
    # Make sure the response is the same as the random 10 byte string
    if response != random_10_byte_string:
        print("Ping failed")
        print("Sent:", random_10_byte_string)
        print("Received:", response)
        print("FAILED")
        break


servomotor.close_serial_port()
del motorX

print("PASSED")
