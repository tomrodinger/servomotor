#!/usr/bin/env python3
"""Read the configured raw PID gains (requires firmware 0.16.0.0 or later)."""
import servomotor

ALIAS = 'X'                            # Device alias; change if needed.
SERIAL_PORT = "MENU"                  # Or specify your USB-RS485 serial port.

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, verbose=0)
try:
    # Reads the device without enabling outputs or changing controller state.
    # Values are raw user gains; working coefficients may be limited internally.
    p, i, d = motor.get_pid_constants()
    print(f"Configured PID constants: P={p} I={i} D={d}")
finally:
    servomotor.close_serial_port()
