#!/bin/bash

# Deployment script for uploading servomotor library to Raspberry Pi Pico 2
# Usage: ./deploy_to_raspberry_pi_pico_2.sh [serial_port]

# Default port for Raspberry Pi Pico 2
PORT="${1:-/dev/cu.usbmodem2101}"

echo "========================================="
echo "Deploying Servomotor Library to Raspberry Pi Pico 2"
echo "========================================="
echo "Port: $PORT"
echo ""

# Check if mpremote is available
if ! command -v mpremote &> /dev/null; then
    echo "ERROR: mpremote not found. Install with: pip install mpremote"
    exit 1
fi

echo "Step 1: Checking Raspberry Pi Pico 2 connection..."
if ! mpremote connect $PORT exec "print('Connected')"; then
    echo "ERROR: Cannot connect to Raspberry Pi Pico 2 at $PORT"
    exit 1
fi
echo "✓ Connection successful"
echo ""

echo "Step 2: Creating servomotor directory on Raspberry Pi Pico 2..."
mpremote connect $PORT exec "import os; os.mkdir('servomotor') if 'servomotor' not in os.listdir() else None"
echo "✓ Directory created"
echo ""

echo "Step 3: Uploading servomotor library files..."
echo "  - Uploading __init__.py..."
mpremote connect $PORT cp servomotor/__init__.py :servomotor/__init__.py

echo "  - Uploading platform_detect.py..."
mpremote connect $PORT cp servomotor/platform_detect.py :servomotor/platform_detect.py

echo "  - Uploading serial_abstraction.py..."
mpremote connect $PORT cp servomotor/serial_abstraction.py :servomotor/serial_abstraction.py

echo "  - Uploading platform_utils.py..."
mpremote connect $PORT cp servomotor/platform_utils.py :servomotor/platform_utils.py

echo "  - Uploading terminal_format_wrapper.py..."
mpremote connect $PORT cp servomotor/terminal_format_wrapper.py :servomotor/terminal_format_wrapper.py

echo "  - Uploading communication.py..."
mpremote connect $PORT cp servomotor/communication.py :servomotor/communication.py

echo "  - Uploading command_loader.py..."
mpremote connect $PORT cp servomotor/command_loader.py :servomotor/command_loader.py

echo "  - Uploading M3.py..."
mpremote connect $PORT cp servomotor/M3.py :servomotor/M3.py

echo "  - Uploading device_detection.py..."
mpremote connect $PORT cp servomotor/device_detection.py :servomotor/device_detection.py

echo "✓ Library files uploaded"
echo ""

echo "Step 4: Uploading JSON configuration files..."
echo "  - Uploading data_types.json..."
mpremote connect $PORT cp servomotor/data_types.json :servomotor/data_types.json

echo "  - Uploading motor_commands.json..."
mpremote connect $PORT cp servomotor/motor_commands.json :servomotor/motor_commands.json

echo "  - Uploading error_codes.json..."
mpremote connect $PORT cp servomotor/error_codes.json :servomotor/error_codes.json

echo "  - Uploading unit_conversions_M3.json..."
mpremote connect $PORT cp servomotor/unit_conversions_M3.json :servomotor/unit_conversions_M3.json

echo "✓ Configuration files uploaded"
echo ""

echo "Step 5: Uploading ball_juggling_demo.py..."
mpremote connect $PORT cp ball_juggling_demo.py :ball_juggling_demo.py
echo "✓ Demo uploaded"
echo ""

echo "Step 6: Verifying installation..."
mpremote connect $PORT exec "
import sys
try:
    import servomotor
    print('✓ servomotor module imported successfully')
    print('  Platform:', servomotor.get_platform())
    print('  Is MicroPython:', servomotor.is_micropython())
    print('  Version:', servomotor.__version__)
except Exception as e:
    print('✗ Error importing servomotor:', e)
    sys.exit(1)
"

echo ""
echo "========================================="
echo "Deployment Complete!"
echo "========================================="
echo ""
echo "To run the ball juggling demo:"
echo "  mpremote connect $PORT run ball_juggling_demo.py"
echo ""
echo "To enter REPL:"
echo "  mpremote connect $PORT repl"
echo ""