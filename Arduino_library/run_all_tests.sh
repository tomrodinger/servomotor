#!/bin/bash

# Default values
SERIAL_PORT="/dev/ttys014"
DEVICE_ID="X"  # Default to alias 'X'

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to display help
show_help() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -p, --port PORT       Serial port to use (default: /dev/ttys014)"
    echo "  -a, --alias ID        Device identifier:"
    echo "                        - If ID is 4 or more characters, it is treated as a unique ID"
    echo "                        - If ID is less than 4 characters, it is treated as an alias"
    echo "                        - Alias can be a single character or number 0-251"
    echo "                        - Unique ID must be a 16-character hex number"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 -p /dev/ttys014 -a X          # Use alias 'X'"
    echo "  $0 -p /dev/ttys014 -a 10         # Use numeric alias 10"
    echo "  $0 -p /dev/ttys014 -a 0123456789ABCDEF  # Use unique ID"
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        -a|--alias)
            DEVICE_ID="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            ;;
    esac
done

# Validate device identifier
if [ ${#DEVICE_ID} -ge 4 ]; then
    # Validate unique ID format (16-character hex)
    if [ ${#DEVICE_ID} -ne 16 ] || ! [[ "$DEVICE_ID" =~ ^[0-9A-Fa-f]{16}$ ]]; then
        echo -e "${RED}Error: Unique ID must be a 16-character hex number${NC}"
        exit 1
    fi
else
    # Validate alias format
    if [ ${#DEVICE_ID} -eq 1 ] && [[ "$DEVICE_ID" =~ [a-zA-Z0-9] ]]; then
        # Valid single character alias
        :
    elif [[ "$DEVICE_ID" =~ ^[0-9]+$ ]]; then
        # Check if it's a valid number
        if [ "$DEVICE_ID" -lt 0 ] || [ "$DEVICE_ID" -gt 251 ]; then
            echo -e "${RED}Error: Alias number must be between 0 and 251${NC}"
            exit 1
        fi
    else
        echo -e "${RED}Error: Alias must be a single character or number 0-251${NC}"
        exit 1
    fi
fi

# Build all tests first
echo "Building all tests..."
./build_tests.sh

# Initialize counters
TOTAL=0
PASSED=0
FAILED=0

# Run all test_* executables
echo -e "\nRunning all tests..."
echo "===================="
echo "Serial Port: $SERIAL_PORT"
echo "Device ID: $DEVICE_ID"
echo "Addressing Mode: $([ ${#DEVICE_ID} -ge 4 ] && echo "unique" || echo "alias")"
echo "===================="

for test in test_*; do
    if [ -x "$test" ] && [ "$test" != "test_framework.cpp" ] && [ "$test" != "test_framework.h" ]; then
        echo -e "\nRunning $test..."
        if [ "$test" = "test_unit_conversions" ]; then
            # test_unit_conversions doesn't need serial port
            ./"$test"
        else
            # Other tests need serial port and device identifier
            ./"$test" "$SERIAL_PORT" "$DEVICE_ID"
        fi
        
        if [ $? -eq 0 ]; then
            ((PASSED++))
            echo -e "${GREEN}$test PASSED${NC}"
        else
            ((FAILED++))
            echo -e "${RED}$test FAILED${NC}"
        fi
        ((TOTAL++))
    fi
done

# Print final summary
echo -e "\nTest Summary"
echo "============"
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo "Total:  $TOTAL"

# Exit with failure if any test failed
[ $FAILED -eq 0 ]
