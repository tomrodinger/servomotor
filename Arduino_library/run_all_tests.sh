#!/bin/bash

# Default values
SERIAL_PORT="/dev/ttys014"
DEVICE_ID="X"  # Default to alias 'X'
UNIQUE_ID="0123456789ABCDEF"  # Default unique ID
RUN_BOTH_MODES=true  # Run tests in both alias and unique ID modes

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to display help
show_help() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -p, --port PORT       Serial port to use (default: /dev/ttys014)"
    echo "  -a, --alias ID        Alias to use (default: X)"
    echo "                        - Alias can be a single character or number 0-251"
    echo "  -u, --unique-id ID    Unique ID to use (default: 0123456789ABCDEF)"
    echo "                        - Unique ID must be a 16-character hex number"
    echo "  -s, --single-mode     Run tests only in the mode specified by -a or -u"
    echo "                        - By default, tests run in both alias and unique ID modes"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 -p /dev/ttys014 -a X -u 0123456789ABCDEF  # Run in both modes"
    echo "  $0 -p /dev/ttys014 -a X -s                   # Run only with alias"
    echo "  $0 -p /dev/ttys014 -u 0123456789ABCDEF -s    # Run only with unique ID"
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
        -u|--unique-id)
            UNIQUE_ID="$2"
            shift 2
            ;;
        -s|--single-mode)
            RUN_BOTH_MODES=false
            shift
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

# Validate alias
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

# Validate unique ID
if [ ${#UNIQUE_ID} -ne 16 ] || ! [[ "$UNIQUE_ID" =~ ^[0-9A-Fa-f]{16}$ ]]; then
    echo -e "${RED}Error: Unique ID must be a 16-character hex number${NC}"
    exit 1
fi

# Build all tests first
echo "Building all tests..."
./build_tests.sh

# Initialize counters
TOTAL=0
PASSED=0
FAILED=0

# Function to run tests with a specific device ID
run_tests_with_id() {
    local id=$1
    local mode=$2
    local local_passed=0
    local local_failed=0
    local local_total=0

    echo -e "\nRunning tests in $mode mode..."
    echo "===================="
    echo "Serial Port: $SERIAL_PORT"
    echo "Device ID: $id"
    echo "Addressing Mode: $mode"
    echo "===================="

    for test in test_*; do
        if [ -x "$test" ] && [ "$test" != "test_framework.cpp" ] && [ "$test" != "test_framework.h" ]; then
            echo -e "\nRunning $test with $mode addressing..."
            if [ "$test" = "test_unit_conversions" ]; then
                # test_unit_conversions doesn't need serial port
                ./"$test"
            else
                # Other tests need serial port and device identifier
                ./"$test" "$SERIAL_PORT" "$id"
            fi
            
            if [ $? -eq 0 ]; then
                ((local_passed++))
                ((PASSED++))
                echo -e "${GREEN}$test PASSED${NC}"
            else
                ((local_failed++))
                ((FAILED++))
                echo -e "${RED}$test FAILED${NC}"
            fi
            ((local_total++))
            ((TOTAL++))
        fi
    done

    echo -e "\n$mode Mode Summary"
    echo "============"
    echo -e "Passed: ${GREEN}$local_passed${NC}"
    echo -e "Failed: ${RED}$local_failed${NC}"
    echo "Total:  $local_total"
}

# Run tests based on mode
if [ "$RUN_BOTH_MODES" = true ]; then
    # Run tests with alias
    run_tests_with_id "$DEVICE_ID" "alias"
    
    # Run tests with unique ID
    run_tests_with_id "$UNIQUE_ID" "unique"
else
    # Determine which mode to run based on which parameter was explicitly set
    if [ "$1" = "-u" ] || [ "$1" = "--unique-id" ]; then
        run_tests_with_id "$UNIQUE_ID" "unique"
    else
        run_tests_with_id "$DEVICE_ID" "alias"
    fi
fi

# Print final summary
echo -e "\nTest Summary"
echo "============"
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo "Total:  $TOTAL"

# Exit with failure if any test failed
[ $FAILED -eq 0 ]
