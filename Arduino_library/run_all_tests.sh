#!/bin/bash

# Serial port to use for tests
SERIAL_PORT="/dev/ttys107"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

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

for test in test_*; do
    if [ -x "$test" ] && [ "$test" != "test_framework.cpp" ] && [ "$test" != "test_framework.h" ]; then
        echo -e "\nRunning $test..."
        if [ "$test" = "test_unit_conversions" ]; then
            # test_unit_conversions doesn't need serial port
            ./"$test"
        else
            # Other tests need serial port
            ./"$test" "$SERIAL_PORT"
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
