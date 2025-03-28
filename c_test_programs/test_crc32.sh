#!/bin/bash

# Set working directory
cd "$(dirname "$0")"

# Compile the C test program
echo "Compiling C test program..."
gcc -o test_crc32_c test_crc32_c.c

if [ $? -ne 0 ]; then
    echo "Error: Failed to compile C test program"
    exit 1
fi

# Make Python script executable
chmod +x test_crc32_python.py

# Define input and output files
INPUT_FILE="test_CRC32_data.txt"
C_OUTPUT="c_crc32_results.txt"
PYTHON_OUTPUT="python_crc32_results.txt"

# Run the C test program
echo "Running C CRC32 test..."
./test_crc32_c "$INPUT_FILE" "$C_OUTPUT"

# Run the Python test program
echo "Running Python CRC32 test..."
./test_crc32_python.py "$INPUT_FILE" "$PYTHON_OUTPUT"

# Compare the results
echo "Comparing results..."
if diff -q "$C_OUTPUT" "$PYTHON_OUTPUT" >/dev/null; then
    echo "SUCCESS: C and Python CRC32 calculations match!"
else
    echo "ERROR: C and Python CRC32 calculations do not match!"
    echo "Differences:"
    diff "$C_OUTPUT" "$PYTHON_OUTPUT"
fi

echo "Test complete. Results are in $C_OUTPUT and $PYTHON_OUTPUT"
