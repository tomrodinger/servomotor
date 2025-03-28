#!/usr/bin/env python3

import zlib
import sys

def calculate_crc32(data):
    """Calculate CRC32 checksum for a byte array"""
    return zlib.crc32(data) & 0xffffffff

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_file> <output_file>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    results = []
    
    with open(input_file, 'r') as f:
        for line in f:
            # Skip comments
            if line.startswith('#'):
                continue
                
            line = line.strip()
            
            # Handle empty line as empty buffer test case
            if not line:
                print("Testing empty buffer (0 bytes)")
                byte_array = bytearray()
                crc32_value = calculate_crc32(byte_array)
                results.append(f"[empty buffer] => {crc32_value:08X}")
            else:
                # Parse hex bytes
                try:
                    # Remove any comments (text after #)
                    if '#' in line:
                        line = line.split('#')[0].strip()
                    
                    # Skip if line is empty after removing comments
                    if not line:
                        continue
                        
                    # Split by spaces and convert each hex string to a byte
                    hex_values = line.split()
                    if not hex_values:
                        continue
                        
                    byte_array = bytearray([int(x, 16) for x in hex_values])
                    
                    # Calculate CRC32
                    crc32_value = calculate_crc32(byte_array)
                    
                    # Format the result
                    hex_string = ' '.join([f"{b:02X}" for b in byte_array])
                    results.append(f"{hex_string} => {crc32_value:08X}")
                except ValueError as e:
                    print(f"Error parsing line: {line}")
                    print(f"Error details: {e}")
    
    # Write results to output file
    with open(output_file, 'w') as f:
        for result in results:
            f.write(f"{result}\n")

if __name__ == "__main__":
    main()
