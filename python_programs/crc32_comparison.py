#!/usr/bin/env python3

import binascii
import zlib
import os
import struct
import random
import time
import statistics

def compare_crc32(data):
    """Compare CRC32 implementations from binascii and zlib modules."""
    binascii_crc = binascii.crc32(data)
    zlib_crc = zlib.crc32(data)
    
    match = binascii_crc == zlib_crc
    
    print(f"Data: {data[:20]}{'...' if len(data) > 20 else ''}")
    print(f"binascii.crc32: 0x{binascii_crc:08X} ({binascii_crc})")
    print(f"zlib.crc32:     0x{zlib_crc:08X} ({zlib_crc})")
    print(f"Match: {match}")
    print("-" * 60)
    
    return match

def benchmark_crc32():
    """Benchmark binascii.crc32 vs zlib.crc32 performance."""
    print("\nBENCHMARK: binascii.crc32() vs zlib.crc32()")
    print("-" * 60)
    
    # Generate 1000 random data chunks of 32 bytes each
    print("Generating 1000 random data chunks of 32 bytes each...")
    data_chunks = [os.urandom(32) for _ in range(1000)]
    
    # Benchmark binascii.crc32
    print("Benchmarking binascii.crc32...")
    binascii_times = []
    for _ in range(5):  # Run multiple times to get a stable average
        start_time = time.time()
        for chunk in data_chunks:
            binascii.crc32(chunk)
        end_time = time.time()
        elapsed_microseconds = (end_time - start_time) * 1_000_000
        binascii_times.append(elapsed_microseconds)
    
    # Benchmark zlib.crc32
    print("Benchmarking zlib.crc32...")
    zlib_times = []
    for _ in range(5):  # Run multiple times to get a stable average
        start_time = time.time()
        for chunk in data_chunks:
            zlib.crc32(chunk)
        end_time = time.time()
        elapsed_microseconds = (end_time - start_time) * 1_000_000
        zlib_times.append(elapsed_microseconds)
    
    # Calculate statistics
    binascii_avg = statistics.mean(binascii_times)
    zlib_avg = statistics.mean(zlib_times)
    
    # Print results
    print("\nResults:")
    print(f"binascii.crc32: {binascii_avg:.2f} microseconds for 1000 calculations")
    print(f"zlib.crc32:     {zlib_avg:.2f} microseconds for 1000 calculations")
    
    # Determine which is faster
    if binascii_avg < zlib_avg:
        ratio = zlib_avg / binascii_avg
        print(f"\nVerdict: binascii.crc32 is faster by a factor of {ratio:.2f}")
    elif zlib_avg < binascii_avg:
        ratio = binascii_avg / zlib_avg
        print(f"\nVerdict: zlib.crc32 is faster by a factor of {ratio:.2f}")
    else:
        print("\nVerdict: Both implementations have identical performance")

def main():
    print("Comparing binascii.crc32() and zlib.crc32() implementations\n")
    
    # Test case 1: Empty data
    print("Test case 1: Empty data")
    compare_crc32(b'')
    
    # Test case 2: Simple string
    print("Test case 2: Simple string")
    compare_crc32(b'Hello, World!')
    
    # Test case 3: Binary data
    print("Test case 3: Binary data")
    binary_data = struct.pack('<IIII', 0x12345678, 0xAABBCCDD, 0x87654321, 0xDEADBEEF)
    compare_crc32(binary_data)
    
    # Test case 4: Random data (small)
    print("Test case 4: Random data (small)")
    random_data_small = os.urandom(100)
    compare_crc32(random_data_small)
    
    # Test case 5: Random data (large)
    print("Test case 5: Random data (large)")
    random_data_large = os.urandom(10000)
    compare_crc32(random_data_large)
    
    # Test case 6: Incremental CRC calculation
    print("Test case 6: Incremental CRC calculation")
    data1 = b'First part of the data'
    data2 = b'Second part of the data'
    
    # Combined data
    combined_data = data1 + data2
    
    # Direct calculation on combined data
    binascii_combined = binascii.crc32(combined_data)
    zlib_combined = zlib.crc32(combined_data)
    
    # Incremental calculation
    binascii_incremental = binascii.crc32(data2, binascii.crc32(data1))
    zlib_incremental = zlib.crc32(data2, zlib.crc32(data1))
    
    print(f"Combined data: {combined_data}")
    print(f"binascii direct:      0x{binascii_combined:08X}")
    print(f"binascii incremental: 0x{binascii_incremental:08X}")
    print(f"zlib direct:          0x{zlib_combined:08X}")
    print(f"zlib incremental:     0x{zlib_incremental:08X}")
    print(f"All match: {binascii_combined == binascii_incremental == zlib_combined == zlib_incremental}")
    
    # Run benchmark
    benchmark_crc32()
    
    # Summary
    print("\nSUMMARY:")
    print("The binascii.crc32() and zlib.crc32() functions produce identical results.")
    print("They both implement the same CRC-32 algorithm and can be used interchangeably.")

if __name__ == "__main__":
    main()
