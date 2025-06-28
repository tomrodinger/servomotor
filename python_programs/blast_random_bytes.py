#!/usr/bin/env python3
"""
Random Bytes Blaster

This script sends random bytes to a specified serial port as fast as possible
for a specified duration. It tracks the total bytes sent and calculates the
average bytes per second transmission rate.
"""

import argparse
import random
import sys
import time
import threading
import serial


class RandomBytesBlaster:
    def __init__(self, port, baud, duration, verbose=False):
        self.port = port
        self.baud = baud
        self.duration = duration
        self.verbose = verbose
        self.bytes_sent = 0
        self.start_time = None
        self.running = False
        self.stats_lock = threading.Lock()
        
    def print_stats(self):
        """Print current statistics"""
        with self.stats_lock:
            if self.start_time is None:
                return
                
            elapsed_time = time.time() - self.start_time
            if elapsed_time > 0:
                bytes_per_second = self.bytes_sent / elapsed_time
                
                print(f"Time: {elapsed_time:.1f}s | "
                      f"Total Bytes: {self.bytes_sent} | "
                      f"Bytes/sec: {bytes_per_second:.0f}")
    
    def stats_printer_thread(self):
        """Thread function to print stats every 5 seconds"""
        while self.running:
            time.sleep(5.0)
            if self.running:
                self.print_stats()
    
    def blast_random_bytes(self):
        """Send random bytes as fast as possible"""
        print(f"Random Bytes Blaster")
        print("=" * 30)
        print(f"Port: {self.port}")
        print(f"Baud rate: {self.baud}")
        print(f"Duration: {self.duration} seconds")
        print(f"Stats will be printed every 5 seconds")
        print()
        
        # Start stats printing thread
        self.running = True
        stats_thread = threading.Thread(target=self.stats_printer_thread, daemon=True)
        stats_thread.start()
        
        try:
            # Open serial port
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                print(f"Successfully opened serial port: {self.port}")
                
                # Record start time
                with self.stats_lock:
                    self.start_time = time.time()
                
                # Blast random bytes as fast as possible
                while True:
                    current_time = time.time()
                    if current_time - self.start_time >= self.duration:
                        break
                    
                    # Generate random byte
                    random_byte = random.randint(0, 255)
                    
                    try:
                        # Send the byte
                        bytes_written = ser.write(bytes([random_byte]))
                        if bytes_written == 1:
                            with self.stats_lock:
                                self.bytes_sent += 1
                                
                            if self.verbose:
                                print(f"Sent byte #{self.bytes_sent}: 0x{random_byte:02x} ({random_byte})")
                                
                    except serial.SerialException as e:
                        print(f"Error writing to serial port: {e}", file=sys.stderr)
                        break
                    
                    # No delay - send as fast as possible
                    
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}", file=sys.stderr)
            return False
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.running = False
            
        # Final statistics
        final_time = time.time()
        total_elapsed = final_time - self.start_time if self.start_time else 0
        
        print(f"\nRandom bytes blast completed!")
        print(f"Total duration: {total_elapsed:.2f} seconds")
        print(f"Total bytes sent: {self.bytes_sent}")
        
        if total_elapsed > 0:
            bytes_per_second = self.bytes_sent / total_elapsed
            print(f"Average bytes per second: {bytes_per_second:.0f}")
        
        return self.bytes_sent > 0


def main():
    parser = argparse.ArgumentParser(description='Blast random bytes to a serial port')
    parser.add_argument('-p', '--port', required=True, 
                       help='Serial port to use (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=230400, 
                       help='Baud rate (default: 230400)')
    parser.add_argument('-d', '--duration', type=float, default=10.0,
                       help='Duration to send bytes in seconds (default: 10.0)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Print each byte being sent')
    
    args = parser.parse_args()
    
    # Validate duration
    if args.duration <= 0:
        print(f"Error: Duration must be positive, got {args.duration}", file=sys.stderr)
        sys.exit(1)
    
    # Validate baud rate
    if args.baud <= 0:
        print(f"Error: Baud rate must be positive, got {args.baud}", file=sys.stderr)
        sys.exit(1)
    
    try:
        # Create and run random bytes blaster
        blaster = RandomBytesBlaster(args.port, args.baud, args.duration, args.verbose)
        success = blaster.blast_random_bytes()
        
        if success:
            print(f"\nSUCCESS: Sent {blaster.bytes_sent} random bytes")
            return 0
        else:
            print(f"\nFAILED: No bytes were sent")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())