#!/usr/bin/env python3
"""
Ping Blast Test Program

This script sends ping commands as rapidly as possible to trigger overrun errors.
It sends pings to alias 255 (broadcast) so devices won't respond, allowing
maximum transmission speed. The program tracks total pings sent and bytes per second.
"""

import argparse
import sys
import time
import threading
import servomotor
from servomotor import M3


class PingBlaster:
    def __init__(self, duration, verbose=False):
        self.duration = duration
        self.verbose = verbose
        self.ping_count = 0
        self.start_time = None
        self.running = False
        self.stats_lock = threading.Lock()
        
        # Ping payload: "0123456789" as specified
        self.ping_payload = "0123456789"
        
        # Calculate bytes per ping (rough estimate)
        # Each ping command has overhead + payload
        # Rough estimate: command header (3 bytes) + payload (10 bytes) + CRC (~4 bytes) = ~17 bytes
        self.bytes_per_ping = 17
        
    def print_stats(self):
        """Print current statistics"""
        with self.stats_lock:
            if self.start_time is None:
                return
                
            elapsed_time = time.time() - self.start_time
            if elapsed_time > 0:
                pings_per_second = self.ping_count / elapsed_time
                bytes_per_second = (self.ping_count * self.bytes_per_ping) / elapsed_time
                
                print(f"Time: {elapsed_time:.1f}s | "
                      f"Total Pings: {self.ping_count} | "
                      f"Pings/sec: {pings_per_second:.1f} | "
                      f"Bytes/sec: {bytes_per_second:.0f}")
    
    def stats_printer_thread(self):
        """Thread function to print stats every 5 seconds"""
        while self.running:
            time.sleep(5.0)
            if self.running:
                self.print_stats()
    
    def blast_pings(self):
        """Send pings as fast as possible to alias 255"""
        # Create motor object with alias 255 (broadcast address)
        # Devices won't respond to this address, allowing maximum speed
        motor = M3(255, verbose=self.verbose)
        
        print(f"Starting ping blast test:")
        print(f"  Target: Alias 255 (broadcast - no responses expected)")
        print(f"  Payload: '{self.ping_payload}'")
        print(f"  Duration: {self.duration} seconds")
        print(f"  Stats will be printed every 5 seconds")
        print()
        
        # Start stats printing thread
        self.running = True
        stats_thread = threading.Thread(target=self.stats_printer_thread, daemon=True)
        stats_thread.start()
        
        # Record start time
        with self.stats_lock:
            self.start_time = time.time()
        
        try:
            # Blast pings as fast as possible
            while True:
                current_time = time.time()
                if current_time - self.start_time >= self.duration:
                    break
                
                try:
                    # Send ping command - we expect this to timeout since alias 255 won't respond
                    # We catch the timeout and continue to send as fast as possible
                    motor.ping(self.ping_payload, verbose=self.verbose)
                except Exception as e:
                    # Expected - devices at alias 255 won't respond
                    if self.verbose:
                        print(f"Ping timeout (expected): {e}")
                
                with self.stats_lock:
                    self.ping_count += 1
                
                # No delay - send as fast as possible
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        finally:
            self.running = False
            
        # Final statistics
        final_time = time.time()
        total_elapsed = final_time - self.start_time
        
        print(f"\nPing blast test completed!")
        print(f"Total duration: {total_elapsed:.2f} seconds")
        print(f"Total pings sent: {self.ping_count}")
        
        if total_elapsed > 0:
            pings_per_second = self.ping_count / total_elapsed
            bytes_per_second = (self.ping_count * self.bytes_per_ping) / total_elapsed
            
            print(f"Average pings per second: {pings_per_second:.1f}")
            print(f"Average bytes per second: {bytes_per_second:.0f}")
        
        return self.ping_count


def main():
    parser = argparse.ArgumentParser(description='Blast ping commands to trigger overrun errors')
    parser.add_argument('-p', '--port', help='Serial port device name (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-P', '--PORT', action='store_true', 
                       help='Show available ports and prompt for selection')
    parser.add_argument('-d', '--duration', type=float, default=30.0,
                       help='Duration to send pings in seconds (default: 30.0)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Validate duration
    if args.duration <= 0:
        print(f"Error: Duration must be positive, got {args.duration}", file=sys.stderr)
        sys.exit(1)
    
    # Set up serial port
    servomotor.set_serial_port_from_args(args)
    
    serial_port_opened = False
    
    try:
        # Open serial port
        servomotor.open_serial_port()
        serial_port_opened = True
        
        print("Ping Blast Test - Overrun Error Generator")
        print("=" * 50)
        
        # Create and run ping blaster
        blaster = PingBlaster(args.duration, args.verbose)
        total_pings = blaster.blast_pings()
        
        if total_pings > 0:
            print(f"\nSUCCESS: Sent {total_pings} ping commands")
            return 0
        else:
            print(f"\nFAILED: No pings were sent")
            return 1
            
    except Exception as e:
        print(f"Error during test: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1
    
    finally:
        # Close serial port only if it was successfully opened
        if serial_port_opened:
            try:
                servomotor.close_serial_port()
            except:
                pass  # Ignore errors when closing


if __name__ == "__main__":
    sys.exit(main())