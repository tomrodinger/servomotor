#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
from datetime import datetime

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribe to all statistics topics
    client.subscribe("servomotor/statistics/#")

def on_message(client, userdata, msg):
    try:
        # Parse the JSON message
        data = json.loads(msg.payload.decode())
        
        # Print a timestamp
        print("\n" + "="*100)
        print(f"Message received at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} on topic {msg.topic}")
        print("="*100)
        
        # Print test duration and iterations
        print(f"\nTest running for: {data['test_duration']} seconds")
        print(f"Total movement iterations: {data['total_iterations']}")
        print(f"Total iterations with errors: {data['iterations_with_errors']}")
        
        if data['total_iterations'] > 0:
            print(f"Percentage of iterations with errors: {100 * data['iterations_with_errors'] / data['total_iterations']:.2f}%")
        
        print(f"Total unique devices detected: {data['total_devices']}")
        
        # Print histogram
        print("\nHistogram of number of devices detected:")
        histogram = data['devices_histogram']
        max_devices = max([int(k) for k in histogram.keys()]) if histogram else 0
        for i in range(max_devices + 1):
            count = histogram.get(str(i), 0)
            bar = '#' * min(count, 50)  # Limit bar length to 50 characters
            print(f"{i:2d} devices: {bar} ({count})")
        
        # Print per-device statistics
        for device in data['devices']:
            print(f"\nDevice {device['unique_id']} (Alias: {device['alias']}):")
            print(f"   Total fatal errors: {device['total_fatal_errors']}")
            if device['error_counts']:
                print("      Fatal error breakdown:")
                for error_code, count in device['error_counts'].items():
                    print(f"      Error code {error_code}: {count} occurrences")
        
        # Print total error counts
        print("\nSum over error types:")
        for error_code, count in data['total_error_counts'].items():
            print(f"   Error code {error_code}: {count} total occurrences")
            
    except json.JSONDecodeError:
        print("Error: Received invalid JSON data")
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print("Connecting to broker.hivemq.com...")
    client.connect("broker.hivemq.com", 1883, 60)

    print("Waiting for messages... (Press Ctrl+C to exit)")
    client.loop_forever()

if __name__ == "__main__":
    main()
