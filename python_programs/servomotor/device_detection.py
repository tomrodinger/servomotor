#!/usr/bin/env python3

"""
Device Detection Module for Servomotor RS485 Bus

This module provides functions for detecting devices on the RS485 bus and resetting them.
Functions will be copied from existing implementations and minimally edited to work as a module.
"""

import time
from typing import List, Tuple
import servomotor

DONT_GO_TO_BOOTlOADER_RESET_TIME = 1.5

class Device:
    def __init__(self, unique_id, alias):
        self.unique_id = unique_id
        self.alias = alias

def detect_devices_iteratively(n_detections: int = 3, verbose: bool = False) -> List[Device]:
    """
    Detect devices multiple times and combine results.
    
    Args:
        n_detections: Number of detection iterations (default: 3)
        verbose: Enable verbose output (default: False)
    
    Returns:
        List of Device objects
    """
    # Let's detect all devices (possible multiple times) and store the data (unique_id and alias) in a dictionary
    motor255 = servomotor.M3(alias_or_unique_id=255, verbose=verbose)
    device_dict = {}
    successful_detect_devices_count = 0
    detect_devices_attempt_count = 0
    min_detection_time = 1000000000.0
    while 1:
        print("* * * * * Resetting the system * * * * *")
        motor255.system_reset()
        time.sleep(DONT_GO_TO_BOOTlOADER_RESET_TIME)

        if successful_detect_devices_count >= n_detections:
            break

        print("Flushing the receive buffer")
        servomotor.flush_receive_buffer()
        print(f"Detecting devices attempt {detect_devices_attempt_count+1}/{n_detections}")
        detection_start_time = time.time()
        try:
            response = motor255.detect_devices()
            successful_detect_devices_count += 1
        except Exception as e:
            response = None
            print(f"Communication error: {e}")
        detection_time = time.time() - detection_start_time
        if (detection_time < min_detection_time):
            min_detection_time = detection_time
        print("The detection time was:", detection_time, "and the minimum detection time currently is:", min_detection_time)
        if (detection_time < 1.1):
            additional_sleep_time = 1.1 - detection_time
            print("Sleeping an additional amount of time:", additional_sleep_time)
            time.sleep(additional_sleep_time)
        if response != None:
            detect_devices_attempt_count += 1
            print("Detected devices:")
            for device in response:
                unique_id = device[0]
                alias = device[1]
                print(f"Unique ID: {unique_id:016X}, Alias: {alias}")
                if unique_id in device_dict:
                    print(f"This unique ID {unique_id:016X} is already in the device dictionary, so not adding it again")
                    if alias != device_dict[unique_id].alias:
                        print(f"Error: we discovered an inconsistency: the alias is different: {alias} vs {device_dict[unique_id].alias}")
                else:
                    new_device = Device(unique_id, alias)
                    device_dict[unique_id] = new_device
    #            else:
    #                print(f"Unique ID: {unique_id:016X}, Alias: {alias}, CRC: {crc:08X} (CHECK FAILED: computed crc: {crc32_value:08X} vs. received crc: {crc:08X})")
    del motor255
    return list(device_dict.values())
