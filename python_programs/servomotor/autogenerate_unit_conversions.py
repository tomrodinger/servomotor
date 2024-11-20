#!/usr/bin/env python3

import sys
import os
import json
import math
import argparse
import importlib.util
from datetime import datetime

def import_settings(motor_type):
    """
    Dynamically import the settings file for the specified motor type.
    Returns the imported module or None if import fails.
    """
    settings_file = f"SETTINGS_{motor_type}.py"
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(current_dir))
    settings_path = os.path.join(project_root, settings_file)
    
    if not os.path.exists(settings_path):
        print(f"\nError: Settings file '{settings_file}' not found at:")
        print(f"{settings_path}")
        print("\nThis program calculates unit conversions for different motor types.")
        print("It requires a settings file that defines the following constants:")
        print("  - N_COMMUTATION_STEPS")
        print("  - N_COMMUTATION_SUB_STEPS")
        print("  - ONE_REVOLUTION_ELECTRICAL_CYCLES")
        print("\nUsage:")
        print("  python3 autogenerate_unit_conversions.py <motor_type>")
        print("\nExample:")
        print("  python3 autogenerate_unit_conversions.py M3")
        sys.exit(1)
    
    try:
        spec = importlib.util.spec_from_file_location("settings", settings_path)
        if spec is None:
            raise ImportError(f"Could not load spec for {settings_path}")
        
        settings = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(settings)
        
        # Verify required constants exist
        required_constants = ['N_COMMUTATION_STEPS', 'N_COMMUTATION_SUB_STEPS', 'ONE_REVOLUTION_ELECTRICAL_CYCLES']
        for constant in required_constants:
            if not hasattr(settings, constant):
                raise AttributeError(f"Required constant '{constant}' not found in {settings_file}")
        
        return settings
        
    except Exception as e:
        print(f"\nError loading settings file: {str(e)}")
        sys.exit(1)

def calculate_time_conversions(INTERNAL_TIME_UNIT_HZ):
    """Calculate time unit conversion factors"""
    print("\nCalculating time unit conversions...")
    
    # Milliseconds to timesteps
    print("\nMilliseconds Conversion Calculation:")
    print(f"1. INTERNAL_TIME_UNIT_HZ = {INTERNAL_TIME_UNIT_HZ:,} Hz")
    
    ms_to_sec = 1/1000
    print(f"2. 1 millisecond = {ms_to_sec} seconds")
    
    ms_factor = INTERNAL_TIME_UNIT_HZ * ms_to_sec
    print(f"3. Final value = {ms_factor:,} timesteps/millisecond")

    # Seconds to timesteps
    print("\nSeconds Conversion Calculation:")
    sec_factor = INTERNAL_TIME_UNIT_HZ
    print(f"1. Final value = {sec_factor:,} timesteps/second")

    # Minutes to timesteps
    print("\nMinutes Conversion Calculation:")
    min_to_sec = 60
    print(f"1. 1 minute = {min_to_sec} seconds")
    
    min_factor = INTERNAL_TIME_UNIT_HZ * min_to_sec
    print(f"2. Final value = {min_factor:,} timesteps/minute")

    # Store time conversion factors
    time_factors = {
        "milliseconds": ms_factor,
        "seconds": sec_factor,
        "minutes": min_factor
    }

    # Calculate time verification values
    time_verification = {
        "1000_milliseconds": 1000 * ms_factor,
        "1_second": 1 * sec_factor,
        "1_60th_minute": (1/60) * min_factor
    }

    return time_factors, time_verification

def calculate_acceleration_conversions(ONE_ROTATION_MICROSTEPS, INTERNAL_TIME_UNIT_HZ):
    """Calculate acceleration unit conversion factors"""
    print("\nCalculating acceleration unit conversions...")
    
    # Rotations/second² to internal units
    print("\nRotations/Second² Conversion Calculation:")
    print(f"1. ONE_ROTATION_MICROSTEPS = {ONE_ROTATION_MICROSTEPS:,} counts/rotation")
    print(f"2. INTERNAL_TIME_UNIT_HZ = {INTERNAL_TIME_UNIT_HZ:,} Hz")
    
    # Convert rotations/sec² to microsteps/sec²
    rot_microsteps_per_second_squared = ONE_ROTATION_MICROSTEPS
    print(f"3. Microsteps/second² = {rot_microsteps_per_second_squared:,}")
    
    # Convert to microsteps/timestep²
    rot_microsteps_per_timestep_squared = rot_microsteps_per_second_squared / (INTERNAL_TIME_UNIT_HZ * INTERNAL_TIME_UNIT_HZ)
    print(f"4. Microsteps/timestep² = {rot_microsteps_per_timestep_squared}")
    
    # Convert to internal units (multiply by 2³²)
    rot_internal_units = rot_microsteps_per_timestep_squared * (2**32)
    print(f"5. Internal units = {rot_internal_units:,}")
    
    # Convert to communication units (divide by 2⁸)
    rot_factor = rot_internal_units / (2**8)
    print(f"6. Final value = {rot_factor:,}")

    # RPM/second to internal units
    print("\nRPM/Second Conversion Calculation:")
    # Convert RPM/sec to rotations/sec²
    rpm_to_rps = 1/60
    print(f"1. 1 RPM/second = {rpm_to_rps} rotations/second²")
    
    # Use rotations/sec² conversion factor
    rpm_factor = rpm_to_rps * rot_factor
    print(f"2. Final value = {rpm_factor:,}")

    # Degrees/second² to internal units
    print("\nDegrees/Second² Conversion Calculation:")
    # Convert degrees/sec² to rotations/sec²
    deg_to_rotation = 1/360
    print(f"1. 1 degree/second² = {deg_to_rotation} rotations/second²")
    
    # Use rotations/sec² conversion factor
    deg_factor = deg_to_rotation * rot_factor
    print(f"2. Final value = {deg_factor:,}")

    # Radians/second² to internal units
    print("\nRadians/Second² Conversion Calculation:")
    # Convert radians/sec² to rotations/sec²
    rad_to_rotation = 1/(2*math.pi)
    print(f"1. 1 radian/second² = {rad_to_rotation} rotations/second²")
    
    # Use rotations/sec² conversion factor
    rad_factor = rad_to_rotation * rot_factor
    print(f"2. Final value = {rad_factor:,}")

    # Counts/second² to internal units
    print("\nCounts/Second² Conversion Calculation:")
    # Convert counts/sec² to rotations/sec²
    counts_to_rotation = 1/ONE_ROTATION_MICROSTEPS
    print(f"1. 1 count/second² = {counts_to_rotation} rotations/second²")
    
    # Use rotations/sec² conversion factor
    counts_factor = counts_to_rotation * rot_factor
    print(f"2. Final value = {counts_factor:,}")

    # Store acceleration conversion factors
    acceleration_factors = {
        "rotations_per_second_squared": rot_factor,
        "rpm_per_second": rpm_factor,
        "degrees_per_second_squared": deg_factor,
        "radians_per_second_squared": rad_factor,
        "counts_per_second_squared": counts_factor
    }

    # Calculate acceleration verification values
    acceleration_verification = {
        "1_rotation_per_second_squared": 1 * rot_factor,
        "60_rpm_per_second": 60 * rpm_factor,
        "360_degrees_per_second_squared": 360 * deg_factor,
        "2pi_radians_per_second_squared": 2 * math.pi * rad_factor,
        "one_rotation_counts_per_second_squared": ONE_ROTATION_MICROSTEPS * counts_factor
    }

    return acceleration_factors, acceleration_verification

def calculate_velocity_conversions(ONE_ROTATION_MICROSTEPS, INTERNAL_TIME_UNIT_HZ):
    """Calculate velocity unit conversion factors"""
    print("\nCalculating velocity unit conversions...")
    
    # RPM to counts_per_timestep
    print("\nRPM Conversion Calculation:")
    print(f"1. ONE_ROTATION_MICROSTEPS = {ONE_ROTATION_MICROSTEPS:,} counts/rotation")
    print(f"2. INTERNAL_TIME_UNIT_HZ = {INTERNAL_TIME_UNIT_HZ:,} Hz")
    
    rpm_to_rps = 1/60
    print(f"3. 1 RPM = {rpm_to_rps} rotations/second")
    
    rpm_counts_per_second = rpm_to_rps * ONE_ROTATION_MICROSTEPS
    print(f"4. Counts/second = {rpm_counts_per_second:,}")
    
    rpm_counts_per_timestep = rpm_counts_per_second / INTERNAL_TIME_UNIT_HZ
    print(f"5. Counts/timestep = {rpm_counts_per_timestep}")
    
    rpm_factor = rpm_counts_per_timestep * (2**20)
    print(f"6. Final value (scaled by 2^20 for integer transmission) = {rpm_factor:,}")

    # Degrees/second to counts_per_timestep
    print("\nDegrees/Second Conversion Calculation:")
    deg_to_rotation = 1/360
    print(f"1. 1 degree = {deg_to_rotation} rotations")
    
    deg_counts_per_second = deg_to_rotation * ONE_ROTATION_MICROSTEPS
    print(f"2. Counts/second = {deg_counts_per_second:,}")
    
    deg_counts_per_timestep = deg_counts_per_second / INTERNAL_TIME_UNIT_HZ
    print(f"3. Counts/timestep = {deg_counts_per_timestep}")
    
    deg_factor = deg_counts_per_timestep * (2**20)
    print(f"4. Final value (scaled by 2^20 for integer transmission) = {deg_factor:,}")

    # Radians/second to counts_per_timestep
    print("\nRadians/Second Conversion Calculation:")
    rad_to_rotation = 1/(2*math.pi)
    print(f"1. 1 radian = {rad_to_rotation} rotations")
    
    rad_counts_per_second = rad_to_rotation * ONE_ROTATION_MICROSTEPS
    print(f"2. Counts/second = {rad_counts_per_second:,}")
    
    rad_counts_per_timestep = rad_counts_per_second / INTERNAL_TIME_UNIT_HZ
    print(f"3. Counts/timestep = {rad_counts_per_timestep}")
    
    rad_factor = rad_counts_per_timestep * (2**20)
    print(f"4. Final value (scaled by 2^20 for integer transmission) = {rad_factor:,}")

    # Counts/second to counts_per_timestep
    print("\nCounts/Second Conversion Calculation:")
    counts_per_timestep = 1 / INTERNAL_TIME_UNIT_HZ
    print(f"1. Counts/timestep = {counts_per_timestep}")
    
    counts_factor = counts_per_timestep * (2**20)
    print(f"2. Final value (scaled by 2^20 for integer transmission) = {counts_factor}")

    # Store velocity conversion factors
    velocity_factors = {
        "rpm": rpm_factor,
        "degrees_per_second": deg_factor,
        "radians_per_second": rad_factor,
        "counts_per_second": counts_factor
    }

    # Calculate velocity verification values
    velocity_verification = {
        "60_rpm": 60 * rpm_factor,
        "360_degrees_per_second": 360 * deg_factor,
        "2pi_radians_per_second": 2 * math.pi * rad_factor,
        "one_rotation_counts_per_second": ONE_ROTATION_MICROSTEPS * counts_factor
    }

    return velocity_factors, velocity_verification

def calculate_position_conversions(ONE_ROTATION_MICROSTEPS):
    """Calculate position unit conversion factors"""
    print("\nCalculating position unit conversions...")
    
    # Shaft rotations to encoder counts
    print("\nShaft Rotations Conversion Calculation:")
    print(f"1. ONE_ROTATION_MICROSTEPS = {ONE_ROTATION_MICROSTEPS:,} counts/rotation")
    
    shaft_counts = ONE_ROTATION_MICROSTEPS
    print(f"2. Counts per rotation = {shaft_counts:,}")
    
    shaft_factor = shaft_counts
    print(f"3. Final value = {shaft_factor:,}")

    # Degrees to encoder counts
    print("\nDegrees Conversion Calculation:")
    deg_to_rotation = 1/360
    print(f"1. 1 degree = {deg_to_rotation} rotations")
    
    deg_counts = deg_to_rotation * ONE_ROTATION_MICROSTEPS
    print(f"2. Counts per degree = {deg_counts:,}")
    
    deg_factor = deg_counts
    print(f"3. Final value = {deg_factor:,}")

    # Radians to encoder counts
    print("\nRadians Conversion Calculation:")
    rad_to_rotation = 1/(2*math.pi)
    print(f"1. 1 radian = {rad_to_rotation} rotations")
    
    rad_counts = rad_to_rotation * ONE_ROTATION_MICROSTEPS
    print(f"2. Counts per radian = {rad_counts:,}")
    
    rad_factor = rad_counts
    print(f"3. Final value = {rad_factor:,}")

    # Encoder counts to encoder counts
    print("\nEncoder Counts Conversion Calculation:")
    print("1. Converting from encoder counts to encoder counts")
    counts_factor = 1
    print("2. Final value = 1 (no conversion needed)")

    # Store position conversion factors
    position_factors = {
        "shaft_rotations": shaft_factor,
        "degrees": deg_factor,
        "radians": rad_factor,
        "encoder_counts": counts_factor
    }

    # Calculate position verification values
    position_verification = {
        "1_rotation": 1 * shaft_factor,
        "360_degrees": 360 * deg_factor,
        "2pi_radians": 2 * math.pi * rad_factor,
        "one_rotation_counts": ONE_ROTATION_MICROSTEPS * counts_factor
    }

    return position_factors, position_verification

def generate_unit_conversions_json(motor_type, velocity_factors, velocity_verification, 
                                 position_factors, position_verification,
                                 time_factors, time_verification,
                                 acceleration_factors, acceleration_verification):
    """Generate the combined unit conversions JSON file"""
    data = {
        "metadata": {
            "motor_type": motor_type,
            "generated_at": datetime.now().isoformat(),
            "description": "Automatically generated unit conversion factors",
            "scaling_note": "Velocity conversion factors include a 2^20 scaling factor and acceleration conversion factors include a 2^24 scaling factor to maintain precision when transmitting as integers"
        },
        "units": {
            "time": list(time_factors.keys()),
            "position": list(position_factors.keys()),
            "velocity": list(velocity_factors.keys()),
            "acceleration": list(acceleration_factors.keys()),
            "current": ["milliamps", "amps"],
            "voltage": ["volts", "millivolts"],
            "temperature": ["celsius", "fahrenheit", "kelvin"]
        },
        "conversion_factors": {
            **velocity_factors,
            **position_factors,
            **time_factors,
            **acceleration_factors
        },
        "verification": {
            "velocity": {
                "note": "All these values should be approximately equal as they represent 1 rotation per second",
                "values": velocity_verification
            },
            "position": {
                "note": "All these values should be approximately equal as they represent 1 rotation",
                "values": position_verification
            },
            "time": {
                "note": "All these values should be approximately equal as they represent 1 second",
                "values": time_verification
            },
            "acceleration": {
                "note": "All these values should be approximately equal as they represent 1 rotation per second squared",
                "values": acceleration_verification
            }
        }
    }
    
    output_file = f"unit_conversions_{motor_type}.json"
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"\nGenerated unit conversion factors saved to: {output_file}")
    return output_file

def main():
    parser = argparse.ArgumentParser(
        description='Calculate unit conversions for a specific motor type.',
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('motor_type', 
                       help='Motor type (e.g., M3).\nThe program will look for SETTINGS_<motor_type>.py')
    
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)
        
    args = parser.parse_args()

    # Import settings for the specified motor type
    settings = import_settings(args.motor_type)

    # Calculate constants from settings
    ONE_ROTATION_MICROSTEPS = settings.N_COMMUTATION_STEPS * settings.N_COMMUTATION_SUB_STEPS * settings.ONE_REVOLUTION_ELECTRICAL_CYCLES
    INTERNAL_TIME_UNIT_HZ = 64000000 / 2048   # Hz (internal update frequency)

    print(f"Calculating unit conversions for {args.motor_type} motor from first principles...")
    
    # Calculate time conversion factors
    time_factors, time_verification = calculate_time_conversions(INTERNAL_TIME_UNIT_HZ)
    
    # Calculate velocity conversion factors
    velocity_factors, velocity_verification = calculate_velocity_conversions(ONE_ROTATION_MICROSTEPS, INTERNAL_TIME_UNIT_HZ)
    
    # Calculate position conversion factors
    position_factors, position_verification = calculate_position_conversions(ONE_ROTATION_MICROSTEPS)
    
    # Calculate acceleration conversion factors
    acceleration_factors, acceleration_verification = calculate_acceleration_conversions(ONE_ROTATION_MICROSTEPS, INTERNAL_TIME_UNIT_HZ)
    
    # Generate combined JSON file
    output_file = generate_unit_conversions_json(
        args.motor_type,
        velocity_factors,
        velocity_verification,
        position_factors,
        position_verification,
        time_factors,
        time_verification,
        acceleration_factors,
        acceleration_verification
    )
    
    # Print final results
    print("\nFinal Conversion Factors:")
    print("-------------------------")
    print("\nTime Conversion Factors:")
    print(f"Milliseconds to timesteps:            {time_factors['milliseconds']:,.6f}")
    print(f"Seconds to timesteps:                 {time_factors['seconds']:,.6f}")
    print(f"Minutes to timesteps:                 {time_factors['minutes']:,.6f}")
    
    print("\nVelocity Conversion Factors (includes 2^20 scaling):")
    print(f"RPM to counts_per_timestep:           {velocity_factors['rpm']:,.6f}")
    print(f"Degrees/sec to counts_per_timestep:   {velocity_factors['degrees_per_second']:,.6f}")
    print(f"Radians/sec to counts_per_timestep:   {velocity_factors['radians_per_second']:,.6f}")
    print(f"Counts/sec to counts_per_timestep:    {velocity_factors['counts_per_second']:,.6f}")
    
    print("\nPosition Conversion Factors:")
    print(f"Shaft rotations to encoder counts:    {position_factors['shaft_rotations']:,.6f}")
    print(f"Degrees to encoder counts:            {position_factors['degrees']:,.6f}")
    print(f"Radians to encoder counts:            {position_factors['radians']:,.6f}")
    print(f"Encoder counts to encoder counts:     {position_factors['encoder_counts']:,.6f}")
    
    print("\nAcceleration Conversion Factors (includes 2^24 scaling):")
    print(f"RPM/sec to counts_per_timestep²:      {acceleration_factors['rpm_per_second']:,.6f}")
    print(f"Degrees/sec² to counts_per_timestep²: {acceleration_factors['degrees_per_second_squared']:,.6f}")
    print(f"Radians/sec² to counts_per_timestep²: {acceleration_factors['radians_per_second_squared']:,.6f}")
    print(f"Counts/sec² to counts_per_timestep²:  {acceleration_factors['counts_per_second_squared']:,.6f}")
    
    print("\nTime Verification (all should give same internal value):")
    print("------------------------------------------------")
    print(f"1000 milliseconds:        {time_verification['1000_milliseconds']:,.6f}")
    print(f"1 second:                 {time_verification['1_second']:,.6f}")
    print(f"1/60th minute:            {time_verification['1_60th_minute']:,.6f}")
    
    print("\nVelocity Verification (all should give same internal value):")
    print("------------------------------------------------")
    print(f"60 RPM:                   {velocity_verification['60_rpm']:,.6f}")
    print(f"360 degrees/second:       {velocity_verification['360_degrees_per_second']:,.6f}")
    print(f"2π radians/second:        {velocity_verification['2pi_radians_per_second']:,.6f}")
    print(f"3276800 counts/second:    {velocity_verification['one_rotation_counts_per_second']:,.6f}")
    
    print("\nPosition Verification (all should give same internal value):")
    print("------------------------------------------------")
    print(f"1 rotation:               {position_verification['1_rotation']:,.6f}")
    print(f"360 degrees:              {position_verification['360_degrees']:,.6f}")
    print(f"2π radians:               {position_verification['2pi_radians']:,.6f}")
    print(f"3276800 encoder counts:   {position_verification['one_rotation_counts']:,.6f}")
    
    print("\nAcceleration Verification (all should give same internal value):")
    print("------------------------------------------------")
    print(f"1 rotation/second²:       {acceleration_verification['1_rotation_per_second_squared']:,.6f}")
    print(f"60 RPM/second:           {acceleration_verification['60_rpm_per_second']:,.6f}")
    print(f"360 degrees/second²:     {acceleration_verification['360_degrees_per_second_squared']:,.6f}")
    print(f"2π radians/second²:      {acceleration_verification['2pi_radians_per_second_squared']:,.6f}")
    print(f"3276800 counts/second²:  {acceleration_verification['one_rotation_counts_per_second_squared']:,.6f}")

if __name__ == "__main__":
    main()
