#!/usr/bin/env python3

"""
This program generates unit conversion factors for different motor types.
DO NOT EDIT THE OUTPUT JSON FILE DIRECTLY! Instead, modify this program.
"""

import sys
import os
import json
import math
import argparse
import importlib.util
from datetime import datetime
from collections import OrderedDict

def import_settings(motor_type):
    """
    Dynamically import the settings file for the specified motor type.
    Returns the imported module or None if import fails.
    """
    settings_file = f"SETTINGS_{motor_type}.py"
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)  # Parent directory of autogeneration
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
    
    # Seconds to timesteps
    print("\nSeconds Conversion Calculation:")
    sec_factor = INTERNAL_TIME_UNIT_HZ
    print(f"1. Final value = {sec_factor:,} timesteps/second")

    # Milliseconds to timesteps
    print("\nMilliseconds Conversion Calculation:")
    print(f"1. INTERNAL_TIME_UNIT_HZ = {INTERNAL_TIME_UNIT_HZ:,} Hz")
    
    ms_to_sec = 1/1000
    print(f"2. 1 millisecond = {ms_to_sec} seconds")
    
    ms_factor = INTERNAL_TIME_UNIT_HZ * ms_to_sec
    print(f"3. Final value = {ms_factor:,} timesteps/millisecond")

    # Minutes to timesteps
    print("\nMinutes Conversion Calculation:")
    min_to_sec = 60
    print(f"1. 1 minute = {min_to_sec} seconds")
    
    min_factor = INTERNAL_TIME_UNIT_HZ * min_to_sec
    print(f"2. Final value = {min_factor:,} timesteps/minute")

    # Microseconds to timesteps
    print("\nMicroseconds Conversion Calculation:")
    us_to_sec = 1/1000000
    print(f"1. 1 microsecond = {us_to_sec} seconds")
    
    us_factor = INTERNAL_TIME_UNIT_HZ * us_to_sec
    print(f"2. Final value = {us_factor:,} timesteps/microsecond")

    # Store time conversion factors with seconds first
    time_factors = OrderedDict([
        ("seconds", sec_factor),
        ("milliseconds", ms_factor),
        ("minutes", min_factor),
        ("microseconds", us_factor)
    ])

    # Calculate time verification values
    time_verification = OrderedDict([
        ("1_second", 1 * sec_factor),
        ("1000_milliseconds", 1000 * ms_factor),
        ("1_60th_minute", (1/60) * min_factor),
        ("1000000_microseconds", 1000000 * us_factor)
    ])

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

    # Calculate counts_per_timestep_squared conversion factor
    print("\nCounts/Timestep² Conversion Calculation:")
    # Convert counts/timestep² to counts/sec²
    timestep_to_sec = INTERNAL_TIME_UNIT_HZ * INTERNAL_TIME_UNIT_HZ
    print(f"1. 1 count/timestep² = {timestep_to_sec} counts/second²")
    
    # Convert counts/sec² to rotations/sec²
    counts_timestep_to_rotation = timestep_to_sec / ONE_ROTATION_MICROSTEPS
    print(f"2. {timestep_to_sec} counts/second² = {counts_timestep_to_rotation} rotations/second²")
    
    # Use rotations/sec² conversion factor
    counts_timestep_factor = counts_timestep_to_rotation * rot_factor
    print(f"3. Final value = {counts_timestep_factor:,}")

    # Store acceleration conversion factors
    acceleration_factors = {
        "rotations_per_second_squared": rot_factor,
        "rpm_per_second": rpm_factor,
        "degrees_per_second_squared": deg_factor,
        "radians_per_second_squared": rad_factor,
        "counts_per_second_squared": counts_factor,  # Correct conversion
        "counts_per_timestep_squared": counts_timestep_factor  # Correctly scaled for timesteps
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
    
    # Rotations per second to counts_per_timestep
    print("\nRotations per Second Conversion Calculation:")
    print(f"1. ONE_ROTATION_MICROSTEPS = {ONE_ROTATION_MICROSTEPS:,} counts/rotation")
    print(f"2. INTERNAL_TIME_UNIT_HZ = {INTERNAL_TIME_UNIT_HZ:,} Hz")
    
    rps_counts_per_second = ONE_ROTATION_MICROSTEPS
    print(f"3. Counts/second = {rps_counts_per_second:,}")
    
    rps_counts_per_timestep = rps_counts_per_second / INTERNAL_TIME_UNIT_HZ
    print(f"4. Counts/timestep = {rps_counts_per_timestep}")
    
    rps_factor = rps_counts_per_timestep * (2**20)
    print(f"5. Final value (scaled by 2^20 for integer transmission) = {rps_factor:,}")

    # RPM to counts_per_timestep
    print("\nRPM Conversion Calculation:")
    rpm_to_rps = 1/60
    print(f"1. 1 RPM = {rpm_to_rps} rotations/second")
    
    rpm_counts_per_second = rpm_to_rps * ONE_ROTATION_MICROSTEPS
    print(f"2. Counts/second = {rpm_counts_per_second:,}")
    
    rpm_counts_per_timestep = rpm_counts_per_second / INTERNAL_TIME_UNIT_HZ
    print(f"3. Counts/timestep = {rpm_counts_per_timestep}")
    
    rpm_factor = rpm_counts_per_timestep * (2**20)
    print(f"4. Final value (scaled by 2^20 for integer transmission) = {rpm_factor:,}")

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
        "rotations_per_second": rps_factor,
        "rpm": rpm_factor,
        "degrees_per_second": deg_factor,
        "radians_per_second": rad_factor,
        "counts_per_second": rps_factor / ONE_ROTATION_MICROSTEPS,  # Correct conversion
        "counts_per_timestep": rps_factor  # Correct conversion
    }

    # Calculate velocity verification values
    velocity_verification = {
        "1_rotation_per_second": 1 * rps_factor,
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

def calculate_current_conversions():
    """Calculate current unit conversion factors"""
    print("\nCalculating current unit conversions...")
    
    # Internal current units to milliamps and amps
    print("\nCurrent Conversion Calculation:")
    
    # Define the conversion factors
    # The internal unit is a non-standard unit where 390 internal units = 2A
    internal_current_factor = 1.0
    milliamps_factor = 0.195  # 1mA = 0.195 internal units
    amps_factor = 195.0       # 1A = 195 internal units
    
    print(f"1. Internal current unit = {internal_current_factor}")
    print(f"2. Milliamps factor = {milliamps_factor} (1mA = 0.195 internal units)")
    print(f"3. Amps factor = {amps_factor} (1A = 195 internal units)")
    
    # Store current conversion factors
    current_factors = {
        "internal_current_units": internal_current_factor,
        "milliamps": milliamps_factor,
        "amps": amps_factor
    }
    
    # Calculate current verification values
    current_verification = {
        "2_amps": 2 * amps_factor,
        "1_amp": 1 * amps_factor,
        "1000_milliamps": 1000 * milliamps_factor
    }
    
    return current_factors, current_verification

def calculate_voltage_conversions():
    """Calculate voltage unit conversion factors"""
    print("\nCalculating voltage unit conversions...")
    
    # Millivolts and volts to internal units
    print("\nVoltage Conversion Calculation:")
    
    # Define the conversion factors
    # The internal unit is tenths of a volt (245 internal units = 24.5V)
    millivolts_factor = 0.01  # 1mV = 0.01 internal units
    volts_factor = 10.0       # 1V = 10 internal units
    
    print(f"1. Millivolts factor = {millivolts_factor} (1mV = 0.01 internal units)")
    print(f"2. Volts factor = {volts_factor} (1V = 10 internal units)")
    
    # Store voltage conversion factors
    voltage_factors = {
        "millivolts": millivolts_factor,
        "volts": volts_factor
    }
    
    # Calculate voltage verification values
    voltage_verification = {
        "24.5_volts": 24.5 * volts_factor,
        "1_volt": 1 * volts_factor,
        "1000_millivolts": 1000 * millivolts_factor
    }
    
    return voltage_factors, voltage_verification

def calculate_temperature_conversions():
    """Calculate temperature unit conversion factors and offsets"""
    print("\nCalculating temperature unit conversions...")
    
    # Temperature conversions
    print("\nTemperature Conversion Calculation:")
    
    # Define the conversion factors
    celsius_factor = 1.0
    fahrenheit_factor = 5.0/9.0  # Multiply by this to convert F to C
    kelvin_factor = 1.0  # Kelvin has the same scale as Celsius, just with an offset
    
    # Define the conversion offsets
    fahrenheit_to_celsius_offset = -32.0
    celsius_to_fahrenheit_offset = 32.0
    kelvin_to_celsius_offset = -273.15
    celsius_to_kelvin_offset = 273.15
    
    print(f"1. Celsius factor = {celsius_factor}")
    print(f"2. Fahrenheit factor = {fahrenheit_factor}")
    print(f"3. Kelvin factor = {kelvin_factor}")
    print(f"4. Fahrenheit to Celsius offset = {fahrenheit_to_celsius_offset}")
    print(f"5. Celsius to Fahrenheit offset = {celsius_to_fahrenheit_offset}")
    print(f"6. Kelvin to Celsius offset = {kelvin_to_celsius_offset}")
    print(f"7. Celsius to Kelvin offset = {celsius_to_kelvin_offset}")
    
    # Store temperature conversion factors
    temperature_factors = {
        "celsius": celsius_factor,
        "fahrenheit": fahrenheit_factor,
        "kelvin": kelvin_factor
    }
    
    # Store temperature conversion offsets
    temperature_offsets = {
        "fahrenheit_to_celsius": fahrenheit_to_celsius_offset,
        "celsius_to_fahrenheit": celsius_to_fahrenheit_offset,
        "kelvin_to_celsius": kelvin_to_celsius_offset,
        "celsius_to_kelvin": celsius_to_kelvin_offset
    }
    
    # Calculate temperature verification values
    temperature_verification = {
        "25C_to_77F": (25 * 9.0/5.0) + 32.0,
        "32F_to_0C": (32 - 32) * 5.0/9.0,
        "0C_to_273.15K": 0 + 273.15
    }
    
    return temperature_factors, temperature_offsets, temperature_verification

# Get the absolute path of the Arduino library directory
PYTHON_PROGRAMS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../python_programs/servomotor/"))

def generate_unit_conversions_json(motor_type, velocity_factors, velocity_verification,
                                   position_factors, position_verification,
                                   time_factors, time_verification,
                                   acceleration_factors, acceleration_verification,
                                   current_factors, current_verification,
                                   voltage_factors, voltage_verification,
                                   temperature_factors, temperature_offsets, temperature_verification):
    """Generate the combined unit conversions JSON file"""
    
    data = OrderedDict([
        ("metadata", OrderedDict([
            ("warning", "DO NOT EDIT THIS FILE DIRECTLY! Instead, modify the autogenerate_unit_conversions.py program."),
            ("generation_note", f"Generated on: {datetime.now().strftime('%b %-d %Y %H:%M:%S')}"),
            ("motor_type", motor_type),
            ("generated_at", datetime.now().strftime('%b %-d %Y %H:%M:%S')),
            ("description", "Automatically generated unit conversion factors"),
            ("scaling_note", "Velocity conversion factors include a 2^20 scaling factor and acceleration conversion factors include a 2^24 scaling factor to maintain precision when transmitting as integers")
        ])),
        ("units", OrderedDict([
            ("time", ["timesteps", "seconds", "milliseconds", "minutes", "microseconds"]),  # timesteps must be first
            ("position", ["shaft_rotations", "degrees", "radians", "encoder_counts"]),
            ("velocity", ["rotations_per_second", "rpm", "degrees_per_second", "radians_per_second", "counts_per_second", "counts_per_timestep"]),
            ("acceleration", ["rotations_per_second_squared", "rpm_per_second", "degrees_per_second_squared", "radians_per_second_squared", "counts_per_second_squared", "counts_per_timestep_squared"]),
            ("current", ["internal_current_units", "milliamps", "amps"]),  # Renamed from arbitrary_units to internal_current_units
            ("voltage", ["millivolts", "volts"]),
            ("temperature", ["celsius", "fahrenheit", "kelvin"])
        ])),
        ("conversion_factors", OrderedDict([
            # Velocity factors in order
            ("rotations_per_second", velocity_factors["rotations_per_second"]),
            ("rpm", velocity_factors["rpm"]),
            ("degrees_per_second", velocity_factors["degrees_per_second"]),
            ("radians_per_second", velocity_factors["radians_per_second"]),
            ("counts_per_second", velocity_factors["counts_per_second"]),
            ("counts_per_timestep", velocity_factors["counts_per_timestep"]),
            # Position factors
            ("shaft_rotations", position_factors["shaft_rotations"]),
            ("degrees", position_factors["degrees"]),
            ("radians", position_factors["radians"]),
            ("encoder_counts", position_factors["encoder_counts"]),
            # Time factors with timesteps first
            ("timesteps", 1),
            ("seconds", time_factors["seconds"]),
            ("milliseconds", time_factors["milliseconds"]),
            ("minutes", time_factors["minutes"]),
            ("microseconds", time_factors["microseconds"]),
            # Acceleration factors
            ("rotations_per_second_squared", acceleration_factors["rotations_per_second_squared"]),
            ("rpm_per_second", acceleration_factors["rpm_per_second"]),
            ("degrees_per_second_squared", acceleration_factors["degrees_per_second_squared"]),
            ("radians_per_second_squared", acceleration_factors["radians_per_second_squared"]),
            ("counts_per_second_squared", acceleration_factors["counts_per_second_squared"]),
            ("counts_per_timestep_squared", acceleration_factors["counts_per_timestep_squared"]),
            # Current factors
            ("internal_current_units", current_factors["internal_current_units"]),
            ("milliamps", current_factors["milliamps"]),
            ("amps", current_factors["amps"]),
            # Voltage factors
            ("millivolts", voltage_factors["millivolts"]),
            ("volts", voltage_factors["volts"]),
            # Temperature factors
            ("celsius", temperature_factors["celsius"]),
            ("fahrenheit", temperature_factors["fahrenheit"]),
            ("kelvin", temperature_factors["kelvin"])
        ])),
        ("conversion_offsets", OrderedDict([
            # Temperature offsets
            ("fahrenheit_to_celsius", temperature_offsets["fahrenheit_to_celsius"]),
            ("celsius_to_fahrenheit", temperature_offsets["celsius_to_fahrenheit"]),
            ("kelvin_to_celsius", temperature_offsets["kelvin_to_celsius"]),
            ("celsius_to_kelvin", temperature_offsets["celsius_to_kelvin"])
        ])),
        ("verification", OrderedDict([
            ("velocity", OrderedDict([
                ("note", "All these values should be approximately equal as they represent 1 rotation per second"),
                ("values", OrderedDict([
                    ("1_rotation_per_second", velocity_verification["1_rotation_per_second"]),
                    ("60_rpm", velocity_verification["60_rpm"]),
                    ("360_degrees_per_second", velocity_verification["360_degrees_per_second"]),
                    ("2pi_radians_per_second", velocity_verification["2pi_radians_per_second"]),
                    ("one_rotation_counts_per_second", velocity_verification["one_rotation_counts_per_second"])
                ]))
            ])),
            ("position", OrderedDict([
                ("note", "All these values should be approximately equal as they represent 1 rotation"),
                ("values", position_verification)
            ])),
            ("time", OrderedDict([
                ("note", "All these values should be approximately equal as they represent 1 second"),
                ("values", OrderedDict([
                    ("1_second", time_verification["1_second"]),
                    ("1000_milliseconds", time_verification["1000_milliseconds"]),
                    ("1_60th_minute", time_verification["1_60th_minute"]),
                    ("1000000_microseconds", time_verification["1000000_microseconds"])
                ]))
            ])),
            ("acceleration", OrderedDict([
                ("note", "All these values should be approximately equal as they represent 1 rotation per second squared"),
                ("values", acceleration_verification)
            ])),
            ("current", OrderedDict([
                ("note", "These values represent the conversion between different current units"),
                ("values", OrderedDict([
                    ("2_amps", current_verification["2_amps"]),
                    ("1_amp", current_verification["1_amp"]),
                    ("1000_milliamps", current_verification["1000_milliamps"])
                ]))
            ])),
            ("voltage", OrderedDict([
                ("note", "These values represent the conversion between different voltage units"),
                ("values", OrderedDict([
                    ("24.5_volts", voltage_verification["24.5_volts"]),
                    ("1_volt", voltage_verification["1_volt"]),
                    ("1000_millivolts", voltage_verification["1000_millivolts"])
                ]))
            ])),
            ("temperature", OrderedDict([
                ("note", "These values represent the conversion between different temperature units"),
                ("values", OrderedDict([
                    ("25C_to_77F", temperature_verification["25C_to_77F"]),
                    ("32F_to_0C", temperature_verification["32F_to_0C"]),
                    ("0C_to_273.15K", temperature_verification["0C_to_273.15K"])
                ]))
            ]))
        ]))
    ])
    
    print("\nGenerating unit conversion factors JSON file from this data:")
    print(data)
    
    print("\nSaving to JSON file...")

    output_file = os.path.join(PYTHON_PROGRAMS_DIR, f"unit_conversions_{motor_type}.json")
    
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
    
    # Calculate current conversion factors
    current_factors, current_verification = calculate_current_conversions()
    
    # Calculate voltage conversion factors
    voltage_factors, voltage_verification = calculate_voltage_conversions()
    
    # Calculate temperature conversion factors and offsets
    temperature_factors, temperature_offsets, temperature_verification = calculate_temperature_conversions()
    
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
        acceleration_verification,
        current_factors,
        current_verification,
        voltage_factors,
        voltage_verification,
        temperature_factors,
        temperature_offsets,
        temperature_verification
    )
    
    # Print final results
    print("\nFinal Conversion Factors:")
    print("-------------------------")
    print("\nTime Conversion Factors:")
    print(f"Seconds to timesteps:                 {time_factors['seconds']:,.6f}")
    print(f"Milliseconds to timesteps:            {time_factors['milliseconds']:,.6f}")
    print(f"Minutes to timesteps:                 {time_factors['minutes']:,.6f}")
    print(f"Microseconds to timesteps:            {time_factors['microseconds']:,.6f}")
     
    print("\nVelocity Conversion Factors (includes 2^20 scaling):")
    print(f"Rotations/sec to counts_per_timestep: {velocity_factors['rotations_per_second']:,.6f}")
    print(f"RPM to counts_per_timestep:           {velocity_factors['rpm']:,.6f}")
    print(f"Degrees/sec to counts_per_timestep:   {velocity_factors['degrees_per_second']:,.6f}")
    print(f"Radians/sec to counts_per_timestep:   {velocity_factors['radians_per_second']:,.6f}")
    print(f"Counts/sec to counts_per_timestep:    {velocity_factors['counts_per_timestep']:,.6f}")
    
    print("\nPosition Conversion Factors:")
    print(f"Shaft rotations to encoder counts:    {position_factors['shaft_rotations']:,.6f}")
    print(f"Degrees to encoder counts:            {position_factors['degrees']:,.6f}")
    print(f"Radians to encoder counts:            {position_factors['radians']:,.6f}")
    print(f"Encoder counts to encoder counts:     {position_factors['encoder_counts']:,.6f}")
    
    print("\nAcceleration Conversion Factors (includes 2^24 scaling):")
    print(f"Rotations/sec² to counts_per_timestep²: {acceleration_factors['rotations_per_second_squared']:,.6f}")
    print(f"RPM/sec to counts_per_timestep²:      {acceleration_factors['rpm_per_second']:,.6f}")
    print(f"Degrees/sec² to counts_per_timestep²: {acceleration_factors['degrees_per_second_squared']:,.6f}")
    print(f"Radians/sec² to counts_per_timestep²: {acceleration_factors['radians_per_second_squared']:,.6f}")
    print(f"Counts/sec² to counts_per_timestep²:  {acceleration_factors['counts_per_timestep_squared']:,.6f}")
    
    print("\nTime Verification (all should give same internal value):")
    print("------------------------------------------------")
    print(f"1 second:                 {time_verification['1_second']:,.6f}")
    print(f"1000 milliseconds:        {time_verification['1000_milliseconds']:,.6f}")
    print(f"1/60th minute:            {time_verification['1_60th_minute']:,.6f}")
    print(f"1000000 microseconds:     {time_verification['1000000_microseconds']:,.6f}")
    
    print("\nVelocity Verification (all should give same internal value):")
    print("------------------------------------------------")
    print(f"1 rotation/second:        {velocity_verification['1_rotation_per_second']:,.6f}")
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
    
    print("\nCurrent Conversion Factors:")
    print("-------------------------")
    print(f"Internal current units:              {current_factors['internal_current_units']:,.6f}")
    print(f"Milliamps to internal units:         {current_factors['milliamps']:,.6f}")
    print(f"Amps to internal units:              {current_factors['amps']:,.6f}")
    
    print("\nVoltage Conversion Factors:")
    print("-------------------------")
    print(f"Millivolts to internal units:        {voltage_factors['millivolts']:,.6f}")
    print(f"Volts to internal units:             {voltage_factors['volts']:,.6f}")
    
    print("\nTemperature Conversion Factors:")
    print("-------------------------")
    print(f"Celsius factor:                      {temperature_factors['celsius']:,.6f}")
    print(f"Fahrenheit factor:                   {temperature_factors['fahrenheit']:,.6f}")
    
    print("\nTemperature Conversion Offsets:")
    print("-------------------------")
    print(f"Fahrenheit to Celsius offset:        {temperature_offsets['fahrenheit_to_celsius']:,.6f}")
    print(f"Celsius to Fahrenheit offset:        {temperature_offsets['celsius_to_fahrenheit']:,.6f}")
    print(f"Kelvin to Celsius offset:            {temperature_offsets['kelvin_to_celsius']:,.6f}")
    print(f"Celsius to Kelvin offset:            {temperature_offsets['celsius_to_kelvin']:,.6f}")
    
    print("\nCurrent Verification:")
    print("------------------------------------------------")
    print(f"2 amps:                  {current_verification['2_amps']:,.6f}")
    print(f"1 amp:                   {current_verification['1_amp']:,.6f}")
    print(f"1000 milliamps:          {current_verification['1000_milliamps']:,.6f}")
    
    print("\nVoltage Verification:")
    print("------------------------------------------------")
    print(f"24.5 volts:              {voltage_verification['24.5_volts']:,.6f}")
    print(f"1 volt:                  {voltage_verification['1_volt']:,.6f}")
    print(f"1000 millivolts:         {voltage_verification['1000_millivolts']:,.6f}")
    
    print("\nTemperature Verification:")
    print("------------------------------------------------")
    print(f"25°C to Fahrenheit:      {temperature_verification['25C_to_77F']:,.6f}°F")
    print(f"32°F to Celsius:         {temperature_verification['32F_to_0C']:,.6f}°C")
    print(f"0°C to Kelvin:           {temperature_verification['0C_to_273.15K']:,.6f}K")

if __name__ == "__main__":
    main()
