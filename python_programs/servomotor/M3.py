#!/usr/bin/env python3

from .platform_compat import Enum
from .platform_utils import load_json_file, module_path

from . import communication

def load_units_from_json(motor_type="M3"):
    """Load available units from JSON file"""
    data = load_json_file(module_path(f'unit_conversions_{motor_type}.json'))
    return data["units"]

# Create Enum classes dynamically from JSON units
units = load_units_from_json()

TimeUnit = Enum('TimeUnit', {unit.upper(): unit for unit in units["time"]})
PositionUnit = Enum('PositionUnit', {unit.upper(): unit for unit in units["position"]})
VelocityUnit = Enum('VelocityUnit', {unit.upper(): unit for unit in units["velocity"]})
AccelerationUnit = Enum('AccelerationUnit', {unit.upper(): unit for unit in units["acceleration"]})
CurrentUnit = Enum('CurrentUnit', {unit.upper(): unit for unit in units["current"]})
VoltageUnit = Enum('VoltageUnit', {unit.upper(): unit for unit in units["voltage"]})
TemperatureUnit = Enum('TemperatureUnit', {unit.upper(): unit for unit in units["temperature"]})

class UnitConverter:
    """Handles unit conversions based on JSON definitions"""
    def __init__(self, command_dict, motor_type="M3"):
        self.motor_type = motor_type
        self.command_definitions = load_json_file(module_path('motor_commands.json'))
        self.converted_commands = command_dict
        self.conversion_factors = {}
        self._cache_conversion_factors()

    def _load_conversion_factors_from_file(self, filename):
        """Load conversion factors from an external JSON file"""
        try:
            data = load_json_file(module_path(filename.format(motor_type=self.motor_type)))
            return data["conversion_factors"]
        except Exception as e:
            print(f"Error loading conversion factors: {e}")
            return None

    def _rescale_for_internal_unit(self, conv, factors):
        """The time table in unit_conversions_*.json is expressed in timesteps,
        but a few commands' wire unit is a different time unit (e.g. the
        microsecond clock of 'Get current time' / 'Time sync'). For those,
        rescale the table so the declared InternalUnit's own factor becomes 1.
        Only time tables are rescaled: the velocity/acceleration tables
        intentionally carry the firmware's 2^20 / 2^24 wire scaling, so their
        InternalUnit factor is deliberately not 1."""
        if conv['Type'] != 'time':
            return factors
        internal_factor = factors.get(conv['InternalUnit'])
        if internal_factor is None:
            # Fail loudly: silently skipping the rescale would reintroduce the
            # ~32x wrong-conversion bug this mechanism exists to prevent
            raise ValueError(f"conversion table has no factor for internal unit '{conv['InternalUnit']}'")
        if internal_factor == 1:
            return factors
        return {unit: factor / internal_factor for unit, factor in factors.items()}

    def _cache_one_conversion(self, conv):
        key = f"{conv['Type']}_{conv['InternalUnit']}"
        if key in self.conversion_factors:
            return  # already cached; avoids re-reading the factors file per parameter occurrence
        if 'ConversionFactorsFile' in conv:
            factors = self._load_conversion_factors_from_file(conv['ConversionFactorsFile'])
            if factors:
                self.conversion_factors[key] = self._rescale_for_internal_unit(conv, factors)
        elif 'ConversionFactors' in conv:
            self.conversion_factors[key] = self._rescale_for_internal_unit(conv, conv['ConversionFactors'])

    def _cache_conversion_factors(self):
        """Cache conversion factors from command definitions"""
        for cmd in self.command_definitions:
            for io in ('Input', 'Output'):
                if isinstance(cmd.get(io), list):
                    for param in cmd[io]:
                        if isinstance(param, dict) and 'UnitConversion' in param:
                            self._cache_one_conversion(param['UnitConversion'])

    def get_conversion_info(self, command_string, param_name):
        """Get conversion information for a parameter"""
        for cmd in self.command_definitions:
            if cmd['CommandString'] == command_string:
                if isinstance(cmd.get('Input'), list):
                    for param in cmd['Input']:
                        if isinstance(param, dict) and param.get('ParameterName') == param_name and 'UnitConversion' in param:
                            return param['UnitConversion']
                if isinstance(cmd.get('Output'), list):
                    for param in cmd['Output']:
                        if isinstance(param, dict) and param.get('ParameterName') == param_name and 'UnitConversion' in param:
                            return param['UnitConversion']
        return None

    def convert_to_internal(self, value, command_string, param_name, current_unit):
        """Convert from current unit to internal unit"""
        conv_info = self.get_conversion_info(command_string, param_name)
        if not conv_info:
            return value
        
        key = f"{conv_info['Type']}_{conv_info['InternalUnit']}"
        if key not in self.conversion_factors:
            return value
        
        factors = self.conversion_factors[key]
        if current_unit not in factors:
            return value
        
        factor = factors[current_unit]
        result = value * factor
        
        # Round the result to avoid truncation errors
        rounded = round(result)
        return rounded

    def convert_from_internal(self, value, command_string, param_name, target_unit):
        """Convert from internal unit to target unit"""
        conv_info = self.get_conversion_info(command_string, param_name)
        if not conv_info:
            return value
        
        key = f"{conv_info['Type']}_{conv_info['InternalUnit']}"
        if key not in self.conversion_factors:
            return value
        
        factors = self.conversion_factors[key]
        if target_unit not in factors:
            return value

        # Handle both single values and lists
        if isinstance(value, list):
            value = value[0]  # Take first value since we expect single values

        # Special handling for temperature conversions
        if conv_info['Type'] == 'temperature':
            if target_unit == 'fahrenheit':
                return (value * 9/5) + 32
            elif target_unit == 'kelvin':
                return value + 273.15
            else:
                return value
        
        return value / factors[target_unit]

    def get_param_info(self, command_string, param_index):
        """Get parameter info from original JSON definition"""
        for cmd in self.command_definitions:
            if cmd['CommandString'] == command_string:
                if isinstance(cmd.get('Input'), list) and param_index < len(cmd['Input']):
                    return cmd['Input'][param_index]
        return None

def create_command_function(command, command_id, multiple_responses, unit_converter, verbose=2):
    if verbose >= 2:
        print(f"Creating function for command: {command['CommandString']} (ID {command_id})")
    json_cmd = next((cmd for cmd in unit_converter.command_definitions if cmd['CommandString'] == command['CommandString']), None)
    if not json_cmd:
        print(f"Error: Could not find command {command['CommandString']} in JSON definitions")
        exit(1)
    if verbose >= 2:
        print(f"json command: {json_cmd}")
    if json_cmd.get('Output'):
        if not isinstance(json_cmd.get('Output'), list) and json_cmd.get('Output') != "success_response":
            print(f"Error: Expected 'Output' to be a list or success_response, but got: {json_cmd.get('Output')}")
            exit(1)
        has_outputs = True
    else:
        has_outputs = False

    def command_function(self, *args, **kwargs):
        if self.verbose:
            print(f"Executing: {command['CommandString']} (ID {command_id}) with args: {args} and kwargs: {kwargs}")
        
        # Convert input parameters based on unit conversion definitions
        converted_inputs = []
        if isinstance(command.get('Input'), list):
            input_values = list(args)
            json_cmd = next((cmd for cmd in unit_converter.command_definitions 
                          if cmd['CommandString'] == command['CommandString']), None)
            
            if json_cmd:
                # Handle positional arguments
                for i, value in enumerate(input_values):
                    param_info = unit_converter.get_param_info(command['CommandString'], i)
                    if param_info and 'UnitConversion' in param_info:
                        conv = param_info['UnitConversion']
                        unit_type = conv['Type']
                        current_unit = getattr(self, f"_{unit_type}_unit").value
                        converted_value = unit_converter.convert_to_internal(
                            value, command['CommandString'], param_info['ParameterName'], current_unit)
                        if self.verbose:
                            print(f"Converting {param_info['ParameterName']} from {value} {current_unit} "
                                  f"to {converted_value} {conv['InternalUnit']}")
                        # Round any float values to integers before adding to converted_inputs
                        converted_inputs.append(int(round(converted_value)) if isinstance(converted_value, float) else converted_value)
                    else:
                        # Round any float values to integers
                        converted_inputs.append(int(round(value)) if isinstance(value, float) else value)

                # Handle keyword arguments
                if json_cmd.get('Input'):
                    for param in json_cmd['Input']:
                        if isinstance(param, dict):
                            param_name = param.get('ParameterName')
                            if param_name in kwargs:
                                value = kwargs[param_name]
                                if 'UnitConversion' in param:
                                    conv = param['UnitConversion']
                                    unit_type = conv['Type']
                                    current_unit = getattr(self, f"_{unit_type}_unit").value
                                    converted_value = unit_converter.convert_to_internal(
                                        value, command['CommandString'], param_name, current_unit)
                                    if self.verbose:
                                        print(f"Converting {param_name} from {value} {current_unit} "
                                              f"to {converted_value} {conv['InternalUnit']}")
                                    # Round any float values to integers before adding to converted_inputs
                                    converted_inputs.append(int(round(converted_value)) if isinstance(converted_value, float) else converted_value)
                                else:
                                    # Round any float values to integers
                                    converted_inputs.append(int(round(value)) if isinstance(value, float) else value)
            else:
                # If we can't find the command in JSON definitions, round any float values to integers
                converted_inputs = [int(round(v)) if isinstance(v, float) else v for v in (list(args) + list(kwargs.values()))]

        # Execute the command and get response
        possibly_multiple_responses = communication.execute_command(command_id, converted_inputs, alias_or_unique_id=self.alias_or_unique_id,
                                                                    crc32_enabled=self.crc32_enabled, verbose=self.verbose)

        if not isinstance(possibly_multiple_responses, list):
            print("Error: Expected the response to be a list, but got: ", possibly_multiple_responses)
            exit(1)
        if multiple_responses == False and len(possibly_multiple_responses) > 1:
            print("Error: Expected only one item in the list or an empty list, but got: ", possibly_multiple_responses)
            exit(1)

        # Convert output parameters based on unit conversion definitions
        converted_multiple_responses = []
        if has_outputs:
            for response in possibly_multiple_responses:
                converted_response = []
                for i, value in enumerate(response):
                    json_cmd_output = json_cmd['Output']
                    if i >= len(json_cmd_output):
                        print("Error: More response values than we have data for in the JSON definition")
                        exit(1)
                    
                    param = json_cmd_output[i]
                    if isinstance(param, dict) and 'UnitConversion' in param:
                        conv = param['UnitConversion']
                        unit_type = conv['Type']
                        target_unit = getattr(self, f"_{unit_type}_unit").value
                        converted_value = unit_converter.convert_from_internal(value, command['CommandString'], param['ParameterName'], target_unit)
                        converted_response.append(converted_value)
                    else:
                        converted_response.append(value)
                converted_multiple_responses.append(converted_response)
        else:
            converted_multiple_responses = possibly_multiple_responses

        # Handle single response case
        if not multiple_responses:
            if len(converted_multiple_responses) == 1:
                converted_multiple_responses = converted_multiple_responses[0]  # remove outer list since it has just one item
            if isinstance(converted_multiple_responses, list) and len(converted_multiple_responses) == 1:
                converted_multiple_responses = converted_multiple_responses[0]  # remove inner list since it has just one item

        return converted_multiple_responses
            
#        except communication.TimeoutError:
#            print("\nTimeout occurred! Checking motor status...")
#            try:
#                status = self.get_status()
#                print(f"Motor Status: {status}")
#            except Exception as e:
#                print(f"Failed to get motor status: {e}")
#            raise

    return command_function

def define_commands(m3_class, data_type_dict, command_dict, verbose=2):
    """Define command functions on the M3 class"""
    if verbose == 2:
        print('Defining command functions...')
        print(f"Number of commands: {len(command_dict)}")
    
    unit_converter = UnitConverter(command_dict, motor_type="M3")
    
    # Create a function for each command
    for command in command_dict:
        if verbose >= 2:
            print(f"Command: {command}")
        command_id = command['CommandEnum']
        command_str = command['CommandString']
        multiple_responses = command.get('MultipleResponses', False)
        function_name = command_str.replace(" ", "_").lower()
        if verbose:
            print(f"Creating function: {function_name} for command: {command_str}")
        # Define the function in the M3 class
        setattr(m3_class, function_name, create_command_function(command, command_id, multiple_responses, unit_converter, verbose=verbose))
        # Verify the function was created
        if verbose:
            if hasattr(m3_class, function_name):
                print(f"Successfully created function: {function_name}")
            else:
                print(f"Failed to create function: {function_name}")

    # Debug print all defined methods
    if verbose:
        print("\nDefined methods:")
        for attr in dir(m3_class):
            if not attr.startswith('_'):
                print(f"  {attr}")

    communication.set_data_types_and_command_data(data_type_dict, command_dict)

class AllMotors:
    def __init__(self, alias_or_unique_id=None, time_unit=None, position_unit=None, velocity_unit=None, acceleration_unit=None, temperature_unit=None, current_unit=None, voltage_unit=None, verbose=2):
        if isinstance(alias_or_unique_id, int) or (alias_or_unique_id == None):
            self.alias_or_unique_id = alias_or_unique_id
        elif isinstance(alias_or_unique_id, str):
            # Convert string to integer if needed
            if len(alias_or_unique_id) == 1:
                self.alias_or_unique_id = ord(alias_or_unique_id)  # Convert single character to ASCII value
            elif len(alias_or_unique_id) == 16:
                # Exactly 16 hex characters is a 64-bit unique ID (e.g. "AABBCCDDEEFF0011")
                try:
                    self.alias_or_unique_id = int(alias_or_unique_id, 16)
                except ValueError:
                    raise ValueError("A 16-character string must be a hexadecimal unique ID such as AABBCCDDEEFF0011")
                if self.alias_or_unique_id <= 255:
                    raise ValueError("This unique ID falls in the one-byte alias range (0-255), so it would be "
                                     "misinterpreted as an alias; real device unique IDs are never this small")
            else:
                try:
                    self.alias_or_unique_id = int(alias_or_unique_id)
                except ValueError:
                    raise ValueError("Alias or unique_id must be a single character, a decimal integer, or a 16-hex-digit unique ID")
        else:
            raise ValueError("Alias or unique_id must be a single character such as X or an integer such as 123 or a 16-hex-digit unique ID string such as AABBCCDDEEFF0011")
        # alias_or_unique_id may legitimately be None here (a deferred alias,
        # set later via use_this_alias_or_unique_id() or the global -a alias);
        # only range-check a real value -- comparing None to an int crashes.
        if self.alias_or_unique_id is not None:
            if self.alias_or_unique_id < 0 or self.alias_or_unique_id > 0xFFFFFFFFFFFFFFFF:
                raise ValueError("Alias or unique_id must be in the range from 0 to 0xFFFFFFFFFFFFFFFF")
            if self.alias_or_unique_id in (252, 253, 254):
                raise ValueError("Aliases 252, 253 and 254 are reserved by the protocol (response markers "
                                 "and the extended-addressing marker) and cannot address a device")

        if time_unit == None:
            time_unit = next(iter(TimeUnit)).value
        self._time_unit = TimeUnit(time_unit)
        
        if position_unit == None:
            position_unit = next(iter(PositionUnit)).value
        self._position_unit = PositionUnit(position_unit)
        
        if velocity_unit == None:
            velocity_unit = next(iter(VelocityUnit)).value
        self._velocity_unit = VelocityUnit(velocity_unit)
        
        if acceleration_unit == None:
            acceleration_unit = next(iter(AccelerationUnit)).value
        self._acceleration_unit = AccelerationUnit(acceleration_unit)
        
        if temperature_unit == None:
            temperature_unit = next(iter(TemperatureUnit)).value
        self._temperature_unit = TemperatureUnit(temperature_unit)

        if current_unit == None:
            current_unit = next(iter(CurrentUnit)).value
        self._current_unit = CurrentUnit(current_unit)
        
        if voltage_unit == None:
            voltage_unit = next(iter(VoltageUnit)).value
        self._voltage_unit = VoltageUnit(voltage_unit)

        self.crc32_enabled = True  # matches the device's power-on default
        self.verbose = verbose

    def __del__(self):
        pass

    def set_position_unit(self, new_unit):
        """Change the position unit dynamically"""
        if new_unit != None:
            self._position_unit = PositionUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._position_unit = PositionUnit(next(iter(PositionUnit)).value)

    def set_time_unit(self, new_unit):
        """Change the time unit dynamically"""
        if new_unit != None:
            self._time_unit = TimeUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._time_unit = TimeUnit(next(iter(TimeUnit)).value)        

    def set_velocity_unit(self, new_unit):
        """Change the velocity unit dynamically"""
        if new_unit != None:
            self._velocity_unit = VelocityUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._velocity_unit = VelocityUnit(next(iter(VelocityUnit)).value)

    def set_acceleration_unit(self, new_unit):
        """Change the acceleration unit dynamically"""
        if new_unit != None:
            self._acceleration_unit = AccelerationUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._acceleration_unit = AccelerationUnit(next(iter(AccelerationUnit)).value)

    def set_temperature_unit(self, new_unit):
        """Change the temperature unit dynamically"""
        if new_unit != None:
            self._temperature_unit = TemperatureUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._temperature_unit = TemperatureUnit(next(iter(TemperatureUnit)).value)

    def set_current_unit(self, new_unit):
        """Change the current unit dynamically"""
        if new_unit != None:
            self._current_unit = CurrentUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._current_unit = CurrentUnit(next(iter(CurrentUnit)).value)

    def set_voltage_unit(self, new_unit):
        """Change the voltage unit dynamically"""
        if new_unit != None:
            self._voltage_unit = VoltageUnit(new_unit)
        else: # in the case that the user specifies None, we choose a default as the first item in the enum
            self._voltage_unit = VoltageUnit(next(iter(VoltageUnit)).value)

    def set_crc32_enabled(self, enabled):
        """Control whether this object's outgoing commands append a CRC32. Set to False
        after sending 'CRC32 control' 0 to the device (the device's power-on default is enabled)."""
        self.crc32_enabled = bool(enabled)

class M3(AllMotors):
    def __init__(self, alias_or_unique_id=None, time_unit=None, position_unit=None, velocity_unit=None, acceleration_unit=None, temperature_unit=None, current_unit=None, voltage_unit=None, verbose=2):
        super().__init__(alias_or_unique_id, time_unit=time_unit, position_unit=position_unit, velocity_unit=velocity_unit,
                        acceleration_unit=acceleration_unit, temperature_unit=temperature_unit,
                        current_unit=current_unit, voltage_unit=voltage_unit, verbose=verbose)

    def __del__(self):
        super().__del__()

    def use_this_alias_or_unique_id(self, alias_or_unique_id):
        """Set the alias for addressing this motor (standard addressing mode)"""
        self.alias_or_unique_id = alias_or_unique_id

