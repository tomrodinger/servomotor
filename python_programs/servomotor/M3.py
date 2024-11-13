from . import communication

def create_command_function(command, command_id, multiple_responses):
    def command_function(self, *args, **kwargs):
        # Here you would implement the logic to handle the command
        print(f"Executing: {command['CommandString']} (ID {command_id}) with args: {args} and kwargs: {kwargs}")
        # let's package all the arguments and k-arguments into a list of values
        inputs = []
        for arg in args:
            inputs.append(arg)
        for key, value in kwargs.items():
            inputs.append(value)
        response = communication.execute_command(self.alias, command_id, inputs, verbose=self.verbose)
        # You can also process the input/output based on the command definition
        if multiple_responses == False and len(response) == 1:
            response = response[0]
        return response
    return command_function

def dynamically_define_command_functions(data_type_dict, command_dict):
    print('define_on_the_fly_functions() is running')
    for command in command_dict:
        command_id = command['CommandEnum']
        command_str = command['CommandString']
        multiple_responses = command['MultipleResponses']
        function_name = command_str.replace(" ", "_").lower()
        # Define the function in the M3 class
        setattr(M3, function_name, create_command_function(command, command_id, multiple_responses))

    communication.set_data_types_and_command_data(data_type_dict, command_dict)

class AllMotors:
    def __init__(self, alias, motor_type, time_unit, position_unit, velocity_unit, acceleration_unit, current_unit, voltage_unit, temperature_unit, verbose):
        self.alias = alias
        self.motor_type = motor_type
        self.time_unit = time_unit
        self.position_unit = position_unit
        self.velocity_unit = velocity_unit
        self.acceleration_unit = acceleration_unit
        self.current_unit = current_unit
        self.voltage_unit = voltage_unit
        self.temperature_unit = temperature_unit
        self.verbose = verbose

    def __del__(self):
        pass

class M3(AllMotors):
    def __init__(self, alias, motor_type, time_unit, position_unit, velocity_unit, acceleration_unit, current_unit, voltage_unit, temperature_unit, verbose):
        super().__init__(alias, motor_type, time_unit, position_unit, velocity_unit, acceleration_unit, current_unit, voltage_unit, temperature_unit, verbose)

    def __del__(self):
        super().__del__()

