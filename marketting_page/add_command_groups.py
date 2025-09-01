import json

def add_command_groups(commands):
    # Define command group mappings
    group_mappings = {
        'Basic Control': [
            'Enable MOSFETs',
            'Disable MOSFETs',
            'Emergency stop',
            'System reset',
            'Reset time',
            'Zero position'
        ],
        'Motion Control': [
            'Trapezoid move',
            'Go to position',
            'Move with velocity',
            'Move with acceleration',
            'Multi-move',
            'Homing',
            'Go to closed loop'
        ],
        'Configuration': [
            'Set maximum velocity',
            'Set maximum acceleration',
            'Set maximum motor current',
            'Set PID constants',
            'Set safety limits',
            'Set Max allowable position deviation',
            'Test mode',
            'Start calibration'
        ],
        'Status & Monitoring': [
            'Get status',
            'Get position',
            'Get hall sensor position',
            'Get comprehensive position',
            'Get temperature',
            'Get supply voltage',
            'Get current time',
            'Get update frequency',
            'Get max PID error',
            'Get n queued items',
            'Get hall sensor statistics',
            'Control hall sensor statistics',
            'Get debug values'
        ],
        'Device Management': [
            'Detect devices',
            'Set device alias',
            'Get product info',
            'Get product description',
            'Get firmware version',
            'Firmware upgrade',
            'Identify',
            'Ping',
            'Time sync',
            'Vibrate'
        ]
    }

    # Create reverse mapping for easier lookup
    command_to_group = {}
    for group, command_list in group_mappings.items():
        for command in command_list:
            command_to_group[command] = group

    # Add command group to each command
    for command in commands:
        command_string = command['CommandString']
        command['CommandGroup'] = command_to_group.get(command_string, 'Other')

    return commands

def main():
    # Read the existing JSON file
    with open('motor_commands.json', 'r') as f:
        commands = json.load(f)

    # Add command groups
    modified_commands = add_command_groups(commands)

    # Write back to file with proper formatting
    with open('motor_commands.json', 'w') as f:
        json.dump(modified_commands, f, indent=4)

if __name__ == '__main__':
    main()
