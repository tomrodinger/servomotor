#!/usr/bin/env python3

import json

def read_and_validate_json(filepath):
    try:
        with open(filepath, 'r') as file:
            data = json.load(file)
            return data
    except json.JSONDecodeError as e:
        print(f"Invalid JSON: {e}")
        return None
    except FileNotFoundError:
        print(f"File not found: {filepath}")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def print_formatted_json(data):
    formatted_json = json.dumps(data, indent=4)
    print(formatted_json)

def main():
    filepath = 'servomotor/motor_commands.json'
    data = read_and_validate_json(filepath)
    if data is not None:
        print_formatted_json(data)
        print("JSON is valid.")


if __name__ == "__main__":
    main()