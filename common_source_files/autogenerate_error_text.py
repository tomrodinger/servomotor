#!/usr/bin/env python3

import json
import os

def generate_header_file(errors, output_path):
    with open(output_path, 'w') as f:
        f.write('#ifndef SRC_ERROR_TEXT_H_\n')
        f.write('#define SRC_ERROR_TEXT_H_\n\n')
        f.write('#include <stdint.h>\n\n')
        
        # Write enum definition
        f.write('typedef enum {\n')
        last_error = errors[-1]
        for error in errors:
            # Add comment for deprecated errors
            comment = ''
            if 'deprecated' in error['long_desc'].lower():
                comment = '        // TO BE DEPRECATED'
            
            # Format each enum entry, don't add comma for last entry
            if error == last_error:
                f.write(f'    {error["enum"]} = {error["code"]}{comment}\n')
            else:
                f.write(f'    {error["enum"]} = {error["code"]},{comment}\n')
        f.write('} error_code_t;\n\n')
        
        # Write error text array as a macro
        f.write('#define ERROR_TEXT_INITIALIZER \\\n')
        for error in errors:
            # Calculate padding for alignment
            message = error['short_desc']
            padding = ' ' * (40 - len(message) - 2)  # -2 for quotes
            
            # Add comment with enum name and deprecated note if applicable
            comment = error["enum"]
            if 'deprecated' in error['long_desc'].lower():
                comment += " (TO BE DEPRECATED)"
            
            # Write each error message with block comment style
            f.write(f'    "{message}\\0"{padding}/* {comment} */ \\\n')
        
        # Add final null terminator and comment
        f.write('    "\\0"                                      /* this marks the end of the error messages */\n\n')
        
        # Write function declaration
        f.write('const char *get_error_text(uint16_t error_code);\n\n')
        f.write('#endif /* SRC_ERROR_TEXT_H_ */\n')

def main():
    # Read JSON file
    json_path = '../python_programs/servomotor/error_codes.json'
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # Sort errors by code to ensure correct order
    errors = sorted(data['errors'], key=lambda x: x['code'])
    
    # Generate header file
    generate_header_file(errors, '../common_source_files/error_text.h')

if __name__ == '__main__':
    main()
