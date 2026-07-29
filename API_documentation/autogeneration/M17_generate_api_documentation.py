#!/usr/bin/env python3
"""
Servomotor API Documentation Generator
Generates comprehensive API documentation in both PDF and Markdown formats
"""

import json
import os
import sys
import re
from datetime import datetime
from collections import defaultdict
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm, inch
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Frame, PageTemplate, PageBreak, Preformatted, KeepTogether
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib import colors
from reportlab.pdfgen.canvas import Canvas

# Import existing utilities
from styles import create_title_style, create_subtitle_style, create_normal_style, create_heading_style
from utils import get_processed_image
from versioning import get_latest_version
from reportlab.platypus import Flowable, Table, TableStyle
from reportlab.lib.utils import simpleSplit

class APIDocumentationGenerator:
    def __init__(self):
        self.motor_commands_path = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/motor_commands.json"
        self.data_types_path = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/data_types.json"
        self.error_codes_path = "/Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/error_codes.json"
        self.error_handling_text_path = "error_handling.txt"
        self.hardware_setup_path = "hardware_setup.md"
        self.knowhow_path = "knowhow.md"
        self.arduino_essentials_path = "arduino_essentials.md"
        self.command_examples_dir = "../../python_programs/command_examples"
        self.firmware_dir_path = "../../firmware/firmware_releases"
        self.valid_products_path = "VALID_PRODUCTS.txt"
        self.install_instructions_path = "install_instructions_example.sh"
        self.servomotor_command_examples_path = "servomotor_command_utility_examples.sh"
        self.commands = []
        self.data_types = []
        self.error_codes = []
        self.commands_by_group = defaultdict(list)
        self.validation_errors = []
        self.latest_firmware = {}
        
    def load_commands(self):
        """Load and parse motor commands from JSON file"""
        try:
            with open(self.motor_commands_path, 'r') as f:
                self.commands = json.load(f)
            print(f"✓ Loaded {len(self.commands)} commands from motor_commands.json")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find motor_commands.json at {self.motor_commands_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"\n❌ ERROR: Failed to parse motor_commands.json: {e}")
            return False
    
    def load_data_types(self):
        """Load and parse data types from JSON file"""
        try:
            with open(self.data_types_path, 'r') as f:
                self.data_types = json.load(f)
            print(f"✓ Loaded {len(self.data_types)} data types from data_types.json")
            # Sort data types by whether they are integers first, then by name
            self.data_types.sort(key=lambda x: (not x.get('is_integer', False), x['data_type']))
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find data_types.json at {self.data_types_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"\n❌ ERROR: Failed to parse data_types.json: {e}")
            return False
    
    def load_error_codes(self):
        """Load and parse error codes from JSON file"""
        try:
            with open(self.error_codes_path, 'r') as f:
                data = json.load(f)
                self.error_codes = data.get('errors', [])
            print(f"✓ Loaded {len(self.error_codes)} error codes from error_codes.json")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find error_codes.json at {self.error_codes_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"\n❌ ERROR: Failed to parse error_codes.json: {e}")
            return False
    
    def load_error_handling_text(self):
        """Load error handling description text"""
        try:
            with open(self.error_handling_text_path, 'r') as f:
                self.error_handling_text = f.read()
            print(f"✓ Loaded error handling text from {self.error_handling_text_path}")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find error handling text at {self.error_handling_text_path}")
            self.error_handling_text = "Error handling description not available."
            return False

    def load_hardware_setup(self):
        """Load the hardware setup markdown section"""
        try:
            with open(self.hardware_setup_path, 'r') as f:
                self.hardware_setup_md = f.read()
            print(f"✓ Loaded hardware setup from {self.hardware_setup_path}")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find hardware setup text at {self.hardware_setup_path}")
            self.hardware_setup_md = None
            return False

    def load_knowhow(self):
        """Load the know-how / best practices markdown section"""
        try:
            with open(self.knowhow_path, 'r') as f:
                self.knowhow_md = f.read()
            print(f"✓ Loaded know-how from {self.knowhow_path}")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find know-how text at {self.knowhow_path}")
            self.knowhow_md = None
            return False

    def load_arduino_essentials(self):
        """Load the Arduino-only essentials section (error checking, environment)"""
        try:
            with open(self.arduino_essentials_path, 'r') as f:
                self.arduino_essentials_md = f.read()
            print(f"✓ Loaded Arduino essentials from {self.arduino_essentials_path}")
            return True
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find Arduino essentials text at {self.arduino_essentials_path}")
            self.arduino_essentials_md = None
            return False

    # The know-how section is a SINGLE shared source (knowhow.md) rendered into both
    # the Python and the Arduino documents, so the two can never drift apart. Two
    # mechanisms adapt it to the reader's language:
    #
    #  1. <!--LANG:PYTHON--> ... <!--LANG:END--> and <!--LANG:ARDUINO--> ... <!--LANG:END-->
    #     blocks, for content that is genuinely different between the libraries
    #     (program skeletons, exception handling, return-value shapes).
    #  2. The curated symbol table below, for prose that is identical apart from the
    #     spelling of a method name. Every entry is an exact symbol verified to exist
    #     in Arduino_library/Servomotor.h. Longest patterns are applied first so that
    #     no substitution can partially consume another.
    ARDUINO_API_SUBSTITUTIONS = [
        ('go_to_closed_loop()', 'goToClosedLoop()'),
        ('get_n_queued_items()', 'getNQueuedItems()'),
        ('move_with_velocity(', 'moveWithVelocity('),
        ('set_position_unit(', 'setPositionUnit('),
        ('emergency_stop()', 'emergencyStop()'),
        ('disable_mosfets()', 'disableMosfets()'),
        ('enable_mosfets()', 'enableMosfets()'),
        ('trapezoid_move(', 'trapezoidMove('),
        ('zero_position()', 'zeroPosition()'),
        ('system_reset()', 'systemReset()'),
        ('get_status()', 'getStatus()'),
        ('time_sync(', 'timeSync('),
    ]

    def render_shared_section(self, md_text, language):
        """Resolve the <!--LANG:...--> blocks in a shared markdown source for one
        language, and (for Arduino) rewrite Python method names into their Arduino
        spellings. Returns markdown."""
        if md_text is None:
            return None
        keep = 'PYTHON' if language == 'python' else 'ARDUINO'
        out_lines = []
        skipping = False
        for line in md_text.split('\n'):
            marker = line.strip()
            if marker.startswith('<!--LANG:'):
                name = marker[len('<!--LANG:'):].split('-->')[0].strip()
                if name == 'END':
                    skipping = False
                else:
                    skipping = (name != keep)
                continue
            if not skipping:
                out_lines.append(line)
        rendered = '\n'.join(out_lines)
        if language == 'arduino':
            for python_symbol, arduino_symbol in self.ARDUINO_API_SUBSTITUTIONS:
                rendered = rendered.replace(python_symbol, arduino_symbol)
        # Collapse the blank-line runs that removing a block can leave behind.
        rendered = re.sub(r'\n{3,}', '\n\n', rendered)
        return rendered

    def load_command_example(self, command):
        """Load the real runnable example program for a command, if one exists"""
        if command['CommandString'] == 'Firmware upgrade':
            # Deliberately no example program: raw page writes brick devices when
            # done wrong. Emit a pointer to the supported tool instead of letting
            # the caller fall back to a synthesized stub with placeholder values.
            return ("# There is intentionally no minimal example for 'Firmware upgrade'.\n"
                    "# Upgrading firmware requires correct page sequencing, model/compatibility\n"
                    "# checks, and CRC handling; use the supported tool instead. It is installed\n"
                    "# as a command by 'pip install servomotor' (version 0.12.0 and later):\n"
                    "#\n"
                    "#   upgrade_firmware -p <PORT> -a <ALIAS> <firmware_file.firmware>\n"
                    "#\n"
                    "# Address ONE motor with -a <ALIAS>. The tool's default is the broadcast\n"
                    "# address 255, which flashes every matching device on the bus at once but\n"
                    "# receives no replies, so it reports every page as written even when nothing\n"
                    "# was written at all.\n"
                    "#\n"
                    "# The model code and firmware compatibility code are checked by the DEVICE,\n"
                    "# not by the tool: the bootloader silently ignores pages whose codes do not\n"
                    "# match its own. Run 'Get product info' first and compare its productCode and\n"
                    "# firmwareCompatibility fields against the <MODEL> and scc<N> parts of the\n"
                    "# file name. See the firmware upgrade section of this document for the full\n"
                    "# procedure.\n")
        method_name = self.get_python_method_name(command['CommandString'])
        example_path = os.path.join(self.command_examples_dir, f"example_{method_name}.py")
        try:
            with open(example_path, 'r') as f:
                return f.read()
        except FileNotFoundError:
            return None

    @staticmethod
    def demote_md_headings(md_text):
        """Add one '#' to every markdown heading so an embedded document
        nests one level below the enclosing section."""
        out_lines = []
        in_code = False
        for line in md_text.split('\n'):
            if line.strip().startswith('```'):
                in_code = not in_code
            if not in_code and line.startswith('#'):
                line = '#' + line
            out_lines.append(line)
        return '\n'.join(out_lines)

    def add_markdown_section_to_pdf(self, story, doc, md_text, heading_style, normal_style, code_style):
        """Render a limited markdown subset (headings, paragraphs, bullets,
        numbered lists, fenced code blocks) into reportlab story elements."""
        sub_heading_style = ParagraphStyle(
            'MdSubHeading', parent=normal_style, fontSize=13,
            fontName='Helvetica-Bold', textColor=colors.black,
            spaceBefore=12, spaceAfter=6)
        sub_sub_heading_style = ParagraphStyle(
            'MdSubSubHeading', parent=normal_style, fontSize=11,
            fontName='Helvetica-Bold', textColor=colors.black,
            spaceBefore=10, spaceAfter=4)

        table_cell_style = ParagraphStyle(
            'MdTableCell', parent=normal_style, fontSize=8, leading=10)
        table_header_cell_style = ParagraphStyle(
            'MdTableHeaderCell', parent=normal_style, fontSize=8, leading=10,
            fontName='Helvetica-Bold', textColor=colors.whitesmoke)

        def escape(text):
            return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')

        def inline(text):
            """Escape for reportlab, then honour **bold** and `code` markup."""
            out = escape(text)
            out = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', out)
            out = re.sub(r'`([^`]+?)`', r'<font face="Courier">\1</font>', out)
            return out

        def split_table_row(row):
            return [cell.strip() for cell in row.strip().strip('|').split('|')]

        def is_table_separator(row):
            cells = split_table_row(row)
            return bool(cells) and all(re.fullmatch(r':?-{2,}:?', c) for c in cells)

        in_code = False
        code_lines = []
        paragraph_lines = []
        table_rows = []
        # A bullet or quote whose source text wraps over several lines is one item.
        # Without this, each continuation line became its own paragraph, which also
        # split any **bold** or `code` span that straddled the line break.
        pending = {'style': None, 'prefix': ''}

        def flush_paragraph():
            if paragraph_lines:
                text = inline(' '.join(paragraph_lines))
                if pending['style'] is not None:
                    story.append(Paragraph(pending['prefix'] + text, pending['style']))
                    story.append(Spacer(1, 3))
                    pending['style'] = None
                    pending['prefix'] = ''
                else:
                    story.append(Paragraph(text, normal_style))
                    story.append(Spacer(1, 6))
                paragraph_lines.clear()

        def flush_table():
            if not table_rows:
                return
            rows = [r for r in table_rows if not is_table_separator(r)]
            table_rows.clear()
            if not rows:
                return
            parsed = [split_table_row(r) for r in rows]
            n_cols = max(len(r) for r in parsed)
            data = []
            for i, cells in enumerate(parsed):
                cells = cells + [''] * (n_cols - len(cells))
                style = table_header_cell_style if i == 0 else table_cell_style
                data.append([Paragraph(inline(c), style) for c in cells])
            table = Table(data, colWidths=[(doc.width - 20) / n_cols] * n_cols, repeatRows=1)
            table.setStyle(TableStyle([
                ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#4a4a4a')),
                ('GRID', (0, 0), (-1, -1), 0.5, colors.grey),
                ('VALIGN', (0, 0), (-1, -1), 'TOP'),
                ('LEFTPADDING', (0, 0), (-1, -1), 4),
                ('RIGHTPADDING', (0, 0), (-1, -1), 4),
                ('TOPPADDING', (0, 0), (-1, -1), 3),
                ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
            ]))
            story.append(table)
            story.append(Spacer(1, 8))

        def flush_all():
            flush_paragraph()
            flush_table()

        for line in md_text.split('\n'):
            stripped = line.strip()
            if stripped.startswith('```'):
                flush_all()
                if in_code:
                    code_box = self.CodeBox('\n'.join(code_lines), doc.width - 20, code_style)
                    story.append(code_box)
                    story.append(Spacer(1, 8))
                    code_lines.clear()
                in_code = not in_code
                continue
            if in_code:
                code_lines.append(line)
                continue
            if stripped.startswith('|') and stripped.endswith('|'):
                flush_paragraph()
                table_rows.append(stripped)
                continue
            flush_table()
            if stripped.startswith('# '):
                flush_paragraph()
                story.append(Paragraph(inline(stripped[2:]), heading_style))
                story.append(Spacer(1, 8))
            elif stripped.startswith('## '):
                flush_paragraph()
                story.append(Paragraph(inline(stripped[3:]), sub_heading_style))
            elif stripped.startswith('### '):
                flush_paragraph()
                story.append(Paragraph(inline(stripped[4:]), sub_sub_heading_style))
            elif stripped.startswith('> '):
                flush_paragraph()
                pending['style'] = normal_style
                pending['prefix'] = ''
                paragraph_lines.append(stripped[2:])
            elif stripped.startswith('- '):
                flush_paragraph()
                pending['style'] = normal_style
                pending['prefix'] = '• '
                paragraph_lines.append(stripped[2:])
            elif len(stripped) > 2 and stripped[0].isdigit() and '. ' in stripped[:4]:
                flush_paragraph()
                num, _, rest = stripped.partition('. ')
                pending['style'] = normal_style
                pending['prefix'] = f'<b>{escape(num)}.</b> '
                paragraph_lines.append(rest)
            elif stripped == '':
                flush_paragraph()
            else:
                paragraph_lines.append(stripped)
        flush_all()


    UNIT_NOTES = {
        'seconds': 'time in seconds',
        'milliseconds': 'time in milliseconds',
        'minutes': 'time in minutes',
        'microseconds': 'time in microseconds',
        'timesteps': 'raw internal time unit of 32 microseconds (31,250 per second)',
        'shaft_rotations': 'rotations of the motor shaft',
        'degrees': 'degrees of rotation',
        'radians': 'radians of rotation',
        'encoder_counts': 'raw encoder counts (3,276,800 per shaft rotation)',
        'rotations_per_second': 'rotations per second',
        'rpm': 'revolutions per minute',
        'degrees_per_second': 'degrees per second',
        'radians_per_second': 'radians per second',
        'counts_per_second': 'encoder counts per second',
        'counts_per_timestep': 'encoder counts per 32-microsecond timestep',
        'rotations_per_second_squared': 'rotations per second squared',
        'rpm_per_second': 'RPM per second',
        'degrees_per_second_squared': 'degrees per second squared',
        'radians_per_second_squared': 'radians per second squared',
        'counts_per_second_squared': 'encoder counts per second squared',
        'counts_per_timestep_squared': 'encoder counts per timestep squared',
        'internal_current_units': 'raw internal current units (about 150-200 is a typical working value)',
        'milliamps': 'milliamperes',
        'amps': 'amperes',
        'millivolts': 'millivolts',
        'volts': 'volts',
        'celsius': 'degrees Celsius',
        'fahrenheit': 'degrees Fahrenheit',
        'kelvin': 'kelvin',
    }

    def build_unit_reference(self):
        """Build the unit reference from unit_conversions_M3.json — the same file
        the library builds its unit enums from, so the documented lists and
        defaults can never drift from the code. The FIRST unit of each list is
        the library's default for that unit type."""
        conversions_path = os.path.join(os.path.dirname(self.motor_commands_path),
                                        'unit_conversions_M3.json')
        with open(conversions_path, 'r') as f:
            units = json.load(f)['units']
        sections = []
        for type_name, unit_list in units.items():
            lines = []
            for i, unit in enumerate(unit_list):
                note = self.UNIT_NOTES.get(unit, unit.replace('_', ' '))
                suffix = ' (default)' if i == 0 else ''
                lines.append(f"{unit} - {note}{suffix}")
            sections.append((f"{type_name.capitalize()} Units", lines))
        return sections

    def load_install_instructions(self):
        """Load installation instructions from the symlinked file"""
        try:
            with open(self.install_instructions_path, 'r') as f:
                instructions = f.read().strip()
            print(f"✓ Loaded installation instructions from {self.install_instructions_path}")
            return instructions
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find installation instructions at {self.install_instructions_path}")
            return None
    
    def load_servomotor_command_examples(self):
        """Load servomotor command utility examples"""
        try:
            with open(self.servomotor_command_examples_path, 'r') as f:
                examples = f.read().strip()
            print(f"✓ Loaded servomotor command examples from {self.servomotor_command_examples_path}")
            return examples
        except FileNotFoundError:
            print(f"\n❌ ERROR: Could not find servomotor command examples at {self.servomotor_command_examples_path}")
            return None
    
    def convert_to_windows_commands(self, unix_commands):
        """Convert Unix/macOS installation commands to Windows equivalents"""
        lines = unix_commands.split('\n')
        windows_lines = []
        
        for line in lines:
            line = line.strip()
            if line.startswith('#'):
                # Keep comments as-is
                windows_lines.append(line)
            elif 'python3 -m venv venv' in line:
                windows_lines.append('python -m venv venv')
            elif 'source venv/bin/activate' in line:
                windows_lines.append('venv\\Scripts\\activate')
            elif 'pip3 install' in line:
                # Convert pip3 to pip for Windows
                windows_lines.append(line.replace('pip3', 'pip'))
            elif line:
                # Keep other non-empty lines as-is
                windows_lines.append(line)
        
        return '\n'.join(windows_lines)
    
    def load_valid_products(self):
        """Load valid product names from VALID_PRODUCTS.txt"""
        try:
            with open(self.valid_products_path, 'r') as f:
                products = [line.strip() for line in f if line.strip()]
            print(f"✓ Loaded {len(products)} valid products: {', '.join(products)}")
            return products
        except FileNotFoundError:
            print(f"⚠️  Warning: Could not find VALID_PRODUCTS.txt")
            return []
    
    def find_latest_firmware(self):
        """Find the latest firmware file for each valid product"""
        valid_products = self.load_valid_products()
        if not valid_products:
            return {}
        
        latest_firmware = {}
        
        try:
            # List all firmware files
            firmware_files = os.listdir(self.firmware_dir_path)
            
            for product in valid_products:
                # Find all firmware files for this product
                product_files = [f for f in firmware_files if f.startswith(product)]
                
                if product_files:
                    # Parse firmware versions and find the latest
                    latest_file = None
                    latest_version = None
                    
                    for filename in product_files:
                        # Extract version using regex (looking for fw followed by version numbers)
                        match = re.search(r'fw(\d+\.\d+\.\d+\.\d+)', filename)
                        if match:
                            version_str = match.group(1)
                            version_tuple = tuple(map(int, version_str.split('.')))
                            
                            if latest_version is None or version_tuple > latest_version:
                                latest_version = version_tuple
                                latest_file = filename
                    
                    if latest_file:
                        # Extract model name for display
                        model_match = re.search(r'servomotor_(M\d+)', latest_file)
                        if model_match:
                            model_name = f"Model {model_match.group(1)}"
                            latest_firmware[model_name] = latest_file
                            print(f"✓ Found latest firmware for {model_name}: {latest_file}")
            
            self.latest_firmware = latest_firmware
            return latest_firmware
            
        except FileNotFoundError:
            print(f"⚠️  Warning: Firmware directory not found at {self.firmware_dir_path}")
            return {}
        except Exception as e:
            print(f"⚠️  Warning: Error scanning firmware directory: {e}")
            return {}
    
    def validate_commands(self):
        """Validate that all commands have CommandGroup defined"""
        print("\nValidating commands...")
        commands_without_group = []
        
        for i, cmd in enumerate(self.commands):
            if 'CommandGroup' not in cmd or not cmd['CommandGroup']:
                commands_without_group.append({
                    'index': i,
                    'name': cmd.get('CommandString', 'Unknown'),
                    'enum': cmd.get('CommandEnum', 'Unknown')
                })
            else:
                # Group commands by category
                self.commands_by_group[cmd['CommandGroup']].append(cmd)
        
        if commands_without_group:
            print("\n" + "="*80)
            print("❌ VALIDATION ERROR: Commands without CommandGroup")
            print("="*80)
            print("\nThe following commands are missing the 'CommandGroup' field:")
            for cmd in commands_without_group:
                print(f"  - Index {cmd['index']}: {cmd['name']} (Enum: {cmd['enum']})")
            
            print("\n📝 TO FIX THIS ERROR:")
            print("1. Open: /Users/tom/Documents/Move_the_Needle/Servomotor/python_programs/servomotor/motor_commands.json")
            print("2. Add a 'CommandGroup' field to each command listed above")
            print("3. Valid group names include: 'Basic Control', 'Motion Control', 'Configuration', 'Status & Monitoring', 'Device Management'")
            print("4. Example: \"CommandGroup\": \"Basic Control\"")
            print("\n" + "="*80)
            return False
        
        print(f"✓ All {len(self.commands)} commands have valid CommandGroup fields")
        print(f"✓ Found {len(self.commands_by_group)} command groups:")
        for group in sorted(self.commands_by_group.keys()):
            print(f"  - {group}: {len(self.commands_by_group[group])} commands")
        return True
    
    def get_python_method_name(self, command_string):
        """Convert command string to Python method name"""
        # Handle special cases
        special_cases = {
            "Disable MOSFETs": "disable_mosfets",
            "Enable MOSFETs": "enable_mosfets",
            "Emergency stop": "emergency_stop",
            "System reset": "system_reset",
            "Get n queued items": "get_n_queued_items",
            "Zero position": "zero_position"
        }
        
        if command_string in special_cases:
            return special_cases[command_string]
        
        # General conversion: lowercase and replace spaces with underscores
        return command_string.lower().replace(' ', '_').replace('-', '_')
    
    def generate_python_example(self, command):
        """Generate Python code example for a command with explicit variable assignments"""
        method_name = self.get_python_method_name(command['CommandString'])
        
        example_lines = []
        MAX_LINE_LENGTH = 70  # Maximum characters per line for PDF display
        
        # Handle input parameters
        param_vars = []
        if command['Input'] and command['Input'] != "null":
            for param in command['Input']:
                param_name = param.get('ParameterName', 'value')
                var_name = f"{param_name}_value"
                example_lines.append(f"{var_name} = 0  # Replace with your desired value")
                param_vars.append(f"{param_name}={var_name}")
            if param_vars:
                example_lines.append("")
        
        # Handle the function call
        if command['Output'] and command['Output'] != "success_response":
            # Command returns values
            if isinstance(command['Output'], list):
                result_vars = [out.get('ParameterName', f'value{i}') for i, out in enumerate(command['Output'])]
                
                # Check if we need to wrap the return variables
                result_vars_str = ', '.join(result_vars)
                
                if len(command['Output']) == 1:
                    # Single return value
                    result_var = result_vars[0]
                    call_base = f"{result_var} = motor.{method_name}"
                else:
                    # Multiple return values - check if we need to wrap them
                    if len(result_vars_str) > 40:  # Break return values into multiple lines
                        example_lines.append("(")
                        for i, var in enumerate(result_vars):
                            if i < len(result_vars) - 1:
                                example_lines.append(f"    {var},")
                            else:
                                example_lines.append(f"    {var}")
                        call_base = f") = motor.{method_name}"
                    else:
                        call_base = f"{result_vars_str} = motor.{method_name}"
                
                # Build the function call
                if param_vars:
                    # Check if the entire line would be too long
                    full_line = f"{call_base}({', '.join(param_vars)})"
                    if len(full_line) > MAX_LINE_LENGTH or len(param_vars) > 2:
                        # Wrap parameters
                        if len(result_vars) > 1 and len(result_vars_str) > 40:
                            # Already started multi-line for return values
                            example_lines.append(f") = motor.{method_name}(")
                        else:
                            example_lines.append(f"{call_base}(")
                        for i, param in enumerate(param_vars):
                            if i < len(param_vars) - 1:
                                example_lines.append(f"    {param},")
                            else:
                                example_lines.append(f"    {param}")
                        example_lines.append(")")
                    else:
                        example_lines.append(full_line)
                else:
                    # No parameters
                    if len(result_vars) > 1 and len(result_vars_str) > 40:
                        example_lines.append(f") = motor.{method_name}()")
                    else:
                        example_lines.append(f"{call_base}()")
                
                # Add print statements
                for var in result_vars:
                    example_lines.append(f"print(f\"{var}: {{{var}}}\")")
            else:
                # Non-list output
                if param_vars:
                    full_line = f"response = motor.{method_name}({', '.join(param_vars)})"
                    if len(full_line) > MAX_LINE_LENGTH:
                        example_lines.append(f"response = motor.{method_name}(")
                        for i, param in enumerate(param_vars):
                            if i < len(param_vars) - 1:
                                example_lines.append(f"    {param},")
                            else:
                                example_lines.append(f"    {param}")
                        example_lines.append(")")
                    else:
                        example_lines.append(full_line)
                else:
                    example_lines.append(f"response = motor.{method_name}()")
                example_lines.append("print(f\"Response: {response}\")")
        else:
            # Command doesn't return values (just success response)
            if param_vars:
                # Check if the entire line would be too long
                full_line = f"motor.{method_name}({', '.join(param_vars)})"
                if len(full_line) > MAX_LINE_LENGTH or len(param_vars) > 2:
                    # Wrap parameters
                    example_lines.append(f"motor.{method_name}(")
                    for i, param in enumerate(param_vars):
                        if i < len(param_vars) - 1:
                            example_lines.append(f"    {param},")
                        else:
                            example_lines.append(f"    {param}")
                    example_lines.append(")")
                else:
                    example_lines.append(full_line)
            else:
                example_lines.append(f"motor.{method_name}()")
        
        return '\n'.join(example_lines)
    
    def generate_markdown(self):
        """Generate both Python and Arduino Markdown documentation"""
        # Generate Python documentation
        self.generate_python_markdown()
        # Generate Arduino documentation
        self.generate_arduino_markdown()
        return True
    
    def generate_arduino_example(self, command):
        """Generate Arduino code example for a command - simplified without Serial output"""
        cmd_string = command['CommandString']
        method_name = ''.join(word.capitalize() if i > 0 else word.lower()
                             for i, word in enumerate(cmd_string.split()))
        
        example_lines = []
        example_lines.append(f"// {cmd_string}")
        
        # Handle input parameters - declare variables
        param_names = []
        if command['Input'] and command['Input'] != "null":
            for param in command['Input']:
                param_name = param.get('ParameterName', 'value')
                param_desc = param.get('Description', '')
                
                # Determine data type and value based on description
                if 'float' in param_desc.lower() or 'f32' in param_desc:
                    data_type = "float"
                    default_value = "10.0"
                    if 'velocity' in param_name.lower():
                        default_value = "2.0"
                    elif 'acceleration' in param_name.lower():
                        default_value = "4.0"
                    elif 'time' in param_name.lower() or 'duration' in param_name.lower():
                        default_value = "3.0"
                elif 'bool' in param_desc.lower():
                    data_type = "bool"
                    default_value = "true"
                elif 'u32' in param_desc or 'uint32' in param_desc.lower():
                    data_type = "uint32_t"
                    default_value = "1000"
                elif 'i32' in param_desc or 'int32' in param_desc.lower():
                    data_type = "int32_t"
                    default_value = "1000"
                elif 'u16' in param_desc or 'uint16' in param_desc.lower():
                    data_type = "uint16_t"
                    default_value = "100"
                elif 'i16' in param_desc or 'int16' in param_desc.lower():
                    data_type = "int16_t"
                    default_value = "100"
                elif 'u8' in param_desc or 'uint8' in param_desc.lower():
                    data_type = "uint8_t"
                    default_value = "1"
                elif 'i8' in param_desc or 'int8' in param_desc.lower():
                    data_type = "int8_t"
                    default_value = "1"
                else:
                    data_type = "float"
                    default_value = "0"
                
                example_lines.append(f"{data_type} {param_name} = {default_value};")
                param_names.append(param_name)
            
            if param_names:
                example_lines.append("")
        
        # Handle the function call and return values
        if command['Output'] and command['Output'] != "success_response":
            # Command returns values - Always returns a structure in Arduino
            # Structure name is based on method name + Response
            struct_name = f"{method_name}Response"
            
            # Instantiate the structure and call the function
            if param_names:
                example_lines.append(f"{struct_name} response = motor.{method_name}({', '.join(param_names)});")
            else:
                example_lines.append(f"{struct_name} response = motor.{method_name}();")
        else:
            # Command doesn't return values (just success response)
            if param_names:
                example_lines.append(f"motor.{method_name}({', '.join(param_names)});")
            else:
                example_lines.append(f"motor.{method_name}();")
        
        return '\n'.join(example_lines)
    
    def generate_python_markdown(self):
        """Generate Python Markdown documentation"""
        print("\nGenerating Python Markdown documentation...")
        
        md_content = []
        md_content.append("# Servomotor Python API Documentation\n")
        md_content.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # Add firmware information if available
        if self.latest_firmware:
            md_content.append("## Latest Firmware Versions\n")
            md_content.append("At the time of generating this API reference, the latest released firmware versions for the servomotors are:\n")
            for model, firmware_file in sorted(self.latest_firmware.items()):
                md_content.append(f"- **{model}**: `{firmware_file}`\n")
            md_content.append("\nIf you are experiencing problems, you can try to set the firmware of your product to this version and try again, and report the problem to us using the feedback page.\n")
        
        # Table of Contents
        md_content.append("## Table of Contents\n")
        md_content.append("1. [Hardware Setup](#hardware-setup)")
        md_content.append("2. [Install the Python Library](#install-the-python-library)")
        md_content.append("3. [Controlling the Servomotor From the Command Line](#controlling-the-servomotor-from-the-command-line)")
        md_content.append("4. [Getting Started](#getting-started)")
        md_content.append("5. [Know-How, Best Practices, and Gotchas](#know-how-best-practices-and-gotchas)")
        md_content.append("6. [Data Types](#data-types)")
        md_content.append("7. [Command Reference](#command-reference)")
        toc_index = 8
        for group in sorted(self.commands_by_group.keys()):
            anchor = group.lower().replace('&', '').replace(' ', '-')
            md_content.append(f"{toc_index}. [{group}](#{anchor})")
            toc_index += 1
        md_content.append(f"{toc_index}. [Unit Conversions](#unit-conversions)")
        toc_index += 1
        md_content.append(f"{toc_index}. [Error Handling](#error-handling)")
        toc_index += 1
        md_content.append(f"{toc_index}. [Error Codes](#error-codes)\n")

        # Hardware Setup Section (embedded markdown, headings demoted one level)
        if getattr(self, 'hardware_setup_md', None):
            md_content.append(self.demote_md_headings(self.hardware_setup_md) + "\n")

        # Install the Python Library Section
        md_content.append("## Install the Python Library\n")
        md_content.append("You need to install the servomotor Python library before you can use it in your code. Run this command:\n")
        md_content.append("```bash")
        md_content.append("pip3 install servomotor")
        md_content.append("```\n")
        
        # Load installation instructions from symlinked file
        install_instructions = self.load_install_instructions()
        if install_instructions is None:
            print("\n" + "="*80)
            print("❌ FATAL ERROR: Installation instructions file not found")
            print("="*80)
            print(f"\nCould not find installation instructions at: {self.install_instructions_path}")
            print("\n📝 TO FIX THIS ERROR:")
            print("1. Ensure the symlink exists:")
            print(f"   ls -la {self.install_instructions_path}")
            print("2. If the symlink is broken, recreate it:")
            print(f"   ln -sf ../../python_programs/install_instructions_example.sh {self.install_instructions_path}")
            print("3. Ensure the source file exists:")
            print("   ls -la ../../python_programs/install_instructions_example.sh")
            print("\n" + "="*80)
            return False
        
        md_content.append("If you want to work in a virtual environment, you can create it, activate it, and install the library:\n")
        md_content.append("**For macOS/Linux:**")
        md_content.append("```bash")
        md_content.append(install_instructions)
        md_content.append("```\n")
        
        # Convert to Windows commands
        windows_instructions = self.convert_to_windows_commands(install_instructions)
        md_content.append("**For Windows:**")
        md_content.append("```bash")
        md_content.append(windows_instructions)
        md_content.append("```\n")
        md_content.append("After installation, you can verify the servomotor library is installed correctly by running:")
        md_content.append("```bash")
        md_content.append("python3 -c \"import servomotor; print('Servomotor library installed successfully!')\"")
        md_content.append("```\n")
        
        # Controlling the Servomotor From the Command Line Section
        md_content.append("## Controlling the Servomotor From the Command Line\n")
        md_content.append("You can send commands to the servomotor from the command line using the servomotor_command utility, which gets installed along with the Python library. Make sure to install that library according to the instructions above. After installation, the servomotor_command program should be in the path. You can try running some of the following commands to communicate with the servomotor(s):\n")
        
        # Load and include servomotor command examples
        command_examples = self.load_servomotor_command_examples()
        if command_examples is None:
            print("\n" + "="*80)
            print("❌ FATAL ERROR: Servomotor command examples file not found")
            print("="*80)
            print(f"\nCould not find servomotor command examples at: {self.servomotor_command_examples_path}")
            print("\n📝 TO FIX THIS ERROR:")
            print("1. Ensure the file exists:")
            print(f"   ls -la {self.servomotor_command_examples_path}")
            print("2. Create the file if missing with example commands")
            print("\n" + "="*80)
            return False
        
        md_content.append("```bash")
        md_content.append(command_examples)
        md_content.append("```\n")
        
        # Getting Started Section
        md_content.append("## Getting Started\n")
        md_content.append("This section provides a complete example showing how to initialize and control a servomotor.\n")
        md_content.append("### Complete Example Program\n")
        md_content.append("```python")
        
        # Read and include the example file
        try:
            with open('python_library_example.py', 'r') as f:
                example_code = f.read()
            md_content.append(example_code)
        except FileNotFoundError:
            md_content.append("# Example file not found")
        
        md_content.append("```\n")

        # Know-How Section (embedded markdown, headings demoted one level)
        if getattr(self, 'knowhow_md', None):
            knowhow = self.render_shared_section(self.knowhow_md, 'python')
            md_content.append(self.demote_md_headings(knowhow) + "\n")

        # Data Types Section
        md_content.append("## Data Types\n")
        md_content.append("This section describes the various data types used in the Servomotor API commands.\n")
        
        # Add data types information if loaded
        if self.data_types:
            # Separate integer and non-integer types
            integer_types = [dt for dt in self.data_types if dt.get('is_integer', False)]
            special_types = [dt for dt in self.data_types if not dt.get('is_integer', False)]
            
            md_content.append("### Integer Data Types\n")
            md_content.append("| Type | Size (bytes) | Range | Description |")
            md_content.append("|------|--------------|-------|-------------|")
            for dt in integer_types:
                min_val = dt.get('min_value', 'N/A')
                max_val = dt.get('max_value', 'N/A')
                if min_val != 'N/A' and max_val != 'N/A':
                    range_str = f"{min_val:,} to {max_val:,}"
                else:
                    range_str = "N/A"
                md_content.append(f"| {dt['data_type']} | {dt['size']} | {range_str} | {dt['description']} |")
            md_content.append("")
            
            md_content.append("### Special Data Types\n")
            md_content.append("| Type | Size (bytes) | Description |")
            md_content.append("|------|--------------|-------------|")
            for dt in special_types:
                size = dt.get('size', 'Variable')
                if size is None:
                    size = 'Variable'
                md_content.append(f"| {dt['data_type']} | {size} | {dt['description']} |")
            md_content.append("")
        
        # Command Reference
        md_content.append("## Command Reference\n")
        md_content.append("This section documents all available commands organized by category.\n")
        
        # Process each command group
        for group in sorted(self.commands_by_group.keys()):
            md_content.append(f"### {group}\n")
            
            for cmd in self.commands_by_group[group]:
                # Command name and description - make more prominent
                md_content.append(f"## 🔧 {cmd['CommandString']}\n")
                md_content.append(f"**Description:** {cmd['Description']}\n")
                
                # Parameters
                if cmd['Input'] and cmd['Input'] != "null":
                    md_content.append("**Parameters:**")
                    for param in cmd['Input']:
                        param_name = param.get('ParameterName', 'unknown')
                        param_desc = param.get('Description', 'No description')
                        md_content.append(f"- `{param_name}`: {param_desc}")
                        
                        # Add unit conversion info if available
                        if 'UnitConversion' in param:
                            unit_type = param['UnitConversion'].get('Type', '')
                            internal_unit = param['UnitConversion'].get('InternalUnit', '')
                            if unit_type:
                                md_content.append(f"  - Unit type: {unit_type} (internal: {internal_unit})")
                    md_content.append("")
                
                # Return values
                if cmd['Output'] and cmd['Output'] != "success_response":
                    md_content.append("**Returns:**")
                    if isinstance(cmd['Output'], list):
                        for output in cmd['Output']:
                            out_name = output.get('ParameterName', 'unknown')
                            out_desc = output.get('Description', 'No description')
                            md_content.append(f"- `{out_name}`: {out_desc}")
                    md_content.append("")
                
                # Python example: prefer the real runnable example program
                real_example = self.load_command_example(cmd)
                if real_example:
                    md_content.append("**Example program:**")
                    md_content.append("```python")
                    md_content.append(real_example.rstrip())
                    md_content.append("```\n")
                else:
                    md_content.append("**Example:**")
                    md_content.append("```python")
                    md_content.append(self.generate_python_example(cmd))
                    md_content.append("```\n")
        
        # Unit Conversions Section
        md_content.append("## Unit Conversions\n")
        md_content.append("The servomotor library supports multiple unit systems for convenience.\n")
        for section_title, unit_lines in self.build_unit_reference():
            md_content.append(f"### {section_title}")
            for line in unit_lines:
                unit, _, note = line.partition(' - ')
                md_content.append(f"- `{unit}`: {note}")
            md_content.append("")
        
        md_content.append("### Setting Units")
        md_content.append("You can set the units for a motor instance during initialization or at runtime:\n")
        md_content.append("```python")
        md_content.append("# During initialization (the alias is a single character, an integer 0-251,")
        md_content.append("# or a 64-bit unique ID passed as an int)")
        md_content.append("motor = servomotor.M3(")
        md_content.append("    'X',")
        md_content.append("    time_unit='seconds',")
        md_content.append("    position_unit='degrees',")
        md_content.append("    velocity_unit='rpm',")
        md_content.append("    acceleration_unit='rpm_per_second'")
        md_content.append(")")
        md_content.append("")
        md_content.append("# At runtime")
        md_content.append("motor.set_position_unit('radians')")
        md_content.append("motor.set_velocity_unit('rotations_per_second')")
        md_content.append("```\n")
        
        # Error Handling Section
        md_content.append("## Error Handling\n")
        if hasattr(self, 'error_handling_text'):
            md_content.append(self.error_handling_text + "\n")
        else:
            md_content.append("Error handling description not available.\n")
        
        # Error Codes Section
        md_content.append("## Error Codes\n")
        md_content.append("This section lists all possible error codes that can be returned by the servomotor.\n")
        
        if self.error_codes:
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue
                    
                md_content.append(f"### Error {error['code']}: {error['enum']}\n")
                md_content.append(f"**Short Description:** {error['short_desc']}\n")
                md_content.append(f"**Description:** {error['long_desc']}\n")
                
                if error.get('causes'):
                    md_content.append("**Possible Causes:**")
                    for cause in error['causes']:
                        md_content.append(f"- {cause}")
                    md_content.append("")
                
                if error.get('solutions'):
                    md_content.append("**Solutions:**")
                    for solution in error['solutions']:
                        md_content.append(f"- {solution}")
                    md_content.append("")
        else:
            md_content.append("Error codes not available.\n")
        
        # Write to file in parent directory
        output_file = "../M17_servomotor_Python_API_documentation.md"
        with open(output_file, 'w') as f:
            f.write('\n'.join(md_content))
        
        print(f"✓ Generated Python Markdown documentation: {output_file}")
        return True
    def generate_data_types_for_pdf(self, story, doc, heading_style, normal_style):
        """Generate Data Types section for PDF - shared by Python and Arduino"""
        story.append(Paragraph('Data Types', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section describes the various data types used in the Servomotor API commands.', normal_style))
        story.append(Spacer(1, 12))
        
        # Separate integer and non-integer types
        integer_types = [dt for dt in self.data_types if dt.get('is_integer', False)]
        special_types = [dt for dt in self.data_types if not dt.get('is_integer', False)]
        
        # Integer Data Types
        story.append(Paragraph('<b>Integer Data Types</b>', heading_style))
        story.append(Spacer(1, 8))
        
        # Create style for table cells
        table_cell_style = ParagraphStyle(
            'TableCell',
            parent=normal_style,
            fontSize=9,
            leading=11
        )
        
        table_header_style = ParagraphStyle(
            'TableHeader',
            parent=normal_style,
            fontSize=10,
            fontName='Helvetica-Bold',
            textColor=colors.whitesmoke
        )
        
        # Create table for integer types
        int_table_data = [[
            Paragraph('Type', table_header_style),
            Paragraph('Size (bytes)', table_header_style),
            Paragraph('Range', table_header_style),
            Paragraph('Description', table_header_style)
        ]]
        
        for dt in integer_types:
            min_val = dt.get('min_value', 'N/A')
            max_val = dt.get('max_value', 'N/A')
            if min_val != 'N/A' and max_val != 'N/A':
                range_str = f"{min_val:,} to {max_val:,}"
            else:
                range_str = "N/A"
            
            int_table_data.append([
                Paragraph(dt['data_type'], table_cell_style),
                Paragraph(str(dt['size']), table_cell_style),
                Paragraph(range_str, table_cell_style),
                Paragraph(dt['description'], table_cell_style)
            ])
        
        int_table = Table(int_table_data, colWidths=[50, 50, 100, doc.width - 200])
        int_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#34a853')),
            ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
            ('GRID', (0, 0), (-1, -1), 1, colors.black),
            ('BOX', (0, 0), (-1, -1), 1, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ]))
        story.append(int_table)
        story.append(Spacer(1, 12))
        
        # Special Data Types
        story.append(Paragraph('<b>Special Data Types</b>', heading_style))
        story.append(Spacer(1, 8))
        
        # Create table for special types
        special_table_data = [[
            Paragraph('Type', table_header_style),
            Paragraph('Size (bytes)', table_header_style),
            Paragraph('Description', table_header_style)
        ]]
        
        for dt in special_types:
            size = dt.get('size', 'Variable')
            if size is None:
                size = 'Variable'
            
            special_table_data.append([
                Paragraph(dt['data_type'], table_cell_style),
                Paragraph(str(size), table_cell_style),
                Paragraph(dt['description'], table_cell_style)
            ])
        
        special_table = Table(special_table_data, colWidths=[70, 60, doc.width - 130])
        special_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#34a853')),
            ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
            ('GRID', (0, 0), (-1, -1), 1, colors.black),
            ('BOX', (0, 0), (-1, -1), 1, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ]))
        story.append(special_table)
        story.append(Spacer(1, 12))
    
    
    def generate_arduino_markdown(self):
        """Generate Arduino Markdown documentation"""
        print("\nGenerating Arduino Markdown documentation...")
        
        md_content = []
        md_content.append("# Servomotor Arduino API Documentation\n")
        md_content.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # Add firmware information if available
        if self.latest_firmware:
            md_content.append("## Latest Firmware Versions\n")
            md_content.append("At the time of generating this API reference, the latest released firmware versions for the servomotors are:\n")
            for model, firmware_file in sorted(self.latest_firmware.items()):
                md_content.append(f"- **{model}**: `{firmware_file}`\n")
            md_content.append("\nIf you are experiencing problems, you can try to set the firmware of your product to this version and try again.\n")
        
        # Table of Contents
        md_content.append("## Table of Contents\n")
        md_content.append("1. [Hardware Setup](#hardware-setup)")
        md_content.append("2. [Getting Started](#getting-started)")
        md_content.append("3. [Arduino Essentials: Checking for Errors and Setting Up Your Environment](#arduino-essentials-checking-for-errors-and-setting-up-your-environment)")
        md_content.append("4. [Know-How, Best Practices, and Gotchas](#know-how-best-practices-and-gotchas)")
        md_content.append("5. [Data Types](#data-types)")
        md_content.append("6. [Command Reference](#command-reference)")
        toc_index = 7
        for group in sorted(self.commands_by_group.keys()):
            anchor = group.lower().replace('&', '').replace(' ', '-')
            md_content.append(f"{toc_index}. [{group}](#{anchor})")
            toc_index += 1
        md_content.append(f"{toc_index}. [Error Handling](#error-handling)")
        toc_index += 1
        md_content.append(f"{toc_index}. [Error Codes](#error-codes)\n")

        # Hardware Setup Section (embedded markdown, headings demoted one level)
        if getattr(self, 'hardware_setup_md', None):
            md_content.append(self.demote_md_headings(self.hardware_setup_md) + "\n")

        # Getting Started Section
        md_content.append("## Getting Started\n")
        md_content.append("This section provides a complete example showing how to control a servomotor with Arduino.\n")
        md_content.append("### Trapezoid Move Example\n")
        md_content.append("```cpp")
        
        # Read and include the Arduino example file
        try:
            with open('arduino_library_example.cpp', 'r') as f:
                example_code = f.read()
            md_content.append(example_code)
        except FileNotFoundError:
            md_content.append("// Example file arduino_library_example.cpp not found")

        md_content.append("```\n")

        # Arduino Essentials (error checking + environment) — Arduino only, and
        # deliberately placed before the command reference: a reader who stops
        # after Getting Started must still have seen how to detect failures.
        if getattr(self, 'arduino_essentials_md', None):
            essentials = self.render_shared_section(self.arduino_essentials_md, 'arduino')
            md_content.append(self.demote_md_headings(essentials) + "\n")

        # Know-How Section — the same shared source the Python document uses,
        # rendered with Arduino method names and Arduino code blocks.
        if getattr(self, 'knowhow_md', None):
            knowhow = self.render_shared_section(self.knowhow_md, 'arduino')
            md_content.append(self.demote_md_headings(knowhow) + "\n")

        # Data Types Section
        md_content.append("## Data Types\n")
        md_content.append("This section describes the various data types used in the Servomotor Arduino API.\n")
        
        # Add data types information if loaded
        if self.data_types:
            # Separate integer and non-integer types
            integer_types = [dt for dt in self.data_types if dt.get('is_integer', False)]
            special_types = [dt for dt in self.data_types if not dt.get('is_integer', False)]
            
            md_content.append("### Integer Data Types\n")
            md_content.append("| Type | Size (bytes) | Range | Description |")
            md_content.append("|------|--------------|-------|-------------|")
            for dt in integer_types:
                min_val = dt.get('min_value', 'N/A')
                max_val = dt.get('max_value', 'N/A')
                if min_val != 'N/A' and max_val != 'N/A':
                    range_str = f"{min_val:,} to {max_val:,}"
                else:
                    range_str = "N/A"
                md_content.append(f"| {dt['data_type']} | {dt['size']} | {range_str} | {dt['description']} |")
            md_content.append("")
            
            md_content.append("### Special Data Types\n")
            md_content.append("| Type | Size (bytes) | Description |")
            md_content.append("|------|--------------|-------------|")
            for dt in special_types:
                size = dt.get('size', 'Variable')
                if size is None:
                    size = 'Variable'
                md_content.append(f"| {dt['data_type']} | {size} | {dt['description']} |")
            md_content.append("")
        
        # Command Reference
        md_content.append("## Command Reference\n")
        md_content.append("This section documents all available commands organized by category.\n")
        
        # Generate commands by group for Arduino
        for group in sorted(self.commands_by_group.keys()):
            md_content.append(f"### {group}\n")
            
            for cmd in self.commands_by_group[group]:
                # Command name
                md_content.append(f"#### {cmd['CommandString']}\n")
                
                # Description
                md_content.append(f"**Description:** {cmd['Description']}\n")
                
                # Parameters
                if cmd['Input']:
                    md_content.append("**Parameters:**")
                    for param in cmd['Input']:
                        param_desc = param.get('Description', 'No description')
                        param_name = param.get('ParameterName', 'parameter')
                        md_content.append(f"- `{param_name}`: {param_desc}")
                    md_content.append("")
                
                # Return values
                if cmd['Output'] and cmd['Output'] != "success_response":
                    if isinstance(cmd['Output'], list):
                        md_content.append("**Returns:**")
                        for output in cmd['Output']:
                            out_name = output.get('ParameterName', 'unknown')
                            out_desc = output.get('Description', 'No description')
                            md_content.append(f"- `{out_name}`: {out_desc}")
                        md_content.append("")
                
                # Arduino example - use the proper generator function
                md_content.append("**Example:**")
                md_content.append("```cpp")
                example_code = self.generate_arduino_example(cmd)
                md_content.append(example_code)
                md_content.append("```\n")
        
        # Error Handling Section
        md_content.append("## Error Handling\n")
        if hasattr(self, 'error_handling_text'):
            for paragraph in self.error_handling_text.split('\n'):
                if paragraph.strip():
                    md_content.append(paragraph)
            md_content.append("")
        else:
            md_content.append("Error handling description not available.\n")
        
        # Error Codes Section
        md_content.append("## Error Codes\n")
        md_content.append("This section lists all possible error codes that can be returned by the servomotor.\n")
        
        if self.error_codes:
            md_content.append("Look the code up here after `getError()` returns a positive value, or after "
                              "`getStatus().fatalErrorCode` is nonzero, or by counting the red LED blinks.\n")
            md_content.append("### Error Code Summary\n")
            md_content.append("| Code | Enum | Meaning |")
            md_content.append("|------|------|---------|")
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue
                # Pipes inside a cell would silently shift every later column.
                short_desc = str(error.get('short_desc', '')).replace('|', '\\|')
                md_content.append(f"| {error['code']} | {error['enum']} | {short_desc} |")
            md_content.append("")

            # Full detail, including causes and solutions. Previously the Arduino
            # document stopped at the summary table above, so the actionable half of
            # error_codes.json reached Python readers only.
            md_content.append("### Error Code Details\n")
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue

                md_content.append(f"#### Error {error['code']}: {error['enum']}\n")
                md_content.append(f"**Short Description:** {error['short_desc']}\n")
                md_content.append(f"**Description:** {error['long_desc']}\n")

                if error.get('causes'):
                    md_content.append("**Possible Causes:**")
                    for cause in error['causes']:
                        md_content.append(f"- {cause}")
                    md_content.append("")

                if error.get('solutions'):
                    md_content.append("**Solutions:**")
                    for solution in error['solutions']:
                        md_content.append(f"- {solution}")
                    md_content.append("")

        # Write to file in parent directory
        output_file = "../M17_servomotor_Arduino_API_documentation.md"
        with open(output_file, 'w') as f:
            f.write('\n'.join(md_content))
        
        print(f"✓ Generated Arduino Markdown documentation: {output_file}")
        return True
    
    class CodeBox(Flowable):
        """A custom flowable for code blocks with light grey background and black border"""
        def __init__(self, text, width, style):
            Flowable.__init__(self)
            self.text = text
            self.width = width
            self.style = style
            self.height = 0
            self.line_height = 10  # Reduced line height for better fit
            
        def wrap(self, availWidth, availHeight):
            # Calculate height needed
            lines = self.text.split('\n')
            self.height = len(lines) * self.line_height + 16  # Line height + padding (8 top + 8 bottom)
            return (self.width, self.height)

        def split(self, availWidth, availHeight):
            """Allow a long code block to flow across a page break. Without this,
            any example longer than one page aborts the whole PDF build."""
            lines = self.text.split('\n')
            n_fit = int((availHeight - 16) // self.line_height)
            # Keep at least two lines on each side, otherwise let the block move
            # to the next frame whole (reportlab retries there with a full page).
            if n_fit < 2 or n_fit >= len(lines) - 1:
                return []
            head = self.__class__('\n'.join(lines[:n_fit]), self.width, self.style)
            tail = self.__class__('\n'.join(lines[n_fit:]), self.width, self.style)
            return [head, tail]


        def draw(self):
            # Draw background rectangle with light grey fill
            self.canv.setFillColor(colors.HexColor('#f0f0f0'))
            self.canv.setStrokeColor(colors.black)
            self.canv.setLineWidth(0.5)
            self.canv.rect(0, 0, self.width, self.height, fill=1, stroke=1)
            
            # Draw text
            text_obj = self.canv.beginText(8, self.height - 12)  # Start 12 points from top
            text_obj.setFont('Courier', 8)
            text_obj.setFillColor(colors.black)
            
            for line in self.text.split('\n'):
                # Don't truncate, lines should already be properly wrapped
                text_obj.textLine(line)
            
            self.canv.drawText(text_obj)
    
    def generate_pdf(self):
        """Generate both Python and Arduino PDF documentation"""
        # Generate Python documentation
        self.generate_python_pdf()
        # Generate Arduino documentation
        self.generate_arduino_pdf()
        return True
    
    def generate_python_pdf(self):
        """Generate Python PDF documentation"""
        print("\nGenerating Python PDF documentation...")
        
        # Get version info for document content
        version, date_str = get_latest_version()
        # Use the specified filename format in parent directory
        output_filename = '../M17_servomotor_Python_API_documentation.pdf'
        
        # Create PDF document
        doc = SimpleDocTemplate(
            output_filename,
            pagesize=A4,
            rightMargin=18*mm,
            leftMargin=18*mm,
            topMargin=20*mm,
            bottomMargin=20*mm
        )
        
        # Initialize story
        story = []
        
        # Styles
        title_style = create_title_style()
        subtitle_style = create_subtitle_style()
        heading_style = create_heading_style()
        normal_style = create_normal_style()
        
        # Code style for examples
        code_style = ParagraphStyle(
            'Code',
            parent=getSampleStyleSheet()['Code'],
            fontName='Courier',
            fontSize=8,
            leftIndent=10,
            rightIndent=10,
            spaceBefore=6,
            spaceAfter=6,
            backColor=colors.HexColor('#f5f5f5')
        )
        
        # Command name style - larger and green (matching version/date color)
        command_style = ParagraphStyle(
            'CommandName',
            parent=normal_style,
            fontSize=14,
            textColor=colors.HexColor('#34a853'),  # Green color matching version/date
            fontName='Helvetica-Bold',
            spaceBefore=12,
            spaceAfter=6
        )
        
        # Title page
        story.append(Paragraph('Servomotor Python API Documentation', title_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph(f'Version {version}', subtitle_style))
        story.append(Paragraph(f'Generated: {datetime.now().strftime("%Y-%m-%d")}', subtitle_style))
        story.append(Spacer(1, 12))
        
        # Add firmware information if available
        if self.latest_firmware:
            firmware_style = ParagraphStyle(
                'FirmwareInfo',
                parent=normal_style,
                fontSize=10,
                textColor=colors.HexColor('#34a853'),
                spaceBefore=6,
                spaceAfter=6
            )
            
            story.append(Paragraph('<b>Latest Firmware Versions</b>', firmware_style))
            story.append(Spacer(1, 6))
            story.append(Paragraph('At the time of generating this API reference, the latest released firmware versions for the servomotors are:', normal_style))
            story.append(Spacer(1, 6))
            
            for model, firmware_file in sorted(self.latest_firmware.items()):
                story.append(Paragraph(f'• <b>{model}:</b> {firmware_file}', normal_style))
            
            story.append(Spacer(1, 8))
            story.append(Paragraph('If you are experiencing problems, you can try to set the firmware of your product to this version and try again, and report the problem to us using the feedback page.', normal_style))
        
        story.append(PageBreak())
        
        # Table of Contents
        story.append(Paragraph('Table of Contents', heading_style))
        story.append(Spacer(1, 12))
        
        # TOC style - black, bigger, and bold
        toc_style = ParagraphStyle(
            'TOCItem',
            parent=normal_style,
            fontSize=12,  # Bigger than normal (10)
            textColor=colors.black,
            fontName='Helvetica-Bold',
            leading=16,
            spaceBefore=2,
            spaceAfter=2
        )
        
        toc_items = [
            '1. Hardware Setup',
            '2. Install the Python Library',
            '3. Controlling the Servomotor From the Command Line',
            '4. Getting Started',
            '5. Know-How, Best Practices, and Gotchas',
            '6. Data Types',
            '7. Command Reference'
        ]
        toc_index = 8
        for group in sorted(self.commands_by_group.keys()):
            toc_items.append(f'   {toc_index}. {group}')
            toc_index += 1
        toc_items.append(f'{toc_index}. Unit Conversions')
        toc_index += 1
        toc_items.append(f'{toc_index}. Error Handling')
        toc_index += 1
        toc_items.append(f'{toc_index}. Error Codes')
        
        for item in toc_items:
            story.append(Paragraph(item, toc_style))
        story.append(PageBreak())

        # Hardware Setup Section
        if getattr(self, 'hardware_setup_md', None):
            self.add_markdown_section_to_pdf(story, doc, self.hardware_setup_md,
                                             heading_style, normal_style, code_style)
            story.append(PageBreak())

        # Install the Python Library Section
        story.append(Paragraph('Install the Python Library', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('You need to install the servomotor Python library before you can use it in your code. Run this command:', normal_style))
        story.append(Spacer(1, 8))
        
        # pip install command box
        install_cmd = "pip3 install servomotor"
        code_box = self.CodeBox(install_cmd, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 12))
        
        # Load installation instructions from symlinked file
        install_instructions = self.load_install_instructions()
        if install_instructions is None:
            print("\n" + "="*80)
            print("❌ FATAL ERROR: Installation instructions file not found")
            print("="*80)
            print(f"\nCould not find installation instructions at: {self.install_instructions_path}")
            print("\n📝 TO FIX THIS ERROR:")
            print("1. Ensure the symlink exists:")
            print(f"   ls -la {self.install_instructions_path}")
            print("2. If the symlink is broken, recreate it:")
            print(f"   ln -sf ../../python_programs/install_instructions_example.sh {self.install_instructions_path}")
            print("3. Ensure the source file exists:")
            print("   ls -la ../../python_programs/install_instructions_example.sh")
            print("\n" + "="*80)
            return []
        
        story.append(Paragraph('If you want to work in a virtual environment, you can create it, activate it, and install the library:', normal_style))
        story.append(Spacer(1, 8))
        
        story.append(Paragraph('<b>For macOS/Linux:</b>', normal_style))
        story.append(Spacer(1, 6))
        code_box = self.CodeBox(install_instructions, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 8))
        
        story.append(Paragraph('<b>For Windows:</b>', normal_style))
        story.append(Spacer(1, 6))
        windows_instructions = self.convert_to_windows_commands(install_instructions)
        code_box = self.CodeBox(windows_instructions, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 12))
        
        story.append(Paragraph('After installation, you can verify the servomotor library is installed correctly by running:', normal_style))
        story.append(Spacer(1, 6))
        verify_cmd = 'python3 -c "import servomotor; print(\'Servomotor library installed successfully!\')"'
        code_box = self.CodeBox(verify_cmd, doc.width - 20, code_style)
        story.append(code_box)
        story.append(Spacer(1, 16))
        
        # Controlling the Servomotor From the Command Line Section
        story.append(Paragraph('Controlling the Servomotor From the Command Line', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('You can send commands to the servomotor from the command line using the servomotor_command utility, which gets installed along with the Python library. Make sure to install that library according to the instructions above. After installation, the servomotor_command program should be in the path. You can try running some of the following commands to communicate with the servomotor(s):', normal_style))
        story.append(Spacer(1, 8))
        
        # Load and include servomotor command examples
        command_examples = self.load_servomotor_command_examples()
        if command_examples is None:
            print("\n" + "="*80)
            print("❌ FATAL ERROR: Servomotor command examples file not found")
            print("="*80)
            print(f"\nCould not find servomotor command examples at: {self.servomotor_command_examples_path}")
            print("\n📝 TO FIX THIS ERROR:")
            print("1. Ensure the file exists:")
            print(f"   ls -la {self.servomotor_command_examples_path}")
            print("2. Create the file if missing with example commands")
            print("\n" + "="*80)
            return []
        
        code_box = self.CodeBox(command_examples, doc.width - 20, code_style)
        story.append(code_box)
        
        story.append(PageBreak())
        
        # Getting Started Section
        story.append(Paragraph('Getting Started', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section provides a complete example showing how to initialize and control a servomotor.', normal_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('<b>Complete Example Program:</b>', normal_style))
        story.append(Spacer(1, 6))
        
        # Read and include the example file
        try:
            with open('python_library_example.py', 'r') as f:
                example_code = f.read()
            
            # Use CodeBox for the example
            code_box = self.CodeBox(example_code, doc.width - 20, code_style)
            story.append(code_box)
        except FileNotFoundError:
            story.append(Paragraph('Example file not found', normal_style))
        
        story.append(PageBreak())

        # Know-How Section, rendered for Python from the shared source
        if getattr(self, 'knowhow_md', None):
            self.add_markdown_section_to_pdf(
                story, doc, self.render_shared_section(self.knowhow_md, 'python'),
                heading_style, normal_style, code_style)
            story.append(PageBreak())

        # Data Types Section - use the modular function
        self.generate_data_types_for_pdf(story, doc, heading_style, normal_style)

        story.append(PageBreak())

        # Command Reference
        story.append(Paragraph('Command Reference', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section documents all available commands organized by category.', normal_style))
        story.append(Spacer(1, 12))

        # Subheading style - black, bigger, and bold
        subheading_style = ParagraphStyle(
            'SubHeading',
            parent=heading_style,
            fontSize=16,  # Bigger than normal headings
            textColor=colors.black,
            fontName='Helvetica-Bold',
            spaceBefore=12,
            spaceAfter=8
        )
        
        # Process each command group
        for group in sorted(self.commands_by_group.keys()):
            story.append(Paragraph(f'{group}', subheading_style))
            story.append(Spacer(1, 12))
            
            for cmd in self.commands_by_group[group]:
                # Keep command together on same page if possible
                cmd_content = []
                
                # Command name - use new prominent style
                cmd_content.append(Paragraph(cmd["CommandString"], command_style))
                cmd_content.append(Spacer(1, 6))
                
                # Description
                cmd_content.append(Paragraph(f'{cmd["Description"]}', normal_style))
                cmd_content.append(Spacer(1, 6))
                
                # Parameters
                if cmd['Input'] and cmd['Input'] != "null":
                    param_text = '<b>Parameters:</b><br/>'
                    for param in cmd['Input']:
                        param_name = param.get('ParameterName', 'unknown')
                        param_desc = param.get('Description', 'No description')
                        # Clean up description
                        param_desc = param_desc.replace('<', '&lt;').replace('>', '&gt;')
                        param_text += f'• <i>{param_name}</i>: {param_desc}<br/>'
                    cmd_content.append(Paragraph(param_text, normal_style))
                    cmd_content.append(Spacer(1, 6))
                
                # Return values
                if cmd['Output'] and cmd['Output'] != "success_response":
                    if isinstance(cmd['Output'], list):
                        output_text = '<b>Returns:</b><br/>'
                        for output in cmd['Output']:
                            out_name = output.get('ParameterName', 'unknown')
                            out_desc = output.get('Description', 'No description')
                            # Clean up description
                            out_desc = out_desc.replace('<', '&lt;').replace('>', '&gt;')
                            output_text += f'• <i>{out_name}</i>: {out_desc}<br/>'
                        cmd_content.append(Paragraph(output_text, normal_style))
                        cmd_content.append(Spacer(1, 6))
                
                # Python example with CodeBox: prefer the real runnable example program
                real_example = self.load_command_example(cmd)
                if real_example:
                    cmd_content.append(Paragraph('<b>Example program:</b>', normal_style))
                    example_code = real_example.rstrip()
                else:
                    cmd_content.append(Paragraph('<b>Example:</b>', normal_style))
                    example_code = self.generate_python_example(cmd)

                # A CodeBox cannot split across pages; chunk long examples so
                # each box fits on a page, and keep only short ones in the
                # KeepTogether block with the command header.
                MAX_CODEBOX_LINES = 45
                code_lines = example_code.split('\n')
                if len(code_lines) <= MAX_CODEBOX_LINES:
                    code_box = self.CodeBox(example_code, doc.width - 20, code_style)
                    cmd_content.append(code_box)
                    cmd_content.append(Spacer(1, 12))
                    story.append(KeepTogether(cmd_content))
                else:
                    story.append(KeepTogether(cmd_content))
                    for i in range(0, len(code_lines), MAX_CODEBOX_LINES):
                        chunk = '\n'.join(code_lines[i:i + MAX_CODEBOX_LINES])
                        story.append(self.CodeBox(chunk, doc.width - 20, code_style))
                        story.append(Spacer(1, 4))
                    story.append(Spacer(1, 8))
            
            story.append(PageBreak())
        
        # Unit Conversions Section
        story.append(Paragraph('Unit Conversions', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('The servomotor library supports multiple unit systems for convenience.', normal_style))
        story.append(Spacer(1, 12))
        
        units_info = [(f"<b>{title}:</b>", lines) for title, lines in self.build_unit_reference()]
        
        for title, units in units_info:
            story.append(Paragraph(title, normal_style))
            for unit in units:
                story.append(Paragraph(f'• {unit}', normal_style))
            story.append(Spacer(1, 12))
        
        # Setting units example
        story.append(Paragraph('<b>Setting Units:</b>', normal_style))
        story.append(Spacer(1, 6))
        story.append(Paragraph('You can set the units for a motor instance during initialization or at runtime:', normal_style))
        story.append(Spacer(1, 6))
        
        units_example = """# During initialization (the alias is a single character, an integer
# 0-251, or a 64-bit unique ID passed as an int)
motor = servomotor.M3(
    'X',
    time_unit='seconds',
    position_unit='degrees',
    velocity_unit='rpm',
    acceleration_unit='rpm_per_second'
)

# At runtime
motor.set_position_unit('radians')
motor.set_velocity_unit('rotations_per_second')"""
        
        # Use CodeBox for the units example
        code_box = self.CodeBox(units_example, doc.width - 20, code_style)
        story.append(code_box)
        
        story.append(PageBreak())
        
        self.generate_error_sections_for_pdf(story, doc, heading_style, normal_style)

        # Build PDF
        doc.build(story)
        print(f"✓ Generated Python PDF documentation: {output_filename}")
        return True

    def generate_error_sections_for_pdf(self, story, doc, heading_style, normal_style):
        """Generate the Error Handling and Error Codes sections for PDF - shared by
        Python and Arduino. The Arduino PDF listed both in its table of contents but
        never emitted them until this became a shared function."""
        # Error Handling Section
        story.append(Paragraph('Error Handling', heading_style))
        story.append(Spacer(1, 12))
        if hasattr(self, 'error_handling_text'):
            # Split text into paragraphs for better formatting
            for paragraph in self.error_handling_text.split('\n'):
                if paragraph.strip():
                    story.append(Paragraph(paragraph, normal_style))
                    story.append(Spacer(1, 6))
        else:
            story.append(Paragraph('Error handling description not available.', normal_style))
        story.append(Spacer(1, 12))

        # Error Codes Section
        story.append(Paragraph('Error Codes', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This section lists all possible error codes that can be returned by the servomotor.', normal_style))
        story.append(Spacer(1, 12))

        if self.error_codes:
            # Create style for error code headers
            error_code_style = ParagraphStyle(
                'ErrorCode',
                parent=normal_style,
                fontSize=12,
                textColor=colors.HexColor('#d32f2f'),  # Red color for error codes
                fontName='Helvetica-Bold',
                spaceBefore=10,
                spaceAfter=6
            )
            
            error_label_style = ParagraphStyle(
                'ErrorLabel',
                parent=normal_style,
                fontSize=10,
                fontName='Helvetica-Bold',
                spaceBefore=4,
                spaceAfter=2
            )
            
            for error in self.error_codes:
                if error['code'] == 0:  # Skip ERROR_NONE
                    continue
                
                # Keep error information together
                error_content = []
                
                # Error header
                error_content.append(Paragraph(f"Error {error['code']}: {error['enum']}", error_code_style))
                
                # Short description
                if error.get('short_desc'):
                    error_content.append(Paragraph(f"<b>Short Description:</b> {error['short_desc']}", normal_style))
                    error_content.append(Spacer(1, 4))
                
                # Long description
                error_content.append(Paragraph(f"<b>Description:</b> {error['long_desc']}", normal_style))
                error_content.append(Spacer(1, 6))
                
                # Possible causes
                if error.get('causes') and len(error['causes']) > 0:
                    error_content.append(Paragraph('<b>Possible Causes:</b>', error_label_style))
                    causes_text = ''
                    for cause in error['causes']:
                        # Escape XML special characters
                        cause_escaped = cause.replace('<', '&lt;').replace('>', '&gt;')
                        causes_text += f'• {cause_escaped}<br/>'
                    error_content.append(Paragraph(causes_text, normal_style))
                    error_content.append(Spacer(1, 4))
                
                # Solutions
                if error.get('solutions') and len(error['solutions']) > 0:
                    error_content.append(Paragraph('<b>Solutions:</b>', error_label_style))
                    solutions_text = ''
                    for solution in error['solutions']:
                        # Escape XML special characters
                        solution_escaped = solution.replace('<', '&lt;').replace('>', '&gt;')
                        solutions_text += f'• {solution_escaped}<br/>'
                    error_content.append(Paragraph(solutions_text, normal_style))
                    error_content.append(Spacer(1, 8))
                
                # Add as KeepTogether to avoid splitting error codes
                story.append(KeepTogether(error_content))
        else:
            story.append(Paragraph('Error codes not available.', normal_style))

    def generate_arduino_pdf(self):
        """Generate Arduino PDF documentation"""
        print("\nGenerating Arduino PDF documentation...")
        
        # Get version info for document content
        version, date_str = get_latest_version()
        # Output to parent directory
        output_filename = '../M17_servomotor_Arduino_API_documentation.pdf'
        
        # Create PDF document
        doc = SimpleDocTemplate(
            output_filename,
            pagesize=A4,
            rightMargin=18*mm,
            leftMargin=18*mm,
            topMargin=20*mm,
            bottomMargin=20*mm
        )
        
        # Initialize story
        story = []
        
        # Styles
        title_style = create_title_style()
        subtitle_style = create_subtitle_style()
        heading_style = create_heading_style()
        normal_style = create_normal_style()
        
        # Code style for examples
        code_style = ParagraphStyle(
            'Code',
            parent=getSampleStyleSheet()['Code'],
            fontName='Courier',
            fontSize=8,
            leftIndent=10,
            rightIndent=10,
            spaceBefore=6,
            spaceAfter=6,
            backColor=colors.HexColor('#f5f5f5')
        )
        
        # Command name style
        command_style = ParagraphStyle(
            'CommandName',
            parent=normal_style,
            fontSize=14,
            textColor=colors.HexColor('#34a853'),
            fontName='Helvetica-Bold',
            spaceBefore=12,
            spaceAfter=6
        )
        
        # Title page
        story.append(Paragraph('Servomotor Arduino API Documentation', title_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph(f'Version {version}', subtitle_style))
        story.append(Paragraph(f'Generated: {datetime.now().strftime("%Y-%m-%d")}', subtitle_style))
        story.append(Spacer(1, 12))
        
        # Add firmware information if available
        if self.latest_firmware:
            firmware_style = ParagraphStyle(
                'FirmwareInfo',
                parent=normal_style,
                fontSize=10,
                textColor=colors.HexColor('#34a853'),
                spaceBefore=6,
                spaceAfter=6
            )
            
            story.append(Paragraph('<b>Latest Firmware Versions</b>', firmware_style))
            story.append(Spacer(1, 6))
            story.append(Paragraph('At the time of generating this API reference, the latest released firmware versions for the servomotors are:', normal_style))
            story.append(Spacer(1, 6))
            
            for model, firmware_file in sorted(self.latest_firmware.items()):
                story.append(Paragraph(f'• <b>{model}:</b> {firmware_file}', normal_style))
            
            story.append(Spacer(1, 8))
        
        story.append(PageBreak())
        
        # Table of Contents
        story.append(Paragraph('Table of Contents', heading_style))
        story.append(Spacer(1, 12))
        
        # TOC style
        toc_style = ParagraphStyle(
            'TOC',
            parent=normal_style,
            fontSize=12,
            textColor=colors.black,
            fontName='Helvetica-Bold',
            spaceBefore=6,
            spaceAfter=6
        )
        
        story.append(Paragraph('1. Hardware Setup', toc_style))
        story.append(Paragraph('2. Getting Started', toc_style))
        story.append(Paragraph('3. Arduino Essentials: Checking for Errors and Setting Up Your Environment', toc_style))
        story.append(Paragraph('4. Know-How, Best Practices, and Gotchas', toc_style))
        story.append(Paragraph('5. Data Types', toc_style))
        story.append(Paragraph('6. Command Reference', toc_style))

        toc_index = 7
        for group in sorted(self.commands_by_group.keys()):
            story.append(Paragraph(f'{toc_index}. {group}', toc_style))
            toc_index += 1

        story.append(Paragraph(f'{toc_index}. Error Handling', toc_style))
        story.append(Paragraph(f'{toc_index + 1}. Error Codes', toc_style))

        story.append(PageBreak())

        # Hardware Setup Section
        if getattr(self, 'hardware_setup_md', None):
            self.add_markdown_section_to_pdf(story, doc, self.hardware_setup_md,
                                             heading_style, normal_style, code_style)
            story.append(PageBreak())

        # Getting Started Section
        story.append(Paragraph('Getting Started', heading_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph('This example demonstrates a trapezoid move with the servomotor:', normal_style))
        story.append(Spacer(1, 12))
        
        # Read and include the Arduino example file in a grey box
        try:
            with open('arduino_library_example.cpp', 'r') as f:
                example_code = f.read()
            # Use CodeBox to display in grey box
            code_box = self.CodeBox(example_code, doc.width - 20, code_style)
            story.append(code_box)
        except FileNotFoundError:
            story.append(Paragraph('Example file arduino_library_example.cpp not found', normal_style))

        story.append(PageBreak())

        # Arduino Essentials Section (error checking + environment)
        if getattr(self, 'arduino_essentials_md', None):
            self.add_markdown_section_to_pdf(
                story, doc, self.render_shared_section(self.arduino_essentials_md, 'arduino'),
                heading_style, normal_style, code_style)
            story.append(PageBreak())

        # Know-How Section, rendered for Arduino from the shared source
        if getattr(self, 'knowhow_md', None):
            self.add_markdown_section_to_pdf(
                story, doc, self.render_shared_section(self.knowhow_md, 'arduino'),
                heading_style, normal_style, code_style)
            story.append(PageBreak())

        # Data Types Section - use the same modular function as Python
        self.generate_data_types_for_pdf(story, doc, heading_style, normal_style)
        
        story.append(PageBreak())
        
        # Command Reference section
        story.append(Paragraph('Command Reference', heading_style))
        story.append(Spacer(1, 12))
        
        for group in sorted(self.commands_by_group.keys()):
            story.append(Paragraph(group, heading_style))
            story.append(Spacer(1, 12))
            
            for cmd in self.commands_by_group[group]:
                # Keep command information together
                cmd_content = []
                
                # Command name
                cmd_content.append(Paragraph(cmd['CommandString'], command_style))
                
                # Description
                cmd_content.append(Paragraph(cmd['Description'], normal_style))
                cmd_content.append(Spacer(1, 6))
                
                # Parameters
                if cmd['Input']:
                    param_text = '<b>Parameters:</b><br/>'
                    for param in cmd['Input']:
                        param_desc = param.get('Description', 'No description')
                        param_name = param.get('ParameterName', 'parameter')
                        param_desc = param_desc.replace('<', '&lt;').replace('>', '&gt;')
                        param_text += f'• <i>{param_name}</i>: {param_desc}<br/>'
                    cmd_content.append(Paragraph(param_text, normal_style))
                    cmd_content.append(Spacer(1, 6))
                
                # Arduino example - use the proper generator function
                cmd_content.append(Paragraph('<b>Example:</b>', normal_style))
                example_code = self.generate_arduino_example(cmd)
                
                code_box = self.CodeBox(example_code, doc.width - 20, code_style)
                cmd_content.append(code_box)
                cmd_content.append(Spacer(1, 12))
                
                # Add as KeepTogether
                story.append(KeepTogether(cmd_content))

            story.append(PageBreak())

        # Error Handling and Error Codes - the same shared function the Python PDF uses
        self.generate_error_sections_for_pdf(story, doc, heading_style, normal_style)

        # Build PDF
        doc.build(story)
        print(f"✓ Generated Arduino PDF documentation: {output_filename}")
        return True
    
    def run(self):
        """Main execution function"""
        print("\n" + "="*80)
        print("SERVOMOTOR API DOCUMENTATION GENERATOR")
        print("="*80)
        
        # Load commands
        if not self.load_commands():
            sys.exit(1)
        
        # Load data types
        if not self.load_data_types():
            sys.exit(1)
        
        # Load error codes
        if not self.load_error_codes():
            print("⚠️  Warning: Error codes not loaded. Documentation will be generated without error codes.")
        
        # Load error handling text
        if not self.load_error_handling_text():
            print("⚠️  Warning: Error handling text not loaded. Using default text.")

        # Load hardware setup and know-how sections
        if not self.load_hardware_setup():
            print("⚠️  Warning: Hardware setup section not loaded. Documentation will omit it.")
        if not self.load_knowhow():
            print("⚠️  Warning: Know-how section not loaded. Documentation will omit it.")
        if not self.load_arduino_essentials():
            print("⚠️  Warning: Arduino essentials section not loaded. Documentation will omit it.")

        # Load installation instructions
        if self.load_install_instructions() is None:
            sys.exit(1)
        
        # Load servomotor command examples
        if self.load_servomotor_command_examples() is None:
            sys.exit(1)
        
        # Find latest firmware versions
        self.find_latest_firmware()
        
        # Validate commands
        if not self.validate_commands():
            print("\n⚠️  Documentation generation aborted due to validation errors.")
            print("Please fix the errors above and run again.")
            sys.exit(1)
        
        # Generate documentation
        success = True
        try:
            if not self.generate_markdown():
                success = False
        except Exception as e:
            print(f"❌ Error generating Markdown: {e}")
            success = False
        
        try:
            if not self.generate_pdf():
                success = False
        except Exception as e:
            print(f"❌ Error generating PDF: {e}")
            success = False
        
        if success:
            print("\n" + "="*80)
            print("✅ DOCUMENTATION GENERATION COMPLETE!")
            print("="*80)
            print("\nGenerated files (in parent directory):")
            print("  • ../M17_servomotor_Python_API_documentation.md")
            print("  • ../M17_servomotor_Python_API_documentation.pdf")
            print("  • ../M17_servomotor_Arduino_API_documentation.md")
            print("  • ../M17_servomotor_Arduino_API_documentation.pdf")
        else:
            print("\n❌ Documentation generation failed. Please check the errors above.")
            sys.exit(1)

if __name__ == "__main__":
    generator = APIDocumentationGenerator()
    generator.run()