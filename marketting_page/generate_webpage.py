#!/usr/bin/env python3

import os
import json
from datetime import datetime
from web_utils import (
    create_html_section, create_html_table, create_image_element,
    create_code_block, wrap_html_document, save_css_file
)
from web_styles import generate_css

def resize_image(input_path, output_path, max_width=600):
    """Resize image to optimize for web, creating a smaller version"""
    try:
        from PIL import Image
        
        # Check if input file exists
        if not os.path.exists(input_path):
            print(f"Warning: Image {input_path} not found, skipping resize")
            return False
            
        # Open and resize image
        with Image.open(input_path) as img:
            # Calculate new size maintaining aspect ratio
            width_percent = max_width / float(img.size[0])
            new_height = int(float(img.size[1]) * width_percent)
            
            # Only resize if image is larger than max_width
            if img.size[0] > max_width:
                img_resized = img.resize((max_width, new_height), Image.Resampling.LANCZOS)
                img_resized.save(output_path, optimize=True, quality=85)
                print(f"Created optimized image: {output_path}")
            else:
                # If already small enough, just copy with optimization
                img.save(output_path, optimize=True, quality=85)
                print(f"Optimized image: {output_path}")
            
            return True
    except ImportError:
        print("Warning: PIL not installed. Install with: pip install Pillow")
        return False
    except Exception as e:
        print(f"Error resizing image {input_path}: {e}")
        return False

def get_optimized_image_name(image_name):
    """Generate optimized image filename"""
    name, ext = os.path.splitext(image_name)
    return f"{name}_small{ext}"

def read_file_content(filename):
    """Read content from a text file"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        return ''

def read_features():
    """Read features from features.txt"""
    try:
        with open('features.txt', 'r', encoding='utf-8') as f:
            features = []
            for line in f:
                line = line.strip()
                if line:
                    features.append(line)
            return features
    except FileNotFoundError:
        return []

def load_json_file(filename):
    """Load data from a JSON file"""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except:
        return {}

def get_latest_version():
    """Get the latest version information"""
    try:
        with open('versions.txt', 'r') as f:
            lines = f.readlines()
            for line in lines:
                if line.strip() and not line.startswith('#'):
                    parts = line.strip().split(' - ')
                    if len(parts) >= 2:
                        version = parts[0].replace('Version', '').strip()
                        date = parts[1].strip()
                        return version, date
    except:
        pass
    return '1.0', 'December 2024'

def generate_header_section():
    """Generate the header section with title and logo"""
    html = '<header class="hero-section">\n'
    html += '  <div class="container">\n'
    html += '    <img src="Gearotons_Logo.png" alt="Gearotons Logo" class="logo">\n'
    html += '    <h1 class="main-title">M17 Series Servomotors</h1>\n'
    html += '    <p class="subtitle">Affordable and Simple All-in-One Motion Control</p>\n'
    html += '    <p class="subtitle">From Education to Innovation</p>\n'
    html += '    <img src="M17_series_overview.jpg" alt="M17 Series Overview" class="hero-image">\n'
    html += '  </div>\n'
    html += '</header>\n\n'
    return html

def generate_introduction_section():
    """Generate the introduction section"""
    html = '<section id="introduction" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Introduction</h2>\n'
    
    content = read_file_content('introduction.txt')
    if content:
        paragraphs = content.split('\n')
        for paragraph in paragraphs:
            if paragraph.strip():
                html += f'    <p>{paragraph.strip()}</p>\n'
    
    # Resize and add image to introduction section
    original_img = 'one_motor.jpg'
    optimized_img = get_optimized_image_name(original_img)
    resize_image(original_img, optimized_img)
    
    html += '    <div class="section-image-container">\n'
    html += f'      <img src="{optimized_img}" alt="M17 Servomotor" class="section-image">\n'
    html += '    </div>\n'
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_features_section():
    """Generate the key features section from features.txt"""
    html = '<section id="features" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Key Features</h2>\n'
    
    # Resize and add image to features section
    original_img = 'kit_with_three_motors.jpg'
    optimized_img = get_optimized_image_name(original_img)
    resize_image(original_img, optimized_img)
    
    html += '    <div class="section-image-container">\n'
    html += f'      <img src="{optimized_img}" alt="M17 Servomotor Kit" class="section-image">\n'
    html += '    </div>\n'
    
    html += '    <ul class="features-list-simple">\n'
    
    features = read_features()
    for feature in features:
        html += f'      <li>{feature}</li>\n'
    
    html += '    </ul>\n'
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_connection_diagram_section():
    """Generate the connection diagram section"""
    html = '<section id="connection" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Connection Diagram</h2>\n'
    html += '    <img src="connection_diagram.jpg" alt="Connection Diagram" class="diagram-image">\n'
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_unit_system_section():
    """Generate the unit system section"""
    html = '<section id="unit-system" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Unit System</h2>\n'
    html += '    <p>The M17 Series Servomotors have certain internal units so that they can perform the calculations '
    html += 'associated with motion efficiently (using integer math). It is the responsibility of the controlling '
    html += 'software to support multiple units of measurement for various quantities. Our Python and Arduino libraries handle '
    html += 'unit conversions automatically, allowing you to work with your preferred units. Below are the supported '
    html += 'units for each quantity:</p>\n'
    
    # Try to load unit conversion data
    unit_data = load_json_file('../python_programs/servomotor/unit_conversions_M3.json')
    if unit_data and 'units' in unit_data:
        units = unit_data['units']
        table_data = [['Quantity', 'Available Units']]
        
        for quantity, unit_list in units.items():
            quantity_name = quantity.replace('_', ' ').title()
            unit_text = ', '.join(unit.replace('_', ' ') for unit in unit_list)
            table_data.append([quantity_name, unit_text])
        
        html += create_html_table(table_data, 'specs-table')  # Use specs-table class for consistent styling
    else:
        # Fallback table with common units
        table_data = [
            ['Quantity', 'Available Units'],
            ['Position', 'degrees, radians, revolutions, encoder counts'],
            ['Speed', 'RPM, degrees/sec, radians/sec'],
            ['Acceleration', 'degrees/sec², radians/sec²'],
            ['Time', 'seconds, milliseconds, microseconds'],
            ['Current', 'milliamps, amps'],
            ['Voltage', 'millivolts, volts']
        ]
        html += create_html_table(table_data, 'specs-table')
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_specifications_section():
    """Generate the technical and mechanical specifications section"""
    html = '<section id="specifications" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Technical Specifications</h2>\n'
    
    # Mechanical specs table (displayed first in the PDF)
    html += '    <h3>Mechanical Specifications</h3>\n'
    mech_data = [
        ['Parameter', 'M17-60', 'M17-48', 'M17-40'],
        ['Dimensions (LxW)', '42.2x42.2 mm', '42.2x42.2 mm', '42.2x42.2 mm'],
        ['Height', '59.8 mm', '48.6 mm', '41.6 mm'],
        ['Shaft Length', '20.4 mm', '20.4 mm', '18.5 mm'],
        ['Shaft Diameter', '5 mm', '5 mm', '5 mm'],
        ['Weight', '470g', '360g', '285g'],
        ['Protection Class', 'IP20', 'IP20', 'IP20']
    ]
    html += create_html_table(mech_data, 'specs-table')
    
    # Add dimension images for each model
    html += '    <div class="dimension-images">\n'
    for model in ['M17-60', 'M17-48', 'M17-40']:
        html += f'      <div class="dimension-image-container">\n'
        html += f'        <img src="{model}_dimensions.png" alt="{model} Dimensions" class="dimension-image">\n'
        html += f'        <p class="dimension-label">{model}</p>\n'
        html += f'      </div>\n'
    html += '    </div>\n'
    
    # Technical specs table
    html += '    <h3>Electrical Specifications</h3>\n'
    tech_data = [
        ['Parameter', 'M17-60', 'M17-48', 'M17-40'],
        ['Operating Voltage', '12-24V', '12-24V', '12-24V'],
        ['Rated Torque', '0.65 N.m', '0.55 N.m', '0.42 N.m'],
        ['Maximum Speed', '560 RPM', '560 RPM', '560 RPM'],
        ['Maximum Current', '1.1A', '1.1A', '1.1A'],
        ['Rated Power', '38W', '32W', '25W']
    ]
    html += create_html_table(tech_data, 'specs-table')
    
    # Operating conditions
    html += '    <h3>Operating Conditions</h3>\n'
    conditions_data = [
        ['Parameter', 'Specification'],
        ['Operating Temperature', '0°C to +80°C'],
        ['Storage Temperature', '-20°C to +60°C'],
        ['Humidity Range', '20% to 80% RH (non-condensing)'],
        ['Installation Environment', 'Indoor use only']
    ]
    html += create_html_table(conditions_data, 'conditions-table')
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_indicators_section():
    """Generate the Indicator LEDs and Buttons section"""
    html = '<section id="indicators" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Indicator LEDs and Buttons</h2>\n'
    
    # Resize and add image showing motor back with LEDs and buttons
    original_img = 'motor_back.jpg'
    optimized_img = get_optimized_image_name(original_img)
    resize_image(original_img, optimized_img)
    
    html += '    <div class="section-image-container">\n'
    html += f'      <img src="{optimized_img}" alt="Motor Back with LEDs and Buttons" class="section-image">\n'
    html += '    </div>\n'
    
    # LED description from indicators.py
    led_description = (
        "The servomotor has two status LEDs (Green and Red). The green LED flashes slowly to show a heart beat "
        "and quickly to indicate that the bootloader is running rather than the application. The red LED will "
        "light up briefly to show communication on the bus and will indicate fatal error codes by flashing a "
        "certain number of times."
    )
    
    # Button description from indicators.py
    button_description = (
        "The servomotor has two buttons labelled \"Reset\" and \"Test\". The Reset button will reset the internal "
        "microcontroller and all state will go back to default values. The Test button will cause the motor to spin. "
        "Press briefly to let it spin one way and press for more than 0.3 seconds and release to let it spin the other "
        "way. Hold down for at least 2 seconds and release to cause the motor to go to closed loop mode. Hold down for "
        "more than 15 seconds and release to let the motor perform a calibration on itself. Note that it will spin during "
        "calibration and must be able to spin freely for calibration to be successful, so remove any loads before doing "
        "this operation."
    )
    
    html += f'    <p>{led_description}</p>\n'
    html += f'    <p>{button_description}</p>\n'
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_protocol_section():
    """Generate the communication protocol section"""
    html = '<section id="protocol" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Communication Protocol</h2>\n'
    
    # Resize and add adapter and wire image
    original_img = 'adapter_and_wire.jpg'
    optimized_img = get_optimized_image_name(original_img)
    resize_image(original_img, optimized_img)
    
    html += '    <div class="section-image-container">\n'
    html += f'      <img src="{optimized_img}" alt="RS-485 Adapter and Wire" class="section-image">\n'
    html += '    </div>\n'
    
    # Protocol overview
    content = read_file_content('protocol.txt')
    if content:
        html += f'    <p>{content}</p>\n'
    else:
        html += '    <p>The M17 series uses RS-485 communication with a simple command-based protocol. Multiple motors can be daisy-chained on a single bus, each with a unique ID.</p>\n'
    
    # Command reference - matching the actual datasheet
    html += '    <h3>Command Reference Summary</h3>\n'
    html += '    <p>For the up to date source of truth for all available commands, you can look at this document.</p>\n'
    
    # Add link to GitHub
    html += '    <div class="command-link">\n'
    html += '      <img src="click_here.png" alt="Click Here" class="click-icon-small">\n'
    html += '      <a href="https://github.com/tomrodinger/servomotor/blob/main/python_programs/servomotor/motor_commands.json" target="_blank">\n'
    html += '        https://github.com/tomrodinger/servomotor/blob/main/python_programs/servomotor/motor_commands.json\n'
    html += '      </a>\n'
    html += '    </div>\n'
    
    html += '    <p>You can also run this command:</p>\n'
    
    # Add command line instructions
    commands = '''pip3 install servomotor   # run this just once to install the library and programs
motor_command.py -c'''
    html += create_code_block(commands, 'bash')
    
    html += '    <p>This will print out the information contained in the motor_commands.json file in a nicer way and give some usage information for sending commands to the motor from the command line.</p>\n'
    
    # Since we don't have motor_commands.json, provide a link to documentation
    html += '    <p>The commands are grouped by functionality including Basic Control, Motion Control, Configuration, Status & Monitoring, and Device Management.</p>\n'
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_library_section():
    """Generate the library support section"""
    html = '<section id="libraries" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Library Support</h2>\n'
    
    # Python library
    html += '    <div class="library-section">\n'
    html += '      <h3>Python Library</h3>\n'
    html += '      <p>Easy-to-use Python library for controlling M17 servomotors:</p>\n'
    
    # Try to read the actual Python example
    python_code = read_file_content('python_library_example.py')
    if not python_code:
        # Fallback example
        python_code = '''from m17_servo import M17Controller

# Initialize controller
controller = M17Controller('/dev/ttyUSB0')

# Connect to motor with ID 1
motor = controller.get_motor(1)

# Move to position
motor.move_to(90)  # Move to 90 degrees
motor.move_by(45)  # Move 45 degrees relative

# Read position
position = motor.get_position()
print(f"Current position: {position}°")'''
    
    html += create_code_block(python_code, 'python')
    html += '    </div>\n'
    
    # Arduino library
    html += '    <div class="library-section">\n'
    html += '      <h3>Arduino Library</h3>\n'
    html += '      <p>Arduino library for easy integration with Arduino boards:</p>\n'
    
    # Try to read the actual Arduino example
    arduino_code = read_file_content('arduino_library_example.cpp')
    if not arduino_code:
        # Fallback example
        arduino_code = '''#include <M17Servomotor.h>

M17Servomotor motor(1);  // Motor ID 1

void setup() {
  Serial1.begin(115200);  // RS-485 communication
  motor.begin(&Serial1);
}

void loop() {
  motor.moveTo(90);   // Move to 90 degrees
  delay(2000);
  motor.moveTo(0);    // Move to 0 degrees
  delay(2000);
}'''
    
    html += create_code_block(arduino_code, 'cpp')
    html += '    </div>\n'
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_getting_started_section():
    """Generate the getting started guide section"""
    html = '<section id="getting-started" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Getting Started Guide</h2>\n'
    html += '    <p>To help you get started with your M17 Series Servomotor, we provide a comprehensive online guide '
    html += 'that covers everything from initial setup to advanced protocol implementations. This guide includes:</p>\n'
    
    html += '    <ul class="guide-features">\n'
    html += '      <li>Step-by-step setup instructions</li>\n'
    html += '      <li>Detailed communication protocol documentation</li>\n'
    html += '      <li>Programming examples and code snippets</li>\n'
    html += '      <li>Description of error codes</li>\n'
    html += '      <li>Troubleshooting tips and best practices</li>\n'
    html += '    </ul>\n'
    
    html += '    <div class="guide-link">\n'
    html += '      <img src="click_here.png" alt="Click Here" class="click-icon">\n'
    html += '      <a href="https://servo-tutorial.netlify.app/" target="_blank" class="btn btn-primary">Click Here to Visit our Getting Started Guide</a>\n'
    html += '    </div>\n'
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_applications_section():
    """Generate the applications section with images"""
    html = '<section id="applications" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Applications</h2>\n'
    html += '    <p>The M17 Series servomotors are perfect for a wide range of applications, from educational projects to industrial automation.</p>\n'
    html += '    <div class="applications-grid">\n'
    
    # Robotics application - resize image
    robotics_img = 'robotics.jpg'
    robotics_optimized = get_optimized_image_name(robotics_img)
    resize_image(robotics_img, robotics_optimized)
    
    html += '      <div class="application-card">\n'
    html += f'        <img src="{robotics_optimized}" alt="Robotics Application" class="application-image">\n'
    html += '        <h3>Robotics</h3>\n'
    html += '        <p>Build precise robotic arms, mobile robots, and educational robotics platforms with easy-to-control servomotors.</p>\n'
    html += '      </div>\n'
    
    # Automation application - resize image
    automation_img = 'automation.jpg'
    automation_optimized = get_optimized_image_name(automation_img)
    resize_image(automation_img, automation_optimized)
    
    html += '      <div class="application-card">\n'
    html += f'        <img src="{automation_optimized}" alt="Automation Application" class="application-image">\n'
    html += '        <h3>Automation</h3>\n'
    html += '        <p>Perfect for automated systems, CNC machines, 3D printers, and industrial control applications.</p>\n'
    html += '      </div>\n'
    
    html += '    </div>\n'
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_company_profile_section():
    """Generate the company profile section"""
    html = '<section id="company-profile" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Company Profile</h2>\n'
    html += '    <div class="company-content">\n'
    
    # Read company profile content
    profile_content = read_file_content('company_profile.txt')
    if profile_content:
        paragraphs = profile_content.split('\n')
        for paragraph in paragraphs:
            if paragraph.strip():
                html += f'      <p>{paragraph.strip()}</p>\n'
    
    # Resize and add test rack image
    test_rack_img = 'test_rack.jpg'
    test_rack_optimized = get_optimized_image_name(test_rack_img)
    resize_image(test_rack_img, test_rack_optimized)
    
    html += '      <div class="company-image-container">\n'
    html += f'        <img src="{test_rack_optimized}" alt="Test Rack" class="company-image">\n'
    html += '        <p class="image-caption">Our testing facility ensures every motor meets quality standards</p>\n'
    html += '      </div>\n'
    
    html += '    </div>\n'
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_open_source_section():
    """Generate the open source section"""
    html = '<section id="open-source" class="content-section">\n'
    html += '  <div class="container">\n'
    html += '    <h2>Open Source</h2>\n'
    html += '    <div class="open-source-content">\n'
    # Use the real description from open_source.py
    html += '      <p>We believe in making the world better through technology. All software, firmware, and PCB design files are available here:</p>\n'
    html += '      <div class="github-link">\n'
    html += '        <img src="click_here.png" alt="Click Here" class="click-icon-small">\n'
    html += '        <a href="https://github.com/tomrodinger/servomotor" target="_blank">https://github.com/tomrodinger/servomotor</a>\n'
    html += '      </div>\n'
    html += '      <div class="open-source-logos">\n'
    html += '        <img src="Open-source-hardware-logo.svg.png" alt="Open Source Hardware" class="osh-logo">\n'
    html += '        <img src="Open_Source_Initiative.svg.png" alt="Open Source Initiative" class="osi-logo">\n'
    html += '      </div>\n'
    html += '    </div>\n'
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def generate_footer_section():
    """Generate the footer section"""
    version, date = get_latest_version()
    
    html = '<footer class="footer">\n'
    html += '  <div class="container">\n'
    html += '    <img src="Gearotons_Logo.png" alt="Gearotons" class="footer-logo">\n'
    html += f'    <p class="version-info">Version {version} - {date}</p>\n'
    html += '    <p class="copyright">© 2024 Gearotons. All specifications subject to change without notice.</p>\n'
    html += '    <p>For more information and technical support, please contact our sales team.</p>\n'
    html += '  </div>\n'
    html += '</footer>\n'
    return html

def generate_html():
    """Generate the complete HTML marketing page"""
    # Build the body content
    body_content = ''
    
    # Add all sections (matching the PDF datasheet order)
    body_content += generate_header_section()
    body_content += generate_introduction_section()
    body_content += generate_features_section()
    body_content += generate_connection_diagram_section()
    body_content += generate_unit_system_section()
    body_content += generate_getting_started_section()
    body_content += generate_indicators_section()
    body_content += generate_protocol_section()
    body_content += generate_specifications_section()
    body_content += generate_library_section()
    body_content += generate_applications_section()  # NEW
    body_content += generate_company_profile_section()  # NEW
    body_content += generate_open_source_section()
    body_content += generate_footer_section()
    
    # Wrap in HTML document structure
    html = wrap_html_document(
        body_content,
        title='M17 Series Servomotors - Marketing',
        css_file='marketing.css'
    )
    
    # Save HTML file
    with open('marketing.html', 'w', encoding='utf-8') as f:
        f.write(html)
    
    # Generate and save CSS
    css_content = generate_css()
    save_css_file('marketing.css', css_content)
    
    print("Marketing page generated successfully!")
    print("- HTML: marketing.html")
    print("- CSS: marketing.css")

if __name__ == '__main__':
    generate_html()