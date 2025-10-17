from reportlab.platypus import Paragraph, Spacer, Preformatted, KeepTogether, Table, TableStyle, Flowable
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from styles import create_heading_style, create_feature_style, PRIMARY_COLOR
from utils import get_processed_image

class CodeBox(Flowable):
    """A custom flowable for code blocks with border and background"""
    def __init__(self, text, width):
        Flowable.__init__(self)
        self.text = text.rstrip()  # Remove trailing whitespace/newlines
        self.width = width
        self.style = ParagraphStyle(
            'CodeStyle',
            fontName='Courier',
            fontSize=8,
            leading=10,
            textColor=colors.black
        )
        # Calculate height based on actual lines after stripping
        self.lines = self.text.split('\n') if self.text else []
        line_count = len(self.lines)
        self.height = line_count * 10 + 20  # line height + padding
        
    def wrap(self, availWidth, availHeight):
        return (self.width, self.height)
        
    def draw(self):
        # Draw background
        self.canv.setFillColor(colors.Color(0.95, 0.95, 0.95))
        self.canv.rect(0, 0, self.width, self.height, fill=1, stroke=0)
        
        # Draw border
        self.canv.setStrokeColor(colors.Color(0.7, 0.7, 0.7))
        self.canv.setLineWidth(1)
        self.canv.rect(0, 0, self.width, self.height, fill=0, stroke=1)
        
        # Draw text
        self.canv.setFont('Courier', 8)
        self.canv.setFillColor(colors.black)
        
        # Draw text using stored lines
        y_position = self.height - 12  # Start from top with padding
        for line in self.lines:
            self.canv.drawString(10, y_position, line)
            y_position -= 10  # Move down for next line

def create_code_style():
    """Create a style for inline code"""
    return ParagraphStyle(
        'CodeStyle',
        fontName='Courier',
        fontSize=9,
        leading=11,
        leftIndent=10,
        rightIndent=10,
        spaceBefore=6,
        spaceAfter=6,
        backColor=colors.Color(0.95, 0.95, 0.95)
    )

def add_arduino_library_section(story, normal_style):
    """Add Arduino Library section to the datasheet"""
    
    # Add section heading
    story.append(Paragraph('Arduino Library', create_heading_style()))
    story.append(Spacer(1, 5))
    
    # Add introduction
    intro_text = """
    The Arduino library for M17 Series Servomotors provides an easy-to-use interface for 
    controlling motors from Arduino boards. The library supports hardware serial communication 
    and includes all essential motor control functions.
    """
    story.append(Paragraph(intro_text, normal_style))
    story.append(Spacer(1, 5))
    
    # Installation instructions
    story.append(Paragraph('<b>Installation</b>', normal_style))
    story.append(Spacer(1, 3))
    
    install_text = """
    Install the library through the Arduino Library Manager:
    """
    story.append(Paragraph(install_text, normal_style))
    story.append(Spacer(1, 3))
    
    install_steps = [
        '1. Open Arduino IDE',
        '2. Go to Sketch → Include Library → Manage Libraries',
        '3. Search for "Servomotor"',
        '4. Find the library called "Servomotor by Gearotons" and click Install'
    ]
    
    feature_style = create_feature_style()
    for step in install_steps:
        story.append(Paragraph(step, feature_style))
    story.append(Spacer(1, 5))
    
    # Add installation figure
    try:
        # Calculate appropriate image width (about 80% of content width)
        page_width, _ = A4
        margin = 18 * mm
        content_width = page_width - (2 * margin)
        img_width = content_width * 0.8
        
        installation_img = get_processed_image('arduino_library_installation.png', img_width)
        story.append(installation_img)
        story.append(Spacer(1, 10))
    except Exception as e:
        # Hard failure if image cannot be loaded
        print(f"ERROR: Could not load arduino_library_installation.png: {e}")
        print("INSTRUCTIONS: Please ensure the file 'arduino_library_installation.png' exists in the current directory.")
        print("This image is required for the Arduino Library Installation section of the datasheet.")
        raise SystemExit(1)
    
    # Alternative installation
    story.append(Paragraph('<b>Manual Installation</b>', normal_style))
    story.append(Spacer(1, 3))
    
    manual_text = """
    Alternatively, if you are an expert, you can grab the code for the Arduino library here:
    https://github.com/tomrodinger/Servomotor_Arduino_Library
    """
    story.append(Paragraph(manual_text, normal_style))
    story.append(Spacer(1, 5))
    
    # Example code section
    story.append(Paragraph('<b>Example: Basic Motor Control</b>', normal_style))
    story.append(Spacer(1, 3))
    
    example_text = """
    This example shows how to initialize a motor, enable mosfets, and perform a trapezoid move:
    """
    story.append(Paragraph(example_text, normal_style))
    story.append(Spacer(1, 3))
    
    # Read Arduino example code from file
    try:
        with open('arduino_library_example.cpp', 'r') as f:
            example_code = f.read()
    except FileNotFoundError:
        example_code = """// Example file not found
// Please create arduino_library_example.cpp"""
    
    # Calculate content width
    page_width, _ = A4
    margin = 18 * mm
    content_width = page_width - (2 * margin)
    
    # Create code box with proper styling
    code_box = CodeBox(example_code, content_width)
    
    # Create elements list for keeping together
    elements = []
    elements.append(code_box)
    elements.append(Spacer(1, 5))
    
    # Keep all elements together if possible
    story.append(KeepTogether(elements))