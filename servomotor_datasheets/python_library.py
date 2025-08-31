from reportlab.platypus import Paragraph, Spacer, KeepTogether, Flowable
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from styles import create_heading_style, create_feature_style, PRIMARY_COLOR

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

def add_python_library_section(story, normal_style):
    """Add Python Library section to the datasheet"""
    
    # Add section heading
    story.append(Paragraph('Python Library', create_heading_style()))
    story.append(Spacer(1, 5))
    
    # Add introduction
    intro_text = """
    Our Python library provides a comprehensive interface for controlling M17 Series Servomotors. 
    The library handles all low-level communication protocols and unit conversions, making it easy 
    to integrate servomotor control into your Python projects.
    """
    story.append(Paragraph(intro_text, normal_style))
    story.append(Spacer(1, 5))
    
    # Installation instructions
    story.append(Paragraph('<b>Installation</b>', normal_style))
    story.append(Spacer(1, 3))
    
    install_text = """
    Install the servomotor library using pip:
    """
    story.append(Paragraph(install_text, normal_style))
    story.append(Spacer(1, 3))
    
    # Calculate content width for CodeBox
    page_width, _ = A4
    margin = 18 * mm
    content_width = page_width - (2 * margin)
    
    # Installation command using CodeBox for consistent styling
    install_code = """pip3 install servomotor"""
    install_box = CodeBox(install_code, content_width)
    story.append(install_box)
    story.append(Spacer(1, 5))
    
    # Example code section
    story.append(Paragraph('<b>Example: Basic Motor Control</b>', normal_style))
    story.append(Spacer(1, 3))
    
    example_text = """
    The following example demonstrates how to connect to a servomotor, enable the mosfets,
    and perform a trapezoid move:
    """
    story.append(Paragraph(example_text, normal_style))
    story.append(Spacer(1, 3))
    
    # Read Python example code from file
    try:
        with open('python_library_example.py', 'r') as f:
            example_code = f.read()
    except FileNotFoundError:
        example_code = """# Example file not found
# Please create python_library_example.py"""
    
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
    
    # Add features
    elements.append(Paragraph('<b>Key Features</b>', normal_style))
    elements.append(Spacer(1, 3))
    
    features = [
        'Support for all motor commands',
        'Automatic unit conversion for time, position, velocity, acceleration, current, voltage, and temperature',
        'Error handling and status monitoring',
        'Support for multiple motors on the same bus',
        'Get up and running in just a few lines of code'
    ]
    
    feature_style = create_feature_style()
    for feature in features:
        elements.append(Paragraph(feature, feature_style))
    
    elements.append(Spacer(1, 5))
    
    # Documentation link
    elements.append(Paragraph('<b>Documentation</b>', normal_style))
    elements.append(Spacer(1, 3))
    doc_text = """
    For complete API documentation and more examples, visit:
    https://github.com/tomrodinger/servomotor/tree/main/python_programs
    """
    elements.append(Paragraph(doc_text, normal_style))
    elements.append(Spacer(1, 10))
    
    # Keep all elements together if possible
    story.append(KeepTogether(elements))