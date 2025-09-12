from reportlab.platypus import Paragraph, Spacer, Table, TableStyle, KeepTogether, PageBreak, Preformatted, Flowable
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from styles import (
    create_heading_style, create_group_heading_style,
    PRIMARY_COLOR, create_command_table_style, SECONDARY_COLOR
)
from utils import load_json_file
from content import IconLink

class CodeBox(Flowable):
    """A custom flowable for code blocks with border and background"""
    def __init__(self, text, width):
        Flowable.__init__(self)
        self.text = text.rstrip()  # Remove trailing whitespace/newlines
        self.width = width
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

def add_protocol_overview(story, style):
    """Add protocol overview section"""
    elements = []
    elements.append(Paragraph('Communication Protocol Overview', create_heading_style()))
    
    # Read overview text from file
    with open('communication.txt', 'r') as f:
        overview_text = f.read()
    
    elements.append(Paragraph(overview_text, style))
    elements.append(Spacer(1, 10))
    
    story.append(KeepTogether(elements))

def create_description_cell(text):
    """Create a paragraph for the description cell with proper wrapping"""
    style = ParagraphStyle(
        'Description',
        fontSize=11,
        leading=13,
        spaceBefore=0,
        spaceAfter=0
    )
    return Paragraph(text, style)

def create_command_cell(text):
    """Create a paragraph for the command cell with proper wrapping"""
    style = ParagraphStyle(
        'Command',
        fontSize=11,
        leading=13,
        spaceBefore=0,
        spaceAfter=0
    )
    return Paragraph(text, style)

def create_command_table(title, commands, content_width):
    """Create a command table with its heading"""
    elements = []
    
    # Add title
    elements.append(Paragraph(title, create_group_heading_style()))
    elements.append(Spacer(1, 4))
    
    # Create table data
    data = [['Command', 'Description']]
    for cmd in commands:
        data.append([
            create_command_cell(cmd['CommandString']),
            create_description_cell(cmd['Description'])
        ])
    
    # Create table with adjusted column widths
    col1_width = content_width * 0.35  # Increased from 0.3 to 0.35
    col2_width = content_width * 0.65  # Decreased from 0.7 to 0.65
    table = Table(data, colWidths=[col1_width, col2_width])
    table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), PRIMARY_COLOR),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTSIZE', (0, 0), (-1, 0), 11),
        ('FONTSIZE', (0, 1), (-1, -1), 11),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('GRID', (0, 0), (-1, -1), 1, colors.black),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
    ]))
    
    elements.append(table)
    return elements

def add_command_reference(story, style):
    """Add command reference section"""
    story.append(Paragraph('Command Reference Summary', create_heading_style()))
    story.append(Spacer(1, 4))
    
    # Calculate content width
    page_width, _ = A4
    margin = 20 * mm
    content_width = page_width - (2 * margin)
    
    # Add descriptive text about the command reference
    description_text = """For the up to date source of truth for all available commands, you can look at this document."""
    story.append(Paragraph(description_text, style))
    story.append(Spacer(1, 4))
    
    # Create a link style for the URL
    link_style = ParagraphStyle(
        'LinkStyle',
        fontSize=11,
        leading=14,
        textColor=SECONDARY_COLOR,
        underline=True,
        alignment=0,  # Left align
        leftIndent=35  # Make room for the icon
    )
    
    # Add the icon and URL together
    url = 'https://github.com/tomrodinger/servomotor/blob/main/python_programs/servomotor/motor_commands.json'
    
    # Add icon with inline image tag
    link_text = f'<img src="click_here.png" width="25" height="25" valign="middle"/> <link href="{url}">{url}</link>'
    story.append(Paragraph(link_text, link_style))
    story.append(Spacer(1, 6))
    
    # Add text about running command
    command_text = """You can also run this command:"""
    story.append(Paragraph(command_text, style))
    story.append(Spacer(1, 4))
    
    # Add command line instructions in grey box with border
    commands = """pip3 install servomotor   # run this just once to install the library and programs
servomotor_command.py -c"""
    
    # Use CodeBox for consistent styling with code examples
    command_box = CodeBox(commands, content_width)
    story.append(command_box)
    story.append(Spacer(1, 4))
    
    # Add explanation text
    explanation_text = """This will print out the information contained in the motor_commands.json file in a nicer way and give some usage information for sending commands to the motor from the command line."""
    story.append(Paragraph(explanation_text, style))
    story.append(Spacer(1, 8))
    
    # Load commands from JSON file
    commands = load_json_file('motor_commands.json')
    
    # Group commands by their CommandGroup
    command_groups = {}
    for cmd in commands:
        group = cmd['CommandGroup']
        if group not in command_groups:
            command_groups[group] = []
        command_groups[group].append(cmd)
    
    # Sort groups to maintain consistent order
    group_order = ['Basic Control', 'Motion Control', 'Configuration', 'Status & Monitoring', 'Device Management', 'Other']
    
    # Add each command group
    for group in group_order:
        if group in command_groups:
            # Add page break before Status & Monitoring section
            if group == 'Status & Monitoring':
                story.append(PageBreak())
            
            elements = create_command_table(group, command_groups[group], content_width)
            story.append(KeepTogether(elements))
            story.append(Spacer(1, 6))

def add_all_protocol(story, style):
    """Add all protocol-related sections"""
    add_protocol_overview(story, style)
    add_command_reference(story, style)
