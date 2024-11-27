from reportlab.platypus import Paragraph, Spacer, Table, TableStyle, KeepTogether, PageBreak
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from styles import (
    create_heading_style, create_group_heading_style, 
    PRIMARY_COLOR, create_command_table_style
)
from utils import load_json_file

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
    story.append(Spacer(1, 8))
    
    # Calculate content width
    page_width, _ = A4
    margin = 20 * mm
    content_width = page_width - (2 * margin)
    
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
