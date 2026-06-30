from reportlab.platypus import Paragraph, Spacer, Table, KeepTogether, PageBreak, Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from styles import create_heading_style, create_table_style, create_subtitle_style
from utils import create_data_table, get_image_size

def get_content_width():
    """Calculate content width to match main.py"""
    page_width, _ = A4
    margin = 20 * mm
    return page_width - (2 * margin)

def add_technical_specs(story, style):
    """Add technical specifications section"""
    # Create elements to keep together
    elements = []
    elements.append(Paragraph('Technical Specifications', create_heading_style()))
    elements.append(Spacer(1, 6))  # Reduced spacing
    
    data = [
        ['Parameter', 'M17-60', 'M17-48', 'M17-40', 'M17-34'],
        ['Operating Voltage', '12-24V', '12-24V', '12-24V', '12-24V'],
        ['Rated Torque', '0.65 N.m', '0.55 N.m', '0.42 N.m', '0.28 N.m'],
        ['Maximum Speed', '560 RPM', '560 RPM', '560 RPM', '560 RPM'],
        ['Maximum Current', '1.1A', '1.1A', '1.1A', '1.0A'],
        ['Rated Power', '26.4W', '26.4W', '26.4W', '24.0W']
    ]

    content_width = get_content_width()
    first_col_width = content_width * 0.24  # 20% wider first column
    other_col_width = (content_width - first_col_width) / 4
    table = Table(data, colWidths=[first_col_width] + [other_col_width] * 4)
    table.setStyle(create_table_style())
    elements.append(table)
    elements.append(Spacer(1, 8))
    
    # Keep heading and table together
    story.append(KeepTogether(elements))

def add_mechanical_specs(story, style):
    """Add mechanical specifications section"""
    # Create elements to keep together
    elements = []
    elements.append(Paragraph('Mechanical Specifications', create_heading_style()))
    elements.append(Spacer(1, 2))  # Minimal spacing after heading
    
    data = [
        ['Parameter', 'M17-60', 'M17-48', 'M17-40', 'M17-34'],
        ['Dimensions (LxW)', '42.2x42.2 mm', '42.2x42.2 mm', '42.2x42.2 mm', '42.2x42.2 mm'],
        ['Height', '59.7 mm', '48.7 mm', '40.1 mm', '33.5 mm'],
        ['Shaft Length', '20.6 mm', '20.6 mm', '20.6 mm', '20.6 mm'],
        ['Weight', '470g', '360g', '285g', '210g'],
        ['Protection Class', 'IP20', 'IP20', 'IP20', 'IP20']
    ]

    content_width = get_content_width()
    # Wider first column so "Dimensions (LxW)" fits without overflowing into
    # the value column (matches the Technical Specifications table below)
    first_col_width = content_width * 0.24
    other_col_width = (content_width - first_col_width) / 4
    table = Table(data, colWidths=[first_col_width] + [other_col_width] * 4)
    table.setStyle(create_table_style())
    elements.append(table)
    elements.append(Spacer(1, 4))  # Reduced spacing after table
    
    # Fixed height for all images - reduced to fit 4 models on one page
    fixed_height = 130
    
    # Add dimension images with fixed height
    for model in ['M17-60', 'M17-48', 'M17-40', 'M17-34']:
        # Calculate width based on original aspect ratio
        img_path = f'{model}_dimensions.png'
        _, orig_height = get_image_size(img_path, content_width)
        aspect_ratio = orig_height / content_width
        new_width = fixed_height / aspect_ratio
        
        tech_img = Image(img_path, width=new_width, height=fixed_height)
        tech_img.hAlign = 'CENTER'
        elements.append(tech_img)
        elements.append(Spacer(1, 4))  # Keep minimal spacing between images
    
    # Keep everything together on one page
    story.append(KeepTogether(elements))

def add_operating_conditions(story, style):
    """Add operating conditions section"""
    # Create elements to keep together
    elements = []
    elements.append(Paragraph('Operating Conditions', create_heading_style()))
    elements.append(Spacer(1, 6))
    
    data = [
        ['Parameter', 'Specification'],
        ['Operating Temperature', '0C to +80C'],
        ['Storage Temperature', '-20C to +60C'],
        ['Humidity Range', '20% to 80% RH (non-condensing)'],
        ['Installation Environment', 'Indoor use only']
    ]
    
    content_width = get_content_width()
    col1_width = content_width * 0.3  # 30% for parameter name
    col2_width = content_width * 0.7  # 70% for specification
    table = Table(data, colWidths=[col1_width, col2_width])
    table.setStyle(create_table_style())
    elements.append(table)
    elements.append(Spacer(1, 4))
    
    # Keep heading and table together
    story.append(KeepTogether(elements))

def add_all_specs(story, style):
    """Add all specification sections"""
    add_mechanical_specs(story, style)
    add_technical_specs(story, style)
    add_operating_conditions(story, style)
