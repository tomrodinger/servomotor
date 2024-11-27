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
    elements.append(Spacer(1, 10))
    
    data = [
        ['Parameter', 'M3-60', 'M3-48', 'M3-40'],
        ['Operating Voltage', '12-30V', '12-30V', '12-30V'],
        ['Rated Torque', '0.65 N.m', '0.55 N.m', '0.42 N.m'],
        ['Maximum Speed', '560 RPM', '560 RPM', '560 RPM'],
        ['Maximum Current', '1.1A', '1.1A', '1.1A'],
        ['Rated Power', '38W', '32W', '25W']
    ]
    
    content_width = get_content_width()
    col_width = content_width / 4  # Equal width for 4 columns
    table = Table(data, colWidths=[col_width] * 4)
    table.setStyle(create_table_style())
    elements.append(table)
    elements.append(Spacer(1, 15))
    
    # Keep heading and table together
    story.append(PageBreak())
    story.append(KeepTogether(elements))

def add_mechanical_specs(story, style):
    """Add mechanical specifications section"""
    # Create elements to keep together
    elements = []
    elements.append(Paragraph('Mechanical Specifications', create_heading_style()))
    elements.append(Spacer(1, 2))  # Minimal spacing after heading
    
    data = [
        ['Parameter', 'M3-60', 'M3-48', 'M3-40'],
        ['Dimensions (LxW)', '42.2x42.2 mm', '42.2x42.2 mm', '42.2x42.2 mm'],
        ['Height', '59.8 mm', '48.6 mm', '41.6 mm'],
        ['Shaft Length', '20.4 mm', '20.4 mm', '18.5 mm'],
        ['Weight', '470g', '360g', '285g'],
        ['Protection Class', 'IP20', 'IP20', 'IP20']
    ]
    
    content_width = get_content_width()
    col_width = content_width / 4  # Equal width for 4 columns
    table = Table(data, colWidths=[col_width] * 4)
    table.setStyle(create_table_style())
    elements.append(table)
    elements.append(Spacer(1, 8))  # Reduced spacing after table
    
    # Fixed height for all images (in points) - increased by another 5%
    fixed_height = 173  # Previously 165
    
    # Add dimension images with fixed height
    for model in ['M3-60', 'M3-48', 'M3-40']:
        # Calculate width based on original aspect ratio
        img_path = f'{model}_dimensions.png'
        _, orig_height = get_image_size(img_path, content_width)
        aspect_ratio = orig_height / content_width
        new_width = fixed_height / aspect_ratio
        
        tech_img = Image(img_path, width=new_width, height=fixed_height)
        tech_img.hAlign = 'CENTER'
        elements.append(tech_img)
        elements.append(Spacer(1, 8))  # Reduced spacing between images
    
    # Keep everything together on one page
    story.append(KeepTogether(elements))

def add_operating_conditions(story, style):
    """Add operating conditions section"""
    # Create elements to keep together
    elements = []
    elements.append(Paragraph('Operating Conditions', create_heading_style()))
    elements.append(Spacer(1, 10))
    
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
    elements.append(Spacer(1, 8))  # Reduced from 15 to 8
    
    # Keep heading and table together
    story.append(KeepTogether(elements))

def add_all_specs(story, style):
    """Add all specification sections"""
    add_mechanical_specs(story, style)
    add_technical_specs(story, style)
    add_operating_conditions(story, style)
