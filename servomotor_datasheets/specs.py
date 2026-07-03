from reportlab.platypus import Paragraph, Spacer, Table, KeepTogether, PageBreak, Image
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from styles import create_heading_style, create_table_style, create_subtitle_style
from utils import create_data_table, get_image_size
from i18n import tr

def get_content_width():
    """Calculate content width to match main.py"""
    page_width, _ = A4
    margin = 20 * mm
    return page_width - (2 * margin)

def label_cell(text):
    """Wrap a (translated) table label in a Paragraph so long words wrap
    inside the cell instead of overflowing into the next column"""
    style = ParagraphStyle('SpecLabel', fontSize=12, leading=14)
    return Paragraph(tr(text), style)

def header_cell(text):
    """Like label_cell, but for cells in the green header row: a Paragraph
    ignores the table style's TEXTCOLOR, so set the white text color here"""
    style = ParagraphStyle('SpecHeader', fontSize=12, leading=14,
                           textColor=colors.whitesmoke)
    return Paragraph(tr(text), style)

def add_technical_specs(story, style):
    """Add technical specifications section"""
    # Create elements to keep together
    elements = []
    elements.append(Paragraph(tr('Technical Specifications'), create_heading_style()))
    elements.append(Spacer(1, 6))  # Reduced spacing

    data = [
        [header_cell('Parameter'), 'M17-60', 'M17-48', 'M17-40', 'M17-34'],
        [label_cell('Operating Voltage'), '12-24V', '12-24V', '12-24V', '12-24V'],
        [label_cell('Rated Torque'), '0.65 N.m', '0.55 N.m', '0.42 N.m', '0.28 N.m'],
        [label_cell('Maximum Speed'), '560 RPM', '560 RPM', '560 RPM', '560 RPM'],
        [label_cell('Maximum Current'), '1.1A', '1.1A', '1.1A', '1.0A'],
        [label_cell('Rated Power'), '26.4W', '26.4W', '26.4W', '24.0W']
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
    elements.append(Paragraph(tr('Mechanical Specifications'), create_heading_style()))
    elements.append(Spacer(1, 2))  # Minimal spacing after heading

    data = [
        [header_cell('Parameter'), 'M17-60', 'M17-48', 'M17-40', 'M17-34'],
        [label_cell('Dimensions (LxW)'), '42.2x42.2 mm', '42.2x42.2 mm', '42.2x42.2 mm', '42.2x42.2 mm'],
        [label_cell('Height'), '59.7 mm', '48.7 mm', '40.1 mm', '33.5 mm'],
        [label_cell('Shaft Length'), '20.6 mm', '20.6 mm', '20.6 mm', '20.6 mm'],
        [label_cell('Weight'), '470g', '360g', '285g', '210g'],
        [label_cell('Protection Class'), 'IP20', 'IP20', 'IP20', 'IP20']
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
    elements.append(Paragraph(tr('Operating Conditions'), create_heading_style()))
    elements.append(Spacer(1, 6))

    data = [
        [header_cell('Parameter'), header_cell('Specification')],
        [label_cell('Operating Temperature'), label_cell('0C to +80C')],
        [label_cell('Storage Temperature'), label_cell('-20C to +60C')],
        [label_cell('Humidity Range'), label_cell('20% to 80% RH (non-condensing)')],
        [label_cell('Installation Environment'), label_cell('Indoor use only')]
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
