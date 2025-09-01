from reportlab.platypus import Paragraph, Spacer, Image, Table
from reportlab.lib.units import mm
from styles import create_link_style, create_click_icon, SECONDARY_COLOR, create_heading_style
from utils import get_processed_image
from content import IconLink

def add_open_source_section(story, normal_style):
    """Add the Open Source section to the datasheet"""
    
    # Add section heading
    heading_style = create_heading_style()
    story.append(Paragraph('Open Source', heading_style))
    story.append(Spacer(1, 5))
    
    # Add description text
    description = "We believe in making the world better through technology. All software, firmware, and PCB design files are available here:"
    story.append(Paragraph(description, normal_style))
    story.append(Spacer(1, 5))
    
    # Calculate content width
    page_width = 595.27  # A4 width in points
    margin = 18 * mm
    content_width = page_width - (2 * margin)
    
    # Add custom icon link flowable
    icon_link = IconLink(
        'click_here.png',
        'https://github.com/tomrodinger/servomotor',
        'https://github.com/tomrodinger/servomotor',
        content_width
    )
    story.append(icon_link)
    story.append(Spacer(1, 10))
    
    # Calculate image widths (50% and 40% of original size)
    base_width = 170 * mm * 0.3  # Original base width
    hw_width = base_width * 0.5   # 50% of original
    osi_width = base_width * 0.4  # 40% of original
    
    # Create both logos
    hw_logo = get_processed_image('Open-source-hardware-logo.svg.png', hw_width)
    osi_logo = get_processed_image('Open_Source_Initiative.svg.png', osi_width)
    
    # Create a table for side-by-side layout
    logo_table = Table([[hw_logo, osi_logo]], colWidths=[hw_width + 10, osi_width + 10])
    logo_table.hAlign = 'CENTER'
    
    # Add the table to the story
    story.append(logo_table)
    story.append(Spacer(1, 15))
