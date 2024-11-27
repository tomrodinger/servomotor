from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, CondPageBreak

from styles import (
    create_title_style, create_subtitle_style, 
    create_normal_style, create_footer_style
)
from utils import get_image_size, create_image_table, get_processed_image
from content import add_all_content
from specs import add_all_specs
from protocol import add_all_protocol
from company_info import add_company_info
from versioning import add_version_info, get_latest_version
from open_source import add_open_source_section
from indicators import add_indicators_section

def generate_pdf():
    """Generate the complete servomotor datasheet PDF"""
    # Get version info for filename
    version, date = get_latest_version()
    # Clean up date string: remove periods and commas, replace spaces with underscores
    date = date.replace(".", "").replace(",", "").replace(" ", "_")
    output_filename = f'servomotor_datasheet_v{version}_{date}.pdf'
    
    # Define page properties
    page_width, page_height = A4
    margin = 18 * mm  # Slightly reduced margin for better text fitting
    content_width = page_width - (2 * margin)
    
    # Create PDF document with tighter margins
    doc = SimpleDocTemplate(
        output_filename,
        pagesize=A4,
        rightMargin=margin,
        leftMargin=margin,
        topMargin=margin,
        bottomMargin=margin
    )
    
    # Initialize story
    story = []
    
    # Get styles
    title_style = create_title_style()
    subtitle_style = create_subtitle_style()
    normal_style = create_normal_style()
    
    # Add title
    story.append(Paragraph('M3 Series Servomotors - DATASHEET', title_style))
    story.append(Spacer(1, 10))
    
    # Add logo with correct aspect ratio
    logo_width = content_width * 0.2
    logo_img = get_processed_image('Gearotons_Logo.png', logo_width)
    logo_img.hAlign = 'CENTER'
    story.append(logo_img)
    story.append(Spacer(1, 5))
    
    # Add subtitles with reduced spacing
    story.append(Paragraph('Affordable and Simple All-in-One Motion Control', subtitle_style))
    story.append(Spacer(1, -10))
    story.append(Paragraph('From Education to Innovation', subtitle_style))
    story.append(Spacer(1, 10))
    
    # Add images
    try:
        # Main image
        main_width = content_width * 1.05
        main_img = get_processed_image('m3_series_overview.jpg', main_width)
        story.append(main_img)
        story.append(Spacer(1, 5))
        
        # Add introduction and other content sections
        add_all_content(story, normal_style)
        
    except Exception as e:
        print(f"Warning: Could not process images: {e}")
        pass
    
    # Add indicators section right after Getting Started Guide
    add_indicators_section(story, normal_style)
    
    # Add protocol sections
    add_all_protocol(story, normal_style)
    
    # Add specification sections
    add_all_specs(story, normal_style)
    
    # Add company information
    add_company_info(story, normal_style)
    
    # Add open source section
    add_open_source_section(story, normal_style)
    
    # Add version information
    add_version_info(story, normal_style)
    
    # Add footer
    story.append(Spacer(1, 10))
    footer_style = create_footer_style()
    story.append(Paragraph('(c) 2024 All specifications subject to change without notice.', footer_style))
    story.append(Paragraph('For more information and technical support, please contact our sales team.', footer_style))
    
    # Generate PDF
    doc.build(story)

if __name__ == '__main__':
    generate_pdf()
