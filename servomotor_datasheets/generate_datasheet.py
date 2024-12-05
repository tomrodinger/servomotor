from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm, inch
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Frame, PageTemplate
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors

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

def firstPage(canvas, doc):
    """First page has no footer"""
    pass

def laterPages(canvas, doc):
    """Draw footer on later pages"""
    page_width, page_height = A4
    
    # Add logo to footer (centered) - moved 30% closer to bottom
    logo_width = 35*mm
    logo_height = logo_width * 0.3
    logo_x = (page_width - logo_width) / 2
    logo_y = 3.5*mm  # Reduced from 5mm
    canvas.drawImage('Gearotons_Logo_and_Gearotons_Name.png', 
                    logo_x, logo_y, 
                    width=logo_width, 
                    height=logo_height,
                    preserveAspectRatio=True)
    
    # Add page number (right side) - moved 30% closer to bottom
    canvas.setFont('Helvetica', 10)
    canvas.setFillColor(colors.black)
    page_num_x = page_width - 20*mm
    page_num_y = 5*mm  # Reduced from 7mm
    canvas.drawString(page_num_x, page_num_y, str(doc.page))

def generate_pdf():
    """Generate the complete servomotor datasheet PDF"""
    # Get version info for filename
    version, date = get_latest_version()
    date = date.replace(".", "").replace(",", "").replace(" ", "_")
    output_filename = f'servomotor_datasheet_v{version}_{date}.pdf'
    
    # Define page properties
    page_width, page_height = A4
    margin = 18*mm
    top_margin = 3.6*mm
    bottom_margin = 3.6*mm  # Match top margin
    footer_space = 8*mm  # Reduced from 12mm
    
    # Create PDF document
    doc = SimpleDocTemplate(
        output_filename,
        pagesize=A4,
        rightMargin=margin,
        leftMargin=margin,
        topMargin=top_margin,
        bottomMargin=bottom_margin + footer_space
    )
    
    # Create page templates
    frame = Frame(
        doc.leftMargin,
        doc.bottomMargin,
        doc.width,
        doc.height - footer_space,
        id='normal'
    )
    
    templates = [
        PageTemplate(id='First', frames=frame, onPage=firstPage),
        PageTemplate(id='Later', frames=frame, onPage=laterPages)
    ]
    doc.addPageTemplates(templates)
    
    # Initialize story
    story = []
    
    # Get styles
    title_style = create_title_style()
    subtitle_style = create_subtitle_style()
    normal_style = create_normal_style()
    
    # Add title
    story.append(Paragraph('M3 Series Servomotors - DATASHEET', title_style))
    story.append(Spacer(1, 3))
    
    # Add logo
    logo_width = doc.width * 0.15
    logo_img = get_processed_image('Gearotons_Logo.png', logo_width)
    logo_img.hAlign = 'CENTER'
    story.append(logo_img)
    story.append(Spacer(1, 2))
    
    # Add subtitles
    story.append(Paragraph('Affordable and Simple All-in-One Motion Control', subtitle_style))
    story.append(Spacer(1, -3))
    story.append(Paragraph('From Education to Innovation', subtitle_style))
    story.append(Spacer(1, 3))
    
    # Add main image
    try:
        main_width = doc.width * 0.85
        main_img = get_processed_image('m3_series_overview.jpg', main_width)
        main_img.hAlign = 'CENTER'
        story.append(main_img)
        story.append(Spacer(1, 2))
    except Exception as e:
        print(f"Warning: Could not process main image: {e}")
    
    # Add content sections
    try:
        add_all_content(story, normal_style)
        add_indicators_section(story, normal_style)
        add_all_protocol(story, normal_style)
        add_all_specs(story, normal_style)
        add_company_info(story, normal_style)
        add_open_source_section(story, normal_style)
        add_version_info(story, normal_style)
    except Exception as e:
        print(f"Warning: Error adding content sections: {e}")
    
    # Add footer text
    story.append(Spacer(1, 3))
    footer_style = create_footer_style()
    story.append(Paragraph('(c) 2024 All specifications subject to change without notice.', footer_style))
    story.append(Paragraph('For more information and technical support, please contact our sales team.', footer_style))
    
    # Generate PDF
    doc.build(story)

if __name__ == '__main__':
    generate_pdf()
