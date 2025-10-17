from reportlab.platypus import Paragraph, Spacer, PageBreak, Table, TableStyle, KeepTogether, Image, Flowable
from reportlab.lib import colors
from reportlab.lib.units import mm
from styles import (
    create_heading_style, create_link_style, create_feature_style, PRIMARY_COLOR,
    SECONDARY_COLOR
)
from utils import read_file_content, read_features, load_json_file, get_processed_image
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.lib.pagesizes import A4

class IconLink(Flowable):
    def __init__(self, icon_path, text, url, width):
        Flowable.__init__(self)
        self.icon_path = icon_path
        self.text = text
        self.url = url
        self.width = width
        self.height = 30
        
    def wrap(self, *args):
        return (self.width, self.height)
        
    def draw(self):
        # Calculate center position
        text_width = self.canv.stringWidth(self.text, 'Helvetica', 15)
        total_width = 30 + 5 + text_width  # icon width + small gap + text width
        x_start = (self.width - total_width) / 2
        
        # Draw icon
        self.canv.drawImage(self.icon_path, x_start, 0, 30, 30)
        
        # Draw text with link annotation
        self.canv.setFont('Helvetica', 15)
        self.canv.setFillColor(SECONDARY_COLOR)
        
        # Create link annotation first
        self.canv.linkURL(
            self.url,
            (x_start + 35, 0, x_start + 35 + text_width, 30),
            relative=1
        )
        
        # Then draw the text
        self.canv.drawString(x_start + 35, 8, self.text)

def add_introduction(story, style):
    """Add introduction section"""
    story.append(Paragraph('Introduction', create_heading_style()))
    intro_text = read_file_content('introduction.txt')
    story.append(Paragraph(intro_text, style))
    story.append(Spacer(1, 6))  # Reduced spacing

def add_features(story, style):
    """Add key features section"""
    # Create elements list for keeping together
    elements = []
    
    # Add heading and features
    elements.append(Paragraph('Key Features', create_heading_style()))
    features = read_features()
    feature_style = create_feature_style()
    for feature in features:
        elements.append(Paragraph(feature, feature_style))
    
    # Keep all elements together
    story.append(KeepTogether(elements))
    story.append(Spacer(1, 6))  # Reduced spacing

def add_connection_diagram(story, style):
    """Add connection diagram section"""
    # Create elements list for keeping together
    elements = []
    
    # Add heading
    elements.append(Paragraph('Connection Diagram', create_heading_style()))
    
    # Calculate content width to match main.py
    page_width, _ = A4
    margin = 18 * mm  # Using the same margin as in main.py
    content_width = page_width - (2 * margin)
    
    # Add the connection diagram image at full width
    diagram_img = get_processed_image('connection_diagram.jpg', content_width)
    diagram_img.hAlign = 'CENTER'
    elements.append(diagram_img)
    
    # Keep all elements together
    story.append(KeepTogether(elements))
    story.append(Spacer(1, 6))  # Reduced spacing

def add_unit_system(story, style):
    """Add unit system section"""
    # Create elements list for keeping together
    elements = []
    
    # Add heading and description
    elements.append(Paragraph('Unit System', create_heading_style()))
    elements.append(Spacer(1, 4))  # Reduced spacing
    
    elements.append(Paragraph(
        'The M17 Series Servomotors have certain internal units so that they can perform the calculations '
        'associated with motion efficiently (using integer math). It is the responsibility of the controlling '
        'software to support multiple units of measurement for various quantities. Our Python and Arduino libraries handle '
        'unit conversions automatically, allowing you to work with your preferred units. Below are the supported '
        'units for each quantity:', style))
    elements.append(Spacer(1, 4))  # Reduced spacing
    
    # Calculate content width to match main.py
    page_width, _ = A4
    margin = 20 * mm
    content_width = page_width - (2 * margin)
    
    # Load unit conversion data
    unit_data = load_json_file('../python_programs/servomotor/unit_conversions_M3.json')
    units = unit_data['units']
    
    # Create a paragraph style for table cells
    cell_style = ParagraphStyle(
        'CellStyle',
        fontSize=10,
        leading=12,
        alignment=TA_LEFT
    )
    
    # Create table data with Paragraph objects for wrapping
    table_data = [['Quantity', 'Available Units']]  # Header row
    
    # Add each quantity and its units
    for quantity, unit_list in units.items():
        # Format quantity name
        quantity_name = quantity.replace('_', ' ').title()
        # Format unit list with line breaks for better readability
        unit_text = ', '.join(unit.replace('_', ' ') for unit in unit_list)
        # Wrap both columns in Paragraph objects
        table_data.append([
            Paragraph(quantity_name, cell_style),
            Paragraph(unit_text, cell_style)
        ])
    
    # Create table with width matching content_width
    col1_width = content_width * 0.25  # 25% of content width
    col2_width = content_width * 0.75  # 75% of content width
    table = Table(data=table_data, colWidths=[col1_width, col2_width])
    table.setStyle(TableStyle([
        ('GRID', (0, 0), (-1, -1), 1, colors.black),
        ('BACKGROUND', (0, 0), (-1, 0), PRIMARY_COLOR),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTSIZE', (0, 0), (-1, 0), 10),
        ('BOTTOMPADDING', (0, 0), (-1, 0), 8),  # Reduced padding
        ('TOPPADDING', (0, 0), (-1, 0), 8),  # Reduced padding
        ('BOTTOMPADDING', (0, 1), (-1, -1), 6),  # Reduced padding
        ('TOPPADDING', (0, 1), (-1, -1), 6),  # Reduced padding
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
    ]))
    
    elements.append(table)
    elements.append(Spacer(1, 6))  # Reduced spacing
    
    # Keep all elements together
    story.append(KeepTogether(elements))

def add_getting_started_section(story, style):
    """Add getting started guide section"""
    story.append(Paragraph('Getting Started Guide', create_heading_style()))
    
    story.append(Paragraph(
        'To help you get started with your M17 Series Servomotor, we provide a comprehensive online guide '
        'that covers everything from initial setup to advanced protocol implementations. This guide includes:',
        style
    ))
    
    guide_features = [
        'Step-by-step setup instructions',
        'Detailed communication protocol documentation',
        'Programming examples and code snippets',
        'Description of error codes',
        'Troubleshooting tips and best practices'
    ]
    
    feature_style = create_feature_style()
    for feature in guide_features:  # Fixed: using guide_features instead of features
        story.append(Paragraph(feature, feature_style))
    
    story.append(Spacer(1, 4))  # Reduced spacing
    
    # Calculate content width
    page_width, _ = A4
    margin = 20 * mm
    content_width = page_width - (2 * margin)
    
    # Add custom icon link flowable
    icon_link = IconLink(
        'click_here.png',
        'Click Here to Visit our Getting Started Guide: tutorial.gearotons.com',
        'https://tutorial.gearotons.com/',
        content_width
    )
    story.append(icon_link)
    story.append(Spacer(1, 6))  # Reduced spacing

def add_feedback_section(story, style):
    """Add feedback section"""
    story.append(Paragraph('Send Us Feedback', create_heading_style()))
    
    story.append(Paragraph(
        'If you find errors in this document or have questions, please let us know by going to: '
        'tutorial.gearotons.com/feedback. We would really appreciate your suggestions on how '
        'we can improve the product or documentation. Thanks very much!',
        style
    ))
    
    story.append(Spacer(1, 4))  # Reduced spacing
    
    # Calculate content width
    page_width, _ = A4
    margin = 20 * mm
    content_width = page_width - (2 * margin)
    
    # Add custom icon link flowable
    icon_link = IconLink(
        'click_here.png',
        'Visit our Feedback Page',
        'https://tutorial.gearotons.com/feedback',
        content_width
    )
    story.append(icon_link)
    story.append(Spacer(1, 6))  # Reduced spacing

def add_all_content(story, style):
    """Add all content sections"""
    add_introduction(story, style)
    add_feedback_section(story, style)
    add_features(story, style)
    add_connection_diagram(story, style)
    add_unit_system(story, style)
    add_getting_started_section(story, style)
