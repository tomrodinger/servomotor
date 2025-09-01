from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.platypus import TableStyle
from reportlab.graphics.shapes import Drawing, Circle, String
from reportlab.lib.colors import white

# Define colors
PRIMARY_COLOR = colors.HexColor('#1b4d3e')  # Dark green
SECONDARY_COLOR = colors.HexColor('#34a853')  # Medium green
TEXT_COLOR = colors.HexColor('#333333')

def create_click_icon(width=30, height=30):  # Increased from 20 to 30
    """Create a click icon drawing"""
    d = Drawing(width, height)
    # Add circular background
    d.add(Circle(width/2, height/2, width/2-2, fillColor=SECONDARY_COLOR))
    # Add centered arrow symbol (adjusted position and size)
    d.add(String(width/2-5, height/2-6, '➜', fillColor=white, fontSize=16))
    return d

def create_title_style():
    return ParagraphStyle(
        'CustomTitle',
        parent=getSampleStyleSheet()['Heading1'],
        fontSize=24,
        textColor=PRIMARY_COLOR,
        alignment=TA_CENTER,
        spaceAfter=15
    )

def create_subtitle_style():
    return ParagraphStyle(
        'CustomSubTitle',
        parent=getSampleStyleSheet()['Heading2'],
        fontSize=16,
        textColor=SECONDARY_COLOR,
        alignment=TA_CENTER,
        spaceAfter=0,
        leading=20
    )

def create_heading_style():
    return ParagraphStyle(
        'CustomHeading',
        parent=getSampleStyleSheet()['Heading2'],
        fontSize=14,
        textColor=PRIMARY_COLOR,
        spaceBefore=10,
        spaceAfter=6
    )

def create_group_heading_style():
    return ParagraphStyle(
        'GroupHeading',
        parent=getSampleStyleSheet()['Normal'],
        fontSize=14,
        textColor=TEXT_COLOR,
        spaceBefore=8,
        spaceAfter=4,
        leading=16
    )

def create_normal_style():
    return ParagraphStyle(
        'CustomNormal',
        parent=getSampleStyleSheet()['Normal'],
        fontSize=10,
        textColor=TEXT_COLOR,
        leading=12
    )

def create_feature_style():
    return ParagraphStyle(
        'Feature',
        parent=getSampleStyleSheet()['Normal'],
        fontSize=10,
        textColor=TEXT_COLOR,
        leading=14,
        bulletColor=SECONDARY_COLOR,
        bulletText='●',
        bulletFontSize=14,
        leftIndent=20,
        firstLineIndent=0,
        spaceBefore=2,
        spaceAfter=2
    )

def create_link_style():
    return ParagraphStyle(
        'CustomLink',
        parent=getSampleStyleSheet()['Normal'],
        fontSize=15,  # 50% bigger than normal 10
        textColor=SECONDARY_COLOR,  # Changed to green
        alignment=TA_CENTER,
        spaceBefore=8,
        spaceAfter=8
    )

def create_footer_style():
    return ParagraphStyle(
        'Footer',
        parent=getSampleStyleSheet()['Normal'],
        fontSize=10,
        textColor=colors.gray,
        alignment=TA_CENTER
    )

def create_table_style():
    """Style for specification tables"""
    return TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), PRIMARY_COLOR),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTSIZE', (0, 0), (-1, 0), 12),
        ('FONTSIZE', (0, 1), (-1, -1), 12),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('GRID', (0, 0), (-1, -1), 1, colors.black),
        ('BOX', (0, 0), (-1, -1), 1, colors.black),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
    ])

def create_command_table_style():
    """Style specifically for command tables"""
    return TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), PRIMARY_COLOR),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('FONTSIZE', (0, 0), (-1, 0), 12),
        ('FONTSIZE', (0, 1), (-1, -1), 12),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
        ('GRID', (0, 0), (-1, -1), 1, colors.black),
        ('BOX', (0, 0), (-1, -1), 1, colors.black),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
    ])
