from reportlab.platypus import Paragraph, Spacer, PageBreak
from styles import create_heading_style

def add_company_info(story, style):
    """Add company information section"""
    story.append(Paragraph('Company Information', create_heading_style()))
    story.append(Spacer(1, 8))
    
    company_info = """
    Green Eco Technology (Shenzhen) Company Limited
    Room C301E, 3F, Block CD, Tianjing Building
    Shatou Street, Futian District
    Shenzhen City, Guangdong Province, China
    """
    
    story.append(Paragraph(company_info.strip(), style))
    story.append(Spacer(1, 15))
