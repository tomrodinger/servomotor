#!/usr/bin/env python3

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm, inch
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Frame, PageTemplate
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from datetime import datetime

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
from python_library import add_python_library_section
from arduino_library import add_arduino_library_section
import i18n
from i18n import tr

import argparse
import shutil

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

def generate_pdf(lang='en'):
    """Generate the complete servomotor datasheet PDF for one language.

    Returns the number of missing translations (0 when complete)."""
    i18n.set_language(lang)

    # Get version info for filename. Filenames must stay at 25 characters or
    # less, e.g. 'datasheet_v1.9_de.pdf' / 'datasheet_latest_de.pdf'
    version, date = get_latest_version()
    output_filename = f'datasheet_v{version}_{lang}.pdf'
    latest_filename = f'datasheet_latest_{lang}.pdf'
    for name in (output_filename, latest_filename):
        assert len(name) <= 25, f"Output filename '{name}' exceeds 25 characters"

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
    story.append(Paragraph(tr('M17 Series Servomotors - DATASHEET'), title_style))
    story.append(Spacer(1, 3))
    
    # Add logo
    logo_width = doc.width * 0.15
    logo_img = get_processed_image('Gearotons_Logo.png', logo_width)
    logo_img.hAlign = 'CENTER'
    story.append(logo_img)
    story.append(Spacer(1, 2))
    
    # Add subtitles
    story.append(Paragraph(tr('Affordable and Simple All-in-One Motion Control'), subtitle_style))
    story.append(Spacer(1, -3))
    story.append(Paragraph(tr('From Education to Innovation'), subtitle_style))
    story.append(Spacer(1, 3))
    
    # Add main image
    try:
        main_width = doc.width * 0.85
        main_img = get_processed_image('m17_series_overview.jpg', main_width)
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
        add_python_library_section(story, normal_style)
        add_arduino_library_section(story, normal_style)
        add_company_info(story, normal_style)
        add_open_source_section(story, normal_style)
        add_version_info(story, normal_style)
    except Exception as e:
        print(f"Warning: Error adding content sections: {e}")
    
    # Add footer text
    story.append(Spacer(1, 3))
    footer_style = create_footer_style()
    current_year = datetime.now().year
    footer_text = tr('(c) {year} All specifications subject to change without notice.').format(year=current_year)
    story.append(Paragraph(footer_text, footer_style))

    # Generate PDF
    try:
        doc.build(story)
        # Copy to latest
        shutil.copy(output_filename, latest_filename)
        print("Success: Datasheet generated successfully.")
        print(f"Generated files: {output_filename}, {latest_filename}")
    except Exception as e:
        print(f"Error generating datasheet: {e}")
        return 1

    # Report any strings that fell back to English (writes
    # translations/missing_<lang>.json listing exactly what to translate)
    return i18n.report_missing()

def main():
    parser = argparse.ArgumentParser(description='Generate the servomotor datasheet PDF(s)')
    parser.add_argument('--lang', choices=i18n.LANGUAGES, default=None,
                        help='generate a single language (default: en)')
    parser.add_argument('--all', action='store_true',
                        help='generate all languages: ' + ', '.join(i18n.LANGUAGES))
    args = parser.parse_args()

    languages = i18n.LANGUAGES if args.all else [args.lang or 'en']
    total_missing = 0
    for lang in languages:
        print(f"\n=== Generating '{lang}' datasheet ===")
        total_missing += generate_pdf(lang)

    if total_missing:
        print(f"\nWARNING: {total_missing} missing translation(s) in total. "
              "Translate the entries in translations/missing_*.json, merge them into "
              "translations/<lang>.json, and regenerate.")
    return 1 if total_missing else 0

if __name__ == '__main__':
    raise SystemExit(main())
