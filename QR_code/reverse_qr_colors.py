#!/usr/bin/env python3
"""
QR Code Color Reverser and PDF Generator
Reverses the colors of QR codes in SVG format from black dots on white background
to white dots on black background, and generates PDF files for both variants.
"""

import os
import re
from pathlib import Path
try:
    import cairosvg
except ImportError:
    print("Warning: cairosvg not installed. PDF generation will be skipped.")
    print("Install with: pip install cairosvg")
    cairosvg = None

def reverse_svg_colors(svg_content):
    """
    Reverse black and white colors in SVG content.
    Changes #ffffff (white) to #000000 (black) and vice versa.
    """
    # Create a temporary placeholder to avoid double replacement
    temp_placeholder = "#TEMP_COLOR_PLACEHOLDER"
    
    # Step 1: Replace white with placeholder
    svg_content = svg_content.replace('#ffffff', temp_placeholder)
    svg_content = svg_content.replace('#FFFFFF', temp_placeholder)
    
    # Step 2: Replace black with white
    svg_content = svg_content.replace('#000000', '#ffffff')
    svg_content = svg_content.replace('#000', '#ffffff')  # Handle short form
    
    # Step 3: Replace placeholder with black
    svg_content = svg_content.replace(temp_placeholder, '#000000')
    
    return svg_content

def convert_svg_to_pdf(svg_content, output_path):
    """
    Convert SVG content to PDF file.
    
    Args:
        svg_content (str): SVG content as string
        output_path (Path): Path where PDF should be saved
    
    Returns:
        bool: True if successful, False otherwise
    """
    if cairosvg is None:
        return False
    
    try:
        cairosvg.svg2pdf(bytestring=svg_content.encode('utf-8'),
                        write_to=str(output_path))
        return True
    except Exception as e:
        print(f"Error converting to PDF: {str(e)}")
        return False

def process_qr_codes():
    """
    Main function to process all SVG files from source to destination folders.
    Generates both reversed SVG files and PDF files for both color variants.
    """
    # Define source and destination directories
    source_dir = Path("./QR_codes_black_dots_on_white_background")
    dest_dir = Path("./QR_codes_white_dots_on_black_background")
    pdf_black_on_white_dir = Path("./QR_codes_PDF_black_on_white")
    pdf_white_on_black_dir = Path("./QR_codes_PDF_white_on_black")
    
    # Check if source directory exists
    if not source_dir.exists():
        print(f"Error: Source directory '{source_dir}' does not exist!")
        return
    
    # Create destination directories if they don't exist
    dest_dir.mkdir(parents=True, exist_ok=True)
    pdf_black_on_white_dir.mkdir(parents=True, exist_ok=True)
    pdf_white_on_black_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Created/verified destination directories:")
    print(f"  SVG (white on black): {dest_dir}")
    print(f"  PDF (black on white): {pdf_black_on_white_dir}")
    print(f"  PDF (white on black): {pdf_white_on_black_dir}")
    
    # Get all SVG files from source directory
    svg_files = list(source_dir.glob("*.svg"))
    
    if not svg_files:
        print(f"No SVG files found in {source_dir}")
        return
    
    print(f"Found {len(svg_files)} SVG file(s) to process")
    print("-" * 50)
    
    # Process each SVG file
    processed_count = 0
    pdf_generated_count = 0
    
    for svg_file in svg_files:
        try:
            # Read the original SVG content
            with open(svg_file, 'r', encoding='utf-8') as f:
                original_content = f.read()
            
            # Reverse the colors
            reversed_content = reverse_svg_colors(original_content)
            
            # Define file paths
            dest_file = dest_dir / svg_file.name
            pdf_original_file = pdf_black_on_white_dir / f"{svg_file.stem}.pdf"
            pdf_reversed_file = pdf_white_on_black_dir / f"{svg_file.stem}.pdf"
            
            # Write the reversed SVG content to the new file
            with open(dest_file, 'w', encoding='utf-8') as f:
                f.write(reversed_content)
            
            processed_count += 1
            print(f"✓ SVG processed: {svg_file.name} -> {dest_file}")
            
            # Generate PDF files if cairosvg is available
            if cairosvg is not None:
                # Generate PDF from original (black on white)
                if convert_svg_to_pdf(original_content, pdf_original_file):
                    print(f"✓ PDF generated: {svg_file.name} -> {pdf_original_file}")
                    pdf_generated_count += 1
                else:
                    print(f"✗ Failed to generate PDF: {pdf_original_file}")
                
                # Generate PDF from reversed colors (white on black)
                if convert_svg_to_pdf(reversed_content, pdf_reversed_file):
                    print(f"✓ PDF generated: {svg_file.name} -> {pdf_reversed_file}")
                    pdf_generated_count += 1
                else:
                    print(f"✗ Failed to generate PDF: {pdf_reversed_file}")
            
        except Exception as e:
            print(f"✗ Error processing {svg_file.name}: {str(e)}")
    
    print("-" * 50)
    print(f"Successfully processed {processed_count}/{len(svg_files)} SVG file(s)")
    if cairosvg is not None:
        print(f"Successfully generated {pdf_generated_count}/{len(svg_files) * 2} PDF file(s)")
    else:
        print("PDF generation skipped - cairosvg not available")
    print(f"Output directories:")
    print(f"  SVG files: {dest_dir}")
    if cairosvg is not None:
        print(f"  PDF files (black on white): {pdf_black_on_white_dir}")
        print(f"  PDF files (white on black): {pdf_white_on_black_dir}")

if __name__ == "__main__":
    print("QR Code Color Reverser")
    print("=" * 50)
    process_qr_codes()