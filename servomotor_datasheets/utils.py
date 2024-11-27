from PIL import Image as PILImage
from reportlab.platypus import Image, Table, TableStyle
from styles import PRIMARY_COLOR
import os

def get_image_size(image_path, target_width):
    """Calculate image dimensions while maintaining aspect ratio"""
    with PILImage.open(image_path) as img:
        w, h = img.size
        aspect = h / float(w)
        return target_width, target_width * aspect

def create_image_table(detail_img, tech_img, img_width):
    """Create a table containing two images side by side"""
    image_table = Table([[detail_img, tech_img]], colWidths=[img_width + 8, img_width + 8])
    image_table.setStyle(TableStyle([
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('TOPPADDING', (0, 0), (-1, -1), 8),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 8),
    ]))
    return image_table

def read_file_content(filename):
    """Read content from a file"""
    with open(filename, 'r') as file:
        return file.read()

def read_features():
    """Read features from features.txt"""
    with open('features.txt', 'r') as file:
        return [line.strip() for line in file.readlines() if line.strip()]

def load_json_file(filename):
    """Load and parse a JSON file"""
    import json
    # Special handling for motor_commands.json
    if filename == 'motor_commands.json':
        filename = os.path.join('..', 'python_programs', 'servomotor', 'motor_commands.json')
    with open(filename, 'r') as file:
        return json.load(file)

def create_data_table(data, col_widths, style):
    """Create a formatted table from data"""
    table = Table(data, colWidths=col_widths)
    table.setStyle(style)
    return table

def get_processed_image(image_path, target_width):
    """Get image with correct size
    
    Args:
        image_path: Path to the image
        target_width: Desired width in points/pixels
    
    Returns:
        reportlab Image object ready for PDF insertion
    """
    # Calculate dimensions
    width, height = get_image_size(image_path, target_width)
    
    # Create reportlab Image
    img = Image(image_path, width=width, height=height)
    
    return img
