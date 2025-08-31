from reportlab.platypus import Paragraph, Spacer
from styles import create_heading_style

def get_latest_version():
    """Get the latest version information from versions.txt"""
    try:
        with open('versions.txt', 'r') as f:
            # Read all lines and get the last valid line
            lines = [line.strip() for line in f.readlines() if line.strip()]
            if not lines:
                return "1.0", "Unknown"
            
            # Get the last line and split into version and date
            latest = lines[-1].split(',', 1)  # Split on first comma only
            if len(latest) != 2:
                return "1.0", "Unknown"
            
            version = latest[0].strip()
            date = latest[1].strip()
            return version, date
    except Exception as e:
        print(f"Error reading version info: {e}")
        return "1.0", "Unknown"

def add_version_info(story, style):
    """Add version information"""
    story.append(Spacer(1, 15))
    
    version, date = get_latest_version()
    version_text = f"Datasheet Version: {version} Release Date: {date}"
    
    story.append(Paragraph(version_text, style))
    story.append(Spacer(1, 15))
