from reportlab.platypus import Paragraph, Spacer
from styles import create_heading_style

def add_indicators_section(story, style):
    """Add the Indicator LEDs and Buttons section"""
    story.append(Paragraph('Indicator LEDs and Buttons', create_heading_style()))
    story.append(Spacer(1, 8))
    
    led_description = (
        "The servomotor has two status LEDs (Green and Red). The green LED flashes slowly to show a heart beat "
        "and quickly to indicate that the bootloader is running rather than the application. The red LED will "
        "light up briefly to show communication on the bus and will indicate fatal error codes by flashing a "
        "certain number of times."
    )
    
    button_description = (
        "The servomotor has two buttons labelled \"Reset\" and \"Test\". The Reset button will reset the internal "
        "microcontroller and all state will go back to default values. The Test button will cause the motor to spin. "
        "Press briefly to let it spin one way and press for more than 0.3 seconds and release to let it spin the other "
        "way. Hold down for at least 2 seconds and release to cause the motor to go to closed loop mode. Hold down for "
        "more than 15 seconds and release to let the motor perform a calibration on itself. Note that it will spin during "
        "calibration and must be able to spin freely for calibration to be successful, so remove any loads before doing "
        "this operation."
    )
    
    story.append(Paragraph(led_description, style))
    story.append(Spacer(1, 8))
    story.append(Paragraph(button_description, style))
    story.append(Spacer(1, 15))
