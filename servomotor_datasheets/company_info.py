from reportlab.platypus import Paragraph, Spacer, PageBreak
from styles import create_heading_style

def add_company_info(story, style):
    """Add company mission section"""
    story.append(Paragraph('Our Mission', create_heading_style()))
    story.append(Spacer(1, 8))
    
    mission_content = """
    At Gearotons, we believe that the future belongs to those who understand and can work alongside artificial intelligence and automation systems. Our mission is to empower the next generation of innovators by making advanced motion control technology accessible, understandable, and affordable for makers, educators, and engineers worldwide.

    We are committed to breaking down the barriers that have traditionally separated students and hobbyists from professional-grade automation tools. By providing integrated solutions that eliminate complexity without sacrificing capability, we enable hands-on learning experiences that prepare young minds for a world where human creativity and machine precision work in harmony.

    Our core belief is that every student should have the opportunity to experiment with the building blocks of modern automation—from robotics and CNC systems to scientific instruments. Through our educational partnerships and open-source approach, we're fostering a generation that doesn't just use technology, but truly understands and shapes it. Together, we're building the foundation for innovators who will define the amazing future.
    """
    
    story.append(Paragraph(mission_content.strip(), style))
    story.append(Spacer(1, 15))
