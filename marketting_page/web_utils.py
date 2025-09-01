#!/usr/bin/env python3

import os

def create_html_section(title, content, section_id=None):
    """Create an HTML section with title and content"""
    id_attr = f' id="{section_id}"' if section_id else ''
    html = f'<section{id_attr} class="content-section">\n'
    html += '  <div class="container">\n'
    html += f'    <h2>{title}</h2>\n'
    
    # Split content into paragraphs if it contains newlines
    if content:
        paragraphs = content.split('\n')
        for paragraph in paragraphs:
            if paragraph.strip():
                html += f'    <p>{paragraph.strip()}</p>\n'
    
    html += '  </div>\n'
    html += '</section>\n\n'
    return html

def create_html_table(data, css_class='data-table'):
    """Create an HTML table from a 2D list"""
    if not data:
        return ''
    
    html = f'    <table class="{css_class}">\n'
    
    # First row is header
    html += '      <thead>\n'
    html += '        <tr>\n'
    for header in data[0]:
        html += f'          <th>{header}</th>\n'
    html += '        </tr>\n'
    html += '      </thead>\n'
    
    # Rest are data rows
    html += '      <tbody>\n'
    for row in data[1:]:
        html += '        <tr>\n'
        for cell in row:
            html += f'          <td>{cell}</td>\n'
        html += '        </tr>\n'
    html += '      </tbody>\n'
    html += '    </table>\n'
    
    return html

def create_image_element(src, alt, css_class=''):
    """Create an HTML image element"""
    class_attr = f' class="{css_class}"' if css_class else ''
    return f'<img src="{src}" alt="{alt}"{class_attr}>'

def create_code_block(code, language=''):
    """Create a formatted code block with syntax highlighting support"""
    lang_class = f' class="language-{language}"' if language else ''
    html = f'    <pre><code{lang_class}>'
    
    # Escape HTML special characters
    code = code.replace('&', '&amp;')
    code = code.replace('<', '&lt;')
    code = code.replace('>', '&gt;')
    
    html += code
    html += '</code></pre>\n'
    return html

def wrap_html_document(body_content, title='', css_file=''):
    """Wrap content in a complete HTML document structure"""
    html = '<!DOCTYPE html>\n'
    html += '<html lang="en">\n'
    html += '<head>\n'
    html += '  <meta charset="UTF-8">\n'
    html += '  <meta name="viewport" content="width=device-width, initial-scale=1.0">\n'
    
    if title:
        html += f'  <title>{title}</title>\n'
    
    if css_file:
        html += f'  <link rel="stylesheet" href="{css_file}">\n'
    
    # Add syntax highlighting support (optional - can use Prism.js or highlight.js)
    html += '  <!-- Optional: Add syntax highlighting library -->\n'
    html += '  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/themes/prism.min.css">\n'
    
    html += '</head>\n'
    html += '<body>\n'
    html += body_content
    
    # Add JavaScript for smooth scrolling and syntax highlighting
    html += '''
  <!-- Optional: Add syntax highlighting script -->
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/prism.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/components/prism-python.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/components/prism-cpp.min.js"></script>
'''
    
    html += '</body>\n'
    html += '</html>\n'
    
    return html

def save_css_file(filename, css_content):
    """Save CSS content to a file"""
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(css_content)

def read_json_features():
    """Read features from features.json if it exists"""
    import json
    features_file = 'features.json'
    
    if os.path.exists(features_file):
        try:
            with open(features_file, 'r') as f:
                return json.load(f)
        except:
            pass
    
    # Fallback to default features
    return [
        {
            "title": "All-in-One Design",
            "description": "Motor, encoder, driver, and controller integrated in a single compact unit"
        },
        {
            "title": "RS-485 Communication",
            "description": "Daisy-chain multiple motors on a single bus for simplified wiring"
        },
        {
            "title": "High Precision",
            "description": "4096 PPR magnetic encoder for accurate position control"
        },
        {
            "title": "Easy Integration",
            "description": "NEMA 17 compatible mounting for drop-in replacement"
        },
        {
            "title": "Wide Voltage Range",
            "description": "Operates from 12V to 24V for flexibility in different applications"
        },
        {
            "title": "Open Source",
            "description": "Full documentation, libraries, and examples available on GitHub"
        }
    ]