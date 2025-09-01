# Servomotor Marketing Web Page Generator

## Quick Overview
Converts the existing PDF datasheet generator into an HTML marketing page generator for integration into an e-commerce site.

## Main Changes from PDF Version
- **Output**: HTML + separate CSS file (not PDF)
- **Layout**: Single continuous scrolling page (no pagination)
- **New Sections**:
  - Company Profile (with test_rack.jpg image)
  - Applications (with robotics.jpg and automation.jpg images)

## Implementation Plan

### Phase 1: Core Conversion
1. Create `generate_webpage.py` - main script that outputs HTML
2. Convert existing modules to HTML generators:
   - Content sections → HTML divs with semantic markup
   - Tables → HTML tables with CSS classes
   - Images → `<img>` tags with responsive sizing
   - Code examples → `<pre><code>` blocks with syntax highlighting support

### Phase 2: Add Marketing Sections
1. **Company Profile Section** (near end)
   - Load content from `company_profile.txt`
   - Include `test_rack.jpg` image
   
2. **Applications Section**
   - Create image gallery with two labeled images:
     - "Robotics" (robotics.jpg)
     - "Automation" (automation.jpg)

### File Structure
```
generate_webpage.py     # Main generator (similar to generate_datasheet.py)
web_styles.py          # CSS generation functions
web_utils.py           # HTML helper functions
marketing.html         # Generated HTML output
marketing.css          # Generated CSS styles
```

### Technical Approach
- Reuse existing content reading functions from current modules
- Keep same data sources (txt files, images)
- Generate separate HTML and CSS files for e-commerce integration
- Output: `marketing.html` + `marketing.css`

