# QR Code Color Reverser and PDF Generator

A Python utility that processes QR codes in SVG format to create color-reversed versions and generate PDF files for both color variants.

## Overview

This program reads SVG QR code files from a source directory, reverses their colors (black ↔ white), and outputs:
- Color-reversed SVG files
- PDF files for original QR codes (black dots on white background)
- PDF files for color-reversed QR codes (white dots on black background)

## Features

- **Color Reversal**: Converts black dots on white background to white dots on black background
- **PDF Generation**: Creates PDF versions of both original and color-reversed QR codes
- **Batch Processing**: Processes all SVG files in the source directory automatically
- **Directory Management**: Automatically creates output directories
- **Graceful Degradation**: Works without PDF generation if dependencies are missing

## Requirements

### Required
- Python 3.6 or higher
- SVG files in the source directory

### Optional (for PDF generation)
- `cairosvg` library

Install the optional dependency. Due to system package management, you may need to use a virtual environment:

**Option 1: Virtual Environment (Recommended)**
```bash
python3 -m venv venv
source venv/bin/activate
pip install cairosvg
```

**Option 2: System-wide (if allowed)**
```bash
pip install cairosvg
```

**Running the Program with Virtual Environment:**
```bash
source venv/bin/activate
python reverse_qr_colors.py
```

## Directory Structure

The program expects and creates the following directory structure:

```
project_root/
├── reverse_qr_colors.py           # Main program
├── README.md                      # This file
├── QR_codes_black_dots_on_white_background/  # Source directory (must exist)
│   └── *.svg                      # Input SVG files
├── QR_codes_white_dots_on_black_background/  # Created automatically
│   └── *.svg                      # Color-reversed SVG files
├── QR_codes_PDF_black_on_white/   # Created automatically
│   └── *.pdf                      # PDF files (original colors)
└── QR_codes_PDF_white_on_black/   # Created automatically
    └── *.pdf                      # PDF files (reversed colors)
```

## Usage

1. Place your SVG QR code files in the `QR_codes_black_dots_on_white_background/` directory
2. Run the program:
   ```bash
   python3 reverse_qr_colors.py
   ```

### Example Output

```
QR Code Color Reverser
==================================================
Created/verified destination directories:
  SVG (white on black): QR_codes_white_dots_on_black_background
  PDF (black on white): QR_codes_PDF_black_on_white
  PDF (white on black): QR_codes_PDF_white_on_black
Found 1 SVG file(s) to process
--------------------------------------------------
✓ SVG processed: M17_1.0_QR_Code.svg -> QR_codes_white_dots_on_black_background/M17_1.0_QR_Code.svg
✓ PDF generated: M17_1.0_QR_Code.svg -> QR_codes_PDF_black_on_white/M17_1.0_QR_Code.pdf
✓ PDF generated: M17_1.0_QR_Code.svg -> QR_codes_PDF_white_on_black/M17_1.0_QR_Code.pdf
--------------------------------------------------
Successfully processed 1/1 SVG file(s)
Successfully generated 2/2 PDF file(s)
Output directories:
  SVG files: QR_codes_white_dots_on_black_background
  PDF files (black on white): QR_codes_PDF_black_on_white
  PDF files (white on black): QR_codes_PDF_white_on_black
```

## How It Works

### Color Reversal Algorithm
The program uses a three-step replacement process to avoid double-replacement issues:
1. Replace white (#ffffff, #FFFFFF) with a temporary placeholder
2. Replace black (#000000, #000) with white (#ffffff)
3. Replace the temporary placeholder with black (#000000)

### PDF Generation
- Uses the `cairosvg` library to convert SVG content to PDF format
- Generates PDFs for both original and color-reversed versions
- Handles conversion errors gracefully and continues processing other files

## Error Handling

- **Missing Source Directory**: Program exits with an error message
- **Missing cairosvg**: PDF generation is skipped, but SVG processing continues
- **File Processing Errors**: Individual file errors are reported but don't stop batch processing
- **PDF Conversion Errors**: Reported individually while continuing with other files

## Supported File Formats

- **Input**: SVG files (*.svg)
- **Output**: 
  - SVG files (color-reversed)
  - PDF files (both color variants, if cairosvg is available)

## License

This project is provided as-is for educational and utility purposes.

## Troubleshooting

### PDF Generation Not Working
- Ensure `cairosvg` is installed: `pip install cairosvg`
- On some systems, you may need additional dependencies for cairosvg

### No Files Processed
- Check that SVG files exist in `QR_codes_black_dots_on_white_background/`
- Ensure file extensions are lowercase `.svg`

### Permission Errors
- Ensure write permissions for the output directories
- Run with appropriate user privileges if needed