# Servomotor Datasheet Generator

The [`generate_datasheet.py`](generate_datasheet.py:53) program is a modular PDF generation system that creates professional datasheets for M17 Series Servomotors using ReportLab.

## Core Functionality
- Generates versioned PDF datasheets with automatic version management via [`versioning.py`](versioning.py:4)
- Creates both versioned files (e.g., `servomotor_datasheet_v1.7_Oct_17_2025.pdf`) and a `servomotor_datasheet_latest.pdf` copy
- Uses modular architecture with separate files for different content sections

## Key Components
- **Content Management**: [`content.py`](content.py:48), [`specs.py`](specs.py:13), [`protocol.py`](protocol.py), [`indicators.py`](indicators.py) handle different datasheet sections
- **Libraries**: [`python_library.py`](python_library.py), [`arduino_library.py`](arduino_library.py) document programming interfaces
- **Styling**: [`styles.py`](styles.py) provides consistent PDF formatting and corporate branding
- **Utilities**: [`utils.py`](utils.py) handles image processing and table creation
- **Company Info**: [`company_info.py`](company_info.py), [`open_source.py`](open_source.py) add corporate and licensing information

## Output Features
- Professional layout with company branding (Gearotons logo)
- Technical specifications tables for M17-40, M17-48, and M17-60 models
- Dimensional drawings, connection diagrams, and installation instructions
- Communication protocol documentation and programming examples
- Automatic page numbering and footer generation

**Dependencies**: ReportLab, Pillow, NumPy (see [`requirements.txt`](requirements.txt:1))