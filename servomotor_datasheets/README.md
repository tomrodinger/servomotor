# Servomotor Datasheet Generator

The [`generate_datasheet.py`](generate_datasheet.py:53) program is a modular PDF generation system that creates professional datasheets for M17 Series Servomotors using ReportLab.

## Core Functionality
- Generates versioned PDF datasheets with automatic version management via [`versioning.py`](versioning.py:4)
- Creates both versioned files (e.g., `datasheet_v1.9_en.pdf`) and a `datasheet_latest_<lang>.pdf` copy per language (all output filenames are kept to 25 characters or less)
- Multilingual: English, German, French, Spanish, Italian, Portuguese
- Uses modular architecture with separate files for different content sections

## Generating

```bash
python3 generate_datasheet.py              # English only
python3 generate_datasheet.py --lang de    # one language
python3 generate_datasheet.py --all        # all languages
```

## Translations

All user-facing text goes through [`i18n.py`](i18n.py)'s `tr()` and is looked up in
`translations/<lang>.json`, keyed by the (whitespace-normalized) **English text itself**.
English is the single source of truth: edit English text in the code, the `.txt` source
files, or `motor_commands.json` descriptions, and the changed strings automatically become
*missing* in every language (the generator falls back to English for them and can never
show a stale translation).

Update workflow after changing any English text:
1. `python3 generate_datasheet.py --all` — prints a warning per language and writes the
   untranslated strings to `translations/missing_<lang>.json` (exit code is non-zero when
   translations are incomplete)
2. Translate the entries in each `missing_<lang>.json` and merge them into
   `translations/<lang>.json` (asking Claude to do this works well); optionally prune keys
   that no longer exist
3. Re-run `python3 generate_datasheet.py --all` — the warnings disappear and the
   `missing_*.json` files are removed

Notes:
- Translations must only use Latin-1/cp1252 characters (the PDF uses built-in Helvetica);
  adding e.g. Polish would require embedding a Unicode TTF font first
- Command names, unit names, URLs, and code samples intentionally stay in English (they
  match the Python/Arduino APIs); command *descriptions* and everything else is translated
- To add a language: add its code to `LANGUAGES` in [`i18n.py`](i18n.py), run
  `generate_datasheet.py --lang <code>`, and translate the generated `missing_<code>.json`
  into `translations/<code>.json`

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