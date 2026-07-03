"""Translation layer for the datasheet generator.

Every user-facing string is passed through tr() and is looked up in
translations/<lang>.json using the (whitespace-normalized) English text as
the key. If the English text in the code or source .txt files changes, the
key no longer matches, so the string automatically shows up as missing in
every language until it is re-translated — English stays the single source
of truth and translations can never silently go stale.

After a generation run, missing keys are written to
translations/missing_<lang>.json; translate those entries and merge them
into translations/<lang>.json.

Translations should only use Latin-1 characters (the PDF uses the built-in
Helvetica font, which cannot render e.g. Polish or Czech letters).
"""

import json
import os
import re

TRANSLATIONS_DIR = 'translations'
LANGUAGES = ['en', 'de', 'fr', 'es', 'it', 'pt']

_current_language = 'en'
_catalog = {}
_missing = {}  # normalized key -> None, insertion-ordered (doubles as template)


def normalize(text):
    """Collapse all runs of whitespace to single spaces and strip.

    Paragraph flowables collapse whitespace when rendering anyway, so this
    changes nothing visually while making catalog keys stable regardless of
    how strings are wrapped in the source code."""
    return re.sub(r'\s+', ' ', text).strip()


def set_language(lang):
    global _current_language, _catalog
    if lang not in LANGUAGES:
        raise ValueError(f"Unsupported language '{lang}'. Supported: {LANGUAGES}")
    _current_language = lang
    _catalog = {}
    _missing.clear()
    if lang != 'en':
        catalog_path = os.path.join(TRANSLATIONS_DIR, f'{lang}.json')
        if os.path.exists(catalog_path):
            with open(catalog_path, 'r', encoding='utf-8') as f:
                raw = json.load(f)
            _catalog = {normalize(k): v for k, v in raw.items() if v}
        else:
            print(f"Warning: {catalog_path} not found; all text will fall back to English")


def get_language():
    return _current_language


def tr(text):
    """Translate a string; falls back to English and records it as missing."""
    key = normalize(text)
    if not key or _current_language == 'en':
        return key
    translated = _catalog.get(key)
    if translated is None:
        _missing[key] = None
        return key
    return translated


def missing_keys():
    return list(_missing)


def report_missing():
    """Print a summary and write missing keys to a per-language file.

    Returns the number of missing translations (0 for English)."""
    if _current_language == 'en':
        return 0
    missing_path = os.path.join(TRANSLATIONS_DIR, f'missing_{_current_language}.json')
    if not _missing:
        if os.path.exists(missing_path):
            os.remove(missing_path)
        print(f"Translations complete for '{_current_language}'")
        return 0
    os.makedirs(TRANSLATIONS_DIR, exist_ok=True)
    with open(missing_path, 'w', encoding='utf-8') as f:
        json.dump({k: "" for k in _missing}, f, ensure_ascii=False, indent=2)
    print(f"Warning: {len(_missing)} missing translation(s) for '{_current_language}' "
          f"(fell back to English) — see {missing_path}")
    return len(_missing)
