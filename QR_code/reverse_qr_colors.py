#!/usr/bin/env python3
"""
QR Code Color Reverser and PDF Generator
Reverses the colors of QR codes in SVG format from black dots on white background
to white dots on black background, and generates PDF files for both variants.
"""

import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
try:
    import cairosvg
except (ImportError, OSError, Exception) as e:
    # cairosvg can be installed but still fail to import if native cairo libs
    # are missing (common on macOS/conda setups).
    print("Warning: cairosvg unavailable (will fall back to ImageMagick if available).")
    print(f"Reason: {e}")
    cairosvg = None


def ensure_svg_background(svg_content: str, background_hex: str) -> str:
    """Ensure the SVG has an explicit background rect.

    Some SVG QR code generators emit only the modules (black dots) with a
    transparent background. For reliable PDF output, insert a full-canvas
    background <rect> when missing.

    Args:
        svg_content: SVG as a string
        background_hex: '#ffffff' or '#000000'
    """
    background_hex = background_hex.strip().lower()
    if background_hex not in {"#ffffff", "#000000"}:
        return svg_content

    SVG_NS = "http://www.w3.org/2000/svg"
    ET.register_namespace("", SVG_NS)

    BLACK_VALUES = {"#000", "#000000", "black"}
    WHITE_VALUES = {"#fff", "#ffffff", "white"}
    wanted = BLACK_VALUES if background_hex == "#000000" else WHITE_VALUES

    def _parse_number(value: str):
        if not value:
            return None
        m = re.match(r"^\s*([0-9]*\.?[0-9]+)", str(value))
        return float(m.group(1)) if m else None

    def _get_canvas_size(root: ET.Element):
        view_box = root.attrib.get("viewBox")
        if view_box:
            parts = re.split(r"[ ,]+", view_box.strip())
            if len(parts) == 4:
                try:
                    return float(parts[2]), float(parts[3])
                except ValueError:
                    pass
        w = _parse_number(root.attrib.get("width"))
        h = _parse_number(root.attrib.get("height"))
        if w is not None and h is not None:
            return w, h
        return None, None

    def _normalize_color(value: str) -> str:
        return str(value).strip().lower()

    def _rect_covers_canvas(el: ET.Element, canvas_w, canvas_h) -> bool:
        if el.tag != f"{{{SVG_NS}}}rect":
            return False
        x = _parse_number(el.attrib.get("x", "0")) or 0.0
        y = _parse_number(el.attrib.get("y", "0")) or 0.0
        w_raw = str(el.attrib.get("width", "")).strip()
        h_raw = str(el.attrib.get("height", "")).strip()
        if w_raw == "100%" and h_raw == "100%" and abs(x) < 1e-9 and abs(y) < 1e-9:
            return True
        w = _parse_number(w_raw)
        h = _parse_number(h_raw)
        if canvas_w is None or canvas_h is None or w is None or h is None:
            return False
        return abs(x) < 1e-9 and abs(y) < 1e-9 and abs(w - canvas_w) < 1e-6 and abs(h - canvas_h) < 1e-6

    try:
        root = ET.fromstring(svg_content)
    except ET.ParseError:
        return svg_content

    canvas_w, canvas_h = _get_canvas_size(root)
    for el in root.iter():
        if not _rect_covers_canvas(el, canvas_w, canvas_h):
            continue
        fill = _normalize_color(el.attrib.get("fill", ""))
        style = el.attrib.get("style", "")
        if fill in wanted:
            return svg_content
        if re.search(r"\bfill\s*:\s*(#ffffff|#fff|white)\b", style, flags=re.IGNORECASE) and background_hex == "#ffffff":
            return svg_content
        if re.search(r"\bfill\s*:\s*(#000000|#000|black)\b", style, flags=re.IGNORECASE) and background_hex == "#000000":
            return svg_content

    rect_attrib = {"x": "0", "y": "0", "fill": background_hex}
    if canvas_w is not None and canvas_h is not None:
        rect_attrib["width"] = str(canvas_w)
        rect_attrib["height"] = str(canvas_h)
    else:
        rect_attrib["width"] = "100%"
        rect_attrib["height"] = "100%"

    root.insert(0, ET.Element(f"{{{SVG_NS}}}rect", rect_attrib))
    return ET.tostring(root, encoding="unicode")

def reverse_svg_colors(svg_content):
    """Reverse black/white colors in SVG content.

    Notes:
    - Some QR SVG generators emit named colors (e.g. fill="black") instead of hex.
      The previous implementation only swapped '#ffffff'/'#000000', so such files
      were left unchanged.
    - Some SVGs also have *no explicit background* (transparent). When reversed,
      you still need a black background for "white dots on black" output.
    """

    SVG_NS = "http://www.w3.org/2000/svg"
    ET.register_namespace("", SVG_NS)

    BLACK_VALUES = {"#000", "#000000", "black"}
    WHITE_VALUES = {"#fff", "#ffffff", "white"}

    def _parse_number(value: str):
        """Parse the leading numeric portion of an SVG length (e.g. '10.0mm')."""
        if not value:
            return None
        m = re.match(r"^\s*([0-9]*\.?[0-9]+)", str(value))
        return float(m.group(1)) if m else None

    def _get_canvas_size(root: ET.Element):
        view_box = root.attrib.get("viewBox")
        if view_box:
            parts = re.split(r"[ ,]+", view_box.strip())
            if len(parts) == 4:
                try:
                    return float(parts[2]), float(parts[3])
                except ValueError:
                    pass
        w = _parse_number(root.attrib.get("width"))
        h = _parse_number(root.attrib.get("height"))
        if w is not None and h is not None:
            return w, h
        return None, None

    def _normalize_color(value: str) -> str:
        return str(value).strip().lower()

    def _invert_color(value: str) -> str:
        v = _normalize_color(value)
        if v in BLACK_VALUES:
            return "#ffffff"
        if v in WHITE_VALUES:
            return "#000000"
        return value

    def _invert_style_attr(style: str) -> str:
        # Swap fill/stroke in inline style declarations.
        # Example: "fill:black;stroke:#fff" -> "fill:#ffffff;stroke:#000000"
        def repl(match: re.Match) -> str:
            prop = match.group(1)
            val = match.group(2)
            return f"{prop}:{_invert_color(val)}"

        return re.sub(r"\b(fill|stroke)\s*:\s*([^;]+)", repl, style, flags=re.IGNORECASE)

    def _rect_covers_canvas(el: ET.Element, canvas_w, canvas_h) -> bool:
        if el.tag != f"{{{SVG_NS}}}rect":
            return False
        x = _parse_number(el.attrib.get("x", "0")) or 0.0
        y = _parse_number(el.attrib.get("y", "0")) or 0.0

        # Allow percentage-based full cover.
        w_raw = str(el.attrib.get("width", "")).strip()
        h_raw = str(el.attrib.get("height", "")).strip()
        if w_raw == "100%" and h_raw == "100%" and abs(x) < 1e-9 and abs(y) < 1e-9:
            return True

        w = _parse_number(w_raw)
        h = _parse_number(h_raw)
        if canvas_w is None or canvas_h is None or w is None or h is None:
            return False
        return abs(x) < 1e-9 and abs(y) < 1e-9 and abs(w - canvas_w) < 1e-6 and abs(h - canvas_h) < 1e-6

    def _has_black_background(root: ET.Element) -> bool:
        canvas_w, canvas_h = _get_canvas_size(root)
        for el in root.iter():
            if not _rect_covers_canvas(el, canvas_w, canvas_h):
                continue
            fill = _normalize_color(el.attrib.get("fill", ""))
            style = el.attrib.get("style", "")
            if fill in BLACK_VALUES or re.search(r"\bfill\s*:\s*(#000000|#000|black)\b", style, flags=re.IGNORECASE):
                return True
        return False

    def _ensure_black_background(root: ET.Element) -> None:
        if _has_black_background(root):
            return

        canvas_w, canvas_h = _get_canvas_size(root)
        rect_attrib = {"x": "0", "y": "0", "fill": "#000000"}
        if canvas_w is not None and canvas_h is not None:
            rect_attrib["width"] = str(canvas_w)
            rect_attrib["height"] = str(canvas_h)
        else:
            rect_attrib["width"] = "100%"
            rect_attrib["height"] = "100%"

        bg = ET.Element(f"{{{SVG_NS}}}rect", rect_attrib)
        # Insert background as the first rendered element.
        root.insert(0, bg)

    # Parse SVG. If parsing fails, fall back to conservative string replacement.
    try:
        root = ET.fromstring(svg_content)
    except ET.ParseError:
        temp_placeholder = "#TEMP_COLOR_PLACEHOLDER"
        svg_content = svg_content.replace("#ffffff", temp_placeholder).replace("#FFFFFF", temp_placeholder)
        svg_content = svg_content.replace("#000000", "#ffffff").replace("#000", "#ffffff")
        return svg_content.replace(temp_placeholder, "#000000")

    # Invert common attributes.
    for el in root.iter():
        for attr in ("fill", "stroke", "stop-color"):
            if attr in el.attrib:
                el.attrib[attr] = _invert_color(el.attrib[attr])
        if "style" in el.attrib:
            el.attrib["style"] = _invert_style_attr(el.attrib["style"])

    # If the source SVG had no explicit white background, the inverted output will
    # have white modules on transparency. Ensure a black background.
    _ensure_black_background(root)

    return ET.tostring(root, encoding="unicode")

def convert_svg_to_pdf(svg_content, output_path):
    """
    Convert SVG content to PDF file.
    
    Args:
        svg_content (str): SVG content as string
        output_path (Path): Path where PDF should be saved
    
    Returns:
        bool: True if successful, False otherwise
    """
    # Prefer true vector conversions (cairosvg or rsvg-convert). ImageMagick is a
    # last-resort fallback and commonly rasterizes the SVG.
    if cairosvg is not None:
        try:
            cairosvg.svg2pdf(bytestring=svg_content.encode("utf-8"), write_to=str(output_path))
            return True
        except Exception as e:
            print(f"Warning: cairosvg failed for {output_path.name}: {e}")

    rsvg_convert = shutil.which("rsvg-convert")
    if rsvg_convert is not None:
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory() as td:
                tmp_svg = Path(td) / "input.svg"
                tmp_svg.write_text(svg_content, encoding="utf-8")

                # librsvg + cairo generates a vector PDF (no rasterization).
                cmd = [
                    rsvg_convert,
                    "--format",
                    "pdf",
                    "--output",
                    str(output_path),
                    str(tmp_svg),
                ]
                subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return True
        except subprocess.CalledProcessError as e:
            stderr = e.stderr.decode("utf-8", errors="replace") if e.stderr else str(e)
            print(f"Error converting to PDF with rsvg-convert for {output_path.name}: {stderr}")
            return False

    magick = shutil.which("magick")
    convert = shutil.which("convert")
    tool = magick or convert
    if tool is None:
        print("Error: no PDF conversion backend available. Install cairosvg+cairo or librsvg (rsvg-convert).")
        return False

    def _guess_background(svg_text: str) -> str:
        """Best-effort guess for background color for alpha removal.

        We primarily rely on `ensure_svg_background()` to make output SVGs have an
        explicit full-canvas background rect, but this keeps things robust.
        """
        SVG_NS = "http://www.w3.org/2000/svg"

        def _parse_number(value: str):
            if not value:
                return None
            m = re.match(r"^\s*([0-9]*\.?[0-9]+)", str(value))
            return float(m.group(1)) if m else None

        def _get_canvas_size(root: ET.Element):
            view_box = root.attrib.get("viewBox")
            if view_box:
                parts = re.split(r"[ ,]+", view_box.strip())
                if len(parts) == 4:
                    try:
                        return float(parts[2]), float(parts[3])
                    except ValueError:
                        pass
            w = _parse_number(root.attrib.get("width"))
            h = _parse_number(root.attrib.get("height"))
            if w is not None and h is not None:
                return w, h
            return None, None

        def _rect_covers_canvas(el: ET.Element, canvas_w, canvas_h) -> bool:
            if el.tag != f"{{{SVG_NS}}}rect":
                return False
            x = _parse_number(el.attrib.get("x", "0")) or 0.0
            y = _parse_number(el.attrib.get("y", "0")) or 0.0
            w_raw = str(el.attrib.get("width", "")).strip()
            h_raw = str(el.attrib.get("height", "")).strip()
            if w_raw == "100%" and h_raw == "100%" and abs(x) < 1e-9 and abs(y) < 1e-9:
                return True
            w = _parse_number(w_raw)
            h = _parse_number(h_raw)
            if canvas_w is None or canvas_h is None or w is None or h is None:
                return False
            return abs(x) < 1e-9 and abs(y) < 1e-9 and abs(w - canvas_w) < 1e-6 and abs(h - canvas_h) < 1e-6

        try:
            root = ET.fromstring(svg_text)
            canvas_w, canvas_h = _get_canvas_size(root)
            for el in root:
                if not _rect_covers_canvas(el, canvas_w, canvas_h):
                    continue
                fill = str(el.attrib.get("fill", "")).strip().lower()
                if fill in {"#000", "#000000", "black"}:
                    return "black"
                if fill in {"#fff", "#ffffff", "white"}:
                    return "white"
        except ET.ParseError:
            pass
        return "white"

    print(
        "Warning: generating PDF via ImageMagick fallback; output is likely rasterized. "
        "Install librsvg (rsvg-convert) for vector PDFs."
    )

    # Write to a temp SVG file and ask ImageMagick to render to PDF.
    # NOTE: operations like `-alpha` require an image to already be read, so those
    # flags must come *after* the input SVG.
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory() as td:
            tmp_svg = Path(td) / "input.svg"
            tmp_svg.write_text(svg_content, encoding="utf-8")

            background = _guess_background(svg_content)

            cmd = [
                tool,
                "-density",
                "600",
                "-background",
                background,
                str(tmp_svg),
                "-alpha",
                "remove",
                "-alpha",
                "off",
                str(output_path),
            ]
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return True
    except subprocess.CalledProcessError as e:
        stderr = e.stderr.decode("utf-8", errors="replace") if e.stderr else str(e)
        print(f"Error converting to PDF with ImageMagick for {output_path.name}: {stderr}")
        return False

def process_qr_codes():
    """
    Main function to process all SVG files from source to destination folders.
    Generates both reversed SVG files and PDF files for both color variants.
    """
    # Define source and destination directories
    source_dir = Path("./QR_codes_black_dots_on_white_background")
    dest_dir = Path("./QR_codes_white_dots_on_black_background")
    pdf_black_on_white_dir = Path("./QR_codes_PDF_black_on_white")
    pdf_white_on_black_dir = Path("./QR_codes_PDF_white_on_black")
    
    # Check if source directory exists
    if not source_dir.exists():
        print(f"Error: Source directory '{source_dir}' does not exist!")
        return
    
    # Create destination directories if they don't exist
    dest_dir.mkdir(parents=True, exist_ok=True)
    pdf_black_on_white_dir.mkdir(parents=True, exist_ok=True)
    pdf_white_on_black_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Created/verified destination directories:")
    print(f"  SVG (white on black): {dest_dir}")
    print(f"  PDF (black on white): {pdf_black_on_white_dir}")
    print(f"  PDF (white on black): {pdf_white_on_black_dir}")
    
    # Get all SVG files from source directory
    svg_files = list(source_dir.glob("*.svg"))
    
    if not svg_files:
        print(f"No SVG files found in {source_dir}")
        return
    
    print(f"Found {len(svg_files)} SVG file(s) to process")
    print("-" * 50)
    
    # Process each SVG file
    processed_count = 0
    pdf_generated_count = 0

    def _ensure_stem_suffix(stem: str, suffix: str) -> str:
        """Append suffix to a stem unless it is already present."""
        return stem if stem.endswith(suffix) else f"{stem}{suffix}"
    
    for svg_file in svg_files:
        try:
            # Read the original SVG content
            with open(svg_file, 'r', encoding='utf-8') as f:
                original_content = f.read()
            
            # Reverse the colors
            reversed_content = reverse_svg_colors(original_content)

            # Ensure explicit backgrounds for both variants (for reliable PDF rendering).
            original_content_for_pdf = ensure_svg_background(original_content, "#ffffff")
            reversed_content_for_pdf = ensure_svg_background(reversed_content, "#000000")
            
            # Define file paths
            stem = svg_file.stem
            stem_black_on_white = _ensure_stem_suffix(stem, "_black_dots_on_white_background")
            stem_white_on_black = _ensure_stem_suffix(stem, "_white_dots_on_black_background")

            dest_file = dest_dir / f"{stem_white_on_black}.svg"
            pdf_original_file = pdf_black_on_white_dir / f"{stem_black_on_white}.pdf"
            pdf_reversed_file = pdf_white_on_black_dir / f"{stem_white_on_black}.pdf"
            
            # Write the reversed SVG content to the new file
            with open(dest_file, 'w', encoding='utf-8') as f:
                f.write(reversed_content)
            
            processed_count += 1
            print(f"✓ SVG processed: {svg_file.name} -> {dest_file}")
            
            # Generate PDF files if cairosvg is available
            # Generate PDF files (via cairosvg if available, else ImageMagick fallback)
            if convert_svg_to_pdf(original_content_for_pdf, pdf_original_file):
                print(f"✓ PDF generated: {svg_file.name} -> {pdf_original_file}")
                pdf_generated_count += 1
            else:
                print(f"✗ Failed to generate PDF: {pdf_original_file}")
            
            if convert_svg_to_pdf(reversed_content_for_pdf, pdf_reversed_file):
                print(f"✓ PDF generated: {svg_file.name} -> {pdf_reversed_file}")
                pdf_generated_count += 1
            else:
                print(f"✗ Failed to generate PDF: {pdf_reversed_file}")
            
        except Exception as e:
            print(f"✗ Error processing {svg_file.name}: {str(e)}")
    
    print("-" * 50)
    print(f"Successfully processed {processed_count}/{len(svg_files)} SVG file(s)")
    print(f"Successfully generated {pdf_generated_count}/{len(svg_files) * 2} PDF file(s)")
    print(f"Output directories:")
    print(f"  SVG files: {dest_dir}")
    print(f"  PDF files (black on white): {pdf_black_on_white_dir}")
    print(f"  PDF files (white on black): {pdf_white_on_black_dir}")

if __name__ == "__main__":
    print("QR Code Color Reverser")
    print("=" * 50)
    process_qr_codes()
