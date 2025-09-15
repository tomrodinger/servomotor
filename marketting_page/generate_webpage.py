#!/usr/bin/env python3

import os
import json
import argparse
import shutil
from datetime import datetime

def ensure_dir(directory):
    """Ensure directory exists, create if it doesn't"""
    if not os.path.exists(directory):
        os.makedirs(directory)
        print(f"Created directory: {directory}")

def copy_file(src, dst):
    """Copy a file from src to dst"""
    shutil.copy2(src, dst)
    print(f"Copied: {src} -> {dst}")

def format_file_size(size_bytes):
    """Format file size in human-readable format"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size_bytes < 1024.0:
            return f"{size_bytes:.1f} {unit}"
        size_bytes /= 1024.0
    return f"{size_bytes:.1f} TB"

def get_image_dimensions(image_path):
    """Get dimensions of an image"""
    try:
        from PIL import Image
        if os.path.exists(image_path):
            with Image.open(image_path) as img:
                return img.size  # Returns (width, height)
    except ImportError:
        print(f"Warning: PIL not installed, cannot get dimensions for {image_path}")
    except Exception as e:
        print(f"Error getting dimensions for {image_path}: {e}")
    return None

def get_smart_max_width(image_name):
    """Determine optimal max width based on image type"""
    # Technical diagrams and dimension drawings - preserve detail
    if 'diagram' in image_name.lower() or 'dimensions' in image_name.lower():
        return 1200
    # Logos and icons - moderate size
    elif 'logo' in image_name.lower() or 'icon' in image_name.lower() or 'click_here' in image_name.lower():
        return 400
    # Overview images - good quality
    elif 'overview' in image_name.lower():
        return 800
    # Regular photos - standard web optimization
    else:
        return 800

def optimize_and_copy_image(src, dst, max_width=None):
    """Copy and optimize image for web, returning metadata"""
    if not os.path.exists(src):
        print(f"Warning: Source image {src} not found")
        return None
    
    # Auto-determine max width if not specified
    if max_width is None:
        max_width = get_smart_max_width(src)
    
    # Get original file info
    original_size = os.path.getsize(src)
    original_dims = get_image_dimensions(src)
    
    # Try to optimize the image
    optimization_result = resize_image(src, dst, max_width)
    
    # Get optimized file info
    if os.path.exists(dst):
        optimized_size = os.path.getsize(dst)
        optimized_dims = get_image_dimensions(dst)
        
        # Calculate compression ratio
        if original_size > 0:
            compression_ratio = ((original_size - optimized_size) / original_size) * 100
        else:
            compression_ratio = 0
        
        return {
            'src_path': src,
            'dst_path': dst,
            'original_size': original_size,
            'optimized_size': optimized_size,
            'original_dims': original_dims,
            'optimized_dims': optimized_dims,
            'compression_ratio': compression_ratio,
            'optimization_applied': optimization_result
        }
    
    return None

def resize_image(input_path, output_path, max_width=600):
    """Resize image to optimize for web, creating a smaller version"""
    try:
        from PIL import Image

        # Check if input file exists
        if not os.path.exists(input_path):
            print(f"Warning: Image {input_path} not found, skipping resize")
            return False

        # Open image
        with Image.open(input_path) as img:
            # Calculate new size maintaining aspect ratio
            if img.size[0] > max_width:
                width_percent = max_width / float(img.size[0])
                new_height = int(float(img.size[1]) * width_percent)
                
                # Resize the image
                img_resized = img.resize((max_width, new_height), Image.Resampling.LANCZOS)
                img_resized.save(output_path, optimize=True, quality=85)
                print(f"Created optimized image: {output_path} ({max_width}x{new_height})")
                return True
            else:
                # If already small enough, just copy with optimization
                img.save(output_path, optimize=True, quality=85)
                print(f"Optimized image: {output_path} (preserved size {img.size[0]}x{img.size[1]})")
                return True

    except ImportError:
        print("Warning: PIL not installed. Install with: pip install Pillow")
        # Fallback to simple copy
        copy_file(input_path, output_path)
        return False
    except Exception as e:
        print(f"Error resizing image {input_path}: {e}")
        # Fallback to simple copy
        copy_file(input_path, output_path)
        return False

def get_optimized_image_name(image_name):
    """Generate optimized image filename"""
    name, ext = os.path.splitext(image_name)
    return f"{name}_small{ext}"

def read_file_content(filename):
    """Read content from a text file"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        return ''

def read_features():
    """Read features from features.txt"""
    try:
        with open('features.txt', 'r', encoding='utf-8') as f:
            features = []
            for line in f:
                line = line.strip()
                if line:
                    features.append(line)
            return features
    except FileNotFoundError:
        return []

def load_json_file(filename):
    """Load data from a JSON file"""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except:
        return {}

def get_latest_version():
    """Get the latest version information"""
    try:
        with open('versions.txt', 'r') as f:
            lines = f.readlines()
            for line in lines:
                if line.strip() and not line.startswith('#'):
                    parts = line.strip().split(' - ')
                    if len(parts) >= 2:
                        version = parts[0].replace('Version', '').strip()
                        date = parts[1].strip()
                        return version, date
    except:
        pass
    return '1.0', 'December 2024'

def generate_marketing_component(image_dimensions):
    """Generate MarketingContent.js component for e-commerce compatibility"""
    # Read template file
    with open('templates/next_index_template.jsx', 'r', encoding='utf-8') as f:
        jsx_template = f.read()
    
    # Use e-commerce compatible import path
    jsx_template = jsx_template.replace("import styles from '../styles/Marketing.module.css'",
                                        "import styles from '@/styles/Marketing.module.css'")

    # Generate dynamic content
    intro_paragraphs = generate_intro_paragraphs()
    features_li = generate_features_list()
    unit_table = generate_unit_table_html()
    company_paragraphs = generate_company_paragraphs()
    version, date = get_latest_version()

    # Replace content tokens in templates
    jsx_content = jsx_template.replace('__INTRO_PARAGRAPHS__', intro_paragraphs)
    jsx_content = jsx_content.replace('__FEATURES_LI__', features_li)
    jsx_content = jsx_content.replace('__UNIT_TABLE__', unit_table)
    jsx_content = jsx_content.replace('__COMPANY_PARAGRAPHS__', company_paragraphs)
    jsx_content = jsx_content.replace('__VERSION__', version)
    jsx_content = jsx_content.replace('__DATE__', date)
    
    # Fix Open Source image filename references
    jsx_content = jsx_content.replace(
        'src="/marketing/images/Open-source-hardware-logo.svg.png"',
        'src="/marketing/images/Open_source_hardware_logo_svg.png"'
    )
    jsx_content = jsx_content.replace(
        'src="/marketing/images/Open_Source_Initiative.svg.png"',
        'src="/marketing/images/Open_Source_Initiative_svg.png"'
    )
    
    # Replace image dimension tokens
    dimension_replacements = {}
    
    for image_name, dims in image_dimensions.items():
        if dims:
            width, height = dims
            # Create token names from image name - handle various formats
            base_name = image_name.replace('.jpg', '').replace('.png', '').replace('.svg', '')
            base_name = base_name.replace('-', '_').replace(' ', '_').replace('.', '_')
            
            # Store dimensions for this image
            dimension_replacements[base_name] = (width, height)
    
    # Apply all dimension replacements
    for base_name, (width, height) in dimension_replacements.items():
        width_token = f'__WIDTH_{base_name}__'
        height_token = f'__HEIGHT_{base_name}__'
        
        # Replace in JSX content
        jsx_content = jsx_content.replace(width_token, str(width))
        jsx_content = jsx_content.replace(height_token, str(height))
    
    # Check for any remaining unreplaced tokens and provide defaults
    import re
    width_pattern = r'__WIDTH_([^_]+(?:_[^_]+)*)__'
    height_pattern = r'__HEIGHT_([^_]+(?:_[^_]+)*)__'
    
    # Find any unreplaced width tokens
    remaining_widths = re.findall(width_pattern, jsx_content)
    remaining_heights = re.findall(height_pattern, jsx_content)
    
    # Provide defaults for any remaining tokens
    for token_name in remaining_widths:
        default_width = "800"  # Default width
        jsx_content = jsx_content.replace(f'__WIDTH_{token_name}__', default_width)
        print(f"Warning: Used default width {default_width} for {token_name}")
    
    for token_name in remaining_heights:
        default_height = "600"  # Default height
        jsx_content = jsx_content.replace(f'__HEIGHT_{token_name}__', default_height)
        print(f"Warning: Used default height {default_height} for {token_name}")
    
    return jsx_content

def generate_css_content():
    """Generate CSS content for Marketing.module.css"""
    with open('templates/marketing_module_template.css', 'r', encoding='utf-8') as f:
        css_content = f.read()
    return css_content

def generate_preview_index():
    """Generate simple pages/index.js for preview that imports the component"""
    return '''import MarketingContent from '../components/MarketingContent';

export default function Home() {
  return <MarketingContent />;
}
'''

def generate_unified_files(output_dir, image_dimensions):
    """Generate unified file structure compatible with e-commerce"""
    # Create directories
    components_dir = os.path.join(output_dir, 'components')
    pages_dir = os.path.join(output_dir, 'pages')
    styles_dir = os.path.join(output_dir, 'styles')
    
    ensure_dir(components_dir)
    ensure_dir(pages_dir)
    ensure_dir(styles_dir)
    
    # Generate MarketingContent.js (e-commerce compatible)
    marketing_component = generate_marketing_component(image_dimensions)
    with open(os.path.join(components_dir, 'MarketingContent.js'), 'w', encoding='utf-8') as f:
        f.write(marketing_component)
    print("✅ Generated components/MarketingContent.js")
    
    # Generate pages/index.js (preview only)
    preview_index = generate_preview_index()
    with open(os.path.join(pages_dir, 'index.js'), 'w', encoding='utf-8') as f:
        f.write(preview_index)
    print("✅ Generated pages/index.js (preview wrapper)")
    
    # Generate CSS Module
    css_content = generate_css_content()
    with open(os.path.join(styles_dir, 'Marketing.module.css'), 'w', encoding='utf-8') as f:
        f.write(css_content)
    print("✅ Generated styles/Marketing.module.css")

def generate_intro_paragraphs():
    """Generate introduction paragraphs for JSX"""
    content = read_file_content('introduction.txt')
    if not content:
        return '<p>The M17 Series Servomotors are all-in-one motion control solutions that integrate a motor, motor driver, motion controller, and encoder in a single compact package.</p>'

    paragraphs = content.split('\n')
    html = ''
    for paragraph in paragraphs:
        if paragraph.strip():
            html += f'<p>{paragraph.strip()}</p>\n'
    return html

def generate_features_list():
    """Generate features list items for JSX"""
    features = read_features()
    html = ''
    for feature in features:
        html += f'<li>{feature}</li>\n'
    return html

def generate_unit_table_html():
    """Generate unit system table for JSX"""
    # Try to load unit conversion data
    unit_data = load_json_file('../python_programs/servomotor/unit_conversions_M3.json')
    if unit_data and 'units' in unit_data:
        units = unit_data['units']
        html = '''<table className={styles.specsTable}>
      <thead>
        <tr>
          <th>Quantity</th>
          <th>Available Units</th>
        </tr>
      </thead>
      <tbody>'''

        for quantity, unit_list in units.items():
            quantity_name = quantity.replace('_', ' ').title()
            unit_text = ', '.join(unit.replace('_', ' ') for unit in unit_list)
            html += f'''
        <tr>
          <td>{quantity_name}</td>
          <td>{unit_text}</td>
        </tr>'''

        html += '''
      </tbody>
    </table>'''
    else:
        # Fallback table with common units
        html = '''<table className={styles.specsTable}>
      <thead>
        <tr>
          <th>Quantity</th>
          <th>Available Units</th>
        </tr>
      </thead>
      <tbody>
        <tr><td>Time</td><td>timesteps, seconds, milliseconds, minutes</td></tr>
        <tr><td>Position</td><td>shaft rotations, degrees, radians, encoder counts</td></tr>
        <tr><td>Velocity</td><td>rotations per second, rpm, degrees per second, radians per second, counts per second, counts per timestep</td></tr>
        <tr><td>Acceleration</td><td>rotations per second squared, rpm per second, degrees per second squared, radians per second squared, counts per second squared, counts per timestep squared</td></tr>
        <tr><td>Current</td><td>internal current units, milliamps, amps</td></tr>
        <tr><td>Voltage</td><td>millivolts, volts</td></tr>
        <tr><td>Temperature</td><td>celsius, fahrenheit, kelvin</td></tr>
      </tbody>
    </table>'''

    return html

def generate_company_paragraphs():
    """Generate company profile paragraphs for JSX"""
    content = read_file_content('company_profile.txt')
    if not content:
        return '<p>We are an innovative startup committed to making precision motion control accessible to everyone: to makers, educators, and engineers alike.</p>'

    paragraphs = content.split('\n')
    html = ''
    for paragraph in paragraphs:
        if paragraph.strip():
            html += f'<p>{paragraph.strip()}</p>\n'
    return html

def print_optimization_summary(image_metadata_list):
    """Print a summary of image optimization results"""
    print("\n" + "="*60)
    print("IMAGE OPTIMIZATION SUMMARY")
    print("="*60)
    
    total_original = 0
    total_optimized = 0
    
    for metadata in image_metadata_list:
        if metadata:
            image_name = os.path.basename(metadata['src_path'])
            original_size_str = format_file_size(metadata['original_size'])
            optimized_size_str = format_file_size(metadata['optimized_size'])
            
            print(f"\n📸 {image_name}")
            
            # Dimensions
            if metadata['original_dims']:
                orig_w, orig_h = metadata['original_dims']
                print(f"   Original:  {orig_w}x{orig_h}, {original_size_str}")
            else:
                print(f"   Original:  {original_size_str}")
            
            if metadata['optimized_dims']:
                opt_w, opt_h = metadata['optimized_dims']
                print(f"   Optimized: {opt_w}x{opt_h}, {optimized_size_str}")
            else:
                print(f"   Optimized: {optimized_size_str}")
            
            # Compression ratio
            if metadata['compression_ratio'] > 0:
                print(f"   Reduction: {metadata['compression_ratio']:.1f}%")
            
            total_original += metadata['original_size']
            total_optimized += metadata['optimized_size']
    
    # Print totals
    if total_original > 0:
        print("\n" + "-"*60)
        print(f"TOTAL ORIGINAL SIZE:  {format_file_size(total_original)}")
        print(f"TOTAL OPTIMIZED SIZE: {format_file_size(total_optimized)}")
        total_reduction = ((total_original - total_optimized) / total_original) * 100
        print(f"TOTAL REDUCTION:      {total_reduction:.1f}%")
        print(f"SPACE SAVED:          {format_file_size(total_original - total_optimized)}")
    
    print("="*60 + "\n")

def copy_assets_to_nextjs(nextjs_path, skip_assets=False):
    """Copy and optimize assets to Next.js project, returning dimension map"""
    if skip_assets:
        print("Skipping asset copy...")
        return {}

    public_dir = os.path.join(nextjs_path, 'public')
    images_dir = os.path.join(public_dir, 'marketing', 'images')
    logos_dir = os.path.join(public_dir, 'marketing', 'logos')

    ensure_dir(images_dir)
    ensure_dir(logos_dir)

    # Track all image metadata
    all_image_metadata = []
    image_dimensions = {}

    # Copy logos (usually don't need heavy optimization)
    logos_to_copy = [
        ('Gearotons_Logo.png', 'Gearotons_Logo.png')
    ]

    for src_name, dst_name in logos_to_copy:
        if os.path.exists(src_name):
            dst_path = os.path.join(logos_dir, dst_name)
            metadata = optimize_and_copy_image(src_name, dst_path, max_width=800)
            if metadata:
                all_image_metadata.append(metadata)
                if metadata['optimized_dims']:
                    image_dimensions[dst_name] = metadata['optimized_dims']

    # Define images to copy with their optimization strategy
    # Format: (source, destination)
    images_to_copy = [
        ('M17_series_overview.jpg', 'M17_series_overview.jpg'),
        ('connection_diagram.jpg', 'connection_diagram.jpg'),
        ('M17-40_dimensions.png', 'M17-40_dimensions.png'),
        ('M17-48_dimensions.png', 'M17-48_dimensions.png'),
        ('M17-60_dimensions.png', 'M17-60_dimensions.png'),
        ('one_motor.jpg', 'one_motor_small.jpg'),
        ('kit_with_three_motors.jpg', 'kit_with_three_motors_small.jpg'),
        ('motor_back.jpg', 'motor_back_small.jpg'),
        ('adapter_and_wire.jpg', 'adapter_and_wire_small.jpg'),
        ('robotics.jpg', 'robotics_small.jpg'),
        ('automation.jpg', 'automation_small.jpg'),
        ('test_rack.jpg', 'test_rack_small.jpg'),
        ('click_here.png', 'click_here.png'),
        ('Open-source-hardware-logo.svg.png', 'Open_source_hardware_logo_svg.png'),
        ('Open_Source_Initiative.svg.png', 'Open_Source_Initiative_svg.png')
    ]

    for src_name, dst_name in images_to_copy:
        if os.path.exists(src_name):
            dst_path = os.path.join(images_dir, dst_name)
            metadata = optimize_and_copy_image(src_name, dst_path)
            if metadata:
                all_image_metadata.append(metadata)
                if metadata['optimized_dims']:
                    image_dimensions[dst_name] = metadata['optimized_dims']

    # Print optimization summary
    print_optimization_summary(all_image_metadata)
    
    return image_dimensions


def create_unified_preview(output_dir='./preview'):
    """Create unified preview structure compatible with e-commerce"""
    print(f"\n🚀 Creating unified marketing content in {output_dir}...")
    
    ensure_dir(output_dir)
    
    # Create package.json for preview
    package_json = {
        "name": "marketing-preview",
        "version": "1.0.0",
        "private": True,
        "scripts": {
            "dev": "next dev",
            "build": "next build",
            "start": "next start"
        },
        "dependencies": {
            "next": "14.0.0",
            "react": "^18",
            "react-dom": "^18"
        }
    }
    
    with open(os.path.join(output_dir, 'package.json'), 'w') as f:
        json.dump(package_json, f, indent=2)
    
    # Create next.config.js for preview
    next_config = """/** @type {import('next').NextConfig} */
const nextConfig = {
  reactStrictMode: true,
  images: {
    unoptimized: true
  }
}

module.exports = nextConfig
"""
    
    with open(os.path.join(output_dir, 'next.config.js'), 'w') as f:
        f.write(next_config)
    
    # Create jsconfig.json for @ alias support in preview
    jsconfig = """{
  "compilerOptions": {
    "baseUrl": ".",
    "paths": {
      "@/*": ["./*"]
    }
  }
}"""
    
    with open(os.path.join(output_dir, 'jsconfig.json'), 'w') as f:
        f.write(jsconfig)
    
    # Copy and optimize assets, generate files
    image_dimensions = copy_assets_to_nextjs(output_dir, skip_assets=False)
    
    # Generate the unified files
    generate_unified_files(output_dir, image_dimensions)
    
    print(f"\n✅ Unified marketing content created in '{output_dir}'!")
    print("📁 Structure: components/MarketingContent.js + pages/index.js + styles/Marketing.module.css")

def copy_to_ecommerce(preview_dir='./preview', ecommerce_path='../../AI_testing/selling_web_site'):
    """Copy generated files from preview to e-commerce site"""
    print(f"\n📁 Copying files from {preview_dir} to {ecommerce_path}...")
    
    # Define what files/folders to copy
    files_to_copy = [
        ('components/MarketingContent.js', 'components/MarketingContent.js'),
        ('styles/Marketing.module.css', 'styles/Marketing.module.css'),
        ('public/marketing/', 'public/marketing/')
    ]
    
    copied_count = 0
    for src_path, dst_path in files_to_copy:
        src_full = os.path.join(preview_dir, src_path)
        dst_full = os.path.join(ecommerce_path, dst_path)
        
        if os.path.exists(src_full):
            if os.path.isdir(src_full):
                # Copy directory
                if os.path.exists(dst_full):
                    shutil.rmtree(dst_full)
                shutil.copytree(src_full, dst_full)
                print(f"  📂 Copied directory: {src_path}")
            else:
                # Copy file
                ensure_dir(os.path.dirname(dst_full))
                shutil.copy2(src_full, dst_full)
                print(f"  📄 Copied file: {src_path}")
            copied_count += 1
        else:
            print(f"  ⚠️  Source not found: {src_path}")
    
    if copied_count > 0:
        print(f"\n✅ Successfully copied {copied_count} items to e-commerce site!")
        print("🚀 The e-commerce site should now have the updated marketing content.")
    else:
        print("\n❌ No files were copied. Please check the preview generation.")

def main():
    """Main function - generates everything automatically"""
    print("🚀 Generating marketing content...")
    
    # Step 1: Generate preview structure
    preview_dir = './preview'
    ecommerce_path = '../../AI_testing/selling_web_site'
    
    create_unified_preview(preview_dir)
    
    # Step 2: Copy to e-commerce site
    copy_to_ecommerce(preview_dir, ecommerce_path)
    
    print("\n🎉 All done! Marketing content generated and deployed.")
    print(f"📋 To test preview: cd {preview_dir} && npm install && npm run dev")

if __name__ == '__main__':
    main()