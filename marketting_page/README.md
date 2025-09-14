# Servomotor Marketing Web Page Generator

## Quick Overview
Generates a Next.js marketing page for M17 Series Servomotors with automatic image optimization and dimension detection.

## ✨ Features (v2.1 - Image Aspect Ratio Fixed!)
- **Next.js Integration**: Generate React components and CSS Modules directly compatible with Next.js
- **Dynamic Image Dimensions**: Automatic detection and injection of actual image dimensions into Next.js Image components
- **Smart Image Optimization**: Different optimization strategies based on image type (diagrams, photos, logos)
- **File Size Reporting**: Comprehensive summary showing optimization results and space savings (~83% average reduction)
- **Standalone Preview**: Create a minimal Next.js app for local testing without needing the full e-commerce site
- **Template-Based Generation**: Safe token replacement system avoiding JSX escaping issues
- **Asset Pipeline**: Automatic image optimization and copying to Next.js public directories (preserves originals)

## Main Output
- **Output**: Next.js JSX components + CSS Modules
- **Layout**: Single continuous scrolling page (no pagination)
- **Sections**: Introduction, Features, Connection Diagram, Specifications, Company Profile, Applications

## 🚀 Quick Start

### Generate for Existing Next.js Site
```bash
python3 generate_webpage.py
```
This generates optimized Next.js components and copies images to your e-commerce site.

### Create Standalone Preview
```bash
python3 generate_webpage.py --preview
cd preview
npm install
npm run dev
```
Then open http://localhost:3000 to see your marketing page locally.

## 📁 File Structure
```
generate_webpage.py           # Main generator script
generate_webpage.sh           # Shell wrapper (if needed)
templates/
  ├── next_index_template.jsx     # Next.js JSX template
  └── marketing_module_template.css # CSS Module template
preview/                      # Standalone preview app (when generated)
  ├── package.json
  ├── pages/index.js
  ├── styles/Marketing.module.css
  └── public/marketing/       # Optimized images
```

## 🎯 Generation Options

```bash
python3 generate_webpage.py [OPTIONS]

Options:
  --preview                Create standalone preview app
  --preview-dir PATH       Directory for preview (default: ./preview)
  --nextjs-path PATH       Path to Next.js project (default: ../../AI_testing/selling_web_site)
  --overwrite-index        Overwrite pages/index.js instead of creating component
  --skip-assets            Skip copying assets
  --skip-css               Skip generating CSS Module
```

## 📋 Content Sources

The generator pulls content from these source files:

- `introduction.txt` → Introduction section paragraphs
- `features.txt` → Key features list items
- `company_profile.txt` → Company profile paragraphs
- `versions.txt` → Version and date information
- Images in current directory → Optimized and copied to Next.js
- `../python_programs/servomotor/unit_conversions_M3.json` → Unit system table

## 🛠️ Integration with Next.js

### Option A: Use as Component (Recommended)
```javascript
// pages/index.js
import MarketingContent from '@/components/MarketingContent';

export default function Home() {
  return (
    <main>
      <MarketingContent />
      {/* Your existing content */}
    </main>
  );
}
```

### Option B: Direct Homepage Override
```bash
python3 generate_webpage.py --nextjs-path ../your-site --overwrite-index
```

## 📚 Documentation

- `NEXTJS_INTEGRATION_USAGE_GUIDE.md` - Complete usage guide with examples
- `NEXTJS_INTEGRATION_TESTING_CHECKLIST.md` - Comprehensive testing checklist
- `NEXTJS_INTEGRATION_LEARNINGS_AND_FAILURE_ANALYSIS.md` - Solution to aspect ratio issues
- `AUTOGENERATOR_NEXTJS_INTEGRATION_RETROSPECTIVE.md` - Technical background

## 🔄 Template System

The Next.js generation uses a safe token replacement system:

- `__INTRO_PARAGRAPHS__` → Introduction content
- `__FEATURES_LI__` → Features list
- `__UNIT_TABLE__` → Unit conversion table
- `__COMPANY_PARAGRAPHS__` → Company profile
- `__VERSION__` and `__DATE__` → Version info
- `__WIDTH_imagename__` and `__HEIGHT_imagename__` → Dynamic image dimensions

## ⚡ Performance Features

- **Smart Image Optimization**: Intelligent sizing based on content type:
  - Technical diagrams: 1200px (preserves readability and aspect ratio)
  - Photos: 800px (standard web optimization)
  - Logos: 400px (balanced size)
  - UI elements: No resize if under 200px
- **Dynamic Dimension Detection**: Automatically detects and uses actual image dimensions
- **CSS Modules**: Scoped styling to prevent conflicts (fixed camelCase naming)
- **Lazy Loading**: Next.js Image component optimization
- **Fallback Handling**: Graceful degradation when PIL is not available
- **Optimization Summary**: Detailed report showing file sizes before/after and space saved

## 🐛 Troubleshooting

### Common Issues
- **PIL not installed**: Images still copy but aren't optimized. Install with: `pip install Pillow`
- **Missing source files**: Fallback content is used automatically
- **Path issues**: Use absolute paths for custom Next.js locations

### Testing Without E-commerce Site
Use the `--preview` flag to create a standalone app for testing:
```bash
python3 generate_webpage.py --preview
cd preview
npm install
npm run dev
```

## 📈 Technical Approach
- **Template-based generation** avoids JSX string concatenation issues
- **Token replacement system** ensures safe content injection
- **Dynamic dimension detection** fixes aspect ratio problems
- **Error handling** with graceful fallbacks for missing dependencies
- **Asset optimization** pipeline for web performance

---

**Ready to generate your marketing content?** Just run `python3 generate_webpage.py --preview` to test locally! 🎉
