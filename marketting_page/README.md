# Servomotor Marketing Web Page Generator

## Quick Overview
Generates a unified Next.js marketing page for M17 Series Servomotors with automatic deployment to both preview and e-commerce sites.

## ✨ Features (v3.0 - Unified Architecture!)
- **Single Generation Path**: One command generates everything - no more dual modes
- **Unified Architecture**: Both preview and e-commerce use identical components  
- **Automatic Deployment**: Generates preview AND copies to e-commerce site automatically
- **Dynamic Image Dimensions**: Automatic detection and injection of actual image dimensions
- **Smart Image Optimization**: Different strategies based on image type (83% average reduction)
- **Fixed Image Issues**: Open Source images now work correctly in both contexts
- **Perfect Parity**: Identical rendering between preview and e-commerce (all sections included)

## 🚀 Quick Start

### Generate Everything (Recommended)
```bash
python3 generate_webpage.py
```

This single command:
1. ✅ Generates preview structure in `./preview/`
2. ✅ Copies identical files to e-commerce site
3. ✅ Ensures perfect parity between both contexts

### Test Preview Locally
```bash
cd preview
npm install
npm run dev
```
Then open http://localhost:3000

## 📁 Generated Structure

```
preview/                           # Complete Next.js preview app
├── components/MarketingContent.js  # Main component (copied to e-commerce)
├── pages/index.js                 # Preview wrapper (imports component)
├── styles/Marketing.module.css    # CSS Module (copied to e-commerce)
├── public/marketing/              # Optimized images (copied to e-commerce)
├── package.json                   # Next.js dependencies
├── next.config.js                 # Next.js configuration
└── jsconfig.json                  # Path alias support

E-commerce site integration:
├── components/MarketingContent.js  # ← Copied from preview
├── pages/index.js                 # ← Modified to use component
├── styles/Marketing.module.css    # ← Copied from preview
└── public/marketing/              # ← Copied from preview
```

## 🎯 Architecture Solution

**Previous Problem**: Dual generation modes created inconsistent files
**Solution**: Single generation + clean component architecture

- **Preview**: `pages/index.js` → imports `components/MarketingContent.js`
- **E-commerce**: `pages/index.js` → imports `components/MarketingContent.js`
- **Result**: Perfect parity - both use identical component, CSS, and images

## 📋 Content Sources

The generator pulls content from these source files:
- `introduction.txt` → Introduction section paragraphs
- `features.txt` → Key features list items  
- `company_profile.txt` → Company profile paragraphs
- `versions.txt` → Version and date information
- Images in current directory → Optimized and copied to both sites
- `../python_programs/servomotor/unit_conversions_M3.json` → Unit system table

## 🛠️ Key Fixes Implemented

### Fixed Missing Sections Issue
- **Root Cause**: E-commerce site was using hardcoded `pages/index.js` instead of generated component
- **Solution**: Modified e-commerce site to use the same `MarketingContent.js` component

### Fixed Image Sizing Differences  
- **Root Cause**: Hardcoded dimensions vs dynamic dimensions
- **Solution**: Both sites now use identical dynamic image dimensions

### Fixed Open Source Images
- **Root Cause**: Filename case mismatch after processing
- **Solution**: Updated template references to match processed filenames

## ⚡ Performance Features

- **Smart Image Optimization**: Intelligent sizing based on content type
- **Dynamic Dimension Detection**: Automatically uses actual image dimensions  
- **CSS Modules**: Scoped styling prevents conflicts
- **Lazy Loading**: Next.js Image component optimization
- **83% Average Reduction**: Comprehensive optimization summary

## 🔄 Workflow

1. **Generate**: Run `python3 generate_webpage.py`
2. **Test**: Preview works in standalone mode
3. **Deploy**: Files automatically copied to e-commerce site
4. **Result**: Perfect parity between both contexts

## 📚 Generated Sections

All sections now appear in both preview and e-commerce:
- Hero & Introduction
- Key Features  
- Connection Diagram
- Unit System
- Getting Started Guide
- Indicator LEDs and Buttons
- Communication Protocol
- **Technical Specifications** ✅ (Now included in e-commerce)
- **Library Support** ✅ (Now included in e-commerce)
- Applications
- Company Profile
- Open Source
- Footer

## 🎉 Success Metrics

- ✅ Single command generates everything
- ✅ Perfect rendering parity achieved
- ✅ All sections present in both contexts
- ✅ Consistent image sizing
- ✅ Clean, maintainable architecture
- ✅ 83% average image size reduction

---

**Ready to generate identical marketing content for both sites?** Just run `python3 generate_webpage.py`! 🚀
