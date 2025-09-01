#!/usr/bin/env python3

def generate_css():
    """Generate the complete CSS stylesheet for the marketing page"""
    css = '''/* Reset and Base Styles */
* {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}

body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
    line-height: 1.6;
    color: #333;
    background-color: #ffffff;
}

.container {
    max-width: 1200px;
    margin: 0 auto;
    padding: 0 20px;
}

/* Colors - matching the PDF branding */
:root {
    --primary-color: #1b4d3e;
    --secondary-color: #34a853;
    --text-color: #333;
    --light-gray: #f5f5f5;
    --border-color: #ddd;
    --code-bg: #f8f8f8;
}

/* Hero Section */
.hero-section {
    padding: 60px 0;
    background: #ffffff;
    text-align: center;
}

.logo {
    max-width: 120px;
    margin-bottom: 20px;
}

.main-title {
    font-size: 3rem;
    color: var(--primary-color);
    margin-bottom: 20px;
    font-weight: 700;
}

.subtitle {
    font-size: 1.25rem;
    color: var(--secondary-color);
    margin-bottom: 10px;
}

.hero-image {
    max-width: 100%;
    height: auto;
    margin-top: 40px;
}

/* Content Sections */
.content-section {
    padding: 60px 0;
    border-bottom: 1px solid var(--border-color);
}

.content-section:last-of-type {
    border-bottom: none;
}

.content-section h2 {
    font-size: 2rem;
    color: var(--primary-color);
    margin-bottom: 30px;
    text-align: center;
}

.content-section h3 {
    font-size: 1.5rem;
    color: var(--primary-color);
    margin: 30px 0 20px;
}

.content-section p {
    margin-bottom: 15px;
    line-height: 1.8;
}

/* Features List - Simplified */
.features-list-simple {
    list-style-type: disc;
    padding-left: 40px;
    margin: 20px 0;
}

.features-list-simple li {
    margin-bottom: 12px;
    line-height: 1.8;
}

/* Section Images */
.section-image-container {
    text-align: center;
    margin: 30px 0;
}

.section-image {
    max-width: 100%;
    height: auto;
    display: inline-block;
}

/* Unit System Table */
.unit-table {
    width: 100%;
    margin: 20px 0;
}

.unit-table th:first-child {
    width: 30%;
}

/* Dimension Images */
.dimension-images {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
    gap: 20px;
    margin: 30px 20px;
    padding: 0 20px;
}

.dimension-image-container {
    text-align: center;
}

.dimension-image {
    max-width: 100%;
    height: auto;
}

.dimension-label {
    text-align: center;
    font-weight: bold;
    color: var(--primary-color);
    margin-top: 10px;
}

/* Tables */
.specs-table, .commands-table, .conditions-table {
    width: 100%;
    border-collapse: collapse;
    margin: 20px 0;
    background: white;
    box-shadow: 0 2px 10px rgba(0,0,0,0.05);
}

.specs-table th,
.commands-table th,
.conditions-table th {
    background-color: var(--primary-color);
    color: white;
    padding: 12px;
    text-align: left;
    font-weight: 600;
}

.specs-table td,
.commands-table td,
.conditions-table td {
    padding: 12px;
    border-bottom: 1px solid var(--border-color);
}

.specs-table tr:hover,
.commands-table tr:hover,
.conditions-table tr:hover {
    background-color: var(--light-gray);
}

/* Code Blocks */
pre {
    background-color: var(--code-bg);
    border: 1px solid var(--border-color);
    border-radius: 5px;
    padding: 20px;
    overflow-x: auto;
    margin: 20px 0;
}

code {
    font-family: 'Courier New', Courier, monospace;
    font-size: 0.9rem;
}

/* Images - ALL images blend seamlessly like hero image */
.diagram-image {
    max-width: 100%;
    height: auto;
    margin: 30px auto;
    display: block;
}

/* Applications Grid */
.applications-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
    gap: 40px;
    margin-top: 40px;
}

.application-card {
    text-align: center;
}

.application-image {
    width: 100%;
    height: auto;
    margin-bottom: 20px;
    display: block;
}

.application-card h3 {
    color: var(--secondary-color);
    margin-bottom: 15px;
}

/* Company Profile */
.company-content {
    margin-top: 30px;
}

.company-image-container {
    margin: 40px 0;
    text-align: center;
}

.company-image {
    max-width: 100%;
    height: auto;
}

.image-caption {
    margin-top: 15px;
    font-style: italic;
    color: #666;
}

/* Open Source Section */
.open-source-content {
    text-align: center;
    margin-top: 30px;
}

.open-source-logos {
    display: flex;
    justify-content: center;
    gap: 40px;
    margin: 30px 0;
}

.osi-logo, .osh-logo {
    height: 80px;
    width: auto;
}

.github-link {
    margin: 20px 0;
    padding: 15px;
    background: var(--light-gray);
    border-radius: 5px;
    display: flex;
    align-items: center;
    justify-content: center;
}

.github-link a {
    color: var(--secondary-color);
    text-decoration: underline;
}

.btn {
    display: inline-block;
    padding: 12px 30px;
    text-decoration: none;
    border-radius: 5px;
    transition: all 0.3s;
    font-weight: 600;
}

.btn-primary {
    background-color: var(--secondary-color);
    color: white;
}

.btn-primary:hover {
    background-color: var(--primary-color);
    transform: translateY(-2px);
    box-shadow: 0 5px 15px rgba(0,0,0,0.2);
}

/* Indicators List */
.indicators-list {
    margin: 20px 0 20px 40px;
    list-style-type: disc;
}

.indicators-list li {
    margin-bottom: 10px;
}

/* Guide Features List */
.guide-features {
    list-style: none;
    padding: 0;
    margin: 20px 0;
}

.guide-features li {
    padding: 10px 10px 10px 40px;
    position: relative;
    margin-bottom: 8px;
}

.guide-features li:before {
    content: '✓';
    position: absolute;
    left: 15px;
    color: var(--secondary-color);
    font-weight: bold;
}

/* Guide Link */
.guide-link {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 15px;
    margin: 30px 0;
}

.click-icon {
    width: 40px;
    height: 40px;
}

.click-icon-small {
    width: 25px;
    height: 25px;
    vertical-align: middle;
    margin-right: 10px;
}

/* Command Link */
.command-link {
    display: flex;
    align-items: center;
    margin: 20px 0;
    padding: 15px;
    background: var(--light-gray);
    border-radius: 5px;
}

.command-link a {
    color: var(--secondary-color);
    text-decoration: underline;
    word-break: break-all;
}

/* Command Tables */
h4 {
    color: var(--primary-color);
    margin: 20px 0 10px;
    font-size: 1.2rem;
}

/* Library Sections */
.library-section {
    margin: 40px 0;
}

.library-section h3 {
    margin-top: 0;
    margin-bottom: 15px;
}

.library-section > p {
    margin-bottom: 20px;
}

/* Footer */
.footer {
    background-color: var(--primary-color);
    color: white;
    padding: 40px 0;
    text-align: center;
    margin-top: 60px;
}

.footer-logo {
    max-width: 200px;
    margin-bottom: 20px;
    /* Removed filter since we're using transparent logo */
}

.version-info {
    font-size: 0.9rem;
    margin: 10px 0;
    opacity: 0.8;
}

.copyright {
    margin: 20px 0 10px;
    font-size: 0.9rem;
}

/* Responsive Design */
@media (max-width: 768px) {
    .main-title {
        font-size: 2rem;
    }
    
    .features-grid,
    .applications-grid {
        grid-template-columns: 1fr;
    }
    
    .open-source-logos {
        flex-direction: column;
        align-items: center;
    }
    
    .specs-table {
        font-size: 0.9rem;
    }
    
    .specs-table th,
    .specs-table td {
        padding: 8px;
    }
}

/* Print Styles */
@media print {
    .content-section {
        page-break-inside: avoid;
    }
    
    .hero-section {
        background: none;
    }
}

/* Smooth Scrolling */
html {
    scroll-behavior: smooth;
}

/* Accessibility */
:focus {
    outline: 2px solid var(--secondary-color);
    outline-offset: 2px;
}

.visually-hidden {
    position: absolute;
    width: 1px;
    height: 1px;
    padding: 0;
    margin: -1px;
    overflow: hidden;
    clip: rect(0, 0, 0, 0);
    white-space: nowrap;
    border: 0;
}
'''
    return css