# PyPI Deployment Guide

## Overview

The servomotor package deployment system has been enhanced to support both TestPyPI and production PyPI deployments with interactive selection and safety features.

## Key Features

### 1. Interactive Deployment Target Selection
The build script now prompts you to choose between:
- **TestPyPI** (test.pypi.org) - For testing packages before production
- **PyPI** (pypi.org) - Production deployment

### 2. Single Source of Truth Versioning
- Version is defined in `servomotor/__init__.py` as `__version__`
- `pyproject.toml` uses dynamic versioning to read from the source
- No need to update version in multiple places

### 3. Production Safety Features
- Confirmation dialog for production deployments
- Dynamic version display in warning messages
- Clear distinction between test and production environments

## Usage

### Basic Deployment

Run the deployment script:
```bash
./build_and_submit_to_PyPi.sh
```

The script will:
1. Build the package in an isolated virtual environment
2. Prompt you to select deployment target
3. Show appropriate warnings for production deployments
4. Upload to the selected repository

### Version Management

To increment the version:
1. Edit `servomotor/__init__.py`
2. Update the `__version__` variable (e.g., from "0.9.1" to "0.9.2")
3. Run the deployment script

Example:
```python
# In servomotor/__init__.py
__version__ = "0.9.2"
```

### TestPyPI Deployment
- Select option 1 when prompted
- Useful for testing package installation and functionality
- Available at: https://test.pypi.org/project/servomotor/

### Production PyPI Deployment
- Select option 2 when prompted
- Requires explicit confirmation with "yes"
- Shows current version in warning message
- Available at: https://pypi.org/project/servomotor/

## Installation Testing

### From TestPyPI
```bash
pip install --index-url https://test.pypi.org/simple/ servomotor==X.Y.Z
```

### From PyPI
```bash
pip install servomotor==X.Y.Z
```

### Verify Installation
```python
import servomotor
print(f"Version: {servomotor.__version__}")
from servomotor import M3
```

## Prerequisites

- PyPI credentials configured in `~/.pypirc`
- Valid API tokens for both TestPyPI and PyPI repositories
- Python 3.8+ environment

## Package Structure

The deployment includes:
- Core `servomotor` package with M3 motor control
- CLI tools: `servomotor_command`, `detect_and_set_alias_all_devices`, `show_device_information_for_all_devices`
- Terminal formatting utilities
- Vendored serial communication libraries

## Command Line Tools

After installation, the following commands are available:
- `servomotor_command` - Main command interface
- `detect_and_set_alias_all_devices` - Device detection and alias assignment
- `show_device_information_for_all_devices` - Display device information

## Troubleshooting

### Common Issues

1. **Version not found on TestPyPI**: Wait a few minutes for propagation
2. **Permission denied**: Check API token permissions
3. **Package already exists**: Increment version number

### Build Warnings

The build process may show setuptools deprecation warnings about license configuration. These are informational and don't affect functionality.

## Future Improvements

Consider updating `pyproject.toml` to use SPDX license expressions to resolve deprecation warnings:
```toml
license = "MIT"
```
Instead of:
```toml
license = { file = "LICENCE" }