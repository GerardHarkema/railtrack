#!/bin/bash

# Script to install Python packages for RailTrack project
# Installs packages to user directory without virtual environment
# Uses --break-system-packages flag to bypass PEP 668 protection

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"

echo "================================================"
echo "RailTrack Python Packages Installation"
echo "================================================"
echo ""

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed"
    echo "Please install Python 3.12 or later"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "Found Python version: $PYTHON_VERSION"
echo ""

# Check if pip is installed
if ! command -v pip &> /dev/null; then
    echo "Error: pip is not installed"
    echo "Please install pip first"
    exit 1
fi

echo "Installing packages to user directory (~/.local/)"
echo "This bypasses the externally-managed-environment protection"
echo ""

# Install requirements
if [ -f "$REQUIREMENTS_FILE" ]; then
    echo "Installing packages from requirements.txt..."
    echo ""
    pip install --user --break-system-packages -r "$REQUIREMENTS_FILE"
else
    echo "Warning: requirements.txt not found at $REQUIREMENTS_FILE"
    echo "Installing basic packages manually..."
    echo ""
    pip install --user --break-system-packages nicegui easygui setuptools==58.2.0
fi

echo ""
echo "================================================"
echo "Installation Complete!"
echo "================================================"
echo ""
echo "Packages installed to: ~/.local/lib/python3.*/site-packages"
echo ""
echo "The packages are now available system-wide for your user account"
echo ""
