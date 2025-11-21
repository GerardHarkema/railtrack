#!/bin/bash

# Script to set up Python virtual environment for RailTrack project
# This creates a virtual environment and installs required packages

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VENV_PATH="$PROJECT_ROOT/.venv"

echo "================================================"
echo "RailTrack Virtual Environment Setup"
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

# Check if virtual environment already exists
if [ -d "$VENV_PATH" ]; then
    echo "Virtual environment already exists at: $VENV_PATH"
    read -p "Do you want to remove and recreate it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing virtual environment..."
        rm -rf "$VENV_PATH"
    else
        echo "Keeping existing virtual environment"
        echo "To activate it manually, run:"
        echo "  source $VENV_PATH/bin/activate"
        exit 0
    fi
fi

# Create virtual environment
echo "Creating virtual environment at: $VENV_PATH"
python3 -m venv "$VENV_PATH"

if [ ! -d "$VENV_PATH" ]; then
    echo "Error: Failed to create virtual environment"
    exit 1
fi

echo "Virtual environment created successfully"
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source "$VENV_PATH/bin/activate"

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"
if [ -f "$REQUIREMENTS_FILE" ]; then
    echo ""
    echo "Installing packages from requirements.txt..."
    pip install -r "$REQUIREMENTS_FILE"
else
    echo ""
    echo "Warning: requirements.txt not found at $REQUIREMENTS_FILE"
    echo "Installing basic packages manually..."
    pip install nicegui easygui setuptools==58.2.0
fi

echo ""
echo "================================================"
echo "Virtual Environment Setup Complete!"
echo "================================================"
echo ""
echo "Virtual environment location: $VENV_PATH"
echo ""
echo "To activate the virtual environment, run:"
echo "  source $VENV_PATH/bin/activate"
echo ""
echo "To deactivate, run:"
echo "  deactivate"
echo ""
