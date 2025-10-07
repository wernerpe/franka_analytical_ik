#!/bin/bash
# Script to build the pip-installable wheel

set -e

echo "Building Franka Analytical IK wheel..."

# Build the wheel
bazel build //franka_analytical_ik:franka_ik_wheel

# Find the wheel file
WHEEL_PATH=$(bazel cquery --output=files //src:franka_ik_wheel 2>/dev/null)

if [ -z "$WHEEL_PATH" ]; then
    echo "Error: Could not find wheel file"
    exit 1
fi

# Create dist directory
mkdir -p dist

# Copy wheel to dist
cp "$WHEEL_PATH" dist/

echo "Wheel built successfully!"
echo "Location: dist/$(basename $WHEEL_PATH)"
echo ""
echo "To install, run:"
echo "  pip install dist/$(basename $WHEEL_PATH)"