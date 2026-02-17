#!/bin/bash
echo "Cleaning up bundle resources..."
find app/Resources -type l -delete
find app/Resources -name "*.a" -delete
find app/Resources -name ".DS_Store" -delete
find app/Resources -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
find app/Resources -name "*.pyc" -delete
find app/Resources -name "*.pyo" -delete

# Strip ALL extended attributes (Google Drive / macOS metadata)
xattr -cr app/Resources
xattr -cr scripts
xattr -cr web

echo "✓ Cleanup complete"