#!/bin/bash
# Fix OpenCV paths in the C++ binary to use bundled libraries

BINARY="scripts/task1_2/stereo_distance"

echo "Fixing OpenCV library paths in $BINARY..."

# Fix each OpenCV library path
for lib in /opt/homebrew/opt/opencv/lib/*.dylib; do
    libname=$(basename "$lib")
    install_name_tool -change "$lib" "@executable_path/../Resources/opencv-libs/$libname" "$BINARY" 2>/dev/null
done

echo "✓ Paths fixed"