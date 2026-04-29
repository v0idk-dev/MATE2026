#!/bin/bash
set -e

# Check for help flag
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "Usage: ./build.sh [OPTIONS] [SIGNING_IDENTITY]"
    echo ""
    echo "Build and sign the MATE Robot Controller application."
    echo ""
    echo "OPTIONS:"
    echo "  -t, --test              Build without code signing (for testing)"
    echo "  -p, --pack              Create app bundle without DMG"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "ARGUMENTS:"
    echo "  SIGNING_IDENTITY        Apple Developer signing identity (required for signed builds)"
    echo "                          Example: \"Developer ID Application: Your Name (TEAMID)\""
    echo ""
    echo "EXAMPLES:"
    echo "  ./build.sh \"Apple Development: John Doe (ABC4EF7H9J)\""
    echo "  ./build.sh --test"
    echo "  ./build.sh --pack"
    exit 0
fi

# Check arguments
if [ -z "$1" ]; then
    printf "\e[31mError: No arguments provided\e[0m\n"
    echo "Run './build.sh --help' for usage information"
    exit 1
fi

# Always run cleanup and build C++
printf "\n\e[97;4;1mBuilding C++ binaries\e[0m\n\n"
make -C scripts/task1_2 clean
make -C scripts/task1_2 dist

printf "\n\e[97;4;1mBuilding native settings addon\e[0m\n\n"
(bash native/settings/build.sh)

printf "\n\e[97;4;1mCleaning bundle resources\e[0m\n\n"

# Clean bundle resources
find app/Resources -type l -delete 2>/dev/null || true
find app/Resources -name "*.a" -delete 2>/dev/null || true
find app/Resources -name ".DS_Store" -delete 2>/dev/null || true
find app/Resources -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find app/Resources -name "*.pyc" -delete 2>/dev/null || true
find app/Resources -name "*.pyo" -delete 2>/dev/null || true

# Strip ALL extended attributes (Google Drive / macOS metadata)
xattr -cr app/Resources 2>/dev/null || true
xattr -cr scripts 2>/dev/null || true
xattr -cr web 2>/dev/null || true

echo "✓ Cleanup complete"

printf "\n\e[97;4;1mCreating DMG background\e[0m\n\n"
# Create DMG background (must be done before signing to avoid extended attribute issues)
bash dmg-bg.sh "$(node -p "require('./package.json').version")" build/background.png
echo "✓ DMG background created"

# Handle different build modes
if [ "$1" = "--test" ] || [ "$1" = "-t" ]; then
    printf "\n\e[97;4;1mBuilding unsigned app\e[0m\n"
    npm run build:dev
    bash fix-dmg-bg.sh
    printf "\n\e[42;30;1m Test build complete \e[0m\n\n"
    
elif [ "$1" = "--pack" ] || [ "$1" = "-p" ]; then
    printf "\n\e[97;4;1mCreating app bundle\e[0m\n"
    npm run pack
    printf "\n\e[42;30;1m Pack complete \e[0m\n\n"
    
else
    # Signed build - requires identity
    SIGNING_IDENTITY="$1"
    
    # Validate identity format (basic check)
    if [[ ! "$SIGNING_IDENTITY" =~ ^(Apple\ (Development|Distribution)|Developer\ ID\ Application):\ .+\ \([A-Z0-9]+\)$ ]]; then
        printf "\e[31mError: Invalid signing identity format\e[0m\n"
        echo "Expected format: \"Developer ID Application: Name (TEAMID)\""
        echo "Run './build.sh --help' for usage information"
        exit 1
    fi
    
    printf "\n\e[97;4;1mSigning nested binaries\e[0m\n\n"
    echo "Using identity: $SIGNING_IDENTITY"
    
    find app/Resources/python-runtime -type f -perm +111 -exec codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime {} \; 2>/dev/null || true
    find app/Resources/opencv-libs -name "*.dylib" -exec codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime {} \; 2>/dev/null || true
    find scripts -type f -perm +111 ! -name "*.sh" -exec codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime {} \; 2>/dev/null || true

    find app/Resources -type f \( -name "*.dylib" -o -name "*.so" -o -perm +111 \) -exec codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime {} \; 2>/dev/null || true
    find scripts -type f -perm +111 ! -name "*.sh" -exec codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime {} \; 2>/dev/null || true

    codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime native/settings/build/Release/libSettingsUI.dylib
    codesign --force --sign "$SIGNING_IDENTITY" --timestamp --options runtime native/settings/build/Release/settings.node
    
    printf "\n\e[97;4;1mBuilding and signing app\e[0m\n"
    CSC_NAME="${SIGNING_IDENTITY#Developer ID Application: }" npm run build
    bash fix-dmg-bg.sh

    printf "\n\e[42;30;1m Signed build complete \e[0m\n\n"
fi