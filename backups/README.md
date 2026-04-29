# MATE 2026 Robot Controller APP

This /app folder stores all code for the distributable app. To package and distribute the app, please follow the file structure, and do not overwrite the core files. After adding your code, make sure to install the dependencies, test the app, build, sign, and then distribute the .dmg file (for arm64 macs only).

## File Structure

```
------------------------------------------  APP  FILES  -----------------------------------------
web/                              - all web-related stuff
  app.py                          - Flask/Werkzeug backend (routing, camera, task integration)
  templates/                      - Frontend HTML
    index.html                    - Main app file
    x_y.html                      - Task x.y helper html
  static/css/main.css             - app styling
  static/js/                      - all javascript files here
     main.js                      - Frontend JavaScript
     1_2.js                       - js specifically for task 1.2
scripts/                          - all cpp scripts for each task
  task1_2/                        - i currently only have a prototype for task 1.2
    include/                      - C++ headers
      purple_detector.hpp         - Purple plate detection via HSV
      stereo_matcher.hpp          - Feature matching & stereo geometry
      distance_calculator.hpp     - Multiple distance estimation methods
      calibration.hpp             - Camera calibration utilities
    src/                          - C++ implementation
      purple_detector.cpp         - HSV segmentation, morphology, contours
      stereo_matcher.cpp          - AKAZE/RANSAC, triangulation
      distance_calculator.cpp     - Triangulation, disparity, known-size
      calibration.cpp             - Chessboard calibration
      main.cpp                    - CLI entry point
    Makefile                      - C++ build for task 1.2
outputs/                          - All task outputs (generated at runtime) (organized per task)
  task1_2/                        - Task 1.2 outputs
    {mm}.{dd}.{yyyy}_{hh}.{mm}    - specific output organized by date&time
uploads/                          - All task uploads (organized per task)
  task1_2/                        - Task 1.2 uploads
    {mm}.{dd}.{yyyy}_{hh}.{mm}    - specific upload organized by date&time
models/                           - AI models for task 2.1 are stored here
------------------------------------------  CORE FILES  -----------------------------------------
native/                           - native macOS addons
  settings/                       - SwiftUI settings window addon
    SettingsUI.swift              - SwiftUI views (all 4 panes) + window manager
    settings.mm                   - ObjC++ NAPI addon (dlopen → libSettingsUI.dylib)
    binding.gyp                   - node-gyp build config
    package.json                  - addon package (node-addon-api dep)
    build.sh                      - builds dylib + .node, fixes rpaths
    build/Release/                - compiled artifacts
cameras.json                      - camera channel defaults (loaded by Flask + General pane)
tasks.json                        - task names and descriptions
main.js                           - Electron app main process
preload.js                        - bridge between Electron & web content
package.json                      - app information
build/                            - finished app resources
  icon.icns                       - app icon
  SpaceMono-Bold.ttf              - bold app font face
  SpaceMono-Regular.ttf           - regular app font face
entitlements.mac.plist            - macOS app entitlements
dmg-bg.sh                         - DMG background creator
fix-dmg-bg.sh                     - applies background to DMG
fix-opencv.sh                     - opencv fix script
build.sh                          - build application
.gitignore                        - gitignore
README.md                         - this file
---------------------------------------  INSTALLED  FILES  --------------------------------------
node_modules/*                    - all node.js dependencies
native/settings/node_modules/*    - native addon node.js dependencies
native/settings/build/release/*   - built addon files
build/background.png              - DMG background image
app/Resources/*                   - opencv & python runtime
scripts/task1_2/build/*           - compiled binaries for task 1.2
scripts/task1_2/stereo_distance   - compiled stereo distance executable
-------------------------------------------------------------------------------------------------
```

## Core Files

A backup of these files are stored in [/backups](/backups). These files need to be added in addition to the dependencies if they are missing.

```
build/*
native/*
build.sh
cameras.json
dmg-bg.sh
entitlements.mac.plist
fix-dmg-bg.sh
fix-opencv.sh
main.js
package.json
preload.js
README.md (this file)
tasks.json
```

## Installing Dependencies

If Homebrew isn't installed, please install that first at [brew.sh](https://brew.sh).

### Python

```bash
# Download and extract a standalone Python 3.13 (arm64, macOS)
curl -L "https://github.com/astral-sh/python-build-standalone/releases/download/20250702/cpython-3.13.5+20250702-aarch64-apple-darwin-install_only_stripped.tar.gz" -o /tmp/python-standalone.tar.gz
tar -xzf /tmp/python-standalone.tar.gz -C /tmp
cp -R /tmp/python app/Resources/python-runtime
cp app/Resources/python-runtime/bin/python3.13 app/Resources/python-runtime/bin/python3
rm -rf /tmp/python /tmp/python-standalone.tar.gz

# Install required packages into the standalone runtime
app/Resources/python-runtime/bin/python3 -m pip install opencv-python blinker click colorama flask itsdangerous jinja2 markupsafe packaging werkzeug numpy ultralytics pillow dmgbuild
```

### C++

```bash
brew install opencv
mkdir -p app/Resources/opencv-libs
cp -L /opt/homebrew/opt/opencv/lib/*.dylib app/Resources/opencv-libs/
```

### Makefile

Add to `/scripts/task1_2/Makefile`:

```makefile
all: $(TARGET)
	@bash ../../fix-opencv.sh
```

### Electron App

**Must use exactly Electron 40.6.1.** Do not upgrade or downgrade — this version is required for compatibility.

```bash
npm install electron@40.6.1 electron-builder --save-dev
```

### Settings Addon

**Requires** Xcode 26.x to be installed (for the macOS 26 SDK)

```bash
bash native/settings/build.sh
```

The addon must be rebuilt any time the Electron version changes.

## Run Commands

```bash
# FOR DEVELOPMENT SERVER
make -C scripts/task1_2 all && app/Resources/python-runtime/bin/python3 web/app.py

# FOR DEVELOPMENT ELECTRON APP
npm start
```

## Build Commands

First, find your correct signing identity from `security find-identity -v -p codesigning`. You will need an apple developer account for this.

Make sure the following are true before building, or the build will fail:

- ***NOTHING*** shows up when you run `find app/Resources -type l` (no symlinks)
- `native/settings/build/Release/settings.node` and `libSettingsUI.dylib` exist (run the Settings Addon install step if not)
- You are **not** building inside a storage provider folder (iCloud, Google Drive, etc.) — storage providers add metadata that breaks the build

**WARNING**: Do *not* build inside of a storage provider folder, such as Google Drive or iCloud.

```bash
# Build with signing (for distribution)
./build.sh "SIGNING-IDENTITY"

# Build without signing (for testing)
./build.sh --test

# Just create the app folder (no DMG)
./build.sh --pack
```

## Notarization Commands

To notarize, you need to use a **Developer ID Application** certificate in the building phase. You will need a **paid** apple developer account for this.
First, if you have never notarized an application before, run these commands first. This will ask for an app-specific password.

```bash
xcrun notarytool store-credentials "notarization" \
  --apple-id APPLEID-EMAIL \
  --team-id TEAMID
```

To check whether or not apple will accept your notarized app, run these commands. They will save a lot of time if the app does not qualify, as notarization takes a while.

```bash
# Checks for signing -- look for "satisfies its Designated Requirement"
codesign -vvv --deep --strict dist/mac-arm64/MATE\ 2026\ Robot\ Controller.app

# Checks what certificate was used for signing -- look for "origin=Developer ID Application..."
spctl -vvv --assess --type exec dist/mac-arm64/MATE\ 2026\ Robot\ Controller.app
```

Next, enter the following commands.

```bash
# This notarizes the DMG
xcrun notarytool submit dist/MATE\ 2026\ Robot\ Controller-VERSION-arm64.dmg \
  --keychain-profile "notarization" \
  --wait

# This "staples" the notarization onto the DMG
xcrun stapler staple dist/MATE\ 2026\ Robot\ Controller-VERSION-arm64.dmg
```

©2026 Doğukan Koç. All Rights Reserved. For private use only, do not distribute.
