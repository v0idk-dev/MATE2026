# MATE 2026 Robot Conroller APP

This /app folder stores all code for the distributable app. To package and distribute the app, please follow the file structure, and do not overwrite the core files. After adding your code, make sure to install the dependencies, test the app, build, sign, and then distribute the .dmg file (for arm64 macs only).

## File Structure

```
------------------------------------------  APP  FILES  -----------------------------------------
web/                              - all web-related stuff
  app.py                          - Flask backend (routing, task integration)
  templates/index.html            - Frontend HTML
  static/css/style.css            - Custom dark theme styling
  static/js/app.js                - Frontend JavaScript
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
------------------------------------------  CORE FILES  -----------------------------------------
app/                              - all app-related things
  Resources/*                     - the python runtime, cpp dependencies, etc.
main.js                           - Electron app main proccess
preload.js                        - bridge btwn Electron & web content
package.json                      - app information
package-lock.json                 - node.js depedencies list
node_modules/*                    - all node.js dependencies
build/                            - finished app resources
  icon.icns                       - app icon
entitlements.mac.plist            - macos app entitlements
fix-opencv.sh                     - opencv fix script
build.sh                          - build application
.gitignore                        - gitignore
README.md                         - this file
-------------------------------------------------------------------------------------------------
```

## Core Files

A backup of these files are store in [/backups](/backups). These files need to be added in addition to the dependencies if they are missing.

```
build/*
package.json
main.js
preload.js
fix-opencv.sh
build.sh
entitlements.mac.plist
.gitignore
README.md (this file)
```

## Installing Dependencies

If Homebrew isn't installed, please install that first at [brew.sh](https://brew.sh).
Firstly, `cd /app`.

### Python

```bash
python3 -m venv --copies app/Resources/python-runtime
app/Resources/python-runtime/bin/pip3 install opencv-python blinker click colorama flask gunicorn itsdangerous jinja2 markupsafe packaging werkzeug flask numpy
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
OPENCV_CFLAGS = -I../../app/Resources/opencv-include
OPENCV_LIBS = -L../../app/Resources/opencv-libs -lopencv_core -lopencv_imgproc -lopencv_calib3d

all: stereo_distance
	@bash ../../fix-opencv.sh

stereo_distance: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
```

### Electron App

```bash
npm init -y
npm install electron electron-builder --save-dev
npm install express ws
```

Add to `/web/static/css/style.css`:

```css
body {
    -webkit-app-region: drag;
    -webkit-user-select: none;
}

button, input, select, textarea, a, .upload-card, img, [onclick], [href] {
    -webkit-app-region: no-drag;
}

.app-container {
    padding-top: 50px;
}
```

## Run Commands

```bash
# FOR DEVELOPMENT SERVER
make -C scripts/task1_2 all && app/Resources/python-runtime/bin/python3 web/app.py

# FOR DEVELOPMENT ELECTRON APP
npm start
```

## Build Commands

First, find your correct signing identity from `security find-identity -v -p codesigning`. You will need an apple developer account for this.
Next, make sure ***NOTHING*** shows up when you run this command: `find app/Resources -type l`. If something does, you did something wrong, and the build will fail.
Also, **WARNING**: do *not* build inside of a storage provider folder, such as Google Drive or iCloud. Build completely on device, or the build will fail, as storage providers add extra metadata not compatible with the build process.

```bash
make -C scripts/task1_2 clean
make -C scripts/task1_2 all
./cleanup.sh

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
