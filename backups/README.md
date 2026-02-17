# MATE 2026 Robot Conroller APP

This /app folder stores all code for the distributable app. To package and distribute the app, please follow the file structure, and do not overwrite the core files. After adding your code, make sure to install the dependencies, test the app, build, sign, and then distribute the .dmg file (for arm64 macs only).

## File Structure

```
web/                              - all web-related stuff
  app.py                          - Flask backend (routing, task integration)
  templates/index.html            - Frontend HTML
  static/css/style.css            - Custom dark theme styling
  static/js/app.js                - Frontend JavaScript
scripts/                          - all cpp scripts for each task
  task1_2/                            - i currently only have a prototype for task 1.2
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
cleanup.sh                        - cleanup all macos 
.gitignore                        - gitignore
README.md                         - this file
```

## Core Files

A backup of these files are store in `../backups`. These files need to be added in addition to the dependencies if they are missing.

```
build/*
package.json
main.js
preload.js
fix-opencv.sh
cleanup.sh
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

First, replace `"identity"` in package.json with correct signage from `security find-identity -v -p codesigning`

```bash
make -C scripts/task1_2 clean
make -C scripts/task1_2 all
./cleanup_bundle.sh
npm run build
```


©2026 Doğukan Koç. All Rights Reserved. For private use only, do not distribute.
