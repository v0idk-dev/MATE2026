# MATE 2026 Robot Controller APP

This /app folder stores all code for the distributable app. To package and distribute the app, please follow the file structure, and do not overwrite the core files. After adding your code, make sure to install the dependencies, test the app, build, sign, and then distribute the .dmg file (for arm64 macs only).

## File Structure

```
app/
├── main.js                           Electron main process — window, IPC, Flask lifecycle, menus
├── preload.js                        Electron preload — exposes electronAPI bridge to renderer
├── package.json                      Electron app manifest + build config
├── package-lock.json                 npm lockfile
├── cameras.json                      Camera channel defaults (loaded by Flask + Settings pane)
├── tasks.json                        Task names and route definitions
├── entitlements.mac.plist            macOS hardened runtime entitlements for code signing
├── build.sh                          Packages + signs the app and builds the DMG
├── dmg-bg.sh                         Generates the DMG background image
├── fix-dmg-bg.sh                     Applies the background to a built DMG
├── fix-opencv.sh                     Patches OpenCV dylib @rpaths for distribution
├── .gitignore                        Git ignore rules for app/
│
├── build/                            Static assets consumed by electron-builder
│   ├── icon.icns                     App icon (macOS)
│   ├── background.png                DMG background image
│   ├── SpaceMono-Bold.ttf            Bundled font
│   ├── SpaceMono-Regular.ttf         Bundled font
│   └── .gitignore
│
├── web/                              Flask web server (Python)
│   ├── app.py                        Main Flask app — camera, settings, task routing, AI proxy
│   ├── task1_2.py                    Flask blueprint for Task 1.2 — spawns C++ binary, returns JSON
│   ├── task1_2_modes.py              Video / hybrid-AI / AI-only analysis modes for Task 1.2
│   ├── templates/
│   │   ├── index.html                Main dashboard UI
│   │   ├── 1_2.html                  Task 1.2 — stereo photogrammetry page
│   │   ├── 2_1.html                  Task 2.1 page
│   │   ├── 2_2.html                  Task 2.2 page
│   │   ├── 2_5.html                  Task 2.5 page
│   │   └── tasks.html                Task switcher overlay
│   └── static/
│       ├── css/
│       │   ├── main.css              Global styles — tokens, dashboard, camera panel, HUD
│       │   └── 1_2.css               Task 1.2 specific styles
│       ├── js/
│       │   ├── main.js               Dashboard JS — camera feed, settings, WS, fullscreen, restart
│       │   ├── 1_2.js                Task 1.2 controller — upload, analyze, scale, enhance
│       │   ├── 1_2_viewer.js         Three.js 3D viewer — sections, plates, dimensions, exports
│       │   └── 1_2_modes.js          Task 1.2 video/AI mode UI helpers
│       ├── fonts/
│       │   ├── SpaceMono-Bold.ttf
│       │   └── SpaceMono-Regular.ttf
│       └── vendor/                   Vendored JS (Three.js r146 + helpers, offline-safe)
│           ├── three.min.js
│           ├── OrbitControls.js
│           ├── GLTFExporter.js
│           └── OBJExporter.js
│
├── native/
│   └── settings/                     Native macOS Settings window (Swift + Node addon)
│       ├── SettingsUI.swift          SwiftUI — all settings panes, store, keychain helpers
│       ├── settings.mm               ObjC++ Node-API addon — dlopen → libSettingsUI.dylib
│       ├── binding.gyp               node-gyp build config for the addon
│       ├── package.json              Addon package (node-addon-api dep)
│       └── build.sh                  Builds dylib + .node, fixes rpaths, copies artifacts
│
├── models/                           YOLO model weights for Task 2.1 crab detection
│   ├── M26.CD.P1.300.81__3.6.2026_21.54.pt
│   ├── M26.CD.P2.116.20__5.2.2026_14.10.pt
│   └── M26.CD.P3.416.7__5.2.2026_14.20.pt
│
└── scripts/
    └── task1_2/                      Task 1.2 — stereo photogrammetry C++ pipeline
        ├── Makefile                  Build — `make all` / `make dist` (dist runs fix-opencv.sh)
        ├── json.hpp                  nlohmann/json single-header (vendored)
        ├── sample_model.json         Example Model3D output for testing
        ├── README.md                 Short overview
        ├── INFO.md                   In-depth pipeline walkthrough + diagrams
        ├── MATH.md                   Derivations for every formula used
        ├── .gitignore                Ignore build files
        │
        ├── include/                  C++ headers
        │   ├── pipeline.hpp          PipelineInput / PipelineOutput / runPipeline()
        │   ├── pipe_pipeline.hpp     PipePipelineConfig / Output / runPipePipeline()
        │   ├── model3d.hpp           Model3D schema, Vec3, Plate, ScaleInfo, applyScale()
        │   ├── calibration_io.hpp    CameraIntrinsics / StereoExtrinsics YAML load/save
        │   ├── stereo_rectifier.hpp  RectifiedPair, rectifyStereoPair()
        │   ├── image_undistort.hpp   Lens distortion removal
        │   ├── manhattan_calib.hpp   Vanishing-point focal estimation + auto-calib
        │   ├── depth_segment.hpp     SGBM → foreground mask + subject distance estimate
        │   ├── lab_segment.hpp       LAB k-means plate segmentation + corner subpix
        │   ├── plate_detector.hpp    HSV plate detector (legacy + PlateDetectorConfig)
        │   ├── plate_fusion.hpp      Union + IoU dedup of LAB and Vision detections
        │   ├── plate_pnp.hpp         IPPE-square PnP per plate
        │   ├── vision_rectangles.hpp Apple VNDetectRectanglesRequest bridge (header)
        │   ├── match_ransac.hpp      RANSAC L↔R plate correspondence
        │   ├── refine_scale.hpp      Umeyama similarity scale refinement
        │   ├── bundle_adjust.hpp     Global LM bundle adjustment with Huber loss
        │   ├── manual_scale.hpp      applyManualWidthOverride / applyManualLengthOverride
        │   ├── pipe_detector.hpp     Legacy Canny+Hough pipe detector
        │   ├── pipe_lsd.hpp          LSD pipe line detector
        │   ├── pvc_segment.hpp       LAB+chroma PVC mask + distance transform + skeleton
        │   ├── pipe_lines_multi.hpp  4-detector line vote ensemble
        │   ├── pipe_parallel_pair.hpp Sobel-ridge parallel-pair pipe detector
        │   ├── pipe_pink_tape.hpp    LAB pink-blob landmark detector
        │   ├── pipe_ransac.hpp       MSAC total-least-squares line refit
        │   ├── pipe_diameter.hpp     Distance-transform diameter gate
        │   ├── sgbm_disparity.hpp    SGBM dense disparity + optional WLS filter
        │   ├── pipe_match_stereo.hpp Epipolar + Sampson stereo line matching
        │   ├── pipe_sampson.hpp      Sampson-optimal endpoint triangulation
        │   ├── pipe_cylinder3d.hpp   RANSAC + LM 3D cylinder fit
        │   ├── pipe_graph.hpp        KD-tree junction graph
        │   ├── pipe_graph_validate.hpp Post-fit graph sanity checks
        │   ├── pipe_bundle.hpp       LM joint bundle adjust over junctions + radii
        │   ├── pipe_template.hpp     3-section parametric template + Umeyama fit + inject
        │   ├── per_pair_model.hpp    Triangulate plates/pipes → per-pair Model3D
        │   ├── multi_pair_fuse.hpp   ICP/Procrustes multi-pair fusion
        │   ├── scale_estimator.hpp   Plate-prior scale observations
        │   ├── triangulator.hpp      DLT triangulation utilities
        │   ├── dense_mvs.hpp         Dense multi-view stereo helpers
        │   ├── wireframe_builder.hpp Model3D → GLB/OBJ wireframe
        │   ├── scene_io.hpp          JSON / GLB / OBJ serialization
        │   ├── underwater.hpp        Sea-thru red-channel restoration (legacy)
        │   ├── underwater_restore.hpp underwaterRestore() — per-channel gain correction
        │   ├── image_quality.hpp     IQC — blur, exposure, clipping checks
        │   ├── ai_enhancer.hpp       AI enhance pass (fork ai_caller.py)
        │   ├── apple_intelligence.hpp FoundationModels bridge (header)
        │   ├── apple_vision.hpp      Vision.framework utilities (header)
        │   ├── metal_compute.hpp     Metal GPU kernel dispatch (header)
        │   ├── accelerate_utils.hpp  Accelerate BLAS/vDSP helpers
        │   ├── stereo_math.hpp       Stereo geometry math utilities
        │   └── legacy_shims.hpp      Free-function adapters for legacy callers
        │
        ├── src/                      C++ implementations
        │   ├── main.cpp              CLI entry point — arg parse, runPipeline / --engine pipe
        │   ├── pipeline.cpp          Plate-first orchestrator (runPipeline)
        │   ├── pipe_pipeline.cpp     Pipe-first orchestrator (runPipePipeline)
        │   ├── model3d.cpp           Model3D serialization, applyScale, recomputeBounds
        │   └── *                     Remaining files mirror include/
        │
        ├── objc/                     Objective-C++ Apple framework bridges
        │   ├── metal_compute.mm      Metal GPU kernels (rectification, NCC)
        │   ├── apple_vision.mm       Vision.framework — contours, sub-pixel features
        │   ├── apple_intelligence.mm FoundationModels bridge (macOS 15+)
        │   └── vision_rectangles.mm  VNDetectRectanglesRequest — neural rectangle detection
        │
        ├── metal/
        │   └── kernels.metal         Metal shaders — image rectification, NCC correlation
        │
        └── python/                   Python helper scripts
            ├── ai_caller.py          AI enhance caller — routes to Apple/cloud provider
            ├── camera_calibration.py Per-camera intrinsics calibration (chessboard)
            ├── stereo_calibrate.py   Stereo extrinsics calibration → stereo_calib.yaml
            └── pkl_to_yaml.py        Converts legacy .pkl calibration files to YAML

─────── INSTALLED / GENERATED ─────────────────────────────────────────────────────────────────────────────────
node_modules/                         Electron + builder dependencies  (npm install)
native/settings/node_modules/         Addon dependencies               (npm install in native/settings)
native/settings/build/Release/        Compiled addon artifacts         (bash native/settings/build.sh)
  ├── libSettingsUI.dylib
  └── settings.node
app/Resources/python-runtime/         Standalone Python 3.13 + all packages
app/Resources/opencv-libs/            Bundled OpenCV dylibs (copied + @rpath-patched)
scripts/task1_2/task1_2               Compiled C++ binary              (make -C scripts/task1_2 dist)
scripts/task1_2/task1_2_kernels.metallib  Compiled Metal kernel library
dist/                                 Built .app + .dmg                (bash build.sh)
───────────────────────────────────────────────────────────────────────────────────────────────────────────────
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
xcodebuild -downloadComponent MetalToolchain
brew install opencv eigen pkg-config
mkdir -p app/Resources/opencv-libs
cp -L /opt/homebrew/opt/opencv/lib/*.dylib app/Resources/opencv-libs/
```

### Makefile

Add to `/scripts/task1_2/Makefile`:

```makefile
.PHONY: all dist clean run help

all: $(TARGET)

dist: $(TARGET)
	@bash ../../fix-opencv.sh
```

### Electron App

**Must use exactly Electron 40.6.1.** Do not upgrade or downgrade; this version is required for compatibility.

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

**WARNING**: Do *not* build inside of a storage provider folder, such as Google Drive or iCloud.

```bash
# Build with signing (for distribution)
bash build.sh "SIGNING-IDENTITY"

# Build without signing (for testing)
bash build.sh --test

# Just create the app folder (no DMG)
bash build.sh --pack
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
