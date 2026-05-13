# Task 1.2 Photogrammetry

***Check [INFO.md](INFO.md) and [MATH.md](MATH.md) for up-to-date information!***

Single, all-Apple-Silicon photogrammetry pipeline for the MATE 2026 ROV
coral-garden measurement task. Replaces the old 4-mode UI with one
**Analyze** flow that accepts N stereo pairs (1 or more), runs all 9
pipeline steps in a single C++ binary, and returns a custom JSON model
plus glTF/OBJ exports.

```
task1_2_overhaul/
├── scripts/task1_2/        ← copy verbatim into your repo's scripts/
│   ├── include/            (all headers)
│   ├── src/                (C++ implementations)
│   ├── objc/               (Apple Metal + Vision + FoundationModels bridges)
│   ├── metal/              (Metal compute kernels — compiled to .metallib)
│   ├── python/             (ai_caller.py, pkl_to_yaml.py, calib generators)
│   ├── json.hpp            (single-header nlohmann/json — unchanged)
│   └── Makefile
└── web/                    ← copy verbatim into your repo's web/
    ├── task1_2.py          (Flask blueprint — replaces task1_2_modes.py)
    ├── templates/1_2.html
    └── static/{css,js}/    (1_2.css, 1_2.js, 1_2_viewer.js, 1_2_modes.js stub)
```

## Build (Mac, Apple Silicon)

```sh
cd scripts/task1_2
make            # builds task1_2 binary + task1_2_kernels.metallib
```

Requires Xcode command-line tools (for `xcrun metal`/`metallib`) and
OpenCV 4 (`brew install opencv`). Honors the existing `OPENCV_PREFIX`
env-var convention from your old Makefile.

### Library dependencies (beyond OpenCV core)

| Lib                                                                                                                   | Status                                                                                 | Used by                                                                                                                 | Install                                                                                                              |
| --------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| **OpenCV 4** modules: `core`, `imgproc`, `calib3d`, `features2d`, `flann`, `highgui`                | required                                                                               | everything                                                                                                              | `brew install opencv`                                                                                              |
| **Eigen 3** (header-only)                                                                                       | **required**                                                                     | `pipe_bundle` (LM bundle adjust), `pipe_template`, `refine_scale`, `bundle_adjust`, optional Manhattan SVD path | `brew install eigen`                                                                                               |
| **opencv_contrib `ximgproc`**                                                                                 | **optional** — auto-detected via `__has_include` / `MATE_HAVE_XIMGPROC_FLD` | FLD line voter, SGBM-WLS disparity smoother                                                                             | `brew install opencv` already pulls contrib on Homebrew; otherwise build OpenCV with `OPENCV_EXTRA_MODULES_PATH` |
| Apple frameworks:`Metal`, `MetalPerformanceShaders`, `Accelerate`, `Vision`, `FoundationModels`, `CoreML` | required (auto-linked on macOS)                                                        | Apple-Silicon GPU/Vision/AI bridges                                                                                     | bundled with Xcode                                                                                                   |

The Manhattan auto-calibration fallback adds **no new lib requirements** —
it uses `cv::createLineSegmentDetector` (in OpenCV core ≥ 4.5; falls back
to `cv::HoughLinesP` via try/catch if LSD is unavailable in your build).

## Flask Wiring

The binary is discovered in this order (first hit wins):

1. `$TASK1_2_BIN`
2. `web/../scripts/task1_2/task1_2`
3. `web/../../scripts/task1_2/task1_2`

## Pipeline steps (mapped to source files)

| # | Step                     | Source                                       |
| - | ------------------------ | -------------------------------------------- |
| 1 | Undistort                | `src/image_undistort.cpp` + Metal          |
| 2 | Pipe detection           | `src/pipe_detector.cpp` (kept) + Vision    |
| 3 | Plate detection          | `src/plate_detector.cpp` (kept) + Vision   |
| 4 | Per-image rough 3D model | `src/per_pair_model.cpp`                   |
| 5 | Stereo align (rectify)   | `src/stereo_rectifier.cpp` (kept)          |
| 6 | Stereo geometry sizing   | `src/per_pair_model.cpp` (metric pass)     |
| 7 | Fuse N pairs             | `src/multi_pair_fuse.cpp`                  |
| 8 | AI enhance (optional)    | `src/ai_enhancer.cpp` + Apple Intelligence |
| 9 | Manual width override    | `src/manual_scale.cpp`                     |

Apple Silicon acceleration is automatically used when present:

* **Metal** kernels for remap, HSV, edges, median (`metal/kernels.metal`)
* **Accelerate** for PCA / Procrustes / median (`src/accelerate_utils.cpp`)
* **Vision** for sub-pixel rectangles, contours, feature prints (`objc/apple_vision.mm`)
* **FoundationModels** for on-device AI refinement (`objc/apple_intelligence.mm`)

## 3-D model format

The pipeline emits the same model in three formats:

* **Custom JSON** (compact, ~1–2 KB): for in-app persistence and AI
  prompting. Schema id `mate.coral_garden.v1`. See `include/model3d.hpp`
  and `src/model3d.cpp` (`toCustomJson` / `fromCustomJson`).
* **glTF binary (.glb)**: Blender-native, opens with File → Import →
  glTF 2.0. One mesh per section + one mesh per plate, two materials.
* **OBJ + MTL**: also Blender-native fallback.

## CLI

The binary works standalone for testing:

```sh
./task1_2 \
  --pair pair0_L.png pair0_R.png \
  --pair pair1_L.png pair1_R.png \
  --left-calib left.yaml --right-calib right.yaml \
  --stereo stereo.yaml \
  --out model.json --glb model.glb --obj model.obj \
  --target-hue 135 --hue-tol 25 --expected-plates 8 \
  --underwater --water-n 1.34 \
  --apple-intelligence \
  --manual-width-m 0.36
```

### Running without calibration YAMLs (Manhattan auto-calib)

For the MATE 2026 PVC coral garden the intra-pair baseline is fixed
at ~10 cm by the rig hardware. When you cannot supply
`--left-calib` / `--right-calib` / `--stereo`, pass the rig baseline
instead and the binary will auto-calibrate from the scene's vanishing
points (pipes meet at right angles → Manhattan world → Caprile–Torre
focal solve):

```sh
./task1_2 \
  --pair pair0_L.png pair0_R.png \
  --rig-baseline-m 0.10 \
  --out model.json --glb model.glb --obj model.obj \
  --target-hue 135 --hue-tol 25 --expected-plates 8
```

- Detects line segments via LSD (Canny+HoughLinesP fallback).
- Recovers ≤3 vanishing points by greedy RANSAC; medians the focal
  estimates from every orthogonal VP pair.
- Synthesises K with zero distortion and a parallel-axis stereo rig
  (`R = I`, `T = (−baseline, 0, 0)`), then runs `cv::stereoRectify`
  exactly like the YAML path.
- Output `model.json` includes a `warning` field flagging that the
  intrinsics were derived rather than calibrated. Ratios are reliable;
  absolute scale carries the uncertainty of the asserted baseline.

Pass `--no-auto-calib` to skip Manhattan recovery even when
`--rig-baseline-m` is set (forces the legacy FOV-based fallback). See
`HOW_IT_WORKS.md` §3 and `MATH.md` §4 for the math.
