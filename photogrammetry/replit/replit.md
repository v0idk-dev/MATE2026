# MATE ROV Competition - Multi-Task Interface

## Overview

A multi-task web application for the MATE ROV robotics competition. Currently implements Task 1.2 (Stereo Vision Distance Calculator) with a modular structure ready for additional tasks (AI crab counting, iceberg threat analysis, etc.).

## Project Architecture

```
web/                              - Shared web interface
  app.py                          - Flask backend (routing, task integration)
  templates/index.html            - Frontend HTML
  static/css/style.css            - Custom dark theme styling
  static/js/app.js                - Frontend JavaScript
scripts/                          - Task-specific processing scripts
  task1_2/                        - Stereo Vision Distance Calculator
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
outputs/                          - All task outputs (generated at runtime)
  uploads/                        - Uploaded images
  task1_2/                        - Task 1.2 debug/result images
```

## Build & Run

- Web app: `make -C scripts/task1_2 all && python web/app.py` (port 5000)
- CLI: `scripts/task1_2/stereo_distance <image1> <image2> [options]`

## Key Dependencies

- C++ 17 (clang++), OpenCV 4.11.0 (via Nix)
- Python 3.11, Flask, Gunicorn
- pkg-config, cmake, gnumake

## Task 1.2 Details

- Detects purple/colored plates in stereo camera images using HSV color segmentation
- Only plates with >=70% detection confidence are used for distance calculations
- Matches ALL high-confidence plates between left and right images using spatial/size similarity
- Calculates distance to EACH matched plate independently
- Outputs annotated image with distance labels overlaid on each plate
- Uses 4 methods per plate: stereo triangulation, disparity-based, known-size (width), known-size (height)
- Consensus distance per plate computed via robust mean of valid estimates
- Outputs distance in both inches and meters (competition uses inches)

## Recent Changes

- 2026-02-15: Added PVC pipe length measurement mode with reference square scale calibration
- 2026-02-15: Dual-mode web UI: "Plate Distance" (stereo) and "Pipe Lengths" (single-image) modes
- 2026-02-15: Pipe detection via edge detection, Hough line transform, collinear segment merging
- 2026-02-15: Reference square detector for 10cm colored squares (scale calibration)
- 2026-02-14: Multi-plate distance: now detects and measures distance to ALL plates (>=70% conf), not just one
- 2026-02-14: Annotated output image with distance labels drawn on each plate
- 2026-02-14: Web UI redesigned to show annotated image prominently + per-plate distance cards
- 2026-02-13: Fixed units from cm to inches for competition compatibility
- 2026-02-13: Added 70% confidence threshold for plate detection filtering
- 2026-02-13: Reorganized project for multi-task structure (scripts/task1_2/, outputs/)
- 2026-02-13: Added Flask web interface with image upload, parameter controls, results visualization
-
