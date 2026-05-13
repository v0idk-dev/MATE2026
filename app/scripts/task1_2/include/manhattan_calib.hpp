#pragma once
// ─────────────────────────────────────────────────────────────────────────
// manhattan_calib.hpp — auto-calibration fallback for when the user has
// not supplied stereo / per-camera calibration YAMLs.
//
// Strategy (texture-independent, single-image):
//   1. Detect line segments in a sample image (LSD → Hough fallback).
//   2. Cluster segments into vanishing points by greedy RANSAC.
//   3. Solve focal length from VP orthogonality (Caprile-Torre, with
//      principal point assumed at image center). If <2 reliable VPs are
//      found, fall back to a typical-FOV assumption (50° HFOV).
//   4. Build a CameraIntrinsics with no distortion (we don't have
//      enough info to recover it without a calibration target). Same K
//      for both L and R — rigid rig assumption.
//   5. Build StereoExtrinsics with R = I and T = (-rig_baseline_m, 0, 0)
//      — assumes the L/R cameras are mounted with parallel optical axes
//      and a horizontal baseline of known magnitude. cv::stereoRectify
//      then fills R1/R2/P1/P2/Q.
//
// The result is fed into the existing rectify → triangulate → fuse path
// exactly as if it had been loaded from disk. Ratios will be correct
// when the rig actually IS parallel-axis (which it almost always is for
// a fixed ROV mount). Absolute scale comes from the supplied baseline.
//
// This is the fallback path the pipeline takes when `calib_ok == false`.
// If Manhattan recovery itself fails (rare — requires very few clean
// straight edges, which the PVC garden almost always provides), the
// pipeline degrades to its pre-existing pixel-units behavior.
// ─────────────────────────────────────────────────────────────────────────

#include "calibration_io.hpp"
#include <opencv2/core.hpp>
#include <string>

namespace mate {

struct ManhattanCalibInput {
    cv::Mat sample_image;            // representative left frame (BGR)
    double  rig_baseline_m = 0.10;   // physical L↔R camera separation
};

struct ManhattanCalibResult {
    bool ok = false;
    CameraIntrinsics K_left;         // populated regardless of ok (best effort)
    CameraIntrinsics K_right;        // identical to K_left (rigid rig)
    StereoExtrinsics extrinsics;     // R=I, T=(-baseline,0,0), R1/R2/P1/P2/Q filled
    int    n_vanishing_points = 0;   // 0..3, diagnostic
    int    n_line_segments    = 0;   // diagnostic
    double estimated_focal_px = 0.0; // either VP-derived or fallback
    bool   focal_from_vps = false;   // false => fell back to FOV assumption
    std::string note;                // human-readable provenance string
};

// Run the auto-calibration. Always returns a populated result; check
// `ok` to know whether it should be used. On `!ok`, the `note` field
// explains what went wrong (e.g. "stereoRectify failed: ...").
ManhattanCalibResult deriveCalibrationFromManhattan(const ManhattanCalibInput& in);

}  // namespace mate
