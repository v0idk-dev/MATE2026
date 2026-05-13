#pragma once
// ─────────────────────────────────────────────────────────────────────────
// stereo_math.hpp — pure stereo geometry helpers.
//
// Centralises the formulas:
//   Z   = f · B / d                        (depth from disparity)
//   d   = f · B / Z                        (inverse)
//   s_px = f · s_m / Z                     (plate pixel size at depth)
//   σ_Z = Z² · σ_d / (f · B)               (depth uncertainty)
//   Q   = disparity-to-3D matrix from (f, c_x, c_y, B)
//   K   = intrinsics estimated from (W, H, FOV)
//
// Units (NEVER mix):
//   • f, c_x, c_y, σ_d, d   — pixels
//   • B, Z, s_m, σ_Z        — meters
//
// Vocabulary:
//   • B (baseline) = distance BETWEEN the two camera lenses (user input,
//     `baseline_m`).
//   • Z (subject distance) = distance from the cameras TO the coral garden
//     (auto-estimated from disparity, never user input).
//
// Full derivations and a worked numerical example: see ../MATH.md.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>

namespace mate {

// Z = f · B / d. Returns NaN for non-positive disparity.
double disparityToDepth(double disparity_px, double focal_px, double baseline_m);

// d = f · B / Z. Returns NaN for non-positive depth.
double depthToDisparity(double depth_m, double focal_px, double baseline_m);

// s_px = f · s_m / Z. Used to size morphological SEs and detection
// minimum-size gates.
double expectedPlatePx(double focal_px, double plate_side_m, double subject_distance_m);

// σ_Z = Z² · σ_d / (f · B). One-σ depth uncertainty for a given disparity
// uncertainty (use 0.5 px for sub-pixel SGBM, 8 px for the bad-calibration
// worst case the user described).
double depthUncertainty(double depth_m, double focal_px,
                        double baseline_m, double sigma_disparity_px);

// Build a Q matrix when no full stereo calibration is loaded. Convention:
// rectified-right camera lies at +B from rectified-left along +X. Returned
// matrix is CV_64F 4×4 suitable for cv::reprojectImageTo3D.
cv::Mat makeQFromBaseline(double focal_px, double cx_px, double cy_px,
                          double baseline_m, double cx_right_px = -1);

// Estimate K from image size + assumed horizontal field-of-view in degrees.
// Used when intrinsics weren't uploaded. Default FOV 60° matches typical
// GoPro / ROV / phone cameras.
cv::Mat estimateIntrinsicsFromImage(int W, int H, double fov_h_deg = 60.0);

// Pull the focal length (f_x ≈ f_y) out of K. Returns NaN if K is bad.
double focalFromK(const cv::Mat& K);

// Sanity gates the user's baseline value. Returns:
//   "ok"      — value is in a reasonable range [0.02, 1.0] m
//   "narrow"  — < 0.02 m (poor depth accuracy)
//   "wide"    — > 1.0 m  (uncommon — confirm with user)
//   "invalid" — non-positive
const char* baselineSanity(double baseline_m);

}  // namespace mate
