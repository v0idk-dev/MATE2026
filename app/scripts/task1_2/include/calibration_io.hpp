#pragma once
// ─────────────────────────────────────────────────────────────────────────
// calibration_io.hpp — load camera calibration data for the photogrammetry
// pipeline. Two flavors of input:
//
//   1. Per-camera intrinsics YAML produced by python/pkl_to_yaml.py from
//      the existing .pkl files (works for pinhole & fisheye distortion
//      models).
//   2. Stereo extrinsics YAML produced by the standalone
//      crab-detect/camera-calibration/stereo_calibrate.py tool.
//
// Both files use cv::FileStorage's YAML format so we read them directly.
// The two are independent: stereo extrinsics also embed K_left/K_right and
// D_left/D_right copies (in case the user only imports the extrinsics
// without the per-camera pkls), but we prefer the dedicated per-camera
// files when available because they may have been recalibrated separately.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <string>
#include <optional>

namespace mate {

enum class DistortionModel { Pinhole, Fisheye };

// Per-camera intrinsics. Distortion is stored as either:
//   • Pinhole:  D shape (1, N) for N ∈ {4, 5, 8, 12, 14}.
//   • Fisheye:  D shape (4, 1) — equidistant model with 4 coefficients.
struct CameraIntrinsics {
    cv::Mat K;                  // 3×3 camera matrix, CV_64F
    cv::Mat D;                  // distortion coefficients, CV_64F
    DistortionModel model = DistortionModel::Pinhole;
    int image_width  = 0;       // pixels at which K was calibrated; 0 = unknown
    int image_height = 0;
    double rms_px = -1.0;       // reprojection RMS reported by the calibrator
                                // ( <0 = unknown / not stored)
    std::string source_path;    // for diagnostics
};

// Stereo extrinsics — rigid transform of right cam in left cam frame.
// The "_provided" copies of K/D are only fallbacks; prefer the standalone
// CameraIntrinsics structs if the user supplied them.
struct StereoExtrinsics {
    cv::Mat R;                  // 3×3 rotation
    cv::Mat T;                  // 3×1 translation
    cv::Mat E;                  // essential matrix (informational)
    cv::Mat F;                  // fundamental matrix (informational)
    cv::Mat R1, R2;             // rectification rotations (per camera)
    cv::Mat P1, P2;             // rectified projections (3×4)
    cv::Mat Q;                  // 4×4 disparity-to-depth reprojection
    int image_width  = 0;
    int image_height = 0;
    std::string unit;           // "cm" or "m"; T and downstream measurements
                                // are in this unit.
    double rms_px = -1.0;       // stereo RMS from cv::stereoCalibrate
    double avg_epipolar_err_px = -1.0;
    double baseline = 0.0;      // ||T||, in `unit`
    int pairs_used = 0;
    // Fallback intrinsics embedded by stereo_calibrate.py — only consulted
    // if the dedicated per-camera files weren't supplied.
    CameraIntrinsics K_left_provided;
    CameraIntrinsics K_right_provided;
    bool has_provided_intrinsics = false;
    std::string source_path;
};

// Load per-camera intrinsics from a YAML produced by pkl_to_yaml.py.
// Returns nullopt on any read error (file missing / malformed / wrong shape).
// On success, the struct's `model`, `K`, `D`, image_width/height, rms_px and
// source_path are all populated.
std::optional<CameraIntrinsics>
loadCameraIntrinsicsYaml(const std::string& path);

// Load stereo extrinsics from a YAML produced by stereo_calibrate.py.
// Same nullopt-on-failure semantics. Embedded K/D are loaded into the
// `_provided` fields when present.
std::optional<StereoExtrinsics>
loadStereoExtrinsicsYaml(const std::string& path);

// Convert any unit field ("cm", "m", "mm", "in") to a multiplier that turns
// values in that unit into METERS. Defaults to meters (×1.0) on unknown.
double unitToMeters(const std::string& unit);

}  // namespace mate
