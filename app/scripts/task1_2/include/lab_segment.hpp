#pragma once
// ─────────────────────────────────────────────────────────────────────────
// lab_segment.hpp — robust plate detection in CIE LAB space.
//
// Replaces the original HSV-threshold plate detector for cases where
// it fails: faded plates, washed-out tape under stage lights, underwater
// colour cast, motion blur, JPEG noise.
//
// IMPORTANT: this module needs the SUBJECT DISTANCE (Z, camera-to-coral),
// NOT the baseline (B, camera-to-camera). Z is auto-estimated by
// depth_segment.cpp before this module runs. If Z is unknown, the caller
// can pass `expected_plate_px` directly, or omit both and the module
// falls back to a wide multi-scale search.
//
// Pipeline:
//   1. Convert BGR → CIE LAB (8U, 0..255).
//   2. Build target LAB centroid from the user's colour-picker hex.
//   3. Mahalanobis distance to centroid using image-derived variance.
//   4. K-means refine (K=2) initialised from {centroid, image-mean}.
//   5. Morphological open/close with disk SE sized to the EXPECTED plate
//      pixel footprint at distance Z (formula: s_px = f · s_m / Z).
//   6. Connected components → minAreaRect → cv::cornerSubPix on each
//      blob's 4 corners.
//   7. Reject blobs with bad aspect/area; keep top expected_plates.
// ─────────────────────────────────────────────────────────────────────────

#include "plate_detector.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct LabSegmentConfig {
    int    target_hue_opencv = 135;   // 0..179 (OpenCV scale)
    int    hue_tol_deg       = 25;
    double plate_side_m      = 0.10;  // physical edge length (10 cm spec)

    // Geometry inputs for SE sizing. Caller sets EITHER subject_distance_m
    // (preferred — auto-estimated by depth_segment) OR expected_plate_px
    // (direct override). If both are unset (≤0), morphology runs at a
    // default scale and aspect/area gates loosen.
    double focal_px               = 0.0;   // 0 = unknown
    double subject_distance_m     = 0.0;   // 0 = unknown
    double expected_plate_px      = 0.0;   // 0 = unknown

    int    expected_plates   = 8;
    bool   refine_corners    = true;
};

std::vector<PlateDetection>
labSegmentPlates(const cv::Mat& bgr, const LabSegmentConfig& cfg);

// Convenience overload (legacy signature, distance-agnostic — uses
// multi-scale fallback).
std::vector<PlateDetection>
labSegmentPlates(const cv::Mat& bgr, int target_hue, int hue_tol);

}  // namespace mate
