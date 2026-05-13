#pragma once
// ─────────────────────────────────────────────────────────────────────────
// vision_rectangles.hpp — Apple Vision colour-agnostic rectangle detector.
//
// Second-opinion plate detector that complements lab_segment, particularly
// for partial occlusions and edge-defined plates that lost their colour.
//
// IMPORTANT (same correction as lab_segment): the geometric size gate
// uses SUBJECT DISTANCE Z, not BASELINE B. The convenience overload that
// takes Z must be called with an auto-estimated subject distance (from
// depth_segment), never the user's baseline_m.
// ─────────────────────────────────────────────────────────────────────────

#include "plate_detector.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct VisionRectConfig {
    double min_aspect_ratio  = 0.85;
    double max_aspect_ratio  = 1.15;
    double min_size_px       = 14;     // skip tiny detections
    double min_confidence    = 0.35;   // VNRectangleObservation.confidence
    int    max_observations  = 32;
    double quad_tolerance    = 0.30;
};

// Vision rectangle detection (no implicit geometry — set min_size_px
// directly via the config).
std::vector<PlateDetection>
visionDetectRectangles(const cv::Mat& bgr, const VisionRectConfig& cfg = {});

// Convenience: geometry-aware overload. Pass plate side, the
// AUTO-ESTIMATED subject distance Z, and the focal length. Sets
// `min_size_px` from the formula s_px = f · s_m / Z. Do NOT pass
// the baseline here.
std::vector<PlateDetection>
visionDetectRectangles(const cv::Mat& bgr,
                       double plate_side_m,
                       double subject_distance_m,
                       double focal_px = 1200.0);

}  // namespace mate
