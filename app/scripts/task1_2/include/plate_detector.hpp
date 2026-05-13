#pragma once
// ─────────────────────────────────────────────────────────────────────────
// plate_detector.hpp — color-based plate detection with sub-pixel corner
// refinement.
//
// Approach:
//   1. Convert to HSV, threshold by hue with a configurable tolerance band.
//      Saturation/value gates filter out faded background pixels of the
//      same hue and dark shadows.
//   2. Morphological close to fuse plate interiors broken by reflections.
//   3. Find contours, keep ones with area in [min_area, max_area].
//   4. For each kept contour: minAreaRect + initial 4 corners.
//   5. Refine each corner via cv::cornerSubPix using a small window. This
//      dramatically improves localization accuracy (~0.5 px → ~0.05 px).
//   6. Compute confidence: a combination of area, fill ratio (contour vs
//      minAreaRect), and aspect-ratio similarity to a square (since plates
//      are 10×10 cm).
//
// Configuration is set per-call so the same detector instance can run
// against different plate colors / tolerances.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PlateDetectorConfig {
    // Target color: H in [0, 179], S in [0, 255], V in [0, 255] — OpenCV
    // convention. Hue is circular; tolerance wraps around 180.
    int target_h = 135;            // ~purple
    int hue_tolerance = 50;        // degrees in OpenCV's H scale (0-179)
    int min_s = 15;                // minimum saturation
    int min_v = 30;                // minimum value
    int max_v = 255;
    // Size gates as fractions of the image area. Plates take up a roughly
    // bounded chunk of the frame; numbers very different from this are
    // either false positives or wildly wrong.
    double min_area_frac = 0.0005; // 0.05%
    double max_area_frac = 0.10;   // 10%
    // Morphological close kernel size in pixels.
    int close_kernel_px = 5;
    // Sub-pixel corner refinement window (half-size).
    int subpix_win = 5;
    int subpix_zerozone = -1;      // -1 disables the zero-zone
    int subpix_max_iter = 30;
    double subpix_eps = 0.01;
    // Aspect-ratio confidence: plates are square, so |w/h - 1| should be
    // small. ar_tol_full is the spread that maps to 0% confidence.
    double ar_tol_full = 1.5;

    // Legacy alias: some older callers (pipeline.cpp) refer to
    // `expected_plate_count`. We keep the new field name as-is for the
    // detector itself but expose the alias for compatibility.
    int expected_plate_count = 8;
};

// Backwards-compat alias kept so legacy `pipeline.cpp` (which spells the
// type `PlateDetectorParams`) still compiles. Identical layout/semantics.
using PlateDetectorParams = PlateDetectorConfig;

struct PlateDetection {
    cv::Point2f center;            // sub-pixel
    std::array<cv::Point2f, 4> corners; // sub-pixel, ordered TL, TR, BR, BL
                                        // (canonical winding from minAreaRect)
    double area_px = 0.0;
    double fill_ratio = 0.0;       // contour area / rotated-rect area
    double aspect_ratio = 1.0;     // long / short side
    double confidence = 0.0;       // 0..100
    int contour_index = -1;        // index into the raw contour list (debug)

    // Optional unique id assigned by detectors / fusers (lab_segment,
    // plate_fusion, …). -1 = unassigned. Kept here so downstream code
    // (match_ransac, scale refinement) can refer to specific detections
    // across passes without keeping parallel index arrays.
    int id = -1;
};

class PlateDetector {
public:
    explicit PlateDetector(const PlateDetectorConfig& cfg = {}) : cfg_(cfg) {}

    void setConfig(const PlateDetectorConfig& cfg) { cfg_ = cfg; }
    const PlateDetectorConfig& config() const { return cfg_; }

    // Run detection. Returns plates sorted by descending confidence.
    // `expected_count` clips the output to (at most) the top-N detections.
    // Setting it to 0 disables clipping.
    std::vector<PlateDetection>
    detect(const cv::Mat& bgr, int expected_count = 0) const;

    // Build a debug visualization (clones bgr, draws each plate).
    cv::Mat
    visualize(const cv::Mat& bgr,
              const std::vector<PlateDetection>& dets) const;

private:
    PlateDetectorConfig cfg_;

    // Build the binary mask from BGR per cfg_.
    cv::Mat buildHsvMask(const cv::Mat& bgr) const;
};

}  // namespace mate
