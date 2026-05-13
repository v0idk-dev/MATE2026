#pragma once
// ─────────────────────────────────────────────────────────────────────────
// sgbm_disparity.hpp — dense stereo disparity via Semi-Global Block
// Matching (Hirschmüller 2005), with optional Weighted-Least-Squares
// post-filter (Lang et al. 2012, available in cv::ximgproc) and
// left-right consistency check.
//
// Why SGBM (not BM/PatchMatch/raw depth):
//   • SGBM aggregates costs along 8 (or in 3WAY mode, 5) directions,
//     enforcing piecewise-smoothness without over-blurring depth edges.
//   • 3WAY mode is the Apple-Silicon-friendly default — 4× faster than
//     full SGBM with negligible accuracy loss on natural scenes.
//   • WLS post-filter recovers thin pipe surfaces that block-matching
//     misses (bilateral weighting along the colour edges).
//
// All parameters are auto-derived from image size and the user-supplied
// baseline B. The caller never hardcodes num_disparities: it is computed
// as the disparity range that covers Z ∈ [Z_near, Z_far] given B and the
// auto-estimated focal length f. See MATH.md §4.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>

namespace mate {

struct SgbmConfig {
    // Auto-tuning bounds. The disparity range is sized to cover this
    // metric depth interval given (f, B).
    double Z_near_m = 0.20;
    double Z_far_m  = 4.00;

    // Manual overrides (0 = auto).
    int    num_disparities_override = 0;     // forced multiple-of-16
    int    block_size_override      = 0;     // odd

    // Standard SGBM tuning.
    int    P1_per_chan = 8;       // P1 = P1_per_chan * channels * block² (smoothness for Δd=1)
    int    P2_per_chan = 32;      // P2 = P2_per_chan * channels * block² (smoothness for Δd>1)
    int    pre_filter_cap     = 31;
    int    uniqueness_ratio   = 12;
    int    speckle_window_size= 100;
    int    speckle_range      = 2;
    int    disp12_max_diff    = 1;     // L↔R consistency tolerance (pixels)
    bool   use_3way_mode      = true;  // SGBM_3WAY (4× faster on Apple Silicon)

    // WLS post-filter (cv::ximgproc); disabled if ximgproc not linked.
    bool   wls_enabled        = true;
    double wls_lambda         = 8000.0;
    double wls_sigma_color    = 1.5;

    // Confidence threshold; pixels below get marked invalid (NaN).
    int    min_confidence     = 64;     // 0..255 from WLS
};

struct SgbmResult {
    cv::Mat disparity;    // CV_32F, in pixels; NaN where invalid
    cv::Mat confidence;   // CV_8U  (255 = best), empty if WLS disabled
    int     num_disparities_used = 0;
    int     block_size_used      = 0;
    int     pixels_valid         = 0;
    double  pct_valid            = 0.0;
};

SgbmResult computeSgbmDisparity(const cv::Mat& left_rect,
                                const cv::Mat& right_rect,
                                double focal_px,
                                double baseline_m,
                                const SgbmConfig& cfg = {});

// Reprojects a disparity map to a 3D point cloud using Q. Pixels with
// NaN disparity are skipped. Returns the cloud in left-rectified-camera
// coordinates, units = baseline unit (metres if Q used metres for B).
struct CloudPoint {
    cv::Point3f p;
    cv::Vec3b   bgr;
    float       disparity;
    int         u;     // pixel column in LEFT rectified view
    int         v;     // pixel row    in LEFT rectified view
};
std::vector<CloudPoint> disparityToCloud(const cv::Mat& disparity,
                                          const cv::Mat& Q,
                                          const cv::Mat& left_rect_bgr,
                                          int max_points = 200000);

}  // namespace mate
