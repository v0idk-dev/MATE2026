// image_quality.hpp — pre-flight image-pair quality gate.
//
// Catches the common "garbage in, garbage out" failure modes before the
// expensive pipeline ever runs:
//
//   * Blur / defocus     →  Laplacian-of-luminance variance (Pertuz et al.,
//                           Pattern Recognition 2013) thresholded against an
//                           absolute floor and against the *other* eye to
//                           reject one-eye-out-of-focus pairs.
//   * Exposure mismatch  →  mean-luminance ratio outside [1/r, r] (default
//                           r = 1.6), which would invalidate disparity
//                           assumptions of constant brightness.
//   * Saturation / black →  fraction of pixels at 0 or 255 in either channel
//                           exceeding a threshold (clipped highlights kill
//                           SGBM in the bright PVC cap area).
//   * Resolution sanity  →  both images present, same size, ≥ 240×320,
//                           single- or three-channel.
//
// All thresholds are configurable. The result carries a single `pass` bool
// plus a list of human-readable reasons so callers can decide whether to
// abort or just warn-and-continue.
//
#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace mate {

struct ImageQualityConfig {
    double blur_var_min        = 60.0;   // Laplacian variance, abs. floor
    double blur_var_ratio_max  = 4.0;    // max(L,R)/min(L,R) for blur
    double exposure_ratio_max  = 1.6;    // max(meanL,meanR)/min(...)
    double clip_frac_max       = 0.05;   // fraction of saturated pixels
    int    min_width           = 320;
    int    min_height          = 240;
};

struct ImageQualityReport {
    bool   pass               = true;
    double blur_var_left      = 0.0;
    double blur_var_right     = 0.0;
    double mean_lum_left      = 0.0;
    double mean_lum_right     = 0.0;
    double clip_frac_left     = 0.0;
    double clip_frac_right    = 0.0;
    std::vector<std::string> reasons;
};

ImageQualityReport
checkImagePairQuality(const cv::Mat& L,
                      const cv::Mat& R,
                      const ImageQualityConfig& cfg = {});

}  // namespace mate
