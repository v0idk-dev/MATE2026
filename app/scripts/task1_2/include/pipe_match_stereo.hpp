#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_match_stereo.hpp — robust L↔R pipe matching for rectified pairs.
//
// Each matched pipe needs one line in the L view and one in the R view.
// In a rectified pair, epipolar lines are horizontal, so for any L line
// the corresponding R line shares ≈ the same y at every x. We exploit
// SGBM-mean disparity within the line mask to *predict* the R endpoints,
// then accept only matches whose:
//   (a) endpoint y-difference < epi_tol_px           (epipolar gate)
//   (b) angle difference      < angle_tol_deg         (geometric gate)
//   (c) Sampson error         < sampson_tol_px        (algebraic gate)
//   (d) length ratio          ∈ [1/len_tol, len_tol]  (sanity gate)
//
// Sampson error on rectified pairs reduces to |y_l - y_r| but we
// generalise to support unrectified inputs too (we then compute the
// fundamental matrix from the projection matrices). This module also
// returns a per-match confidence ∈ [0,1] used by the cylinder fitter.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_diameter.hpp"
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

namespace mate {

struct PipeMatchStereoConfig {
    double epi_tol_px       = 2.0;
    double angle_tol_deg    = 6.0;
    double sampson_tol_px   = 1.5;
    double len_tol          = 1.6;
    double radius_tol       = 1.5;     // ratio of metric radii
};

struct PipeMatch {
    int left_idx  = -1;
    int right_idx = -1;
    cv::Point2f l0, l1, r0, r1;        // endpoints
    double      mean_disparity_px = 0; // d̄ over the line's PVC mask
    double      confidence        = 0; // 0..1
};

std::vector<PipeMatch>
matchPipesStereo(const std::vector<PipeDiameterResult>& L,
                 const std::vector<PipeDiameterResult>& R,
                 const cv::Mat& sgbm_disparity,    // CV_32F from sgbm_disparity
                 const cv::Mat& sgbm_confidence,   // CV_8U or empty
                 const PipeMatchStereoConfig& cfg = {});

}  // namespace mate
