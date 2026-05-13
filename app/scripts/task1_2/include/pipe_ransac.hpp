#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_ransac.hpp — MSAC line re-fit on edge pixels.
//
// After multi-detector fusion we have approximate endpoints; here we
// re-fit each line to the *actual* edge pixels lying inside its
// influence band. We use MSAC (M-estimator sample consensus, Torr &
// Zisserman 2000) instead of vanilla RANSAC — same iteration count but
// the cost function is the truncated squared error, which gives smoother
// transitions near the inlier/outlier boundary and lower bias on noisy
// data.
//
// Process per candidate line:
//   1. Take pixels P from the PVC mask within `band_px` of the line.
//   2. MSAC over (point-pair) samples for `iters` rounds.
//   3. Re-fit with total-least-squares on inliers (cv::fitLine
//      DIST_L2 → orthogonal regression).
//   4. Update endpoints to the projection of the original endpoints
//      onto the refined line.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_lines_multi.hpp"
#include <opencv2/core.hpp>

namespace mate {

struct PipeRansacConfig {
    double band_px         = 6.0;     // distance from line to consider edge pixels
    int    iters           = 200;
    double inlier_tol_px   = 1.5;
    int    min_inliers     = 25;
    double min_inlier_ratio= 0.35;
};

struct PipeRansacResult {
    LinesMultiSegment seg;
    int    inliers     = 0;
    double inlier_ratio= 0.0;
    double rms_px      = 0.0;
    bool   ok          = false;
};

std::vector<PipeRansacResult>
ransacRefitLines(const cv::Mat& bgr,
                 const cv::Mat& pvc_mask,
                 const std::vector<LinesMultiSegment>& candidates,
                 const PipeRansacConfig& cfg = {});

}  // namespace mate
