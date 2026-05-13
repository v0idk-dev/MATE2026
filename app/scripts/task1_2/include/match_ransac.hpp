#pragma once
// ─────────────────────────────────────────────────────────────────────────
// match_ransac.hpp — RANSAC L↔R plate correspondence under epipolar.
//
// Inputs are two plate sets already detected in the rectified pair (so
// epipolar lines are horizontal). For each candidate (l, r):
//   • |y_l - y_r| < epi_tol_px         (epipolar consistency)
//   • size ratio ∈ [1/sz_tol, sz_tol]
//   • LAB colour distance < lab_tol     (caller pre-computes the centres)
//
// Then RANSAC over a piecewise-constant disparity model (one mean
// disparity per coral section, K=3) with `iters` trials, sampling 3
// matches at random and keeping the consensus with the most inliers
// at `inlier_tol_px`.
// ─────────────────────────────────────────────────────────────────────────

#include "plate_detector.hpp"
#include <opencv2/core.hpp>
#include <vector>
#include <utility>

namespace mate {

struct MatchRansacConfig {
    double epi_tol_px       = 1.5;
    double sz_tol           = 1.6;
    double inlier_tol_px    = 0.4;
    int    iters            = 1000;
    int    n_sections       = 3;
};

// Returns inlier (left_idx, right_idx) pairs.
std::vector<std::pair<int, int>>
ransacMatchPlates(const std::vector<PlateDetection>& left,
                  const std::vector<PlateDetection>& right,
                  const MatchRansacConfig& cfg = {});

}  // namespace mate
