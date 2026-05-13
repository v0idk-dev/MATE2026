#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_sampson.hpp — Sampson-error LM endpoint refinement.
//
// `cv::triangulatePoints` solves DLT via SVD — algebraically optimal
// but not statistically optimal. We follow the standard recipe of
// Hartley & Zisserman §12.3:
//
//   1. DLT initialisation                X⁰
//   2. Iteratively reweighted least squares minimising the SAMPSON
//      first-order approximation to the geometric (reprojection) error
//      via Levenberg-Marquardt.
//   3. Stop on |Δχ²/χ²| < 1e-6 or after 30 iterations.
//
// The Sampson cost is a 1-D Newton step away from the Maximum-Likelihood
// estimator under Gaussian image noise, and it's much cheaper than the
// full reprojection-error LM (no need to carry the X gradient — one
// matrix-vector product per iteration). Reference: Hartley & Zisserman,
// Multiple View Geometry §12.3, eq. (12.6).
//
// The same machinery is used for plate corners (so confidence-weighted
// plate triangulation feeds back into bundle_adjust). The function
// signature works for either: pass two image points and two 3×4
// projection matrices.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct SampsonRefineReport {
    int    iters_used   = 0;
    double chi2_initial = -1;
    double chi2_final   = -1;
    bool   converged    = false;
};

cv::Point3d sampsonTriangulate(const cv::Point2d& xL, const cv::Point2d& xR,
                                const cv::Mat& P1, const cv::Mat& P2,
                                SampsonRefineReport* report = nullptr);

// Vectorised wrapper.
std::vector<cv::Point3d>
sampsonTriangulateMany(const std::vector<cv::Point2d>& xL,
                       const std::vector<cv::Point2d>& xR,
                       const cv::Mat& P1, const cv::Mat& P2);

}  // namespace mate
