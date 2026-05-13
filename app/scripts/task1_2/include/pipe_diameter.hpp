#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_diameter.hpp — projected-diameter gate.
//
// PVC pipes used in the MATE coral garden are 0.012–0.055 m diameter
// (1/2", 3/4", 1", 1.5", 2"). Any line segment whose projected pixel
// thickness — measured by the distance-transform of the PVC mask — does
// not back-project to that range at the auto-estimated subject distance
// Z is rejected. This kills:
//   • shadow edges (zero thickness)
//   • chalkboard / table edges (huge thickness)
//   • text / labels in the OSD band (already excluded but defence in
//     depth)
//
// Conversion:
//     r_m = r_px · Z / f
// where r_px = distance-transform value sampled along the line skeleton,
// Z = depth_segment.subject_distance_m_est, f = focal length in pixels
// (auto-estimated from image dimensions if no calibration given). All
// derivations and worked numbers in MATH.md §6.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_ransac.hpp"
#include <opencv2/core.hpp>

namespace mate {

struct PipeDiameterConfig {
    double r_min_m = 0.006;   // 12 mm OD / 2
    double r_max_m = 0.030;   // 60 mm OD / 2 (slack for 2" + slip-fit)
    // Tolerance multiplier: keep pipes whose r_m is within
    // [r_min_m / tol, r_max_m * tol] (because depth uncertainty
    // grows with Z²; see depthUncertainty in stereo_math).
    double tol     = 1.4;
    int    sample_stride_px = 2;
};

struct PipeDiameterResult {
    PipeRansacResult seg;
    double radius_m = 0.0;
    double radius_m_sigma = 0.0;
    bool   ok       = false;
};

std::vector<PipeDiameterResult>
gateByDiameter(const std::vector<PipeRansacResult>& segs,
               const cv::Mat& dist_px,        // from segmentPvc()
               double focal_px,
               double subject_distance_m_Z,
               double subject_distance_m_sigma_Z,
               const PipeDiameterConfig& cfg = {});

}  // namespace mate
