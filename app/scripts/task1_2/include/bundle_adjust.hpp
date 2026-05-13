#pragma once
// ─────────────────────────────────────────────────────────────────────────
// bundle_adjust.hpp — global Levenberg–Marquardt refinement with Huber
// robust loss, hand-rolled on Eigen (no Ceres dependency).
//
// Variables:
//   • Per-section 6-DoF pose:   3 sections × (rx, ry, rz, tx, ty, tz)
//   • Per-plate (u, v) on its assigned section face: N × 2
//   • Global metric scale k (init = 1, since refine_scale already ran)
//
// Residuals (each scaled by 1/σ before Huber-weighting):
//   r1: reprojection error of plate corners, 4 corners × 2 cameras × N pairs
//       σ = 1.0 px after rectification, Huber δ = 2.0 px
//   r2: reprojection error of pipe-segment endpoints
//       σ = 2.0 px,                     Huber δ = 4.0 px (pipes noisier)
//   r3: plate-side prior |corner_i - corner_(i+1)| − 0.10 m
//       σ = 0.001 m  (very tight — the prior is the spec)
//   r4: section-base prior  min_z(section) = 0
//       σ = 0.005 m
//   r5: edge-orthogonality prior  cos(angle between adjacent edges) = 0
//       σ = 0.05
//
// Solver: dense LM with Jacobian by central differences (variables are
// few — typically <60 — so dense Jacobian is fine and cleanest). Stops
// when |Δχ²|/χ² < 1e-6 or 30 iterations.
//
// Effect: takes the post-Umeyama model and squeezes the last few
// millimetres of error out by jointly tuning all parameters. Critical
// when a few plate detections were noisy or partially occluded — the
// robust loss caps their influence so they don't pull the whole model.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"
#include "plate_detector.hpp"
#include "stereo_rectifier.hpp"
#include <vector>

namespace mate {

struct BundleAdjustConfig {
    double huber_px_plate    = 2.0;
    double huber_px_pipe     = 4.0;
    double sigma_plate_side  = 0.001;   // 1 mm
    double sigma_section_base= 0.005;   // 5 mm
    double sigma_orthogonal  = 0.05;
    int    max_iter          = 30;
    double tol_rel_chi2      = 1e-6;
};

struct BundleAdjustReport {
    int    iters_used   = 0;
    double chi2_initial = -1;
    double chi2_final   = -1;
    int    plates_used  = 0;
    int    pipes_used   = 0;
    bool   ok           = false;
};

// All inputs reference frames must be the same as Model3D's frame
// (i.e. left-rectified-cam frame after refine_scale ran). Modifies
// io_model in place.
BundleAdjustReport bundleAdjustModel(
    Model3D& io_model,
    const std::vector<RectifiedPair>& rects,
    const std::vector<std::vector<PlateDetection>>& plates_per_pair_left,
    const std::vector<std::vector<PlateDetection>>& plates_per_pair_right,
    const BundleAdjustConfig& cfg = {});

}  // namespace mate
