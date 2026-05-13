#pragma once
// ─────────────────────────────────────────────────────────────────────────
// refine_scale.hpp — replace stereo-baseline scale with plate-prior scale.
//
// Given an existing Model3D (built by per_pair_model.cpp from disparity),
// and a set of PnP poses for the visible plates (from plate_pnp.cpp),
// compute the optimal uniform scale factor k* that, when applied to the
// model, makes the modeled plate-center positions match the PnP-derived
// positions in a least-squares sense (after a rigid-body alignment).
//
// k* = argmin_k Σ_i || k · m_i  −  R · p_i + t ||²
//
// where m_i are the model's plate centers (in model frame) and p_i are
// the PnP plate centers (in camera frame), with (R, t) the rigid
// alignment found by Procrustes. We compute (R, t, k) jointly via
// closed-form Umeyama similarity alignment, then return:
//
//   • k         — scale factor to apply to all model lengths
//   • rms_m     — alignment residual in meters (a quality metric)
//   • used_n    — number of plate correspondences used
//
// Effect: any 8-px stereo-RMS error in baseline / extrinsics gets
// REPLACED by the much smaller intrinsic-derived plate-prior scale.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"
#include "plate_pnp.hpp"
#include <vector>

namespace mate {

struct ScaleRefinement {
    double k        = 1.0;
    double rms_m    = -1.0;
    int    used_n   = 0;
    bool   ok       = false;
};

// Map plate id → PnP pose in some camera frame (left rectified by
// convention). Only ids present in BOTH the model and the map are used.
ScaleRefinement refineModelScaleFromPlatePriors(
    Model3D& io_model,
    const std::vector<std::pair<int /*plate_id*/, cv::Vec3d /*pnp_center_cam*/>>& priors);

}  // namespace mate
