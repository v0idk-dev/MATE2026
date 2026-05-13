#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_cylinder3d.hpp — RANSAC + LM 3D right-circular-cylinder fit.
//
// For every matched pipe, we collect dense disparity points whose 2D
// projection lies inside the line's PVC mask in the LEFT rectified view.
// Reproject through Q to 3D → cloud {Xi}. We then fit a cylinder
// parameterised as (axis_dir d̂, axis_point p₀, radius r) by:
//
//   Stage 1 — RANSAC:
//     Sample 2 points → axis = (x₂ − x₁)/‖·‖. Project all points onto
//     the plane perpendicular to axis through their centroid.
//     2D-circle-fit (algebraic Pratt) → centre & radius. Score by
//     #{i : | dist(Xi, cyl) | < tol}.
//
//   Stage 2 — Levenberg-Marquardt:
//     5 free params (axis dir as 2 angles + 3 axis-point coords; we
//     constrain axis_point to lie in the plane through cloud-centroid
//     perpendicular to d̂, removing the 1-D gauge freedom along the
//     axis). Cost = Σ (‖proj(Xi - p₀)‖ − r)² with Huber weighting.
//     Closed-form Jacobian, dense solver (5 unknowns).
//
//   Stage 3 — length & extent:
//     Project all inliers onto the axis → 1D values. length = max-min.
//     Endpoints in 3D = p₀ + (max·d̂), p₀ + (min·d̂).
//
// Output is a ready-to-render cylinder: centre, axis (unit vector),
// radius (metres), length (metres), and an inlier 3D point set for
// downstream bundle adjustment.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_match_stereo.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct CylinderConfig {
    int    ransac_iters        = 500;
    double ransac_inlier_tol_m = 0.005;   // 5 mm
    int    lm_max_iter         = 30;
    double lm_huber_m          = 0.003;   // 3 mm
    int    min_inliers         = 50;
    double max_radius_m        = 0.040;   // hard cap: 40 mm radius
};

struct Cylinder3D {
    cv::Point3d center;     // midpoint of pipe along axis
    cv::Vec3d   axis;       // unit vector
    double      radius_m   = 0.0;
    double      length_m   = 0.0;
    cv::Point3d endpoint_a; // p₀ + (min·d̂)
    cv::Point3d endpoint_b; // p₀ + (max·d̂)
    int         inliers    = 0;
    double      rms_m      = 0.0;
    double      confidence = 0.0;     // 0..1
    bool        ok         = false;
};

// `cloud_full` is the entire SGBM cloud for the pair; the function
// internally selects points whose left-pixel projection lies in the
// match's line mask. `mask_left` = PVC mask of the LEFT view (CV_8U).
Cylinder3D fitCylinderFromCloud(const PipeMatch& match,
                                const std::vector<struct CloudPoint>& cloud_full,
                                const cv::Mat& mask_left,
                                const cv::Mat& Q_left_to_3d,
                                int img_w, int img_h,
                                const CylinderConfig& cfg = {});

}  // namespace mate
