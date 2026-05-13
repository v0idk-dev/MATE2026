#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_bundle.hpp — Eigen-only LM bundle adjustment over the pipe
// junction graph.
//
// Variables (X):
//   • Junction positions   J × 3
//   • Per-pipe radius      P × 1   (constrained ≥ 0 via squared substitute)
//
// Residuals:
//   r_a (4 per pipe per view): endpoint reprojection error (Lₐ, Lᵦ in L
//        view; Rₐ, Rᵦ in R view). σ = 1.5 px, Huber δ = 3 px.
//   r_b (1 per pipe): radius prior — radius must lie in [0.006, 0.030] m
//        with σ = 0.005 m, soft.
//   r_c (1 per pipe pair sharing junction): orthogonality / parallelism
//        prior, encouraged for known PVC fittings (T = 90°, elbow = 90°,
//        straight = 180°). σ = 0.10 (rad).
//   r_d (1 per junction): junction-position prior to its initial 3D
//        cluster centre, σ = 0.005 m, prevents drift.
//
// Solver: dense LM with central-difference Jacobian (problem size is
// tiny — typically 20 junctions × 3 + 25 pipes × 1 ≈ 85 unknowns).
// Levenberg damping τ = 1e-3, λ multiplier 10/0.1, max 60 iterations.
//
// This is the same shape as bundle_adjust.hpp (which targets plates +
// section poses), but the variable set is junctions+radii so the two
// can be composed: after pipe_bundle runs, bundle_adjust can run with
// junctions held fixed.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_graph.hpp"
#include <opencv2/core.hpp>

namespace mate {

struct PipeBundleConfig {
    double sigma_px            = 1.5;
    double huber_px            = 3.0;
    double sigma_radius_m      = 0.005;
    double sigma_orthogonal    = 0.10;       // radians
    double sigma_junction_prior_m = 0.005;
    int    max_iter            = 60;
    double tol_rel_chi2        = 1e-7;
};

struct PipeBundleReport {
    int    iters_used   = 0;
    double chi2_initial = -1;
    double chi2_final   = -1;
    bool   ok           = false;
    int    n_junctions  = 0;
    int    n_pipes      = 0;
};

// `P1` and `P2` are the 3×4 projection matrices of the LEFT and RIGHT
// rectified cameras (in metres). `io_graph` is updated in-place.
PipeBundleReport bundleAdjustPipes(PipeGraphResult& io_graph,
                                    const cv::Mat& P1, const cv::Mat& P2,
                                    const PipeBundleConfig& cfg = {});

}  // namespace mate
