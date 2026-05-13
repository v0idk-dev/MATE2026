#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_pipeline.hpp — single entry point that runs the entire pipe
// detection + 3D reconstruction stack on a rectified stereo pair.
//
// The pipeline (see MATH.md §7 and INTEGRATION.md §6 for derivations):
//
//   Per view (L and R) — independent, parallelisable:
//     1. segmentPvc()           → mask, distance, skeleton (LAB+chroma)
//     2. detectLinesMulti()     → ensemble LSD/FLD/Hough/skeleton vote
//     3. ransacRefitLines()     → MSAC re-fit on edge pixels
//     4. gateByDiameter()       → projected diameter gate (uses Z, f)
//
//   Joint (L↔R):
//     5. computeSgbmDisparity() → SGBM dense + WLS post-filter
//     6. matchPipesStereo()     → mean-disparity match + epipolar +
//                                  Sampson + radius gate, mutual-best
//     7. sampsonTriangulate()   → 3D endpoints (LM-refined ML estimator)
//     8. fitCylinderFromCloud() → RANSAC + LM 3D cylinder per pipe
//     9. buildPipeGraph()       → KD-tree junction cluster + degree gate
//    10. bundleAdjustPipes()    → joint LM over junctions+radii (Eigen)
//
// All numeric thresholds are auto-derived from (image dimensions,
// baseline B, auto-estimated focal f, auto-estimated subject distance Z).
// Nothing is hardcoded against scene-specific assumptions.
//
// Returns a list of validated 3D pipes plus the junction graph and per-
// step diagnostics suitable for the debug overlay.
// ─────────────────────────────────────────────────────────────────────────

#include "pvc_segment.hpp"
#include "sgbm_disparity.hpp"
#include "pipe_lines_multi.hpp"
#include "pipe_parallel_pair.hpp"
#include "pipe_pink_tape.hpp"
#include "pipe_ransac.hpp"
#include "pipe_diameter.hpp"
#include "pipe_match_stereo.hpp"
#include "pipe_cylinder3d.hpp"
#include "pipe_graph.hpp"
#include "pipe_bundle.hpp"
#include "pipe_template.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PipePipelineConfig {
    // Geometry (these come from depth_segment + stereo_math; never user-set).
    double focal_px           = 0.0;     // 0 → auto-estimate from img dims
    double baseline_m         = 0.10;    // user-supplied (cam-to-cam)
    double subject_distance_Z = 0.0;     // 0 → auto from SGBM median depth
    double subject_distance_Z_sigma = 0.0;

    // Pre-detection mask config (segmentPvc).
    PvcSegmentConfig       pvc;
    // SGBM.
    SgbmConfig             sgbm;
    // Multi-detector lines.
    LinesMultiConfig       lines;
    // RANSAC re-fit.
    PipeRansacConfig       ransac;
    // Diameter gate.
    PipeDiameterConfig     diameter;
    // Stereo matching.
    PipeMatchStereoConfig  match;
    // Cylinder fit.
    CylinderConfig         cylinder;
    // Graph.
    PipeGraphConfig        graph;
    // Bundle adjustment.
    PipeBundleConfig       bundle;
    // Color-independent parallel-edge-pair detector.
    PipePairConfig         parallel_pair;
    // Pink-tape marker detector.
    PinkTapeConfig         pink_tape;
    // Structural template + Procrustes-RANSAC fit.
    PipeTemplate           tmpl{ makeDefault3SectionTemplate() };
    TemplateFitConfig      tmpl_fit;

    // Toggles.
    bool run_sgbm           = true;
    bool run_cylinder       = true;
    bool run_graph          = true;
    bool run_bundle         = true;
    bool run_parallel_pair  = true;       // color-independent voter
    bool run_pink_tape      = true;       // landmark anchors
    bool run_template_fit   = true;       // snap to canonical 3-section
    bool run_inject_predicted = true;     // synthesise occluded pipes
    bool run_image_quality_check = true;  // pre-flight blur/exposure gate
    bool fail_on_image_quality   = false; // if true → abort; else warn-only
    bool run_graph_validate      = true;  // post-fit sanity check
};

struct PipePipelineDiag {
    int   pvc_pixels_left = 0,  pvc_pixels_right = 0;
    double pvc_radius_px_left = 0, pvc_radius_px_right = 0;
    int   raw_lines_left  = 0,  raw_lines_right  = 0;
    int   ransac_kept_left= 0,  ransac_kept_right= 0;
    int   diam_kept_left  = 0,  diam_kept_right  = 0;
    int   sgbm_disparities_used = 0;
    double sgbm_pct_valid = 0.0;
    int   matches         = 0;
    int   cylinders_ok    = 0;
    int   graph_pipes     = 0;
    int   graph_junctions = 0;
    int   graph_rejected_isolated = 0;
    PipeBundleReport bundle_report{};
    int   parallel_pairs_left  = 0, parallel_pairs_right = 0;
    int   pink_blobs_left      = 0, pink_blobs_right     = 0;
    bool  template_fit_ok      = false;
    int   template_inliers     = 0;
    double template_rms_m      = 0.0;
    double template_scale      = 1.0;
    cv::Vec3d template_scale_axis = {1,1,1};
    InjectionReport injection{};
    // Pre-flight image-pair quality (blur, exposure, clipping, size).
    bool   iqc_pass             = true;
    double iqc_blur_var_left    = 0, iqc_blur_var_right = 0;
    double iqc_mean_lum_left    = 0, iqc_mean_lum_right = 0;
    double iqc_clip_frac_left   = 0, iqc_clip_frac_right = 0;
    std::vector<std::string> iqc_warnings;
    // Post-fit graph sanity check.
    int    graph_long_pipes_flagged = 0;
    int    graph_radius_outliers    = 0;
    int    graph_components         = 0;
    int    graph_max_degree         = 0;
    std::vector<std::string> graph_warnings;
    long long ms_per_step[14] = {};      // 0=IQC, 1..13 main stages
};

struct PipePipelineOutput {
    PipeGraphResult        graph;          // final structure (post-template-injection)
    PipeGraphResult        graph_pre_tmpl; // structure before template injection
    std::vector<PipeMatch> matches;        // for visualisation
    std::vector<Cylinder3D> cylinders;     // pre-graph (for debug)
    cv::Mat                sgbm_disparity; // for visualisation
    cv::Mat                pvc_mask_left;
    cv::Mat                pvc_mask_right;
    std::vector<PipePair>  parallel_pairs_left, parallel_pairs_right;
    std::vector<PinkBlob>  pink_blobs_left,     pink_blobs_right;
    TemplateFitResult      template_fit;   // fitted similarity, if any
    PipePipelineDiag       diag;
    std::string            warning;
    std::string            error;
};

// `Lr`, `Rr` MUST be rectified. `P1`, `P2`, `Q` correspond to that rect.
PipePipelineOutput runPipePipeline(const cv::Mat& Lr, const cv::Mat& Rr,
                                    const cv::Mat& P1, const cv::Mat& P2,
                                    const cv::Mat& Q,
                                    const PipePipelineConfig& cfg);

}  // namespace mate
