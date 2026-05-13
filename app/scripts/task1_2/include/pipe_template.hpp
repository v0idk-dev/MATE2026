#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_template.hpp — parametric structural prior + Umeyama registration.
//
// The MATE 2026 coral-garden rig is a 3-section axis-aligned cuboid
// wireframe. Knowing the topology a-priori lets us:
//
//   1. Reject implausible reconstructions (a 4th detected junction floating
//      in space far from any template node is almost certainly noise).
//   2. Inject "predicted" pipes when the corresponding template edge has
//      no detected match — so we still draw the full rig under heavy
//      occlusion (the user's lab-photo example: chalkboard, people,
//      shelves all blocking half the structure).
//   3. Give every dimension reading a Bayesian prior pulling it toward
//      a feasible value, suppressing per-pipe noise.
//
// The template is *parametric*, NOT hardcoded:
//   • 3 sections with editable (W_i, H_i, D) per section.
//   • Default seed values reflect typical 1/2"–2" PVC structures, but
//     Umeyama auto-rescales to fit the observed metric junctions.
//   • Topology (which corner connects to which) is fixed; coordinates
//     are fit.
//
// Per the user's note: ONLY axis-aligned vertical + horizontal pipes
// (no cross-bracing diagonals — those exist in the schematic drawing
// but not the physical rig).
//
// References
//   • Umeyama 1991 — "Least-squares estimation of transformation
//     parameters between two point patterns."  Closed-form similarity.
//   • Procrustes RANSAC for graph correspondence under unknown labelling.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_graph.hpp"
#include <opencv2/core.hpp>
#include <vector>
#include <string>

namespace mate {

// ─── Template definition ─────────────────────────────────────────────

struct TemplateSection {
    double W = 0.40;  // X-extent (metres)
    double H = 0.40;  // Y-extent (metres, "height" — Y is up)
    // shared D across all sections (depth, Z-extent) makes the rig flat
    // in the depth direction, matching typical MATE structures.
};

struct TemplateNode {
    cv::Point3d  p;             // canonical 3D position
    int          degree_topo;   // topological degree (# edges meeting)
    std::string  label;         // human-readable, e.g. "L_top_back_R"
};

struct TemplateEdge {
    int          a, b;          // node indices
    char         axis;          // 'X', 'Y', or 'Z' (axis-aligned only)
    double       length_m;      // canonical length
};

struct PipeTemplate {
    std::vector<TemplateNode> nodes;
    std::vector<TemplateEdge> edges;
    double                    D = 0.30;       // shared depth
    std::vector<TemplateSection> sections;    // per-section dims (3 by default)
};

// Default canonical 3-section template: L (low) ─ M (tall) ─ R (low),
// vertical and horizontal pipes only, sections share inner-face bottom
// edges. Seed dimensions are typical for MATE coral rigs but the
// Umeyama fit will rescale them to whatever the observation says.
PipeTemplate makeDefault3SectionTemplate(double W_side  = 0.40,
                                          double W_mid   = 0.30,
                                          double H_side  = 0.40,
                                          double H_mid   = 0.60,
                                          double D       = 0.30);

// ─── Registration ────────────────────────────────────────────────────

struct TemplateFitConfig {
    int    ransac_iters     = 500;
    double inlier_tol_m     = 0.05;        // 5 cm node-snap tolerance
    int    min_correspondences = 4;        // min RANSAC sample size
    bool   allow_anisotropic_scale = true; // fit (sx, sy, sz) per axis
                                            // after Umeyama
    bool   inject_missing_pipes    = true; // emit predicted pipes
    double missing_pipe_confidence = 0.30; // confidence for predicted
};

// 7-DOF similarity (rotation + translation + scalar scale), or 9-DOF
// if allow_anisotropic_scale (3 per-axis scales).
struct TemplateFitResult {
    cv::Matx33d R;             // rotation
    cv::Vec3d   t;             // translation
    double      s = 1.0;       // isotropic scale
    cv::Vec3d   s_axis = {1,1,1}; // per-axis scale (when anisotropic)
    double      rms_m  = 0.0;
    int         inliers = 0;
    int         total   = 0;
    bool        ok      = false;
    // For each detected junction j: index of its matched template node,
    // or -1 if outlier.
    std::vector<int> assignment;
};

// Fit the template to the detected junction set using Procrustes RANSAC.
// Strategy:
//   1. Bucket detected junctions by topological degree (1, 2, 3, 4+).
//   2. RANSAC: sample a small set of correspondences whose degrees match
//      template node degrees, solve Umeyama (1991) for closed-form
//      similarity, score by sum-of-squares of all detected junctions
//      against the nearest projected template node within inlier_tol.
//   3. (Optional) Per-axis scale refinement via constrained LSQ.
TemplateFitResult fitTemplate(const PipeGraphResult& graph,
                              const PipeTemplate&   tmpl,
                              const TemplateFitConfig& cfg = {});

// Map a template point through the fitted similarity: x' = s · R·diag(s_axis)·x + t.
cv::Point3d applyTemplateFit(const TemplateFitResult& fit, const cv::Point3d& x);

// Inject "predicted" pipes for every template edge whose endpoints, after
// applying the fit, are not within tol of any detected pipe endpoint pair.
// Returned graph contains: original (detected) pipes UNCHANGED, plus
// extra GraphPipe entries marked with cyl.confidence = cfg.missing_pipe_confidence
// and a cyl.ok = true status. Template-snapped junctions are appended.
struct InjectionReport {
    int n_detected_pipes  = 0;
    int n_predicted_pipes = 0;
    int n_template_edges  = 0;
};
PipeGraphResult injectPredictedPipes(const PipeGraphResult& detected,
                                      const PipeTemplate&    tmpl,
                                      const TemplateFitResult& fit,
                                      const TemplateFitConfig& cfg,
                                      InjectionReport*  report = nullptr);

}  // namespace mate
