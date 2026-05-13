#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_parallel_pair.hpp — color-independent pipe detection.
//
// Why
//   The LAB+chroma path of pvc_segment relies on PVC being bright and
//   achromatic. In the user's lab-photo example (chalkboard background,
//   trophies, white tabletops, people in light clothing, low contrast)
//   that assumption shatters. We need a detector that doesn't care
//   about colour at all.
//
// Idea
//   A cylindrical pipe imaged on any background presents two parallel
//   silhouette edges. The signed-distance between the edges equals the
//   projected pipe diameter:
//
//       w_px = 2 · r_m · f / Z
//
//   For PVC (r_m ∈ [0.006, 0.030] m), f ≈ 700 px, Z ≈ 0.8 m we get
//   w_px ∈ [10, 50] px. Any pair of parallel line segments whose
//   perpendicular spacing falls in that band, with edge support of
//   *opposite* gradient signs (light→dark on one edge, dark→light on
//   the other — i.e. a ridge profile across the pair), is a pipe
//   candidate regardless of colour.
//
// Pipe-ness score
//   For each candidate pair (Lₐ, Lᵦ):
//     • parallelism            (angle diff) — must be ≤ angle_tol
//     • spacing                (perpendicular distance) — must be in
//                              [w_min_px, w_max_px]
//     • length agreement       (max/min len) — must be ≤ len_ratio_tol
//     • opposite gradient signs along the pair — Sobel projection
//     • interior uniformity    — std-dev of intensity on the band
//                              between Lₐ and Lᵦ should be low
//                              (a real pipe has a smooth body)
//     • exterior contrast      — band intensity should differ from
//                              outside-the-pair intensity (the pipe
//                              must *separate* from background)
//
//   Final score = weighted product; pairs above min_score emerge as
//   PipeSegment2D + a "pipe-ness" attribute usable by downstream
//   modules (treated as another voter in pipe_lines_multi).
//
// This is not a re-implementation of LSD; it OPERATES ON LSD/FLD/Hough
// outputs to find their pairings.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_lines_multi.hpp"
#include <opencv2/core.hpp>

namespace mate {

struct PipePairConfig {
    double angle_tol_deg     = 4.0;
    double w_min_px_override = 0.0;   // 0 = derive from (r_min_m, Z, f)
    double w_max_px_override = 0.0;
    double len_ratio_tol     = 1.5;
    double min_overlap_frac  = 0.5;   // fraction of shorter line that must
                                       // overlap when projected to the
                                       // longer line's axis
    double interior_max_std  = 35.0;  // 0..255 grayscale std cap inside
                                       // the band (uniform body)
    double exterior_min_dI   = 8.0;   // 0..255 mean intensity gap
    double min_score         = 0.40;  // accept threshold
};

struct PipePair {
    PipeSegment2D seg;       // medial line of the pair (mid-points)
    double        width_px      = 0.0;
    double        score         = 0.0;
    double        interior_std  = 0.0;
    double        exterior_diff = 0.0;
    int           src_a = -1;     // indices into the input line list
    int           src_b = -1;
};

// Find pipe candidates from PARALLEL EDGE PAIRS in the line list, using
// only image gradients — no colour mask required. The optional `mask`
// (e.g. PVC mask) is *not* required; if supplied it is used as a soft
// boost (pairs whose midline falls inside the mask get a confidence
// bonus) but not as a hard gate.
std::vector<PipePair>
detectPipeParallelPairs(const cv::Mat& bgr,
                         const std::vector<LinesMultiSegment>& lines,
                         double focal_px,
                         double subject_distance_Z,
                         double r_min_m, double r_max_m,
                         const cv::Mat& soft_mask = {},
                         const PipePairConfig& cfg = {});

}  // namespace mate
