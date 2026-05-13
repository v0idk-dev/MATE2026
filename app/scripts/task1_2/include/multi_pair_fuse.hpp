#pragma once
// ─────────────────────────────────────────────────────────────────────────
// multi_pair_fuse.hpp — step 7: combine N per-pair Model3Ds into one.
//
// We don't have inter-pair viewpoint info, so we register pairs against
// each other purely by their *plate constellation*: for each pair, build
// a feature vector of pairwise plate distances, then find the rotation
// (about +Z, since both models share the gravity convention) and
// translation that best aligns each pair to the first.
//
// After alignment:
//   • Section dimensions are taken as the median across pairs (robust to
//     a single bad pair).
//   • Plate (u, v) on each face is averaged across pairs that agree on
//     the same plate ID.
//   • Confidence is the geometric mean of per-pair confidences and the
//     inverse of the registration residual.
//
// If only one pair is supplied, the output is just that pair's model
// with `n_pairs_used = 1`.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"
#include <vector>

namespace mate {

struct FuseConfig {
    // Threshold below which a registration residual is "good".
    double max_residual_m = 0.15;
    // If we cannot register a pair within this threshold, drop it.
    bool drop_outlier_pairs = true;
};

Model3D fuseModels(const std::vector<Model3D>& per_pair,
                   const FuseConfig& cfg = {});

}  // namespace mate
