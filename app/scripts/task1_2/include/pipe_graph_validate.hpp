// pipe_graph_validate.hpp — post-fit sanity check on the assembled 3D
// pipe graph. Produces a list of warnings + an optional list of indices
// to drop. Never modifies the graph itself; the caller decides.
//
// Checks:
//   1. Length sanity        — pipes longer than `max_pipe_length_m` are
//                             almost certainly false matches (e.g. the
//                             whole rig fused into one segment).
//   2. Radius catalog snap  — PVC sold in the U.S. comes in a small set
//                             of nominal sizes. Snap each radius to the
//                             closest catalog entry and report residuals
//                             above `radius_outlier_m` (default 5 mm).
//   3. Connectivity         — number of connected components in the
//                             junction graph; > 1 means at least one
//                             pipe is floating.
//   4. Degree distribution  — junctions with degree > 4 are physically
//                             implausible for PVC fittings and probably
//                             reflect a merge-radius that is too large.
//
#pragma once
#include "pipe_graph.hpp"
#include <string>
#include <vector>

namespace mate {

struct PipeGraphValidateConfig {
    double max_pipe_length_m   = 1.5;     // hard cap (longest single pipe)
    double radius_outlier_m    = 0.005;   // 5 mm off catalog → flag
    int    max_junction_degree = 6;       // pragmatic upper bound
    // U.S. PVC nominal outer-radius catalog (m). Outer ≈ nominal + 0.0032.
    // 1/2", 3/4", 1", 1-1/4", 1-1/2", 2"
    std::vector<double> radius_catalog_m {
        0.0107, 0.0133, 0.0166, 0.0210, 0.0240, 0.0301
    };
};

struct PipeGraphValidateReport {
    int    num_pipes              = 0;
    int    num_junctions          = 0;
    int    num_connected_components = 0;
    int    num_long_pipes_flagged = 0;
    int    num_radius_outliers    = 0;
    int    max_observed_degree    = 0;
    std::vector<int>           drop_pipe_indices;
    std::vector<double>        snapped_radii_m;        // per-pipe
    std::vector<std::string>   warnings;
};

PipeGraphValidateReport
validatePipeGraph(const PipeGraphResult& g,
                  const PipeGraphValidateConfig& cfg = {});

}  // namespace mate
