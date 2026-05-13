#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_graph.hpp — junction graph + 2D/3D connectivity gate.
//
// Real PVC structures form a graph: pipes meet at PVC fittings (tees,
// elbows, crosses). Spurious detections almost never satisfy this.
// We enforce:
//
//   • Each pipe endpoint must lie within `merge_radius_m` of at least
//     one other pipe endpoint (degree ≥ 1 at both ends).
//   • Each junction must also be visible as a 2D intersection in the
//     LEFT rectified view (project both endpoints, check pixel
//     proximity within `merge_radius_px`).
//
// Junctions are clustered with a KD-tree (cv::flann) in 3D. Cluster
// centres become junction nodes; each pipe stores its two junction
// indices. We then drop any pipe that touches a degree-0 cluster on
// either end (i.e. a pipe whose endpoint is alone in 3D space).
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_cylinder3d.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PipeGraphConfig {
    double merge_radius_m   = 0.030;   // 3 cm cluster tolerance
    double merge_radius_px  = 24.0;    // for 2D back-projection check
    bool   require_2d_check = true;
};

struct GraphJunction {
    cv::Point3d position;
    int         degree = 0;
    std::vector<int> pipe_indices;     // edges
};

struct GraphPipe {
    Cylinder3D  cyl;
    int         junction_a = -1;
    int         junction_b = -1;
};

struct PipeGraphResult {
    std::vector<GraphJunction> junctions;
    std::vector<GraphPipe>     pipes;
    int                        rejected_isolated = 0;
};

PipeGraphResult buildPipeGraph(const std::vector<Cylinder3D>& cylinders,
                                const cv::Mat& P1_3x4_left,
                                const PipeGraphConfig& cfg = {});

}  // namespace mate
