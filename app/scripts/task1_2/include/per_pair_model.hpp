#pragma once
// ─────────────────────────────────────────────────────────────────────────
// per_pair_model.hpp — step 4: build a rough 3-section Model3D from one
// stereo pair's 2-D detections + initial triangulations.
//
// Approach:
//   • Triangulate plate centers + corners (rectified pair → 3D).
//   • Triangulate pipe segment endpoints (rectified pair → 3D).
//   • PCA the resulting 3-D point cloud to find principal axes (long /
//     wide / up). Build the model frame from those.
//   • Cluster pipes by which "section" they likely belong to: project
//     pipe midpoints onto the long axis, run a 1-D k-means with k=3
//     (the spec calls out 3 sections). Each cluster becomes a Section
//     whose length is the cluster's extent along the long axis.
//   • Each Section's width is the median pipe extent along the wide axis
//     within its cluster; its height is the extent along the up axis.
//   • Plates are attached to the closest section + face by 3-D distance,
//     with (u, v) computed as the projection onto the face's tangents.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"
#include "plate_detector.hpp"
#include "pipe_detector.hpp"
#include "stereo_rectifier.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PerPairConfig {
    int n_sections = 3;          // user spec: 3 sections (different heights)
    double junction_merge_m = 0.05;
    bool snap_to_axes = true;
    double snap_max_deviation_deg = 25.0;
};

// Inputs are the *already-rectified* 2-D detections (in pixel coords) plus
// the rectified pair's projection matrices and disparity-to-depth matrix.
Model3D buildPerPairModel(const RectifiedPair& rect,
                          const std::vector<PlateDetection>& plates_left,
                          const std::vector<PlateDetection>& plates_right,
                          const std::vector<PipeSegment2D>& pipes_left,
                          const std::vector<PipeSegment2D>& pipes_right,
                          const PerPairConfig& cfg = {},
                          const std::string& unit_in = "m");

}  // namespace mate
