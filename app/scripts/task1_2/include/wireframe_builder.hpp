#pragma once
// ─────────────────────────────────────────────────────────────────────────
// wireframe_builder.hpp — turn 3D pipe segments into a junction graph
// with global length and height annotations.
//
// Inputs (per pipe segment):
//   • Two 3D endpoints (already triangulated and possibly underwater-
//     corrected and globally rescaled).
//
// What we do:
//   1. Cluster endpoints across all pipes: any two endpoints within
//     `junction_merge_distance` are fused into a single junction node.
//     This is what turns "8 disjoint line segments" into a graph where
//     pipes share endpoints.
//   2. Replace each pipe's endpoints with the junction it was assigned.
//   3. Optionally snap pipe directions to PCA principal axes (the coral
//     garden is rectilinear so its three dominant axes form a basis;
//     after PCA we project each pipe onto the closest axis, eliminating
//     small rotational noise).
//   4. Compute scene length = max distance between any two junctions
//     along the principal axis (the "long" direction).
//   5. Compute scene height = max perpendicular extent above the base
//     plane (best-fit plane through the lowest junctions).
//
// All results live in calibration-unit space (typically cm). The caller
// is expected to know the unit and surface it via the SceneOutput.unit
// field.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PipeSegment3D {
    cv::Point3f a;
    cv::Point3f b;
    int junction_a = -1;        // filled after build()
    int junction_b = -1;
    double length = 0.0;        // in calibration units
};

struct WireframeBuilderConfig {
    // Endpoints within this distance of each other (in calibration units)
    // get fused into the same junction node. Default ~ 5 cm in the cm
    // unit system, which matches the typical PVC tee/elbow size.
    double junction_merge_distance = 5.0;
    // Snap pipe directions to PCA axes? Off by default; turn on when the
    // structure is known to be rectilinear (PVC is).
    bool snap_to_axes = true;
    // Reject snapping if the segment's deviation from every axis exceeds
    // this many degrees (means it's truly diagonal and shouldn't be snapped).
    double snap_max_deviation_deg = 25.0;
};

struct WireframeJunction {
    cv::Point3f position;
    int degree = 0;             // number of pipes incident
};

struct Wireframe {
    std::vector<WireframeJunction> junctions;
    std::vector<PipeSegment3D>     pipes;
    double length = 0.0;        // scene principal-axis extent
    double height = 0.0;        // perpendicular extent above base plane
    cv::Point3f principal_axis{1.0f, 0.0f, 0.0f};
    cv::Point3f up_axis{0.0f, 1.0f, 0.0f};
    cv::Point3f base_plane_normal{0.0f, 1.0f, 0.0f};
    double base_plane_offset = 0.0;
};

// Build the wireframe from raw 3D pipe segments. `pipes_inout` is
// modified in place: each pipe's junction_a/junction_b indices get filled
// in.
Wireframe
buildWireframe(std::vector<PipeSegment3D>& pipes_inout,
               const WireframeBuilderConfig& cfg = {});

}  // namespace mate
