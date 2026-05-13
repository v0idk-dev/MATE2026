#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_lsd.hpp — PVC-pipe segment detection via Line-Segment Detector.
//
// PVC pipes are the highest-contrast straight edges in the scene. The
// existing morphological pipe detector (pipe_detector.cpp) struggles
// with cluttered backgrounds (lab tables, chalkboards, books) because
// it works on global thresholding. LSD is a per-pixel, parameter-free,
// NFA-validated edge detector that yields sub-pixel line segments.
//
// Pipeline:
//   1. cv::createLineSegmentDetector(LSD_REFINE_STD).
//   2. Detect raw segments.
//   3. Reject segments shorter than `min_len_frac × image_width`.
//   4. Reject segments inside the timestamp-overlay band (top/bottom
//      `osd_band_frac` of the image — typical CCTV/dive-cam burnt-in
//      timestamps).
//   5. Cluster near-collinear segments (angle < `cluster_angle_deg`,
//      perpendicular distance < `cluster_dist_px`) into one PipeSegment2D.
//   6. Reject segments whose midpoint lies inside any plate bounding
//      polygon (caller passes them in).
//   7. Sort by length, keep top-K.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_detector.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PipeLSDConfig {
    double min_len_frac      = 0.05;   // of image width
    double cluster_angle_deg = 5.0;
    double cluster_dist_px   = 3.0;
    double osd_band_frac     = 0.06;   // top/bottom band to ignore
    int    max_segments      = 64;
};

// Detects pipe-like line segments. Pass in plate polygons so segments
// that overlap them get rejected (a plate isn't a pipe).
std::vector<PipeSegment2D>
lsdPipes(const cv::Mat& bgr,
         const std::vector<std::vector<cv::Point2f>>& plate_polygons = {},
         const PipeLSDConfig& cfg = {});

}  // namespace mate
