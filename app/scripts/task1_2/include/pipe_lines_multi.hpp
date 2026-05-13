#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_lines_multi.hpp — multi-detector line extraction with fusion.
//
// We never trust a single line detector. Instead we run several and
// require corroboration ("ensemble voting"):
//   • LSD (cv::createLineSegmentDetector) — sub-pixel, NFA-validated.
//   • FastLineDetector (cv::ximgproc) when available — faster, similar
//     quality (graceful skip if ximgproc not linked).
//   • HoughLinesP on a Canny edge map masked by the PVC mask.
//   • Skeleton tracing — walk the medial-axis skeleton from segmentPvc()
//     and fit straight runs by Douglas-Peucker.
//
// Each detector emits raw segments; we restrict them to the PVC mask,
// then cluster co-linear ones across detectors (DBSCAN-like in (angle,
// perpendicular-offset) space). A cluster needs at least `min_votes`
// detectors to survive — typical setting (votes = 2) eliminates the
// shadow lines, table edges, and chalkboard marks that any single
// detector finds.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_detector.hpp"  // PipeSegment2D
#include <opencv2/core.hpp>

namespace mate {

struct LinesMultiConfig {
    double min_len_frac     = 0.05;   // of min(W,H)
    double cluster_angle_deg= 5.0;
    double cluster_perp_px  = 4.0;
    int    min_votes        = 2;      // detectors that must agree
    double osd_band_frac    = 0.06;
    int    max_segments     = 96;

    // Per-detector enable flags (graceful no-ops if not available).
    bool   use_lsd          = true;
    bool   use_fld          = true;   // requires opencv_ximgproc
    bool   use_hough        = true;
    bool   use_skeleton     = true;
};

struct LinesMultiSegment {
    PipeSegment2D seg;
    int           votes = 0;       // how many detectors found it
    double        radius_px = 0.0; // median of distance-transform along seg
};

// `mask` and `dist_px` come from segmentPvc(). `skeleton` is optional;
// if empty the skeleton-trace voter is skipped automatically.
std::vector<LinesMultiSegment>
detectLinesMulti(const cv::Mat& bgr,
                 const cv::Mat& mask,
                 const cv::Mat& dist_px,
                 const cv::Mat& skeleton,
                 const std::vector<std::vector<cv::Point2f>>& plates_to_exclude = {},
                 const LinesMultiConfig& cfg = {});

}  // namespace mate
