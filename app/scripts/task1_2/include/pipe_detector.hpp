#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_detector.hpp — find PVC pipe segments in a single rectified image.
//
// PVC pipes are bright, edge-rich, mostly straight, and rectilinear. We
// use:
//   1. Canny edge detection (with auto-tuned thresholds via Otsu).
//   2. Probabilistic Hough lines.
//   3. Merge collinear/overlapping lines (NMS by angle + perpendicular
//      distance).
//   4. Discard segments shorter than a minimum length (in pixels).
//
// Cross-frame matching for stereo is NOT done here; it lives in
// pipe_detector.cpp's pairPipesByEpipolarOrder() helper, called from
// main.cpp after running detect() on both rectified frames.
//
// Pipe diameter detection (for the cylinder-fitting / sanity-check pass)
// is also a future enhancement; step 4 reports endpoint pairs only.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PipeDetectorConfig {
    // Canny low/high are computed via Otsu unless overridden positive.
    int canny_low_override  = 0;
    int canny_high_override = 0;
    // HoughLinesP parameters
    int hough_threshold = 60;
    double hough_rho = 1.0;
    double hough_theta_deg = 1.0;
    // Minimum line length as a fraction of min(image dimension)
    double min_length_frac = 0.06;
    double max_line_gap_px = 12.0;
    // Merge thresholds: lines whose angles agree within this and whose
    // perpendicular distances agree within this are fused.
    double merge_angle_deg = 6.0;
    double merge_perp_px   = 12.0;
    // Reject suspiciously diagonal lines? PVC structure is rectilinear
    // (vertical + horizontal). We don't enforce this in detection because
    // tilted-camera shots produce diagonals; later wireframe_builder snaps
    // angles. Setting reject_diagonal_deg to a positive value (e.g. 25)
    // disables non-rectilinear lines at detection time.
    double reject_diagonal_deg = 0.0;
};

struct PipeSegment2D {
    cv::Point2f p0;
    cv::Point2f p1;
    double angle_deg = 0.0;       // [-90, 90]
    double length_px = 0.0;
    int    id        = -1;        // optional per-detector unique tag
};

class PipeDetector {
public:
    explicit PipeDetector(const PipeDetectorConfig& cfg = {}) : cfg_(cfg) {}

    void setConfig(const PipeDetectorConfig& cfg) { cfg_ = cfg; }
    const PipeDetectorConfig& config() const { return cfg_; }

    std::vector<PipeSegment2D> detect(const cv::Mat& bgr) const;

    // Annotate (clones bgr).
    cv::Mat visualize(const cv::Mat& bgr,
                      const std::vector<PipeSegment2D>& segs) const;

private:
    PipeDetectorConfig cfg_;
};

// After detecting pipes in left and right RECTIFIED frames, pair them by
// y-coordinate proximity (since rectified pairs share epipolar y) and
// orientation similarity. Returns parallel index pairs (left_index,
// right_index). Unmatched pipes are dropped.
std::vector<std::pair<int, int>>
pairPipesByEpipolarOrder(const std::vector<PipeSegment2D>& L,
                         const std::vector<PipeSegment2D>& R,
                         double max_y_diff_px = 18.0,
                         double max_angle_diff_deg = 8.0);

}  // namespace mate
