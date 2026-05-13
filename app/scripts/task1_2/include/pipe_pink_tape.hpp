#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipe_pink_tape.hpp — adaptive pink-tape marker detection.
//
// Pink electrical tape on the MATE rig is the most reliable colour cue
// in the entire scene: it is a unique, isolated, high-saturation hue
// that the surrounding lab clutter cannot fake. Even when half the
// rig is occluded, the visible pink-tape patches anchor the
// reconstruction.
//
// We DON'T use a fixed HSV range — that's exactly the kind of brittle
// hardcoding the user objects to. Instead:
//
//   1. Compute LAB.
//   2. Build a "pinkness" score per pixel:
//        a* > 0  (red side of green-red axis)
//        |b*| < |a*|  (more red than yellow/blue)
//        L*   > L*_min (avoid black)
//   3. Threshold via Otsu on the score → blob candidates.
//   4. For each blob: check shape (rectangle-ish), area, centroid stability.
//   5. Output centroids in pixel coords with a per-blob confidence.
//
// These centroids feed the template fitter as high-confidence landmark
// observations: each pink-tape blob is a known marker on the rig (e.g.
// "front-left vertical post"), which can be triangulated stereoscopically
// and used as anchor correspondences for the Procrustes RANSAC.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PinkTapeConfig {
    int    L_min            = 40;       // ignore very dark pinks (shadow)
    double min_area_frac    = 5e-5;     // of image area
    double max_area_frac    = 5e-3;
    double aspect_max       = 6.0;      // tape is rectangle-ish
    int    morph_kernel     = 0;        // 0 = auto = max(3, min/300)
};

struct PinkBlob {
    cv::Point2f centroid;
    cv::Rect    bbox;
    int         area_px       = 0;
    double      pinkness_mean = 0.0;
    double      confidence    = 0.0;
};

std::vector<PinkBlob> detectPinkTape(const cv::Mat& bgr,
                                       cv::Mat* debug_mask = nullptr,
                                       const PinkTapeConfig& cfg = {});

}  // namespace mate
