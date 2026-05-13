#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pvc_segment.hpp — chromaticity/lightness segmentation for PVC pipe.
//
// PVC is bright (high L*) and nearly achromatic (a*² + b*² small). We
// build a mask of "PVC-likely" pixels purely from image content — no
// hardcoded HSV ranges, thresholds adapt to scene statistics.
//
// Outputs (all aligned to the input image):
//   • mask        : CV_8U {0,255}                                 PVC pixels
//   • dist_px     : CV_32F                                        distance-to-non-PVC, in pixels
//                                                                 = local pipe radius estimate
//   • skeleton    : CV_8U {0,255}                                 1-px medial axis
//
// `dist_px[y,x]` doubles as a per-pixel pipe-radius estimate (Felzenszwalb
// distance transform): once we know subject distance Z and focal f, pipe
// metric radius is r_m(y,x) = dist_px(y,x) · Z / f. This is the core
// signal that lets us reject non-pipe lines whose projected thickness
// doesn't match a 0.012–0.055 m PVC pipe. Adaptive thresholding via
// Otsu-on-L* and MAD-based chroma cutoff means the mask works at any
// exposure / white-balance.
//
// The skeleton is computed via Zhang-Suen iterative thinning (no OpenCV
// dependency on ximgproc — same implementation as scikit-image's
// `morphology.skeletonize`).
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>

namespace mate {

struct PvcSegmentConfig {
    // L* lower bound. Default 0 → use Otsu on L* channel automatically.
    int    L_min_override = 0;
    // Maximum chroma (sqrt(a*² + b*²)) for "achromatic". Default 0 → use
    // MAD-based statistics on the L*-thresholded pixels.
    double chroma_max_override = 0.0;
    // Morphology kernel size (px). 0 → max(3, image_min_dim/300).
    int    morph_kernel_override = 0;
    // Minimum connected-component area (frac of image area).
    double min_cc_area_frac = 1e-4;
    // Skip the top/bottom OSD bands (timestamp burn-ins) so they cannot
    // contaminate the chroma statistics.
    double osd_band_frac    = 0.06;
    // Compute distance transform? (cheap; on by default)
    bool   compute_distance = true;
    // Compute skeleton? (more expensive; on by default)
    bool   compute_skeleton = true;
};

struct PvcSegmentResult {
    cv::Mat mask;       // CV_8U {0,255}
    cv::Mat dist_px;    // CV_32F  (empty if !compute_distance)
    cv::Mat skeleton;   // CV_8U {0,255} (empty if !compute_skeleton)
    int     L_min_used      = 0;
    double  chroma_max_used = 0.0;
    int     pixels_kept     = 0;
    double  median_radius_px = 0.0;   // median of dist_px on the skeleton
};

PvcSegmentResult segmentPvc(const cv::Mat& bgr,
                             const PvcSegmentConfig& cfg = {});

}  // namespace mate
