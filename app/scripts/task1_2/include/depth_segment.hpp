#pragma once
// ─────────────────────────────────────────────────────────────────────────
// depth_segment.hpp — stereo "closeness heatmap" → foreground mask +
//                     SUBJECT-DISTANCE auto-estimator.
//
// CORRECTED API: the user-supplied input is `baseline_m` (B, distance
// between the two camera lenses). The subject distance Z (camera to
// coral garden) is computed from the disparity map itself, never asked
// of the user.
//
// Pipeline:
//   1. SGBM dense disparity (semi-global matching — robust to low-texture
//      surfaces like white PVC).
//   2. (Optional) WLS edge-preserving disparity filter when the OpenCV
//      ximgproc contrib is available.
//   3. Convert to "closeness" map (0=far, 255=near) for visualisation.
//   4. Foreground threshold: top `fg_percentile` of valid disparities.
//      RELATIVE depth ordering is reliable even when calibration scale
//      is wrong, so this works without a metric prior.
//   5. If a Q matrix is available (or can be synthesised from baseline +
//      intrinsics), reproject to metric depth in metres.
//   6. Auto-estimate `subject_distance_m_est = median(Z) over FG`.
//   7. Morphological cleanup + drop tiny blobs.
//
// Returned to caller:
//   • disp_int16            — raw SGBM disparity (CV_16S)
//   • closeness_u8          — 8U heatmap (visualisation)
//   • foreground_u8         — 8U binary FG mask
//   • depth_m_32f           — 32F per-pixel depth in metres (NaN where invalid)
//   • subject_distance_m_est — median FG depth (the auto-estimated Z)
//   • fg_area_frac          — fraction of image covered by FG
//
// Used downstream to:
//   • Mask the rectified images BEFORE plate / pipe detectors run.
//   • Pass `subject_distance_m_est` to lab_segment / vision_rectangles
//     so they can compute the correct expected plate pixel size via
//     s_px = f · s_m / Z.
//   • Reject any plate/pipe detection whose median pixel depth lies
//     outside [Z_est − band_m, Z_est + band_m].
// ─────────────────────────────────────────────────────────────────────────

#include "stereo_rectifier.hpp"
#include <opencv2/core.hpp>

namespace mate {

struct DepthSegmentConfig {
    int    sgbm_min_disp     = 0;
    int    sgbm_num_disp     = 128;     // must be %16==0
    int    sgbm_block_size   = 5;
    int    sgbm_uniqueness   = 10;
    int    sgbm_speckle_win  = 100;
    int    sgbm_speckle_rng  = 32;
    int    sgbm_pre_filter   = 31;

    double fg_percentile     = 0.45;    // top 55 % of "closeness" = FG
    int    morph_close_px    = 9;
    double min_blob_frac     = 0.005;
    bool   use_wls_filter    = true;

    // Used ONLY to construct a synthetic Q matrix when rect.Q is empty.
    // Set to the user's `baseline_m`. Z is auto-estimated, never input.
    double baseline_m_for_q  = 0.10;

    // OSD-band rejection: zero out disparity in the top/bottom band
    // (typical burnt-in CCTV/dive-cam timestamp height).
    double osd_band_frac     = 0.06;
};

struct DepthSegmentResult {
    cv::Mat disp_int16;             // raw SGBM disparity (CV_16S)
    cv::Mat closeness_u8;           // 0=far 255=near (visualisation)
    cv::Mat foreground_u8;          // binary mask, 0/255
    cv::Mat depth_m_32f;            // metric depth (NaN where invalid)

    double  subject_distance_m_est = -1.0;   // median FG depth — pass to detectors
    double  fg_area_frac           = 0.0;
    double  median_disparity_px    = -1.0;   // for QC
    double  baseline_m_used        = 0.0;    // what we used in Q
    double  focal_px_used          = 0.0;    // what we used in Q
    bool    ok                     = false;
};

// Run depth-based foreground segmentation.
//
//   `rect`         — rectified pair. If rect.Q is empty, a synthetic Q is
//                    built from `baseline_m` and intrinsics in rect.
//   `baseline_m`   — user-supplied stereo baseline B (metres).
//                    NOT the subject distance.
DepthSegmentResult segmentForegroundByDepth(
    const RectifiedPair& rect,
    double baseline_m,
    const DepthSegmentConfig& cfg = {});

// Predicate: median depth over `roi` lies inside [Z_est − band_m, Z_est + band_m].
// Returns true if `depth_m_32f` is empty or `subject_distance_m_est` ≤ 0
// (i.e., no prior to reject against).
bool detectionInDepthBand(const cv::Mat& depth_m_32f,
                          const cv::Rect& roi,
                          double subject_distance_m_est,
                          double band_m = 0.5);

}  // namespace mate
