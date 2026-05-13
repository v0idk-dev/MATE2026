#pragma once
// ─────────────────────────────────────────────────────────────────────────
// plate_pnp.hpp — IPPE-square PnP for known-size plates.
//
// Why this is the single biggest accuracy lever for Task 1.2:
//
//   The plates are KNOWN to be 10 cm × 10 cm planar squares.
//   With sub-pixel corners + camera intrinsics, cv::solvePnP using
//   SOLVEPNP_IPPE_SQUARE returns the plate pose (R, t) in the camera
//   frame WITH METRIC SCALE, derived purely from the 10 cm side prior.
//   This means:
//
//     • Every plate position is recovered absolutely, per single image,
//       with NO dependence on stereo baseline or extrinsics.
//     • An 8-px stereo-RMS calibration error becomes IRRELEVANT for
//       plate-derived measurements — they only depend on the intrinsics
//       (focal length / principal point), which are far more stable.
//     • Inter-plate distances on the same coral section have sub-cm
//       accuracy from a single rectified image alone.
//     • Stereo is then only used to refine pipe endpoints between
//       plates, and those endpoints are anchored to known plate positions
//       via the section geometry.
//
// Multi-view fusion: when the same plate is seen in both L and R
// (and across multiple pairs), we average the plate-center camera-frame
// position weighted by 1/rms_px², after rigid alignment.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>
#include <optional>

namespace mate {

struct PnPPlatePose {
    cv::Vec3d t_cam;          // plate center in camera frame, meters
    cv::Matx33d R_cam;        // plate orientation (Z = plate normal)
    std::vector<cv::Point3f> corners_cam;   // 4 corners in camera frame, meters
    double rms_px = -1.0;     // reprojection RMS of the 4 corners
    bool ok = false;
};

// Solve plate pose from 4 (sub-pixel) corner observations.
// `corners_px` must be ordered consistently: TL, TR, BR, BL (CCW around
// the plate seen from the camera). `K` is 3x3 intrinsics (CV_64F).
//
// Performs:
//   1. SOLVEPNP_IPPE_SQUARE for an initial planar-square solution
//   2. solvePnPRefineLM (Levenberg–Marquardt) for sub-pixel refinement
//   3. Reprojection RMS computation
//
// Distortion is assumed to already be removed (caller passes rectified
// coordinates).
PnPPlatePose solvePlatePnP(const cv::Matx33d& K,
                           const std::vector<cv::Point2f>& corners_px,
                           double side_m = 0.10);

// Convenience: weighted multi-view fusion of a single plate seen N times.
// Each observation contributes its 4 corners in some common frame
// (typically the rectified-left camera frame after applying that camera's
// extrinsics). Weight = 1 / rms_px². Returns the weighted-mean center.
cv::Vec3d fuseCenters(const std::vector<PnPPlatePose>& obs);

}  // namespace mate
