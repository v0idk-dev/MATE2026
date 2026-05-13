#pragma once
// ─────────────────────────────────────────────────────────────────────────
// image_undistort.hpp — step 1: undistort an image with cached remap maps.
//
// Wraps cv::initUndistortRectifyMap + cv::remap (or the fisheye variant)
// behind a small cache so repeated pairs at the same resolution do not
// recompute the maps. Optionally dispatches to mate::metal::remapBgr for
// Apple Silicon GPU acceleration.
// ─────────────────────────────────────────────────────────────────────────

#include "calibration_io.hpp"
#include <opencv2/core.hpp>

namespace mate {

cv::Mat undistortImage(const cv::Mat& src_bgr,
                       const CameraIntrinsics& intr,
                       bool use_metal = true);

// Free the internal remap-map cache (rarely needed; resolution changes
// already invalidate the right key).
void clearUndistortCache();

}  // namespace mate
