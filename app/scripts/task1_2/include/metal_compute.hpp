#pragma once
// ─────────────────────────────────────────────────────────────────────────
// metal_compute.hpp — Metal compute kernels for image operations.
//
// Apple Silicon only. We provide Metal-accelerated versions of the
// per-pixel hot paths that dominate runtime when N pairs > 1:
//   • undistort+remap (replaces cv::remap with a single fused kernel)
//   • HSV threshold (replaces cv::cvtColor + cv::inRange)
//   • Sobel/Canny edges (replaces cv::Canny for the pipe detector)
//   • SGBM disparity post-filter (median + speckle on the device)
//
// API shape mirrors the OpenCV calls we replace, so callers can swap
// transparently. When Metal is unavailable (PipelineInput::use_metal=false
// or Linux test build) the implementations forward to OpenCV's CPU paths.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate::metal {

// Lazy-initialize the default Metal device + command queue. Returns false
// if Metal isn't available (non-Apple build, or device init failed).
bool ensureInitialized();

// Pinhole/fisheye undistort. `mapx`/`mapy` are the precomputed remap maps
// (same shape as `cv::initUndistortRectifyMap` produces with CV_32F).
// Returns the undistorted image as CV_8UC3.
cv::Mat remapBgr(const cv::Mat& src_bgr,
                 const cv::Mat& mapx, const cv::Mat& mapy);

// HSV threshold:  out[y,x] = 255 if H∈[h_lo,h_hi] && S>=s_min && V∈[v_min,v_max], else 0.
// Hue wrap (h_lo > h_hi) is handled internally.
cv::Mat hsvThreshold(const cv::Mat& src_bgr,
                     int h_lo, int h_hi, int s_min, int v_min, int v_max);

// Canny-equivalent edge detector: gaussian blur → sobel → non-max suppression
// → double-threshold hysteresis. Output is CV_8UC1 (0 or 255).
cv::Mat cannyEdges(const cv::Mat& src_bgr,
                   int low_thresh, int high_thresh,
                   int gaussian_radius_px = 2);

// Median filter on a CV_32F disparity map (3×3 or 5×5 window).
cv::Mat medianFilterF32(const cv::Mat& disp32, int window_px);

}  // namespace mate::metal
