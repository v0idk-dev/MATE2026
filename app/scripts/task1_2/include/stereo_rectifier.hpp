#pragma once
// ─────────────────────────────────────────────────────────────────────────
// stereo_rectifier.hpp — undistort + rectify a stereo pair.
//
// Two operating modes:
//
//   (A) FULL: per-camera intrinsics + stereo extrinsics both available.
//       This is the accurate path. We honor each camera's distortion
//       model (pinhole or fisheye) for undistortion, then rectify using
//       the stereo R, T. Output rectified images have parallel epipolar
//       lines; subsequent matching can assume horizontal-only disparity.
//
//   (B) PARTIAL: extrinsics are usable but per-camera intrinsics weren't
//       provided. We fall back to the K/D embedded inside the extrinsics
//       file (StereoExtrinsics::K_*_provided). Same accuracy if the
//       embedded intrinsics came from the same calibration session.
//
// If neither mode is achievable, the caller (main.cpp) reports a clear
// error before invoking us.
// ─────────────────────────────────────────────────────────────────────────

#include "calibration_io.hpp"
#include <opencv2/core.hpp>
#include <optional>

namespace mate {

struct RectifiedPair {
    cv::Mat left;          // rectified left image
    cv::Mat right;         // rectified right image
    cv::Mat K_rect_left;   // 3×3 intrinsics in the rectified left frame
    cv::Mat K_rect_right;  // 3×3 intrinsics in the rectified right frame
    cv::Mat P1, P2;        // 3×4 projections in rectified frames
    cv::Mat Q;             // disparity-to-depth, in the unit of T
    double  baseline;      // ||T||, in the unit of T
    std::string unit;      // "cm" | "m" | etc., propagated from extrinsics
    cv::Size size;         // rectified image size (same for both)
};

// Build rectification maps + apply them.
//
// `left_in` and `right_in` must be the original (distorted) frames. They
// don't have to be the same size as the calibration image_width/height —
// if they differ, we scale the intrinsics linearly. The stereo R, T are
// invariant under uniform scaling so they don't need adjustment.
//
// `Kl`, `Kr`: per-camera intrinsics. If empty, the function falls back to
// the embedded `ex.K_*_provided` intrinsics. If those are also empty,
// returns nullopt.
std::optional<RectifiedPair>
rectifyStereoPair(const cv::Mat& left_in,
                  const cv::Mat& right_in,
                  const std::optional<CameraIntrinsics>& left_intr,
                  const std::optional<CameraIntrinsics>& right_intr,
                  const StereoExtrinsics& ex,
                  double alpha = 0.0);
// alpha=0: rectified output crops to valid pixels; alpha=1: keeps all
// pixels including black borders. 0 is what we want for downstream
// pipeline accuracy; alpha=1 is mostly useful for visual debugging.

}  // namespace mate
