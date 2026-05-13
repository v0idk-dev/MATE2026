#pragma once
// ─────────────────────────────────────────────────────────────────────────
// triangulator.hpp — convert paired 2D points in a RECTIFIED stereo image
// pair into 3D points in the left rectified camera frame.
//
// All inputs are assumed already-rectified pixel coordinates: i.e. the y
// coordinates of corresponding points should match (within noise) and only
// disparity (xL - xR) varies. This is what stereo_rectifier.cpp produces.
//
// Output coordinates are in the unit of the stereo extrinsics' translation
// vector — typically centimeters (because stereo_calibrate.py uses cm),
// but anything is fine; downstream code converts.
//
// We use cv::triangulatePoints (DLT), which is reliable and fast for
// rectified pairs. For higher accuracy at long distances we could later
// add an iterative refinement (Sampson-error minimization), but the
// classic DLT solution suffices for the ±5 cm spec at 1–2.5 m subject
// distance with sub-pixel-refined inputs.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct Triangulator {
    // Both must be 3×4, doubles. Take them straight from RectifiedPair.
    cv::Mat P1;
    cv::Mat P2;

    // Triangulate one pair → 3D point in the left rectified camera frame.
    // Returns false if the disparity is non-positive (point at infinity).
    bool triangulate(const cv::Point2f& xL, const cv::Point2f& xR,
                     cv::Point3f& out) const;

    // Vectorized form. Out vector is parallel to inputs. Points that fail
    // (non-positive disparity) get cv::Point3f(NAN, NAN, NAN).
    std::vector<cv::Point3f>
    triangulateMany(const std::vector<cv::Point2f>& xL,
                    const std::vector<cv::Point2f>& xR) const;
};

}  // namespace mate
