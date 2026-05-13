#include "triangulator.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace mate {

bool Triangulator::triangulate(const cv::Point2f& xL, const cv::Point2f& xR,
                               cv::Point3f& out) const {
    // Reject non-positive disparity. After rectification disparity is xL−xR
    // for points in front of the cameras, so we want xL.x > xR.x.
    if (xL.x - xR.x <= 1e-3f) return false;

    cv::Mat ptsL(2, 1, CV_64F), ptsR(2, 1, CV_64F);
    ptsL.at<double>(0, 0) = xL.x; ptsL.at<double>(1, 0) = xL.y;
    ptsR.at<double>(0, 0) = xR.x; ptsR.at<double>(1, 0) = xR.y;
    cv::Mat X4d;  // 4×1
    cv::triangulatePoints(P1, P2, ptsL, ptsR, X4d);
    if (X4d.empty()) return false;
    double w = X4d.at<double>(3, 0);
    if (std::abs(w) < 1e-9) return false;
    out.x = (float)(X4d.at<double>(0, 0) / w);
    out.y = (float)(X4d.at<double>(1, 0) / w);
    out.z = (float)(X4d.at<double>(2, 0) / w);
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z)) {
        return false;
    }
    return out.z > 0;  // must be in front of the camera
}

std::vector<cv::Point3f>
Triangulator::triangulateMany(const std::vector<cv::Point2f>& xL,
                              const std::vector<cv::Point2f>& xR) const {
    std::vector<cv::Point3f> out;
    out.reserve(xL.size());
    const size_t n = std::min(xL.size(), xR.size());
    if (n == 0) return out;

    // Build batched 2×N matrices for OpenCV's triangulatePoints.
    cv::Mat L(2, (int)n, CV_64F), R(2, (int)n, CV_64F);
    for (size_t i = 0; i < n; ++i) {
        L.at<double>(0, (int)i) = xL[i].x;
        L.at<double>(1, (int)i) = xL[i].y;
        R.at<double>(0, (int)i) = xR[i].x;
        R.at<double>(1, (int)i) = xR[i].y;
    }
    cv::Mat X4d;  // 4×N
    cv::triangulatePoints(P1, P2, L, R, X4d);
    for (size_t i = 0; i < n; ++i) {
        const double w = X4d.at<double>(3, (int)i);
        cv::Point3f p(NAN, NAN, NAN);
        const float dx = xL[i].x - xR[i].x;
        if (dx > 1e-3f && std::abs(w) > 1e-9) {
            float x = (float)(X4d.at<double>(0, (int)i) / w);
            float y = (float)(X4d.at<double>(1, (int)i) / w);
            float z = (float)(X4d.at<double>(2, (int)i) / w);
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0) {
                p = cv::Point3f(x, y, z);
            }
        }
        out.push_back(p);
    }
    return out;
}

}  // namespace mate
