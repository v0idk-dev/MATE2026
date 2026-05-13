#include "plate_pnp.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace mate {

PnPPlatePose solvePlatePnP(const cv::Matx33d& K,
                           const std::vector<cv::Point2f>& corners_px,
                           double side_m) {
    PnPPlatePose out;
    if (corners_px.size() != 4 || side_m <= 0.0) return out;

    // Object frame: plate is centered at origin, lies in z=0 plane.
    // Order matches corners_px: TL, TR, BR, BL (CCW from camera view).
    const float s = static_cast<float>(side_m * 0.5);
    std::vector<cv::Point3f> obj = {
        {-s, -s, 0.f}, { s, -s, 0.f}, { s,  s, 0.f}, {-s,  s, 0.f}
    };

    cv::Mat Kmat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            Kmat.at<double>(r, c) = K(r, c);

    cv::Vec3d rvec, tvec;
    bool ok = false;
    try {
        ok = cv::solvePnP(obj, corners_px, Kmat, cv::noArray(),
                          rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
    } catch (const cv::Exception&) {
        ok = false;
    }
    if (!ok) {
        // Fallback to general PnP if IPPE_SQUARE refuses (e.g. degenerate).
        try {
            ok = cv::solvePnP(obj, corners_px, Kmat, cv::noArray(),
                              rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        } catch (const cv::Exception&) { ok = false; }
    }
    if (!ok) return out;

    // Levenberg–Marquardt sub-pixel refinement.
    try {
        cv::solvePnPRefineLM(obj, corners_px, Kmat, cv::noArray(), rvec, tvec);
    } catch (const cv::Exception&) {
        // refinement is optional; ignore failures
    }

    // Reproject to compute RMS.
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, Kmat, cv::noArray(), proj);
    double sse = 0.0;
    for (int i = 0; i < 4; ++i) {
        const double dx = proj[i].x - corners_px[i].x;
        const double dy = proj[i].y - corners_px[i].y;
        sse += dx * dx + dy * dy;
    }
    out.rms_px = std::sqrt(sse / 4.0);

    // Convert R, t to output types and compute corner positions in camera frame.
    cv::Mat Rm; cv::Rodrigues(rvec, Rm);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.R_cam(r, c) = Rm.at<double>(r, c);
    out.t_cam = tvec;
    out.corners_cam.reserve(4);
    for (const auto& p : obj) {
        cv::Vec3d v(p.x, p.y, p.z);
        cv::Vec3d w = out.R_cam * v + tvec;
        out.corners_cam.emplace_back(static_cast<float>(w[0]),
                                     static_cast<float>(w[1]),
                                     static_cast<float>(w[2]));
    }
    out.ok = true;
    return out;
}

cv::Vec3d fuseCenters(const std::vector<PnPPlatePose>& obs) {
    cv::Vec3d acc(0, 0, 0);
    double wsum = 0.0;
    for (const auto& o : obs) {
        if (!o.ok || o.rms_px <= 0) continue;
        const double w = 1.0 / (o.rms_px * o.rms_px);
        acc += w * o.t_cam;
        wsum += w;
    }
    if (wsum <= 0) return cv::Vec3d(0, 0, 0);
    return acc * (1.0 / wsum);
}

}  // namespace mate
