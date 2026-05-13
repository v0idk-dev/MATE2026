#include "stereo_rectifier.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace mate {

namespace {

// Scale a 3×3 intrinsic matrix from the calibration resolution to the
// actual frame resolution. fx, fy, cx, cy all scale linearly with the
// dimension change. K must be 3×3, CV_64F.
cv::Mat scaleK(const cv::Mat& K_in, int calib_w, int calib_h,
               int actual_w, int actual_h) {
    if (calib_w <= 0 || calib_h <= 0) return K_in.clone();
    if (calib_w == actual_w && calib_h == actual_h) return K_in.clone();
    cv::Mat K = K_in.clone();
    const double sx = double(actual_w) / double(calib_w);
    const double sy = double(actual_h) / double(calib_h);
    K.at<double>(0, 0) *= sx;  // fx
    K.at<double>(0, 2) *= sx;  // cx
    K.at<double>(1, 1) *= sy;  // fy
    K.at<double>(1, 2) *= sy;  // cy
    return K;
}

}  // namespace

std::optional<RectifiedPair>
rectifyStereoPair(const cv::Mat& left_in,
                  const cv::Mat& right_in,
                  const std::optional<CameraIntrinsics>& left_intr,
                  const std::optional<CameraIntrinsics>& right_intr,
                  const StereoExtrinsics& ex,
                  double alpha) {
    if (left_in.empty() || right_in.empty()) {
        std::cerr << "rectifyStereoPair: empty image (L="
                  << left_in.cols << "x" << left_in.rows << " R="
                  << right_in.cols << "x" << right_in.rows << ")\n";
        return std::nullopt;
    }
    if (left_in.size() != right_in.size()) {
        // Both frames usually come from the same combined CCTV feed split
        // down the middle, so they're guaranteed equal size at runtime.
        // For independently-captured pairs (e.g. two separate cameras at
        // different resolutions) they may legitimately differ — in that
        // case the user must rescale before calling this function.
        std::cerr << "rectifyStereoPair: size mismatch left="
                  << left_in.cols << "x" << left_in.rows << " right="
                  << right_in.cols << "x" << right_in.rows
                  << " — both frames must be the same resolution\n";
        return std::nullopt;
    }

    // Resolve which intrinsics to use. Prefer dedicated per-cam files; fall
    // back to the embedded ones from the extrinsics file.
    const CameraIntrinsics* L = nullptr;
    const CameraIntrinsics* R = nullptr;
    if (left_intr.has_value() && !left_intr->K.empty())  L = &(*left_intr);
    else if (ex.has_provided_intrinsics && !ex.K_left_provided.K.empty())
        L = &ex.K_left_provided;
    if (right_intr.has_value() && !right_intr->K.empty()) R = &(*right_intr);
    else if (ex.has_provided_intrinsics && !ex.K_right_provided.K.empty())
        R = &ex.K_right_provided;
    if (!L || !R) {
        std::cerr << "rectifyStereoPair: no usable intrinsics (left K "
                  << (L ? "ok" : "missing") << ", right K "
                  << (R ? "ok" : "missing")
                  << ") — supply per-camera YAML/PKL or embed K_left/K_right "
                     "in the stereo extrinsics YAML\n";
        return std::nullopt;
    }
    if (L->model != R->model) {
        // Mixed pinhole + fisheye is not supported; the rig is always one
        // model. If we ever hit this, abort cleanly.
        std::cerr << "rectifyStereoPair: model mismatch — left is "
                  << (L->model == DistortionModel::Fisheye ? "fisheye" : "pinhole")
                  << ", right is "
                  << (R->model == DistortionModel::Fisheye ? "fisheye" : "pinhole")
                  << " — both cameras must use the same distortion model\n";
        return std::nullopt;
    }

    const cv::Size sz = left_in.size();
    const int actual_w = sz.width, actual_h = sz.height;

    // Scale intrinsics from each camera's calibration resolution to the
    // frame resolution we actually got. Calibration resolution may legitimately
    // differ from runtime (e.g. calibrated at 1280x720 but feed delivered
    // 1920x1080). The stereo R,T are invariant under uniform K scaling.
    cv::Mat K1 = scaleK(L->K, L->image_width, L->image_height, actual_w, actual_h);
    cv::Mat K2 = scaleK(R->K, R->image_width, R->image_height, actual_w, actual_h);
    cv::Mat D1 = L->D.clone();
    cv::Mat D2 = R->D.clone();

    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat map1L, map2L, map1R, map2R;

    if (L->model == DistortionModel::Fisheye) {
        // Fisheye stereo rectification.
        cv::fisheye::stereoRectify(
            K1, D1, K2, D2, sz, ex.R, ex.T,
            R1, R2, P1, P2, Q,
            cv::CALIB_ZERO_DISPARITY, sz, 0.0, 1.0);
        cv::fisheye::initUndistortRectifyMap(
            K1, D1, R1, P1, sz, CV_16SC2, map1L, map2L);
        cv::fisheye::initUndistortRectifyMap(
            K2, D2, R2, P2, sz, CV_16SC2, map1R, map2R);
    } else {
        // Standard pinhole stereo rectification.
        cv::Rect roi1, roi2;
        cv::stereoRectify(
            K1, D1, K2, D2, sz, ex.R, ex.T,
            R1, R2, P1, P2, Q,
            cv::CALIB_ZERO_DISPARITY, alpha, sz, &roi1, &roi2);
        cv::initUndistortRectifyMap(
            K1, D1, R1, P1, sz, CV_16SC2, map1L, map2L);
        cv::initUndistortRectifyMap(
            K2, D2, R2, P2, sz, CV_16SC2, map1R, map2R);
    }

    RectifiedPair out;
    out.size = sz;
    cv::remap(left_in,  out.left,  map1L, map2L, cv::INTER_LINEAR);
    cv::remap(right_in, out.right, map1R, map2R, cv::INTER_LINEAR);
    out.P1 = P1;
    out.P2 = P2;
    out.Q  = Q;
    // K_rect_* are the upper-left 3×3 of each P_*. After rectification
    // both cameras share fx, fy, cx, cy; only the P matrices' translation
    // term differs.
    out.K_rect_left  = P1(cv::Rect(0, 0, 3, 3)).clone();
    out.K_rect_right = P2(cv::Rect(0, 0, 3, 3)).clone();
    out.baseline = cv::norm(ex.T);
    out.unit = ex.unit.empty() ? "m" : ex.unit;
    return out;
}

}  // namespace mate
