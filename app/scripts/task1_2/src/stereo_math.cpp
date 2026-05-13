#include "stereo_math.hpp"
#include <cmath>

namespace mate {

double disparityToDepth(double d, double f, double B) {
    if (!(d > 0.0) || !(f > 0.0) || !(B > 0.0)) return std::nan("");
    return f * B / d;
}

double depthToDisparity(double Z, double f, double B) {
    if (!(Z > 0.0) || !(f > 0.0) || !(B > 0.0)) return std::nan("");
    return f * B / Z;
}

double expectedPlatePx(double f, double s_m, double Z) {
    if (!(f > 0.0) || !(s_m > 0.0) || !(Z > 0.0)) return std::nan("");
    return f * s_m / Z;
}

double depthUncertainty(double Z, double f, double B, double sigma_d) {
    if (!(Z > 0.0) || !(f > 0.0) || !(B > 0.0) || !(sigma_d >= 0.0)) return std::nan("");
    return (Z * Z) * sigma_d / (f * B);
}

cv::Mat makeQFromBaseline(double f, double cx, double cy,
                          double B, double cx_right) {
    cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
    if (!(f > 0.0) || !(B > 0.0)) return Q;

    // OpenCV convention: T_x = translation from rectified-right to
    // rectified-left along x = −B (right cam is at +B from left).
    const double Tx = -B;

    // Q · [x y d 1]ᵀ = [X Y Z W]ᵀ, then divide by W.
    //   W = (-1/Tx)·d + (cx − cx')/Tx
    //     = d/B + (cx − cx')/(-B)
    //     = d/B − (cx − cx')/B
    //   For aligned principal points (cx == cx'): W = d/B
    //   Then Z = f / W = f·B/d  ✓
    if (cx_right < 0) cx_right = cx;

    Q.at<double>(0, 0) = 1.0;  Q.at<double>(0, 3) = -cx;
    Q.at<double>(1, 1) = 1.0;  Q.at<double>(1, 3) = -cy;
    Q.at<double>(2, 3) = f;
    Q.at<double>(3, 2) = -1.0 / Tx;             // = 1/B
    Q.at<double>(3, 3) = (cx - cx_right) / Tx;  // = (cx_right - cx)/B
    return Q;
}

cv::Mat estimateIntrinsicsFromImage(int W, int H, double fov_h_deg) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    if (W <= 0 || H <= 0 || !(fov_h_deg > 1.0 && fov_h_deg < 179.0)) return K;
    const double f = (W * 0.5) / std::tan(fov_h_deg * 0.5 * CV_PI / 180.0);
    K.at<double>(0, 0) = f;
    K.at<double>(1, 1) = f;
    K.at<double>(0, 2) = W * 0.5;
    K.at<double>(1, 2) = H * 0.5;
    return K;
}

double focalFromK(const cv::Mat& K) {
    if (K.empty() || K.rows < 2 || K.cols < 2) return std::nan("");
    cv::Mat K64; K.convertTo(K64, CV_64F);
    double fx = K64.at<double>(0, 0);
    double fy = K64.at<double>(1, 1);
    if (!(fx > 0) || !(fy > 0)) return std::nan("");
    return 0.5 * (fx + fy);
}

const char* baselineSanity(double B) {
    if (!(B > 0.0))      return "invalid";
    if (B < 0.02)        return "narrow";   // < 2 cm: depth uncertainty huge
    if (B > 1.0)         return "wide";     // > 1 m: uncommon, double-check
    return "ok";
}

}  // namespace mate
