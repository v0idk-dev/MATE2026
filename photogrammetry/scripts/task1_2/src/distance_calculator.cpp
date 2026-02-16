#include "distance_calculator.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>

double CameraParameters::focal_length_pixels() const {
    return (focal_length_mm / sensor_width_mm) * image_width;
}

CameraParameters CameraParameters::estimate_from_image(int width, int height,
                                                         double focal_length_mm,
                                                         double sensor_width_mm) {
    CameraParameters params;
    params.focal_length_mm = focal_length_mm;
    params.sensor_width_mm = sensor_width_mm;
    params.image_width = width;
    params.image_height = height;

    double fx = (focal_length_mm / sensor_width_mm) * width;
    double fy = fx;
    double cx = width / 2.0;
    double cy = height / 2.0;

    params.intrinsic_matrix = (cv::Mat_<double>(3, 3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1);

    params.distortion_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    return params;
}

DistanceCalculator::DistanceCalculator()
    : known_plate_width_(0.3)
    , known_plate_height_(0.2)
    , params_set_(false) {}

void DistanceCalculator::set_camera_params(const CameraParameters& params) {
    camera_params_ = params;
    params_set_ = true;
}

void DistanceCalculator::set_known_plate_width(double width_meters) {
    known_plate_width_ = width_meters;
}

void DistanceCalculator::set_known_plate_height(double height_meters) {
    known_plate_height_ = height_meters;
}

cv::Point3f DistanceCalculator::triangulate_point(
    const cv::Mat& P1, const cv::Mat& P2,
    const cv::Point2f& pt1, const cv::Point2f& pt2) const {

    cv::Mat A(4, 4, CV_64F);

    A.row(0) = pt1.x * P1.row(2) - P1.row(0);
    A.row(1) = pt1.y * P1.row(2) - P1.row(1);
    A.row(2) = pt2.x * P2.row(2) - P2.row(0);
    A.row(3) = pt2.y * P2.row(2) - P2.row(1);

    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);

    cv::Mat X = vt.row(3).t();
    double W = X.at<double>(3);

    return cv::Point3f(
        static_cast<float>(X.at<double>(0) / W),
        static_cast<float>(X.at<double>(1) / W),
        static_cast<float>(X.at<double>(2) / W)
    );
}

std::vector<cv::Point3f> DistanceCalculator::triangulate_points(
    const cv::Mat& P1, const cv::Mat& P2,
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2) const {

    std::vector<cv::Point3f> result;
    result.reserve(pts1.size());

    for (size_t i = 0; i < pts1.size() && i < pts2.size(); ++i) {
        result.push_back(triangulate_point(P1, P2, pts1[i], pts2[i]));
    }

    return result;
}

DistanceEstimate DistanceCalculator::estimate_from_stereo_triangulation(
    const cv::Mat& R, const cv::Mat& t,
    const cv::Point2f& pt_left, const cv::Point2f& pt_right,
    double baseline_scale) const {

    DistanceEstimate est;
    est.method = "Stereo Triangulation";

    cv::Mat K = camera_params_.intrinsic_matrix;

    cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);

    cv::Mat Rt(3, 4, CV_64F);
    R.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
    cv::Mat t_scaled = t * baseline_scale;
    t_scaled.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
    cv::Mat P2 = K * Rt;

    cv::Point3f pt3d = triangulate_point(P1, P2, pt_left, pt_right);

    est.world_point = pt3d;
    est.distance = cv::norm(cv::Vec3f(pt3d.x, pt3d.y, pt3d.z));
    est.baseline = baseline_scale;
    est.disparity = std::abs(pt_left.x - pt_right.x);

    double max_dist = 1000.0;
    est.confidence = std::max(0.0, 1.0 - est.distance / max_dist) * 100.0;

    if (est.distance < 0.01 || est.distance > max_dist) {
        est.confidence *= 0.1;
    }

    return est;
}

DistanceEstimate DistanceCalculator::estimate_from_disparity(
    const cv::Point2f& pt_left, const cv::Point2f& pt_right,
    double baseline_meters) const {

    DistanceEstimate est;
    est.method = "Disparity-based";

    double disparity = std::abs(pt_left.x - pt_right.x);
    est.disparity = disparity;
    est.baseline = baseline_meters;

    double f = camera_params_.intrinsic_matrix.at<double>(0, 0);

    if (disparity > 0.5) {
        est.distance = (f * baseline_meters) / disparity;
    } else {
        est.distance = -1.0;
        est.confidence = 0.0;
        return est;
    }

    est.confidence = std::min(disparity / 10.0, 1.0) * 100.0;

    double cx = camera_params_.intrinsic_matrix.at<double>(0, 2);
    double cy = camera_params_.intrinsic_matrix.at<double>(1, 2);
    double fy = camera_params_.intrinsic_matrix.at<double>(1, 1);

    est.world_point.x = static_cast<float>(est.distance * (pt_left.x - cx) / f);
    est.world_point.y = static_cast<float>(est.distance * (pt_left.y - cy) / fy);
    est.world_point.z = static_cast<float>(est.distance);

    return est;
}

DistanceEstimate DistanceCalculator::estimate_from_known_size(
    const cv::RotatedRect& detected_rect,
    bool use_width) const {

    DistanceEstimate est;
    est.method = "Known Object Size";

    double f = camera_params_.intrinsic_matrix.at<double>(0, 0);

    double object_size_meters = use_width ? known_plate_width_ : known_plate_height_;
    double object_size_pixels = use_width ?
        std::max(detected_rect.size.width, detected_rect.size.height) :
        std::min(detected_rect.size.width, detected_rect.size.height);

    if (object_size_pixels > 1.0) {
        est.distance = (f * object_size_meters) / object_size_pixels;
    } else {
        est.distance = -1.0;
        est.confidence = 0.0;
        return est;
    }

    double expected_pixels = f * object_size_meters / est.distance;
    double pixel_error = std::abs(object_size_pixels - expected_pixels) / expected_pixels;
    est.confidence = std::max(0.0, (1.0 - pixel_error)) * 100.0;

    double cx = camera_params_.intrinsic_matrix.at<double>(0, 2);
    double cy = camera_params_.intrinsic_matrix.at<double>(1, 2);
    double fy = camera_params_.intrinsic_matrix.at<double>(1, 1);

    est.world_point.x = static_cast<float>(est.distance * (detected_rect.center.x - cx) / f);
    est.world_point.y = static_cast<float>(est.distance * (detected_rect.center.y - cy) / fy);
    est.world_point.z = static_cast<float>(est.distance);

    est.disparity = 0;
    est.baseline = 0;

    return est;
}

std::vector<DistanceEstimate> DistanceCalculator::estimate_all_methods(
    const cv::Mat& R, const cv::Mat& t,
    const cv::Point2f& pt_left, const cv::Point2f& pt_right,
    const cv::RotatedRect& rect_left,
    double baseline_meters) const {

    std::vector<DistanceEstimate> estimates;

    auto stereo_est = estimate_from_stereo_triangulation(R, t, pt_left, pt_right, baseline_meters);
    estimates.push_back(stereo_est);

    auto disparity_est = estimate_from_disparity(pt_left, pt_right, baseline_meters);
    estimates.push_back(disparity_est);

    auto size_est_w = estimate_from_known_size(rect_left, true);
    size_est_w.method += " (width)";
    estimates.push_back(size_est_w);

    auto size_est_h = estimate_from_known_size(rect_left, false);
    size_est_h.method += " (height)";
    estimates.push_back(size_est_h);

    return estimates;
}
