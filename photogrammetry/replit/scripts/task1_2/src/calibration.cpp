#include "calibration.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <iostream>

Calibration::Calibration(int board_width, int board_height, float square_size)
    : board_width_(board_width)
    , board_height_(board_height)
    , square_size_(square_size) {}

std::vector<cv::Point3f> Calibration::create_object_points() const {
    std::vector<cv::Point3f> obj_pts;
    for (int i = 0; i < board_height_; ++i) {
        for (int j = 0; j < board_width_; ++j) {
            obj_pts.emplace_back(j * square_size_, i * square_size_, 0.0f);
        }
    }
    return obj_pts;
}

bool Calibration::find_corners(const cv::Mat& image, std::vector<cv::Point2f>& corners) const {
    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image.clone();

    bool found = cv::findChessboardCorners(gray,
        cv::Size(board_width_, board_height_), corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }

    return found;
}

std::optional<CalibrationResult> Calibration::calibrate_camera(
    const std::vector<cv::Mat>& images) const {

    if (images.empty()) {
        std::cerr << "[Calibration] No images provided\n";
        return std::nullopt;
    }

    std::vector<std::vector<cv::Point3f>> all_obj_points;
    std::vector<std::vector<cv::Point2f>> all_img_points;
    auto obj_pts = create_object_points();

    cv::Size image_size;

    for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty()) continue;

        image_size = images[i].size();
        std::vector<cv::Point2f> corners;

        if (find_corners(images[i], corners)) {
            all_obj_points.push_back(obj_pts);
            all_img_points.push_back(corners);
            std::cout << "[Calibration] Found corners in image " << i + 1 << "\n";
        } else {
            std::cout << "[Calibration] No corners found in image " << i + 1 << "\n";
        }
    }

    if (all_img_points.size() < 3) {
        std::cerr << "[Calibration] Need at least 3 valid calibration images\n";
        return std::nullopt;
    }

    CalibrationResult result;
    result.image_width = image_size.width;
    result.image_height = image_size.height;

    result.rms_error = cv::calibrateCamera(
        all_obj_points, all_img_points, image_size,
        result.camera_matrix, result.dist_coeffs,
        result.rvecs, result.tvecs);

    std::cout << "[Calibration] RMS reprojection error: " << result.rms_error << "\n";
    std::cout << "[Calibration] Camera matrix:\n" << result.camera_matrix << "\n";

    return result;
}

std::optional<StereoCalibrationResult> Calibration::calibrate_stereo(
    const std::vector<cv::Mat>& left_images,
    const std::vector<cv::Mat>& right_images) const {

    if (left_images.size() != right_images.size() || left_images.empty()) {
        std::cerr << "[Calibration] Invalid stereo image pairs\n";
        return std::nullopt;
    }

    auto left_calib = calibrate_camera(left_images);
    auto right_calib = calibrate_camera(right_images);

    if (!left_calib || !right_calib) {
        std::cerr << "[Calibration] Individual calibration failed\n";
        return std::nullopt;
    }

    std::vector<std::vector<cv::Point3f>> all_obj_points;
    std::vector<std::vector<cv::Point2f>> all_left_points, all_right_points;
    auto obj_pts = create_object_points();

    cv::Size image_size = left_images[0].size();

    for (size_t i = 0; i < left_images.size(); ++i) {
        std::vector<cv::Point2f> left_corners, right_corners;

        if (find_corners(left_images[i], left_corners) &&
            find_corners(right_images[i], right_corners)) {
            all_obj_points.push_back(obj_pts);
            all_left_points.push_back(left_corners);
            all_right_points.push_back(right_corners);
        }
    }

    if (all_obj_points.size() < 3) {
        std::cerr << "[Calibration] Not enough valid stereo pairs\n";
        return std::nullopt;
    }

    StereoCalibrationResult result;
    result.left_calib = *left_calib;
    result.right_calib = *right_calib;

    result.rms_error = cv::stereoCalibrate(
        all_obj_points, all_left_points, all_right_points,
        result.left_calib.camera_matrix, result.left_calib.dist_coeffs,
        result.right_calib.camera_matrix, result.right_calib.dist_coeffs,
        image_size, result.R, result.T, result.E, result.F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));

    std::cout << "[Calibration] Stereo RMS error: " << result.rms_error << "\n";

    return result;
}

bool Calibration::save_calibration(const CalibrationResult& result,
                                    const std::string& filename) const {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;

    fs << "camera_matrix" << result.camera_matrix;
    fs << "dist_coeffs" << result.dist_coeffs;
    fs << "image_width" << result.image_width;
    fs << "image_height" << result.image_height;
    fs << "rms_error" << result.rms_error;

    fs.release();
    return true;
}

std::optional<CalibrationResult> Calibration::load_calibration(
    const std::string& filename) const {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return std::nullopt;

    CalibrationResult result;
    fs["camera_matrix"] >> result.camera_matrix;
    fs["dist_coeffs"] >> result.dist_coeffs;
    fs["image_width"] >> result.image_width;
    fs["image_height"] >> result.image_height;
    fs["rms_error"] >> result.rms_error;

    fs.release();
    return result;
}

cv::Mat Calibration::undistort(const cv::Mat& image, const CalibrationResult& calib) const {
    cv::Mat undistorted;
    cv::undistort(image, undistorted, calib.camera_matrix, calib.dist_coeffs);
    return undistorted;
}
