#pragma once

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>
#include <optional>

struct CalibrationResult {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    double rms_error;
    int image_width;
    int image_height;
};

struct StereoCalibrationResult {
    CalibrationResult left_calib;
    CalibrationResult right_calib;
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    double rms_error;
};

class Calibration {
public:
    Calibration(int board_width = 9, int board_height = 6, float square_size = 0.025f);

    std::optional<CalibrationResult> calibrate_camera(
        const std::vector<cv::Mat>& images) const;

    std::optional<StereoCalibrationResult> calibrate_stereo(
        const std::vector<cv::Mat>& left_images,
        const std::vector<cv::Mat>& right_images) const;

    bool save_calibration(const CalibrationResult& result, const std::string& filename) const;
    std::optional<CalibrationResult> load_calibration(const std::string& filename) const;

    cv::Mat undistort(const cv::Mat& image, const CalibrationResult& calib) const;

private:
    int board_width_;
    int board_height_;
    float square_size_;

    std::vector<cv::Point3f> create_object_points() const;
    bool find_corners(const cv::Mat& image, std::vector<cv::Point2f>& corners) const;
};
