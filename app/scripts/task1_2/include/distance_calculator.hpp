#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <optional>

struct CameraParameters {
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coeffs;
    double focal_length_mm;
    double sensor_width_mm;
    int image_width;
    int image_height;

    double focal_length_pixels() const;
    static CameraParameters estimate_from_image(int width, int height,
                                                 double focal_length_mm = 35.0,
                                                 double sensor_width_mm = 36.0);
};

struct DistanceEstimate {
    double distance;
    double baseline;
    double disparity;
    double confidence;
    cv::Point3f world_point;
    std::string method;
};

class DistanceCalculator {
public:
    DistanceCalculator();

    void set_camera_params(const CameraParameters& params);
    void set_known_plate_width(double width_meters);
    void set_known_plate_height(double height_meters);

    DistanceEstimate estimate_from_stereo_triangulation(
        const cv::Mat& R, const cv::Mat& t,
        const cv::Point2f& pt_left, const cv::Point2f& pt_right,
        double baseline_scale = 1.0) const;

    DistanceEstimate estimate_from_disparity(
        const cv::Point2f& pt_left, const cv::Point2f& pt_right,
        double baseline_meters) const;

    DistanceEstimate estimate_from_known_size(
        const cv::RotatedRect& detected_rect,
        bool use_width = true) const;

    std::vector<DistanceEstimate> estimate_all_methods(
        const cv::Mat& R, const cv::Mat& t,
        const cv::Point2f& pt_left, const cv::Point2f& pt_right,
        const cv::RotatedRect& rect_left,
        double baseline_meters = 0.1) const;

    cv::Point3f triangulate_point(
        const cv::Mat& P1, const cv::Mat& P2,
        const cv::Point2f& pt1, const cv::Point2f& pt2) const;

    std::vector<cv::Point3f> triangulate_points(
        const cv::Mat& P1, const cv::Mat& P2,
        const std::vector<cv::Point2f>& pts1,
        const std::vector<cv::Point2f>& pts2) const;

private:
    CameraParameters camera_params_;
    double known_plate_width_;
    double known_plate_height_;
    bool params_set_;
};
