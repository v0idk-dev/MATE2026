#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <optional>

struct PlateDetection {
    cv::Rect bounding_box;
    cv::RotatedRect rotated_rect;
    std::vector<cv::Point> contour;
    cv::Point2f centroid;
    double area;
    double confidence;
    cv::Mat mask;
};

class PurplePlateDetector {
public:
    struct HSVRange {
        cv::Scalar lower;
        cv::Scalar upper;
    };

    PurplePlateDetector();

    void set_min_area(double area);
    void set_max_area(double area);
    void set_aspect_ratio_range(double min_ratio, double max_ratio);
    void add_purple_range(const HSVRange& range);

    std::vector<PlateDetection> detect(const cv::Mat& image) const;
    std::optional<PlateDetection> detect_best(const cv::Mat& image) const;

    cv::Mat visualize(const cv::Mat& image, const std::vector<PlateDetection>& detections) const;

private:
    std::vector<HSVRange> purple_ranges_;
    double min_area_;
    double max_area_;
    double min_aspect_ratio_;
    double max_aspect_ratio_;

    cv::Mat create_purple_mask(const cv::Mat& hsv_image) const;
    double compute_plate_confidence(const cv::Mat& image, const std::vector<cv::Point>& contour) const;
    std::vector<cv::Point> refine_contour(const std::vector<cv::Point>& contour) const;
};
