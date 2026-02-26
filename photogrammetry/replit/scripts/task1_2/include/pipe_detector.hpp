#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <optional>

struct PipeSegment {
    cv::Point2f start;
    cv::Point2f end;
    double length_pixels;
    double length_real;
    double angle;
    double thickness;
    double confidence;
    std::vector<cv::Point2f> points;
};

struct ReferenceSquare {
    cv::Rect bounding_box;
    cv::RotatedRect rotated_rect;
    std::vector<cv::Point> contour;
    cv::Point2f centroid;
    double area;
    double side_length_pixels;
    double confidence;
    cv::Scalar mean_color_bgr;
    std::string color_name;
};

struct ScaleInfo {
    double pixels_per_cm;
    double cm_per_pixel;
    int num_references;
    double scale_confidence;
};

class PipeDetector {
public:
    PipeDetector();

    void set_canny_thresholds(double low, double high);
    void set_hough_params(double rho, double theta_deg, int threshold,
                          double min_line_length, double max_line_gap);
    void set_min_pipe_length_pixels(double len);
    void set_pipe_merge_angle_threshold(double degrees);
    void set_pipe_merge_distance_threshold(double pixels);

    std::vector<PipeSegment> detect(const cv::Mat& image) const;

    cv::Mat visualize(const cv::Mat& image,
                      const std::vector<PipeSegment>& pipes,
                      const ScaleInfo& scale) const;

private:
    double canny_low_;
    double canny_high_;
    double hough_rho_;
    double hough_theta_deg_;
    int hough_threshold_;
    double hough_min_line_length_;
    double hough_max_line_gap_;
    double min_pipe_length_pixels_;
    double merge_angle_threshold_;
    double merge_distance_threshold_;

    std::vector<PipeSegment> merge_collinear_segments(
        const std::vector<PipeSegment>& segments) const;

    bool are_collinear(const PipeSegment& a, const PipeSegment& b) const;

    double point_to_line_distance(const cv::Point2f& pt,
                                   const cv::Point2f& line_start,
                                   const cv::Point2f& line_end) const;
};

class ReferenceSquareDetector {
public:
    ReferenceSquareDetector();

    void set_known_side_cm(double side_cm);
    void set_min_area(double area);
    void set_max_area(double area);

    std::vector<ReferenceSquare> detect(const cv::Mat& image) const;

    ScaleInfo compute_scale(const std::vector<ReferenceSquare>& squares) const;

    cv::Mat visualize(const cv::Mat& image,
                      const std::vector<ReferenceSquare>& squares) const;

private:
    double known_side_cm_;
    double min_area_;
    double max_area_;

    std::string classify_color(const cv::Scalar& bgr) const;
    double compute_squareness(const std::vector<cv::Point>& contour) const;
};
