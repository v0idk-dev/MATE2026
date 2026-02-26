#include "purple_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

PurplePlateDetector::PurplePlateDetector()
    : min_area_(500.0)
    , max_area_(500000.0)
    , min_aspect_ratio_(0.3)
    , max_aspect_ratio_(5.0) {

    purple_ranges_.push_back({{120, 30, 30}, {160, 255, 255}});
    purple_ranges_.push_back({{100, 20, 40}, {130, 255, 255}});
    purple_ranges_.push_back({{140, 25, 25}, {175, 255, 255}});
    purple_ranges_.push_back({{150, 20, 20}, {180, 255, 255}});
    purple_ranges_.push_back({{0, 20, 30}, {10, 255, 255}});
    purple_ranges_.push_back({{160, 15, 30}, {180, 255, 255}});
}

void PurplePlateDetector::set_min_area(double area) { min_area_ = area; }
void PurplePlateDetector::set_max_area(double area) { max_area_ = area; }
void PurplePlateDetector::set_aspect_ratio_range(double min_ratio, double max_ratio) {
    min_aspect_ratio_ = min_ratio;
    max_aspect_ratio_ = max_ratio;
}

void PurplePlateDetector::add_purple_range(const HSVRange& range) {
    purple_ranges_.push_back(range);
}

cv::Mat PurplePlateDetector::create_purple_mask(const cv::Mat& hsv_image) const {
    cv::Mat combined_mask = cv::Mat::zeros(hsv_image.size(), CV_8UC1);

    for (const auto& range : purple_ranges_) {
        cv::Mat mask;
        cv::inRange(hsv_image, range.lower, range.upper, mask);
        cv::bitwise_or(combined_mask, mask, combined_mask);
    }

    cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel_small);
    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE, kernel_large);

    cv::GaussianBlur(combined_mask, combined_mask, cv::Size(5, 5), 0);
    cv::threshold(combined_mask, combined_mask, 127, 255, cv::THRESH_BINARY);

    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_DILATE, kernel_small, cv::Point(-1, -1), 2);
    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_ERODE, kernel_small, cv::Point(-1, -1), 1);

    return combined_mask;
}

std::vector<cv::Point> PurplePlateDetector::refine_contour(const std::vector<cv::Point>& contour) const {
    double epsilon = 0.02 * cv::arcLength(contour, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, epsilon, true);

    if (approx.size() >= 4 && approx.size() <= 8) {
        return approx;
    }

    epsilon = 0.04 * cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, epsilon, true);
    return approx;
}

double PurplePlateDetector::compute_plate_confidence(
    const cv::Mat& image, const std::vector<cv::Point>& contour) const {

    double confidence = 0.0;

    cv::RotatedRect rrect = cv::minAreaRect(contour);
    double contour_area = cv::contourArea(contour);
    double rect_area = rrect.size.width * rrect.size.height;

    if (rect_area > 0) {
        double rectangularity = contour_area / rect_area;
        confidence += rectangularity * 30.0;
    }

    double aspect = std::max(rrect.size.width, rrect.size.height) /
                    std::max(1.0f, std::min(rrect.size.width, rrect.size.height));
    if (aspect >= 1.0 && aspect <= 4.0) {
        confidence += 20.0 * (1.0 - std::abs(aspect - 2.0) / 3.0);
    }

    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours_vec = {contour};
    cv::drawContours(mask, contours_vec, 0, 255, cv::FILLED);

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar mean_hsv = cv::mean(hsv, mask);
    double hue = mean_hsv[0];
    double sat = mean_hsv[1];
    double val = mean_hsv[2];

    if (hue >= 110 && hue <= 170 && sat >= 30 && val >= 30) {
        double hue_score = 1.0 - std::abs(hue - 140.0) / 30.0;
        double sat_score = std::min(sat / 150.0, 1.0);
        confidence += (hue_score * 15.0 + sat_score * 15.0);
    }

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hull_area = cv::contourArea(hull);
    if (hull_area > 0) {
        double convexity = contour_area / hull_area;
        confidence += convexity * 20.0;
    }

    return std::min(confidence, 100.0);
}

std::vector<PlateDetection> PurplePlateDetector::detect(const cv::Mat& image) const {
    std::vector<PlateDetection> detections;

    if (image.empty()) {
        std::cerr << "[PurplePlateDetector] Error: empty image\n";
        return detections;
    }

    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 1.5);

    cv::Mat lab;
    cv::cvtColor(blurred, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_channels;
    cv::split(lab, lab_channels);
    cv::normalize(lab_channels[0], lab_channels[0], 0, 255, cv::NORM_MINMAX);
    cv::merge(lab_channels, lab);
    cv::Mat enhanced;
    cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);

    cv::Mat hsv;
    cv::cvtColor(enhanced, hsv, cv::COLOR_BGR2HSV);

    cv::Mat purple_mask = create_purple_mask(hsv);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_area_ || area > max_area_) continue;

        cv::RotatedRect rrect = cv::minAreaRect(contour);
        float w = rrect.size.width;
        float h = rrect.size.height;
        double aspect = std::max(w, h) / std::max(1.0f, std::min(w, h));

        if (aspect < min_aspect_ratio_ || aspect > max_aspect_ratio_) continue;

        std::vector<cv::Point> refined = refine_contour(contour);
        double confidence = compute_plate_confidence(image, contour);

        if (confidence < 15.0) continue;

        PlateDetection det;
        det.bounding_box = cv::boundingRect(contour);
        det.rotated_rect = rrect;
        det.contour = contour;
        det.centroid = rrect.center;
        det.area = area;
        det.confidence = confidence;

        det.mask = cv::Mat::zeros(image.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> cnt_vec = {contour};
        cv::drawContours(det.mask, cnt_vec, 0, 255, cv::FILLED);

        detections.push_back(det);
    }

    std::sort(detections.begin(), detections.end(),
              [](const PlateDetection& a, const PlateDetection& b) {
                  return a.confidence > b.confidence;
              });

    return detections;
}

std::optional<PlateDetection> PurplePlateDetector::detect_best(const cv::Mat& image) const {
    auto detections = detect(image);
    if (detections.empty()) return std::nullopt;
    return detections.front();
}

cv::Mat PurplePlateDetector::visualize(const cv::Mat& image,
                                        const std::vector<PlateDetection>& detections) const {
    cv::Mat vis = image.clone();

    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];

        cv::Point2f vertices[4];
        det.rotated_rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(vis, vertices[j], vertices[(j + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }

        cv::rectangle(vis, det.bounding_box, cv::Scalar(255, 0, 0), 1);

        cv::circle(vis, det.centroid, 5, cv::Scalar(0, 0, 255), -1);

        std::string label = "Plate #" + std::to_string(i + 1) +
                           " conf=" + std::to_string(static_cast<int>(det.confidence)) + "%";
        cv::putText(vis, label,
                    cv::Point(det.bounding_box.x, det.bounding_box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    }

    return vis;
}
