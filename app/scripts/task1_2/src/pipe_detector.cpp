#include "pipe_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

PipeDetector::PipeDetector()
    : canny_low_(50.0)
    , canny_high_(150.0)
    , hough_rho_(1.0)
    , hough_theta_deg_(1.0)
    , hough_threshold_(50)
    , hough_min_line_length_(30.0)
    , hough_max_line_gap_(15.0)
    , min_pipe_length_pixels_(40.0)
    , merge_angle_threshold_(8.0)
    , merge_distance_threshold_(20.0) {}

void PipeDetector::set_canny_thresholds(double low, double high) {
    canny_low_ = low;
    canny_high_ = high;
}

void PipeDetector::set_hough_params(double rho, double theta_deg, int threshold,
                                     double min_line_length, double max_line_gap) {
    hough_rho_ = rho;
    hough_theta_deg_ = theta_deg;
    hough_threshold_ = threshold;
    hough_min_line_length_ = min_line_length;
    hough_max_line_gap_ = max_line_gap;
}

void PipeDetector::set_min_pipe_length_pixels(double len) {
    min_pipe_length_pixels_ = len;
}

void PipeDetector::set_pipe_merge_angle_threshold(double degrees) {
    merge_angle_threshold_ = degrees;
}

void PipeDetector::set_pipe_merge_distance_threshold(double pixels) {
    merge_distance_threshold_ = pixels;
}

double PipeDetector::point_to_line_distance(const cv::Point2f& pt,
                                             const cv::Point2f& line_start,
                                             const cv::Point2f& line_end) const {
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    double len_sq = dx * dx + dy * dy;

    if (len_sq < 1e-6) {
        double ddx = pt.x - line_start.x;
        double ddy = pt.y - line_start.y;
        return std::sqrt(ddx * ddx + ddy * ddy);
    }

    double cross = std::abs((pt.x - line_start.x) * dy - (pt.y - line_start.y) * dx);
    return cross / std::sqrt(len_sq);
}

bool PipeDetector::are_collinear(const PipeSegment& a, const PipeSegment& b) const {
    double angle_diff = std::abs(a.angle - b.angle);
    if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;
    if (angle_diff > 90.0) angle_diff = 180.0 - angle_diff;

    if (angle_diff > merge_angle_threshold_) return false;

    double d1 = point_to_line_distance(b.start, a.start, a.end);
    double d2 = point_to_line_distance(b.end, a.start, a.end);
    double d3 = point_to_line_distance(a.start, b.start, b.end);
    double d4 = point_to_line_distance(a.end, b.start, b.end);
    double min_dist = std::min({d1, d2, d3, d4});

    if (min_dist > merge_distance_threshold_) return false;

    auto endpoint_dist = [](const PipeSegment& s1, const PipeSegment& s2) {
        double d1 = cv::norm(s1.end - s2.start);
        double d2 = cv::norm(s1.end - s2.end);
        double d3 = cv::norm(s1.start - s2.start);
        double d4 = cv::norm(s1.start - s2.end);
        return std::min({d1, d2, d3, d4});
    };

    double gap = endpoint_dist(a, b);
    double max_gap = std::max(a.length_pixels, b.length_pixels) * 0.5 + merge_distance_threshold_;

    return gap < max_gap;
}

std::vector<PipeSegment> PipeDetector::merge_collinear_segments(
    const std::vector<PipeSegment>& segments) const {

    if (segments.empty()) return segments;

    std::vector<bool> merged(segments.size(), false);
    std::vector<PipeSegment> result;

    for (size_t i = 0; i < segments.size(); ++i) {
        if (merged[i]) continue;

        std::vector<cv::Point2f> all_points;
        all_points.push_back(segments[i].start);
        all_points.push_back(segments[i].end);
        double total_confidence = segments[i].confidence;
        int merge_count = 1;

        for (size_t j = i + 1; j < segments.size(); ++j) {
            if (merged[j]) continue;

            if (are_collinear(segments[i], segments[j])) {
                all_points.push_back(segments[j].start);
                all_points.push_back(segments[j].end);
                total_confidence += segments[j].confidence;
                merge_count++;
                merged[j] = true;
            }
        }

        cv::Vec4f line_params;
        cv::fitLine(all_points, line_params, cv::DIST_L2, 0, 0.01, 0.01);

        double vx = line_params[0], vy = line_params[1];
        double x0 = line_params[2], y0 = line_params[3];

        double min_t = 1e9, max_t = -1e9;
        for (const auto& pt : all_points) {
            double t = (pt.x - x0) * vx + (pt.y - y0) * vy;
            min_t = std::min(min_t, t);
            max_t = std::max(max_t, t);
        }

        PipeSegment merged_seg;
        merged_seg.start = cv::Point2f(
            static_cast<float>(x0 + vx * min_t),
            static_cast<float>(y0 + vy * min_t));
        merged_seg.end = cv::Point2f(
            static_cast<float>(x0 + vx * max_t),
            static_cast<float>(y0 + vy * max_t));
        merged_seg.length_pixels = cv::norm(merged_seg.end - merged_seg.start);
        merged_seg.angle = std::atan2(vy, vx) * 180.0 / CV_PI;
        merged_seg.confidence = total_confidence / merge_count;
        merged_seg.thickness = segments[i].thickness;
        merged_seg.length_real = 0;
        merged_seg.points = all_points;

        result.push_back(merged_seg);
    }

    return result;
}

std::vector<PipeSegment> PipeDetector::detect(const cv::Mat& image) const {
    std::vector<PipeSegment> pipes;

    if (image.empty()) return pipes;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 1.0);

    cv::Mat clahe_img;
    auto clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(blurred, clahe_img);

    cv::Mat edges;
    cv::Canny(clahe_img, edges, canny_low_, canny_high_, 3);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(edges, edges, kernel, cv::Point(-1, -1), 1);
    cv::erode(edges, edges, kernel, cv::Point(-1, -1), 1);

    std::vector<cv::Vec4i> lines;
    double theta_rad = hough_theta_deg_ * CV_PI / 180.0;
    cv::HoughLinesP(edges, lines, hough_rho_, theta_rad, hough_threshold_,
                     hough_min_line_length_, hough_max_line_gap_);

    std::vector<PipeSegment> raw_segments;
    for (const auto& l : lines) {
        PipeSegment seg;
        seg.start = cv::Point2f(static_cast<float>(l[0]), static_cast<float>(l[1]));
        seg.end = cv::Point2f(static_cast<float>(l[2]), static_cast<float>(l[3]));
        seg.length_pixels = cv::norm(seg.end - seg.start);
        seg.angle = std::atan2(seg.end.y - seg.start.y, seg.end.x - seg.start.x) * 180.0 / CV_PI;
        seg.length_real = 0;
        seg.thickness = 1.0;
        seg.confidence = 50.0;

        cv::Point2f mid((seg.start.x + seg.end.x) / 2, (seg.start.y + seg.end.y) / 2);
        double perp_dx = -(seg.end.y - seg.start.y) / seg.length_pixels;
        double perp_dy = (seg.end.x - seg.start.x) / seg.length_pixels;

        int edge_count = 0;
        int sample_count = 0;
        for (double d = -15; d <= 15; d += 1.0) {
            int px = static_cast<int>(mid.x + perp_dx * d);
            int py = static_cast<int>(mid.y + perp_dy * d);
            if (px >= 0 && px < edges.cols && py >= 0 && py < edges.rows) {
                sample_count++;
                if (edges.at<uchar>(py, px) > 0) edge_count++;
            }
        }

        if (sample_count > 0) {
            double edge_ratio = static_cast<double>(edge_count) / sample_count;
            if (edge_ratio > 0.3) {
                seg.confidence = 30.0;
            }
        }

        seg.confidence += std::min(seg.length_pixels / 100.0, 30.0) * 1.0;

        if (seg.length_pixels >= min_pipe_length_pixels_) {
            raw_segments.push_back(seg);
        }
    }

    pipes = merge_collinear_segments(raw_segments);

    std::sort(pipes.begin(), pipes.end(),
              [](const PipeSegment& a, const PipeSegment& b) {
                  return a.length_pixels > b.length_pixels;
              });

    return pipes;
}

cv::Mat PipeDetector::visualize(const cv::Mat& image,
                                 const std::vector<PipeSegment>& pipes,
                                 const ScaleInfo& scale) const {
    cv::Mat vis = image.clone();

    for (size_t i = 0; i < pipes.size(); ++i) {
        const auto& pipe = pipes[i];

        cv::Scalar color(0, 255, 0);
        cv::line(vis, pipe.start, pipe.end, color, 2, cv::LINE_AA);

        cv::circle(vis, pipe.start, 4, cv::Scalar(255, 0, 0), -1);
        cv::circle(vis, pipe.end, 4, cv::Scalar(0, 0, 255), -1);

        cv::Point2f mid((pipe.start.x + pipe.end.x) / 2,
                         (pipe.start.y + pipe.end.y) / 2);

        std::string label;
        if (pipe.length_real > 0) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.1f cm", pipe.length_real);
            label = buf;
        } else if (scale.pixels_per_cm > 0) {
            double len_cm = pipe.length_pixels * scale.cm_per_pixel;
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.1f cm", len_cm);
            label = buf;
        } else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.0f px", pipe.length_pixels);
            label = buf;
        }

        int font = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.55;
        int thickness = 2;
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, font, font_scale, thickness, &baseline);

        cv::Point text_pos(
            static_cast<int>(mid.x - text_size.width / 2),
            static_cast<int>(mid.y - 10));

        text_pos.x = std::max(0, std::min(text_pos.x, vis.cols - text_size.width - 4));
        text_pos.y = std::max(text_size.height + 4, std::min(text_pos.y, vis.rows - 4));

        cv::rectangle(vis,
            cv::Point(text_pos.x - 3, text_pos.y - text_size.height - 3),
            cv::Point(text_pos.x + text_size.width + 3, text_pos.y + baseline + 3),
            cv::Scalar(0, 0, 0), cv::FILLED);
        cv::rectangle(vis,
            cv::Point(text_pos.x - 3, text_pos.y - text_size.height - 3),
            cv::Point(text_pos.x + text_size.width + 3, text_pos.y + baseline + 3),
            color, 1);

        cv::putText(vis, label, text_pos, font, font_scale, cv::Scalar(0, 255, 255), thickness);
    }

    return vis;
}

ReferenceSquareDetector::ReferenceSquareDetector()
    : known_side_cm_(10.0)
    , min_area_(200.0)
    , max_area_(100000.0) {}

void ReferenceSquareDetector::set_known_side_cm(double side_cm) {
    known_side_cm_ = side_cm;
}

void ReferenceSquareDetector::set_min_area(double area) {
    min_area_ = area;
}

void ReferenceSquareDetector::set_max_area(double area) {
    max_area_ = area;
}

std::string ReferenceSquareDetector::classify_color(const cv::Scalar& bgr) const {
    double b = bgr[0], g = bgr[1], r = bgr[2];

    cv::Mat bgr_pixel(1, 1, CV_8UC3, cv::Scalar(b, g, r));
    cv::Mat hsv_pixel;
    cv::cvtColor(bgr_pixel, hsv_pixel, cv::COLOR_BGR2HSV);
    double h = hsv_pixel.at<cv::Vec3b>(0, 0)[0];
    double s = hsv_pixel.at<cv::Vec3b>(0, 0)[1];
    double v = hsv_pixel.at<cv::Vec3b>(0, 0)[2];

    if (s < 40) {
        if (v < 80) return "black";
        if (v > 200) return "white";
        return "gray";
    }

    if (h < 10 || h > 170) return "red";
    if (h >= 10 && h < 25) return "orange";
    if (h >= 25 && h < 35) return "yellow";
    if (h >= 35 && h < 85) return "green";
    if (h >= 85 && h < 130) return "blue";
    if (h >= 130 && h < 170) return "purple";

    return "unknown";
}

double ReferenceSquareDetector::compute_squareness(const std::vector<cv::Point>& contour) const {
    cv::RotatedRect rrect = cv::minAreaRect(contour);
    float w = rrect.size.width;
    float h = rrect.size.height;
    if (w < 1 || h < 1) return 0;

    double aspect = std::max(w, h) / std::min(w, h);
    double aspect_score = std::max(0.0, 1.0 - std::abs(aspect - 1.0) * 2.0);

    double contour_area = cv::contourArea(contour);
    double rect_area = w * h;
    double fill_ratio = (rect_area > 0) ? contour_area / rect_area : 0;

    std::vector<cv::Point> approx;
    double epsilon = 0.04 * cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, epsilon, true);
    double vertex_score = (approx.size() == 4) ? 1.0 : std::max(0.0, 1.0 - std::abs((int)approx.size() - 4) * 0.2);

    return (aspect_score * 0.35 + fill_ratio * 0.35 + vertex_score * 0.3);
}

std::vector<ReferenceSquare> ReferenceSquareDetector::detect(const cv::Mat& image) const {
    std::vector<ReferenceSquare> squares;

    if (image.empty()) return squares;

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat sat_channel;
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    sat_channel = hsv_channels[1];

    cv::Mat sat_thresh;
    cv::threshold(sat_channel, sat_thresh, 50, 255, cv::THRESH_BINARY);

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);

    cv::Mat combined;
    cv::bitwise_or(sat_thresh, edges, combined);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(combined, combined, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_area_ || area > max_area_) continue;

        double squareness = compute_squareness(contour);
        if (squareness < 0.5) continue;

        cv::RotatedRect rrect = cv::minAreaRect(contour);
        float w = rrect.size.width;
        float h = rrect.size.height;
        double aspect = std::max(w, h) / std::min(w, h);
        if (aspect > 1.5) continue;

        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> cnt_vec = {contour};
        cv::drawContours(mask, cnt_vec, 0, 255, cv::FILLED);
        cv::Scalar mean_color = cv::mean(image, mask);

        ReferenceSquare sq;
        sq.bounding_box = cv::boundingRect(contour);
        sq.rotated_rect = rrect;
        sq.contour = contour;
        sq.centroid = rrect.center;
        sq.area = area;
        sq.side_length_pixels = (w + h) / 2.0;
        sq.confidence = squareness * 100.0;
        sq.mean_color_bgr = mean_color;
        sq.color_name = classify_color(mean_color);

        squares.push_back(sq);
    }

    std::sort(squares.begin(), squares.end(),
              [](const ReferenceSquare& a, const ReferenceSquare& b) {
                  return a.confidence > b.confidence;
              });

    return squares;
}

ScaleInfo ReferenceSquareDetector::compute_scale(const std::vector<ReferenceSquare>& squares) const {
    ScaleInfo info;
    info.pixels_per_cm = 0;
    info.cm_per_pixel = 0;
    info.num_references = 0;
    info.scale_confidence = 0;

    if (squares.empty()) return info;

    std::vector<double> scales;
    for (const auto& sq : squares) {
        if (sq.confidence >= 65.0) {
            double ppc = sq.side_length_pixels / known_side_cm_;
            scales.push_back(ppc);
        }
    }

    if (scales.empty()) return info;

    std::sort(scales.begin(), scales.end());
    double median = scales[scales.size() / 2];

    std::vector<double> filtered;
    for (double s : scales) {
        if (std::abs(s - median) < median * 0.5) {
            filtered.push_back(s);
        }
    }

    if (filtered.empty()) filtered = scales;

    double sum = 0;
    for (double s : filtered) sum += s;
    double mean = sum / filtered.size();

    info.pixels_per_cm = mean;
    info.cm_per_pixel = 1.0 / mean;
    info.num_references = static_cast<int>(filtered.size());

    double max_scale = *std::max_element(filtered.begin(), filtered.end());
    double min_scale = *std::min_element(filtered.begin(), filtered.end());
    double range_ratio = (max_scale > 0) ? (max_scale - min_scale) / max_scale : 0;
    info.scale_confidence = std::max(0.0, (1.0 - range_ratio)) * 100.0;

    return info;
}

cv::Mat ReferenceSquareDetector::visualize(const cv::Mat& image,
                                            const std::vector<ReferenceSquare>& squares) const {
    cv::Mat vis = image.clone();

    for (size_t i = 0; i < squares.size(); ++i) {
        const auto& sq = squares[i];

        cv::Point2f vertices[4];
        sq.rotated_rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(vis, vertices[j], vertices[(j + 1) % 4],
                     cv::Scalar(0, 255, 255), 2);
        }

        cv::circle(vis, sq.centroid, 4, cv::Scalar(0, 0, 255), -1);

        std::string label = sq.color_name + " " +
            std::to_string(static_cast<int>(sq.side_length_pixels)) + "px";
        cv::putText(vis, label,
            cv::Point(sq.bounding_box.x, sq.bounding_box.y - 8),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
    }

    return vis;
}
