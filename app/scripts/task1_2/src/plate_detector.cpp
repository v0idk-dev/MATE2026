#include "plate_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace mate {

namespace {

// Order the 4 corners of a minAreaRect canonically: TL, TR, BR, BL, with
// "top" being the smaller-y row. Stable enough for our needs even when
// the plate is rotated.
std::array<cv::Point2f, 4> orderCorners(const cv::Point2f raw[4]) {
    std::array<cv::Point2f, 4> pts{ raw[0], raw[1], raw[2], raw[3] };
    // Sort by y, then split into "top two" and "bottom two".
    std::sort(pts.begin(), pts.end(),
              [](const cv::Point2f& a, const cv::Point2f& b){ return a.y < b.y; });
    cv::Point2f top1 = pts[0], top2 = pts[1];
    cv::Point2f bot1 = pts[2], bot2 = pts[3];
    // Within each pair, smaller-x is "left".
    cv::Point2f tl = (top1.x <= top2.x) ? top1 : top2;
    cv::Point2f tr = (top1.x <= top2.x) ? top2 : top1;
    cv::Point2f bl = (bot1.x <= bot2.x) ? bot1 : bot2;
    cv::Point2f br = (bot1.x <= bot2.x) ? bot2 : bot1;
    return { tl, tr, br, bl };
}

// Confidence in [0, 100], blending three signals:
//   • fill_ratio: contour area / rotated rect area (1.0 ideal, ≥0.85 great).
//   • |aspect_ratio - 1|: plates are square; small spread is good.
//   • size sanity: passes if within [min_area_frac, max_area_frac]; gated
//     before this function is even called, so we always have area>0 here.
double computeConfidence(double fill_ratio, double aspect_ratio) {
    // Fill ratio: 1.0 = perfect, tape on pipe is usually 0.6-0.9
    double fr = std::clamp(fill_ratio, 0.0, 1.0);
    // Aspect: 1.0 = square. Plates are square-ish. 
    // Score: 1.0 at aspect=1, drops to 0 at aspect=3
    double ar = std::clamp((3.0 - aspect_ratio) / 2.0, 0.0, 1.0);
    return 100.0 * (0.5 * fr + 0.5 * ar);
}

}  // namespace

cv::Mat PlateDetector::buildHsvMask(const cv::Mat& bgr) const {
    cv::Mat lab;
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch;
    cv::split(lab, ch);
    // ch[0]=L, ch[1]=a (green↔red), ch[2]=b (blue↔yellow)

    cv::Mat mask;
    int h = cfg_.target_h;

    // Map OpenCV hue ranges to LAB channel thresholds:
    //   Red/Pink/Magenta (h=0-10 or h=140-179): a > 138
    //   Orange/Yellow    (h=11-35):              b > 138
    //   Green            (h=36-85):              a < 118
    //   Blue/Cyan        (h=86-105):             b < 118
    //   Purple           (h=106-139):            a > 133, b < 123
    if ((h >= 0 && h <= 10) || h >= 140) {
        cv::threshold(ch[1], mask, 138, 255, cv::THRESH_BINARY);
    } else if (h >= 11 && h <= 35) {
        cv::threshold(ch[2], mask, 138, 255, cv::THRESH_BINARY);
    } else if (h >= 36 && h <= 85) {
        cv::threshold(ch[1], mask, 118, 255, cv::THRESH_BINARY_INV);
    } else if (h >= 86 && h <= 105) {
        cv::threshold(ch[2], mask, 118, 255, cv::THRESH_BINARY_INV);
    } else {
        // Purple range — both a high and b low
        cv::Mat m1, m2;
        cv::threshold(ch[1], m1, 133, 255, cv::THRESH_BINARY);
        cv::threshold(ch[2], m2, 123, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(m1, m2, mask);
    }

    if (cfg_.close_kernel_px > 0) {
        cv::Mat k = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(cfg_.close_kernel_px, cfg_.close_kernel_px));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
    }
    return mask;
}

std::vector<PlateDetection>
PlateDetector::detect(const cv::Mat& bgr, int expected_count) const {
    std::vector<PlateDetection> out;
    if (bgr.empty()) return out;

    cv::Mat mask = buildHsvMask(bgr);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    const double image_area = double(bgr.cols) * double(bgr.rows);
    const double min_area = cfg_.min_area_frac * image_area;
    const double max_area = cfg_.max_area_frac * image_area;

    // Grayscale once; cornerSubPix is iterative on it.
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    for (size_t ci = 0; ci < contours.size(); ++ci) {
        const auto& c = contours[ci];
        double a = std::abs(cv::contourArea(c));
        if (a < min_area || a > max_area) continue;

        cv::RotatedRect rr = cv::minAreaRect(c);
        double rect_area = double(rr.size.width) * double(rr.size.height);
        if (rect_area < 1.0) continue;

        cv::Point2f raw[4];
        rr.points(raw);
        auto ordered = orderCorners(raw);

        // Refine each corner via cornerSubPix. Skip corners that fall
        // outside the image; cornerSubPix would crash there.
        std::vector<cv::Point2f> refined(ordered.begin(), ordered.end());
        std::vector<cv::Point2f> safe;
        std::vector<int> safe_idx;
        for (int i = 0; i < 4; ++i) {
            const auto& p = refined[i];
            int margin = cfg_.subpix_win + 2;
            if (p.x > margin && p.y > margin &&
                p.x < gray.cols - margin && p.y < gray.rows - margin) {
                safe.push_back(p);
                safe_idx.push_back(i);
            }
        }
        if (!safe.empty()) {
            cv::cornerSubPix(gray, safe,
                             cv::Size(cfg_.subpix_win, cfg_.subpix_win),
                             cv::Size(cfg_.subpix_zerozone, cfg_.subpix_zerozone),
                             cv::TermCriteria(
                                 cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                 cfg_.subpix_max_iter, cfg_.subpix_eps));
            for (size_t k = 0; k < safe_idx.size(); ++k) {
                refined[safe_idx[k]] = safe[k];
            }
        }

        PlateDetection d;
        d.contour_index = (int)ci;
        for (int i = 0; i < 4; ++i) d.corners[i] = refined[i];
        // Center: average of refined corners (more accurate than the
        // unrefined RotatedRect center).
        cv::Point2f c0 = refined[0] + refined[1] + refined[2] + refined[3];
        d.center = cv::Point2f(c0.x * 0.25f, c0.y * 0.25f);

        d.area_px = a;
        d.fill_ratio = a / rect_area;
        double w = std::max(rr.size.width, rr.size.height);
        double h = std::min(rr.size.width, rr.size.height);
        d.aspect_ratio = (h > 1e-6) ? (w / h) : 999.0;
        
        // Measure how close this contour's average color is to the target
        cv::Mat lab_img;
        cv::cvtColor(bgr, lab_img, cv::COLOR_BGR2Lab);
        cv::Mat contour_mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
        cv::drawContours(contour_mask, contours, (int)ci, cv::Scalar(255), -1);
        cv::Scalar mean_lab = cv::mean(lab_img, contour_mask);
        // Distance in a,b channels from the target plate color.
        // Convert target hue to expected LAB a,b direction.
        // Pink/red: high a. Scale distance by how far a is from peak.
        double peak_a = 180.0, peak_b = 128.0; // ideal pink
        double dist_a = std::abs(mean_lab[1] - peak_a) / 60.0;
        double dist_b = std::abs(mean_lab[2] - peak_b) / 60.0;
        double color_dist = std::clamp(std::sqrt(dist_a*dist_a + dist_b*dist_b), 0.0, 1.0);

        d.confidence = computeConfidence(d.fill_ratio, d.aspect_ratio);

        out.push_back(d);
    }

    // Sort by descending confidence.
    std::sort(out.begin(), out.end(),
                [](const PlateDetection& a, const PlateDetection& b){
                    return a.confidence > b.confidence;
                });
        if (expected_count > 0 && (int)out.size() > expected_count) {
            out.resize(expected_count);
        }

        FILE* f = fopen("/tmp/plate_confs.txt", "w");
    if (f) {
        for (size_t i = 0; i < out.size(); ++i)
            fprintf(f, "#%zu  conf=%.1f  fill=%.2f  ar=%.2f  area=%.0f  center=(%.0f,%.0f)\n",
                    i+1, out[i].confidence, out[i].fill_ratio, out[i].aspect_ratio,
                    out[i].area_px, out[i].center.x, out[i].center.y);
        fclose(f);
    }

    return out;
}

cv::Mat
PlateDetector::visualize(const cv::Mat& bgr,
                         const std::vector<PlateDetection>& dets) const {
    cv::Mat out = bgr.clone();
    for (size_t i = 0; i < dets.size(); ++i) {
        const auto& d = dets[i];
        cv::Scalar col = (d.confidence >= 70.0) ? cv::Scalar(0, 255, 0)
                                                : cv::Scalar(0, 165, 255);
        for (int j = 0; j < 4; ++j) {
            cv::line(out, d.corners[j], d.corners[(j + 1) % 4], col, 2);
            cv::circle(out, d.corners[j], 3, cv::Scalar(0, 0, 255), -1);
        }
        cv::circle(out, d.center, 4, cv::Scalar(255, 255, 0), -1);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "#%zu  %.0f%%", i + 1, d.confidence);
        cv::putText(out, buf, d.center + cv::Point2f(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);
    }
    return out;
}

}  // namespace mate
