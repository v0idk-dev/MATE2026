#include "plate_fusion.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>

namespace mate {

static double polyIoU(const std::vector<cv::Point2f>& a,
                      const std::vector<cv::Point2f>& b) {
    if (a.size() < 3 || b.size() < 3) return 0.0;
    std::vector<cv::Point2f> inter;
    float ai = static_cast<float>(cv::intersectConvexConvex(a, b, inter, true));
    if (ai <= 0) return 0.0;
    float aA = static_cast<float>(cv::contourArea(a));
    float aB = static_cast<float>(cv::contourArea(b));
    float u = aA + aB - ai;
    return u > 0 ? ai / u : 0.0;
}

std::vector<PlateDetection>
fusePlates(const std::vector<PlateDetection>& lab,
           const std::vector<PlateDetection>& vision,
           double iou_th) {
    std::vector<PlateDetection> out;
    std::vector<bool> v_used(vision.size(), false);

    for (const auto& l : lab) {
        int best = -1; double best_iou = 0.0;
        for (size_t j = 0; j < vision.size(); ++j) {
            if (v_used[j]) continue;
            std::vector<cv::Point2f> la(l.corners.begin(), l.corners.end());
            std::vector<cv::Point2f> va(vision[j].corners.begin(), vision[j].corners.end());
            double i = polyIoU(la, va);
            if (i > best_iou) { best_iou = i; best = static_cast<int>(j); }
        }
        if (best >= 0 && best_iou >= iou_th) {
            v_used[best] = true;
            // Pick the one with higher confidence; corners_px from the
            // higher-confidence detection. Keep area from LAB (more reliable
            // when colour is present).
            const auto& w = (l.confidence >= vision[best].confidence) ? l : vision[best];
            PlateDetection merged = w;
            merged.area_px    = l.area_px;
            merged.confidence = std::max(l.confidence, vision[best].confidence);
            out.push_back(std::move(merged));
        } else {
            out.push_back(l);
        }
    }
    for (size_t j = 0; j < vision.size(); ++j)
        if (!v_used[j]) out.push_back(vision[j]);

    for (size_t i = 0; i < out.size(); ++i) out[i].id = static_cast<int>(i);
    return out;
}

}  // namespace mate
