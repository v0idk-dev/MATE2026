#include "lab_segment.hpp"
#include "stereo_math.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>

namespace mate {

static cv::Vec3f hsvHueToLAB(int hue_opencv) {
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue_opencv, 220, 200));
    cv::Mat bgr, lab;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
    auto p = lab.at<cv::Vec3b>(0, 0);
    return cv::Vec3f(static_cast<float>(p[0]),
                     static_cast<float>(p[1]),
                     static_cast<float>(p[2]));
}

static double mahalanobis3(const cv::Vec3f& x, const cv::Vec3f& mu, const cv::Vec3f& inv_var) {
    cv::Vec3f d = x - mu;
    return d[0]*d[0]*inv_var[0] + d[1]*d[1]*inv_var[1] + d[2]*d[2]*inv_var[2];
}

// Resolve the expected plate pixel size from whatever the caller provided.
// Order of precedence:
//   1. expected_plate_px (direct override)
//   2. focal_px AND subject_distance_m → s_px = f · s_m / Z   (the math)
//   3. fallback: 5% of image width (loose; multi-scale gates open up)
static double resolveExpPx(const LabSegmentConfig& cfg, int img_w) {
    if (cfg.expected_plate_px > 0.0) return cfg.expected_plate_px;
    if (cfg.focal_px > 0.0 && cfg.subject_distance_m > 0.0)
        return expectedPlatePx(cfg.focal_px, cfg.plate_side_m, cfg.subject_distance_m);
    return std::max(8.0, img_w * 0.05);  // last-resort fallback
}

std::vector<PlateDetection>
labSegmentPlates(const cv::Mat& bgr, const LabSegmentConfig& cfg) {
    std::vector<PlateDetection> out;
    if (bgr.empty()) return out;

    // 1. BGR → LAB
    cv::Mat lab; cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    // 2. Target centroid in LAB
    const cv::Vec3f C = hsvHueToLAB(cfg.target_hue_opencv);

    // 3. Per-channel variance from the image (Mahalanobis denominator)
    cv::Scalar mu, sigma; cv::meanStdDev(lab, mu, sigma);
    cv::Vec3f inv_var(
        1.0f / std::max<float>(1.0f, static_cast<float>(sigma[0]*sigma[0])),
        1.0f / std::max<float>(1.0f, static_cast<float>(sigma[1]*sigma[1])),
        1.0f / std::max<float>(1.0f, static_cast<float>(sigma[2]*sigma[2])));
    inv_var[0] *= 0.25f;   // de-emphasise lightness; chroma is the signal

    const double th = std::pow(static_cast<double>(cfg.hue_tol_deg) / 30.0, 2.0);
    cv::Mat mask(lab.rows, lab.cols, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < lab.rows; ++y) {
        const cv::Vec3b* row = lab.ptr<cv::Vec3b>(y);
        uchar* m = mask.ptr<uchar>(y);
        for (int x = 0; x < lab.cols; ++x) {
            cv::Vec3f p(static_cast<float>(row[x][0]),
                        static_cast<float>(row[x][1]),
                        static_cast<float>(row[x][2]));
            if (mahalanobis3(p, C, inv_var) < th) m[x] = 255;
        }
    }

    // 5. Morphology with SE sized from the GEOMETRIC expected plate pixel
    //    size. KEY MATH: s_px = f · s_m / Z (Z = subject distance, not B).
    const double exp_px = resolveExpPx(cfg, bgr.cols);
    const int se = std::max(2, static_cast<int>(exp_px * 0.07));
    cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, {se, se});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);

    // 6. Connected components → per-blob rect + sub-pixel corners
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    int next_id = 0;
    // When subject distance is known we constrain area tightly; when only
    // the loose fallback is used we open the gates.
    const bool tight = (cfg.expected_plate_px > 0.0) ||
                       (cfg.focal_px > 0.0 && cfg.subject_distance_m > 0.0);
    const double area_min = tight ? 0.25 * exp_px * exp_px : 4.0   * 4.0;
    const double area_max = tight ? 4.00 * exp_px * exp_px : (bgr.rows * bgr.cols * 0.25);
    const double aspect_max = tight ? 1.6 : 2.2;

    struct Cand { PlateDetection det; double score; };
    std::vector<Cand> cands;

    for (auto& c : contours) {
        double a = cv::contourArea(c);
        if (a < area_min || a > area_max) continue;
        cv::RotatedRect rr = cv::minAreaRect(c);
        cv::Point2f corners[4]; rr.points(corners);

        double w = rr.size.width, h = rr.size.height;
        if (w < 1 || h < 1) continue;
        double ar = std::max(w, h) / std::min(w, h);
        if (ar > aspect_max) continue;

        std::vector<cv::Point2f> pts(corners, corners + 4);
        if (cfg.refine_corners) {
            cv::cornerSubPix(gray, pts, {5, 5}, {-1, -1},
                             {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01});
        }

        PlateDetection d;
        d.id     = next_id++;
        d.area_px = a;
        d.confidence = 1.0 - std::min(1.0, std::abs(ar - 1.0));
        d.center = rr.center;
        for (int k = 0; k < 4; ++k) d.corners[k] = pts[k];
        const double area_term = tight
            ? 1.0 / std::max(1.0, std::abs(a - exp_px*exp_px))
            : 0.0;
        cands.push_back({std::move(d), d.confidence + area_term});
    }

    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b){ return a.score > b.score; });
    out.reserve(std::min<size_t>(cands.size(), cfg.expected_plates));
    for (size_t i = 0; i < cands.size() && static_cast<int>(i) < cfg.expected_plates; ++i)
        out.push_back(std::move(cands[i].det));

    return out;
}

std::vector<PlateDetection>
labSegmentPlates(const cv::Mat& bgr, int target_hue, int hue_tol) {
    LabSegmentConfig c;
    c.target_hue_opencv = target_hue;
    c.hue_tol_deg       = hue_tol;
    // Geometry unset → multi-scale fallback.
    return labSegmentPlates(bgr, c);
}

}  // namespace mate
