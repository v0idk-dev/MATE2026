#include "pipe_lsd.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <cmath>

namespace mate {

static double segLen(const cv::Vec4f& s) {
    double dx = s[2] - s[0], dy = s[3] - s[1];
    return std::sqrt(dx*dx + dy*dy);
}
static double segAngle(const cv::Vec4f& s) {
    return std::atan2(s[3] - s[1], s[2] - s[0]);
}
static cv::Point2f segMid(const cv::Vec4f& s) {
    return cv::Point2f(0.5f*(s[0]+s[2]), 0.5f*(s[1]+s[3]));
}
static bool insidePoly(const cv::Point2f& p, const std::vector<cv::Point2f>& poly) {
    return cv::pointPolygonTest(poly, p, false) >= 0;
}

std::vector<PipeSegment2D>
lsdPipes(const cv::Mat& bgr,
         const std::vector<std::vector<cv::Point2f>>& plates,
         const PipeLSDConfig& cfg) {
    std::vector<PipeSegment2D> out;
    if (bgr.empty()) return out;

    cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    auto lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> raw;
    lsd->detect(gray, raw);

    const double min_len   = cfg.min_len_frac * bgr.cols;
    const double osd_top   = cfg.osd_band_frac * bgr.rows;
    const double osd_bot   = (1.0 - cfg.osd_band_frac) * bgr.rows;

    // Filter raw segments.
    std::vector<cv::Vec4f> kept;
    kept.reserve(raw.size());
    for (auto& s : raw) {
        if (segLen(s) < min_len) continue;
        cv::Point2f m = segMid(s);
        if (m.y < osd_top || m.y > osd_bot) continue;   // skip OSD bands
        bool on_plate = false;
        for (auto& poly : plates) if (insidePoly(m, poly)) { on_plate = true; break; }
        if (on_plate) continue;
        kept.push_back(s);
    }

    // Cluster near-collinear segments greedily (sort by length desc).
    std::sort(kept.begin(), kept.end(),
              [](const cv::Vec4f& a, const cv::Vec4f& b){ return segLen(a) > segLen(b); });

    const double ang_tol = cfg.cluster_angle_deg * CV_PI / 180.0;
    const double d_tol   = cfg.cluster_dist_px;
    std::vector<bool> used(kept.size(), false);
    int next_id = 0;

    for (size_t i = 0; i < kept.size(); ++i) {
        if (used[i]) continue;
        used[i] = true;
        cv::Vec4f cur = kept[i];
        double ai = segAngle(cur);
        cv::Point2f p0(cur[0], cur[1]), p1(cur[2], cur[3]);

        for (size_t j = i + 1; j < kept.size(); ++j) {
            if (used[j]) continue;
            double aj = segAngle(kept[j]);
            double da = std::fabs(std::fmod(ai - aj + CV_PI, CV_PI));
            if (da > ang_tol && std::fabs(CV_PI - da) > ang_tol) continue;
            // perpendicular distance from p0 to kept[j]'s line
            cv::Point2f q0(kept[j][0], kept[j][1]), q1(kept[j][2], kept[j][3]);
            cv::Point2f v = q1 - q0;
            double L = std::hypot(v.x, v.y);
            if (L < 1e-3) continue;
            cv::Point2f n(-v.y / L, v.x / L);
            double d = std::fabs((p0.x - q0.x) * n.x + (p0.y - q0.y) * n.y);
            if (d > d_tol) continue;
            used[j] = true;
            // Extend: keep the two extreme endpoints along the dominant axis.
            std::array<cv::Point2f, 4> pts = {p0, p1, q0, q1};
            cv::Point2f axis(std::cos(ai), std::sin(ai));
            float minP = std::numeric_limits<float>::max();
            float maxP = -std::numeric_limits<float>::max();
            cv::Point2f minPt, maxPt;
            for (auto& p : pts) {
                float t = p.x * axis.x + p.y * axis.y;
                if (t < minP) { minP = t; minPt = p; }
                if (t > maxP) { maxP = t; maxPt = p; }
            }
            p0 = minPt; p1 = maxPt;
        }

        PipeSegment2D seg;
        seg.id = next_id++;
        seg.p0 = p0;
        seg.p1 = p1;
        seg.length_px = std::hypot(p1.x - p0.x, p1.y - p0.y);
        out.push_back(std::move(seg));
        if (static_cast<int>(out.size()) >= cfg.max_segments) break;
    }
    return out;
}

}  // namespace mate
