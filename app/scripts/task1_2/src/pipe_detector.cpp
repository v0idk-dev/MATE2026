#include "pipe_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mate {

namespace {

// Angle in [-90, 90]. Lines are undirected so we fold negatives.
double lineAngleDeg(const cv::Point2f& a, const cv::Point2f& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double ang = std::atan2(dy, dx) * 180.0 / CV_PI;
    if (ang > 90.0)  ang -= 180.0;
    if (ang < -90.0) ang += 180.0;
    return ang;
}

double lineLength(const cv::Point2f& a, const cv::Point2f& b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Perpendicular distance from a point to an infinite line defined by two
// points on it. Used for collinearity testing during merge.
double perpDistance(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double L = std::sqrt(dx * dx + dy * dy);
    if (L < 1e-6) return std::hypot(p.x - a.x, p.y - a.y);
    return std::abs(dy * p.x - dx * p.y + b.x * a.y - b.y * a.x) / L;
}

// Project point p onto the infinite line through a→b; return scalar t such
// that projection = a + t*(b-a). Used to find span endpoints during merge.
double projectScalar(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double L2 = dx * dx + dy * dy;
    if (L2 < 1e-9) return 0.0;
    return ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2;
}

// Difference in undirected line angles, returned in [0, 90].
double angleDiff(double a, double b) {
    double d = std::abs(a - b);
    if (d > 90.0) d = 180.0 - d;
    return d;
}

bool linesShouldMerge(const PipeSegment2D& a, const PipeSegment2D& b,
                      double angle_tol, double perp_tol) {
    if (angleDiff(a.angle_deg, b.angle_deg) > angle_tol) return false;
    // Both endpoints of b should lie close to a's infinite line.
    if (perpDistance(b.p0, a.p0, a.p1) > perp_tol) return false;
    if (perpDistance(b.p1, a.p0, a.p1) > perp_tol) return false;
    return true;
}

// Compute the merged extent of two collinear segments by projecting all
// four endpoints onto a's direction and taking the min and max.
PipeSegment2D mergeSegments(const PipeSegment2D& a, const PipeSegment2D& b) {
    cv::Point2f origin = a.p0;
    cv::Point2f dir    = a.p1 - a.p0;
    double t0 = 0.0;
    double t1 = projectScalar(a.p1, origin, a.p1);  // = 1.0 by definition
    double t2 = projectScalar(b.p0, origin, a.p1);
    double t3 = projectScalar(b.p1, origin, a.p1);
    double tmin = std::min({t0, t1, t2, t3});
    double tmax = std::max({t0, t1, t2, t3});
    PipeSegment2D out;
    out.p0 = origin + cv::Point2f((float)(tmin * dir.x), (float)(tmin * dir.y));
    out.p1 = origin + cv::Point2f((float)(tmax * dir.x), (float)(tmax * dir.y));
    out.angle_deg = lineAngleDeg(out.p0, out.p1);
    out.length_px = lineLength(out.p0, out.p1);
    return out;
}

// Auto Canny: derive low/high from Otsu's threshold on the same gray.
std::pair<int, int> autoCanny(const cv::Mat& gray) {
    cv::Mat tmp;
    double otsu = cv::threshold(gray, tmp, 0, 255,
                                cv::THRESH_BINARY | cv::THRESH_OTSU);
    int hi = (int)std::round(otsu);
    int lo = std::max(0, (int)std::round(otsu * 0.5));
    return { lo, hi };
}

}  // namespace

std::vector<PipeSegment2D>
PipeDetector::detect(const cv::Mat& bgr) const {
    std::vector<PipeSegment2D> out;
    if (bgr.empty()) return out;

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    // LSD: parameter-free line segment detection from image gradient.
    // Returns line segments with width — pipes are the wide ones.
    cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
    std::vector<cv::Vec4f> lines;
    std::vector<double> widths, precs, nfas;
    lsd->detect(gray, lines, widths, precs, nfas);

    int min_dim = std::min(bgr.cols, bgr.rows);
    int min_len = std::max(20, (int)std::round(cfg_.min_length_frac * min_dim));

    for (size_t i = 0; i < lines.size(); ++i) {
        PipeSegment2D s;
        s.p0 = cv::Point2f(lines[i][0], lines[i][1]);
        s.p1 = cv::Point2f(lines[i][2], lines[i][3]);
        s.angle_deg = lineAngleDeg(s.p0, s.p1);
        s.length_px = lineLength(s.p0, s.p1);
        if (s.length_px < min_len) continue;

        // LSD gives line width — pipes are wider than thin edges
        double w = (i < widths.size()) ? widths[i] : 0;

        if (cfg_.reject_diagonal_deg > 0.0) {
            double a = std::abs(s.angle_deg);
            if (a > cfg_.reject_diagonal_deg && a < 90.0 - cfg_.reject_diagonal_deg) continue;
        }
        out.push_back(s);
    }

    // Merge collinear
    std::sort(out.begin(), out.end(),
              [](const PipeSegment2D& a, const PipeSegment2D& b){
                  return a.length_px > b.length_px;
              });
    std::vector<bool> consumed(out.size(), false);
    std::vector<PipeSegment2D> merged;
    for (size_t i = 0; i < out.size(); ++i) {
        if (consumed[i]) continue;
        PipeSegment2D cur = out[i];
        for (size_t j = i + 1; j < out.size(); ++j) {
            if (consumed[j]) continue;
            if (linesShouldMerge(cur, out[j], cfg_.merge_angle_deg, cfg_.merge_perp_px)) {
                cur = mergeSegments(cur, out[j]);
                consumed[j] = true;
            }
        }
        if (cur.length_px >= min_len) merged.push_back(cur);
    }

    // Parallel pair filter with brightness between lines
    std::vector<bool> has_pair(merged.size(), false);
    for (size_t i = 0; i < merged.size(); ++i) {
        for (size_t j = i + 1; j < merged.size(); ++j) {
            double da = angleDiff(merged[i].angle_deg, merged[j].angle_deg);
            if (da > 12.0) continue;
            double d1 = perpDistance(merged[j].p0, merged[i].p0, merged[i].p1);
            double d2 = perpDistance(merged[j].p1, merged[i].p0, merged[i].p1);
            double avg_perp = (d1 + d2) / 2.0;
            if (avg_perp < 5 || avg_perp > 80) continue;
            double t0_ = projectScalar(merged[j].p0, merged[i].p0, merged[i].p1);
            double t1_ = projectScalar(merged[j].p1, merged[i].p0, merged[i].p1);
            double ov = std::min(std::max(t0_,t1_), 1.0) - std::max(std::min(t0_,t1_), 0.0);
            if (ov < 0.15) continue;
            int bright = 0, total = 0;
            for (int s = 0; s < 10; ++s) {
                float t = (float)(s+1) / 11.0f;
                cv::Point2f pi(merged[i].p0.x + t*(merged[i].p1.x-merged[i].p0.x),
                               merged[i].p0.y + t*(merged[i].p1.y-merged[i].p0.y));
                cv::Point2f pj(merged[j].p0.x + t*(merged[j].p1.x-merged[j].p0.x),
                               merged[j].p0.y + t*(merged[j].p1.y-merged[j].p0.y));
                cv::Point2f mid((pi.x+pj.x)*0.5f, (pi.y+pj.y)*0.5f);
                int mx = std::clamp((int)mid.x, 0, gray.cols-1);
                int my = std::clamp((int)mid.y, 0, gray.rows-1);
                if (gray.at<uchar>(my, mx) > 100) bright++;
                total++;
            }
            if (total > 0 && bright > total / 2) {
                has_pair[i] = true;
                has_pair[j] = true;
            }
        }
    }
    std::vector<PipeSegment2D> final;
    for (size_t i = 0; i < merged.size(); ++i)
        if (has_pair[i]) final.push_back(merged[i]);
    if (final.size() >= 2) return final;
    return merged;
}


cv::Mat
PipeDetector::visualize(const cv::Mat& bgr,
                        const std::vector<PipeSegment2D>& segs) const {
    cv::Mat o = bgr.clone();
    for (size_t i = 0; i < segs.size(); ++i) {
        const auto& s = segs[i];
        cv::line(o, s.p0, s.p1, cv::Scalar(0, 255, 0), 2);
        cv::circle(o, s.p0, 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(o, s.p1, 3, cv::Scalar(255, 0, 0), -1);
        char buf[24]; std::snprintf(buf, sizeof(buf), "%zu", i);
        cv::putText(o, buf, (s.p0 + s.p1) * 0.5f,
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }
    return o;
}

std::vector<std::pair<int, int>>
pairPipesByEpipolarOrder(const std::vector<PipeSegment2D>& L,
                         const std::vector<PipeSegment2D>& R,
                         double max_y_diff_px, double max_angle_diff_deg) {
    std::vector<std::pair<int, int>> matches;
    std::vector<bool> r_used(R.size(), false);

    auto midY = [](const PipeSegment2D& s){ return 0.5 * (s.p0.y + s.p1.y); };

    // For each left segment, pick the nearest unused right segment whose
    // mid-y matches and whose orientation matches. "Score" is a weighted
    // sum of |Δy| and |Δangle|. After rectification disparities are pure
    // horizontal so endpoints' y values agree; we use mid-y because the
    // segment endpoints can be cropped differently across views.
    for (size_t i = 0; i < L.size(); ++i) {
        double yL = midY(L[i]);
        double aL = L[i].angle_deg;
        double best = 1e9;
        int    pick = -1;
        for (size_t j = 0; j < R.size(); ++j) {
            if (r_used[j]) continue;
            double dy = std::abs(yL - midY(R[j]));
            if (dy > max_y_diff_px) continue;
            double da = std::abs(aL - R[j].angle_deg);
            if (da > 90.0) da = 180.0 - da;
            if (da > max_angle_diff_deg) continue;
            double score = dy + 1.5 * da;
            if (score < best) { best = score; pick = (int)j; }
        }
        if (pick >= 0) {
            matches.push_back({(int)i, pick});
            r_used[pick] = true;
        }
    }
    return matches;
}

}  // namespace mate
