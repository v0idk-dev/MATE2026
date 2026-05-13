// pipe_lines_multi.cpp — multi-detector line extraction with voting.
#include "pipe_lines_multi.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <cmath>

#if __has_include(<opencv2/ximgproc/fast_line_detector.hpp>)
#  include <opencv2/ximgproc/fast_line_detector.hpp>
#  define MATE_HAVE_XIMGPROC_FLD 1
#else
#  define MATE_HAVE_XIMGPROC_FLD 0
#endif

namespace mate {

namespace {

struct RawSeg { cv::Point2f p0, p1; int detector; };

double segLen(const cv::Point2f& a, const cv::Point2f& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}
double segAngleRad(const cv::Point2f& a, const cv::Point2f& b) {
    return std::atan2(b.y - a.y, b.x - a.x);
}
double angleDist(double a, double b) {
    double d = std::fmod(std::fabs(a - b), CV_PI);
    return std::min(d, CV_PI - d);
}
double perpDist(const cv::Point2f& p, const cv::Point2f& q0, const cv::Point2f& q1) {
    cv::Point2f v = q1 - q0;
    double L = std::hypot(v.x, v.y);
    if (L < 1e-6) return std::hypot(p.x - q0.x, p.y - q0.y);
    return std::fabs(((p.x - q0.x) * (-v.y) + (p.y - q0.y) * v.x) / L);
}

bool midInsideMask(const cv::Mat& mask, const cv::Point2f& p0, const cv::Point2f& p1) {
    if (mask.empty()) return true;
    cv::Point2f m = (p0 + p1) * 0.5f;
    if (m.x < 0 || m.y < 0 || m.x >= mask.cols || m.y >= mask.rows) return false;
    return mask.at<uchar>((int)m.y, (int)m.x) > 0;
}
bool insidePoly(const cv::Point2f& p, const std::vector<cv::Point2f>& poly) {
    return cv::pointPolygonTest(poly, p, false) >= 0;
}

void detectLSD(const cv::Mat& gray, std::vector<RawSeg>& out) {
    auto lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> raw; lsd->detect(gray, raw);
    for (auto& s : raw) out.push_back({{s[0],s[1]},{s[2],s[3]}, 0});
}

void detectFLD(const cv::Mat& gray, std::vector<RawSeg>& out) {
#if MATE_HAVE_XIMGPROC_FLD
    auto fld = cv::ximgproc::createFastLineDetector();
    std::vector<cv::Vec4f> raw; fld->detect(gray, raw);
    for (auto& s : raw) out.push_back({{s[0],s[1]},{s[2],s[3]}, 1});
#else
    (void)gray; (void)out;
#endif
}

void detectHough(const cv::Mat& gray_masked, std::vector<RawSeg>& out, int min_len) {
    cv::Mat blurred; cv::GaussianBlur(gray_masked, blurred, {5,5}, 0);
    // Otsu-derived Canny.
    cv::Mat tmp; double otsu = cv::threshold(blurred, tmp, 0, 255,
                                              cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat edges; cv::Canny(blurred, edges, 0.5*otsu, otsu);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1.0, CV_PI/180.0, 60, (double)min_len, 12.0);
    for (auto& l : lines)
        out.push_back({{(float)l[0],(float)l[1]},{(float)l[2],(float)l[3]}, 2});
}

// Trace skeleton into straight runs. We walk every endpoint pixel and
// follow neighbours, breaking the run whenever the cumulative angular
// deviation exceeds 5°. Each kept run becomes one RawSeg.
void detectSkeletonTrace(const cv::Mat& skeleton, std::vector<RawSeg>& out, int min_len) {
    if (skeleton.empty()) return;
    cv::Mat sk = skeleton.clone();
    auto neighbours = [&](int y, int x) {
        std::vector<cv::Point> ns;
        for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
            if (!dy&&!dx) continue;
            int yy=y+dy, xx=x+dx;
            if (yy<0||xx<0||yy>=sk.rows||xx>=sk.cols) continue;
            if (sk.at<uchar>(yy,xx)) ns.push_back({xx,yy});
        }
        return ns;
    };
    for (int y = 0; y < sk.rows; ++y)
        for (int x = 0; x < sk.cols; ++x) {
            if (!sk.at<uchar>(y,x)) continue;
            // start a walk if endpoint or any pixel still set
            cv::Point start(x, y);
            std::vector<cv::Point> path; path.push_back(start);
            sk.at<uchar>(y,x) = 0;
            cv::Point cur = start;
            while (true) {
                auto ns = neighbours(cur.y, cur.x);
                if (ns.empty()) break;
                cv::Point nxt = ns.front();
                path.push_back(nxt);
                sk.at<uchar>(nxt.y, nxt.x) = 0;
                cur = nxt;
            }
            if ((int)path.size() < min_len) continue;
            // split by curvature: greedy DP-lite.
            size_t s = 0;
            while (s + (size_t)min_len < path.size()) {
                size_t e = std::min(path.size()-1, s + (size_t)(2*min_len));
                cv::Point2f p0 = path[s], p1 = path[e];
                bool ok = true;
                for (size_t k = s + 1; k < e; ++k) {
                    if (perpDist(path[k], p0, p1) > 2.0) { ok = false; break; }
                }
                if (!ok) { e = s + min_len; }
                out.push_back({p0, cv::Point2f(path[e]), 3});
                s = e;
            }
        }
}

}  // anonymous

std::vector<LinesMultiSegment>
detectLinesMulti(const cv::Mat& bgr,
                 const cv::Mat& mask,
                 const cv::Mat& dist_px,
                 const cv::Mat& skeleton,
                 const std::vector<std::vector<cv::Point2f>>& plates,
                 const LinesMultiConfig& cfg) {
    std::vector<LinesMultiSegment> out;
    if (bgr.empty()) return out;

    cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_masked = gray;
    if (!mask.empty()) {
        gray_masked = cv::Mat::zeros(gray.size(), CV_8U);
        gray.copyTo(gray_masked, mask);
    }

    int min_len = std::max(8, (int)(cfg.min_len_frac * std::min(bgr.cols, bgr.rows)));

    std::vector<RawSeg> raw;
    if (cfg.use_lsd)      detectLSD(gray_masked, raw);
    if (cfg.use_fld)      detectFLD(gray_masked, raw);
    if (cfg.use_hough)    detectHough(gray_masked, raw, min_len);
    if (cfg.use_skeleton) detectSkeletonTrace(skeleton, raw, min_len);

    // Filter raw: length, OSD band, plate exclusion, mask containment.
    double osd_top = cfg.osd_band_frac * bgr.rows;
    double osd_bot = (1.0 - cfg.osd_band_frac) * bgr.rows;
    std::vector<RawSeg> kept;
    for (auto& s : raw) {
        if (segLen(s.p0, s.p1) < min_len) continue;
        cv::Point2f m = (s.p0 + s.p1) * 0.5f;
        if (m.y < osd_top || m.y > osd_bot) continue;
        if (!midInsideMask(mask, s.p0, s.p1)) continue;
        bool on_plate = false;
        for (auto& poly : plates) if (insidePoly(m, poly)) { on_plate = true; break; }
        if (on_plate) continue;
        kept.push_back(s);
    }

    // DBSCAN-like clustering in (angle, perp-offset) — greedy by length.
    std::sort(kept.begin(), kept.end(), [](const RawSeg& a, const RawSeg& b){
        return segLen(a.p0, a.p1) > segLen(b.p0, b.p1);
    });
    const double atol = cfg.cluster_angle_deg * CV_PI / 180.0;
    const double ptol = cfg.cluster_perp_px;
    std::vector<bool> used(kept.size(), false);

    int next_id = 0;
    for (size_t i = 0; i < kept.size(); ++i) {
        if (used[i]) continue;
        used[i] = true;
        cv::Point2f p0 = kept[i].p0, p1 = kept[i].p1;
        double ai = segAngleRad(p0, p1);
        std::vector<int> dets; dets.push_back(kept[i].detector);
        for (size_t j = i + 1; j < kept.size(); ++j) {
            if (used[j]) continue;
            double aj = segAngleRad(kept[j].p0, kept[j].p1);
            if (angleDist(ai, aj) > atol) continue;
            double d = 0.5 * (perpDist(kept[j].p0, p0, p1) + perpDist(kept[j].p1, p0, p1));
            if (d > ptol) continue;
            used[j] = true;
            dets.push_back(kept[j].detector);
            // Extend bounds along the dominant axis.
            cv::Point2f axis(std::cos(ai), std::sin(ai));
            std::array<cv::Point2f,4> pts = {p0, p1, kept[j].p0, kept[j].p1};
            float minP =  1e30f, maxP = -1e30f;
            cv::Point2f minPt, maxPt;
            for (auto& p : pts) {
                float t = p.x*axis.x + p.y*axis.y;
                if (t < minP) { minP = t; minPt = p; }
                if (t > maxP) { maxP = t; maxPt = p; }
            }
            p0 = minPt; p1 = maxPt;
        }
        std::sort(dets.begin(), dets.end()); dets.erase(std::unique(dets.begin(), dets.end()), dets.end());
        int votes = (int)dets.size();
        if (votes < cfg.min_votes) continue;

        LinesMultiSegment lms;
        lms.seg.id        = next_id++;
        lms.seg.p0        = p0;
        lms.seg.p1        = p1;
        lms.seg.length_px = segLen(p0, p1);
        lms.votes         = votes;
        // Sample distance transform for radius hint.
        if (!dist_px.empty()) {
            std::vector<float> rs;
            int N = std::max(8, (int)(lms.seg.length_px / 4));
            for (int k = 0; k <= N; ++k) {
                float t = (float)k / N;
                cv::Point2f q = p0 * (1.f - t) + p1 * t;
                int xi = (int)q.x, yi = (int)q.y;
                if (xi<0||yi<0||xi>=dist_px.cols||yi>=dist_px.rows) continue;
                rs.push_back(dist_px.at<float>(yi, xi));
            }
            if (!rs.empty()) {
                std::nth_element(rs.begin(), rs.begin()+rs.size()/2, rs.end());
                lms.radius_px = rs[rs.size()/2];
            }
        }
        out.push_back(lms);
        if ((int)out.size() >= cfg.max_segments) break;
    }
    return out;
}

}  // namespace mate
