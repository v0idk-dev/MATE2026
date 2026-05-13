// pipe_ransac.cpp — MSAC line refit on edge pixels in PVC mask.
#include "pipe_ransac.hpp"
#include <opencv2/imgproc.hpp>
#include <random>
#include <cmath>

namespace mate {

namespace {

double perpDistAB(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
    cv::Point2f v = b - a;
    double L = std::hypot(v.x, v.y);
    if (L < 1e-6) return std::hypot(p.x - a.x, p.y - a.y);
    return std::fabs(((p.x - a.x) * (-v.y) + (p.y - a.y) * v.x) / L);
}

void collectEdgePixels(const cv::Mat& bgr,
                       const cv::Mat& mask,
                       const cv::Point2f& a, const cv::Point2f& b,
                       double band,
                       std::vector<cv::Point2f>& pts) {
    cv::Mat gray, edges;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, {3,3}, 0);
    cv::Mat tmp; double otsu = cv::threshold(gray, tmp, 0, 255,
                                              cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Canny(gray, edges, 0.5*otsu, otsu);
    if (!mask.empty()) cv::bitwise_and(edges, mask, edges);

    cv::Point2f mid = (a + b) * 0.5f;
    cv::Point2f v = b - a;
    double len = std::hypot(v.x, v.y);
    int margin = (int)(len * 0.5 + band + 4);
    int x0 = std::max(0, (int)mid.x - margin);
    int y0 = std::max(0, (int)mid.y - margin);
    int x1 = std::min(edges.cols - 1, (int)mid.x + margin);
    int y1 = std::min(edges.rows - 1, (int)mid.y + margin);
    for (int y = y0; y <= y1; ++y) {
        const uchar* ep = edges.ptr<uchar>(y);
        for (int x = x0; x <= x1; ++x) {
            if (!ep[x]) continue;
            cv::Point2f p((float)x, (float)y);
            if (perpDistAB(p, a, b) <= band) pts.push_back(p);
        }
    }
}

}  // namespace

std::vector<PipeRansacResult>
ransacRefitLines(const cv::Mat& bgr,
                 const cv::Mat& mask,
                 const std::vector<LinesMultiSegment>& cand,
                 const PipeRansacConfig& cfg) {
    std::vector<PipeRansacResult> out;
    out.reserve(cand.size());

    // thread_local + nondeterministic seed: avoids identical RANSAC trial
    // sequences across pipes within the same frame (which would correlate
    // failure modes) while staying thread-safe if callers ever parallelise.
    static thread_local std::mt19937 rng{std::random_device{}()};
    for (auto& c : cand) {
        std::vector<cv::Point2f> pts;
        collectEdgePixels(bgr, mask, c.seg.p0, c.seg.p1, cfg.band_px, pts);
        if ((int)pts.size() < cfg.min_inliers) continue;

        // MSAC.
        // Bias against picking two points on the *same* edge of the pipe by
        // splitting edge-pixels along the candidate line direction and
        // forcing samples from opposite halves. Dramatically improves
        // convergence on thick pipes (where uniform sampling biases the
        // line toward whichever edge has more inliers).
        cv::Point2f cdir = c.seg.p1 - c.seg.p0;
        double clen = std::hypot(cdir.x, cdir.y);
        if (clen < 1e-3) continue;
        cv::Point2f cunit(cdir.x/clen, cdir.y/clen);
        cv::Point2f cmid = (c.seg.p0 + c.seg.p1) * 0.5f;
        std::vector<size_t> firstHalf, secondHalf;
        firstHalf.reserve(pts.size()/2); secondHalf.reserve(pts.size()/2);
        for (size_t k = 0; k < pts.size(); ++k) {
            float t = (pts[k].x - cmid.x) * cunit.x + (pts[k].y - cmid.y) * cunit.y;
            (t < 0 ? firstHalf : secondHalf).push_back(k);
        }
        if (firstHalf.empty() || secondHalf.empty()) continue;
        std::uniform_int_distribution<size_t> Uf(0, firstHalf.size() - 1);
        std::uniform_int_distribution<size_t> Us(0, secondHalf.size() - 1);

        cv::Point2f bestA = c.seg.p0, bestB = c.seg.p1;
        int    best_in = 0;
        double best_cost = 1e30;
        const double T = cfg.inlier_tol_px;
        const double T2= T * T;

        for (int it = 0; it < cfg.iters; ++it) {
            size_t i = firstHalf[Uf(rng)], j = secondHalf[Us(rng)];
            cv::Point2f a = pts[i], b = pts[j];
            if (std::hypot(a.x - b.x, a.y - b.y) < clen * 0.3) continue;
            double cost = 0.0; int in = 0;
            for (auto& p : pts) {
                double d = perpDistAB(p, a, b);
                double e2 = d * d;
                if (e2 < T2) { cost += e2; ++in; }
                else cost += T2;
            }
            if (cost < best_cost) {
                best_cost = cost; best_in = in;
                bestA = a; bestB = b;
            }
        }

        if (best_in < cfg.min_inliers) continue;
        if ((double)best_in / pts.size() < cfg.min_inlier_ratio) continue;

        // Total-least-squares refit on inliers.
        std::vector<cv::Point2f> inliers; inliers.reserve(best_in);
        for (auto& p : pts)
            if (perpDistAB(p, bestA, bestB) <= cfg.inlier_tol_px) inliers.push_back(p);
        cv::Vec4f L; cv::fitLine(inliers, L, cv::DIST_L2, 0, 0.01, 0.01);
        // L = (vx, vy, x0, y0)
        cv::Point2f dir(L[0], L[1]);
        cv::Point2f p0(L[2], L[3]);
        // Project original endpoints onto fit.
        auto project = [&](cv::Point2f q) {
            float t = (q.x - p0.x) * dir.x + (q.y - p0.y) * dir.y;
            return cv::Point2f(p0.x + t * dir.x, p0.y + t * dir.y);
        };
        cv::Point2f A = project(c.seg.p0), B = project(c.seg.p1);

        // RMS on inliers.
        double sumsq = 0;
        for (auto& p : inliers) {
            double d = perpDistAB(p, A, B);
            sumsq += d * d;
        }
        double rms = std::sqrt(sumsq / inliers.size());

        PipeRansacResult r;
        r.seg = c;
        r.seg.seg.p0 = A;
        r.seg.seg.p1 = B;
        r.seg.seg.length_px = std::hypot(B.x - A.x, B.y - A.y);
        r.inliers = (int)inliers.size();
        r.inlier_ratio = (double)inliers.size() / pts.size();
        r.rms_px = rms;
        r.ok = true;
        out.push_back(r);
    }
    return out;
}

}  // namespace mate
