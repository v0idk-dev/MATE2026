// pipe_diameter.cpp — projected-diameter gate using distance transform.
#include "pipe_diameter.hpp"
#include <algorithm>
#include <cmath>

namespace mate {

std::vector<PipeDiameterResult>
gateByDiameter(const std::vector<PipeRansacResult>& segs,
               const cv::Mat& dist_px,
               double f_px, double Z, double sigma_Z,
               const PipeDiameterConfig& cfg) {
    std::vector<PipeDiameterResult> out;
    if (segs.empty() || dist_px.empty() || f_px <= 0 || Z <= 0) return out;

    double r_min = cfg.r_min_m / cfg.tol;
    double r_max = cfg.r_max_m * cfg.tol;

    for (auto& s : segs) {
        // Re-sample distance transform along the refined endpoints.
        cv::Point2f a = s.seg.seg.p0, b = s.seg.seg.p1;
        double L = std::hypot(b.x - a.x, b.y - a.y);
        int N = std::max(8, (int)(L / cfg.sample_stride_px));
        std::vector<float> rs; rs.reserve(N + 1);
        for (int k = 0; k <= N; ++k) {
            float t = (float)k / N;
            cv::Point2f q = a * (1.f - t) + b * t;
            int xi = (int)q.x, yi = (int)q.y;
            if (xi < 0 || yi < 0 || xi >= dist_px.cols || yi >= dist_px.rows) continue;
            rs.push_back(dist_px.at<float>(yi, xi));
        }
        if (rs.empty()) continue;
        std::nth_element(rs.begin(), rs.begin()+rs.size()/2, rs.end());
        double r_px = rs[rs.size()/2];
        if (r_px <= 0.5) continue;     // degenerate

        double r_m = r_px * Z / f_px;
        // Propagate Z uncertainty: σ_r ≈ |∂r/∂Z| · σ_Z = (r_px / f) · σ_Z
        double r_sigma = (r_px / f_px) * std::max(0.0, sigma_Z);

        if (r_m < r_min || r_m > r_max) continue;

        PipeDiameterResult d;
        d.seg = s;
        d.radius_m = r_m;
        d.radius_m_sigma = r_sigma;
        d.ok = true;
        out.push_back(d);
    }
    return out;
}

}  // namespace mate
