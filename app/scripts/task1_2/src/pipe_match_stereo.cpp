// pipe_match_stereo.cpp — L↔R pipe matching with SGBM+epipolar+Sampson.
#include "pipe_match_stereo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mate {

namespace {

double mean_disp_along(const cv::Mat& disp,
                       const cv::Point2f& a, const cv::Point2f& b,
                       int stride) {
    if (disp.empty()) return std::nan("");
    int n = std::max(8, (int)(std::hypot(b.x - a.x, b.y - a.y) / stride));
    double sum = 0; int cnt = 0;
    for (int k = 0; k <= n; ++k) {
        float t = (float)k / n;
        cv::Point2f q = a * (1.f - t) + b * t;
        int xi = (int)q.x, yi = (int)q.y;
        if (xi<0||yi<0||xi>=disp.cols||yi>=disp.rows) continue;
        float d = disp.at<float>(yi, xi);
        if (std::isnan(d) || d <= 0) continue;
        sum += d; ++cnt;
    }
    return cnt ? sum / cnt : std::nan("");
}

}  // namespace

std::vector<PipeMatch>
matchPipesStereo(const std::vector<PipeDiameterResult>& Ld,
                 const std::vector<PipeDiameterResult>& Rd,
                 const cv::Mat& disp,
                 const cv::Mat& /*conf*/,
                 const PipeMatchStereoConfig& cfg) {
    std::vector<PipeMatch> out;
    if (Ld.empty() || Rd.empty()) return out;

    const double atol = cfg.angle_tol_deg * CV_PI / 180.0;

    // Score every (i,j); keep best-per-i with mutual best-of-j check.
    std::vector<int>    best_j(Ld.size(), -1);
    std::vector<double> best_score(Ld.size(), 1e30);
    std::vector<double> best_dbar(Ld.size(), 0);
    std::vector<int>    best_i_for_j(Rd.size(), -1);
    std::vector<double> best_score_j(Rd.size(), 1e30);

    auto angle = [](cv::Point2f a, cv::Point2f b){
        return std::atan2(b.y-a.y, b.x-a.x);
    };

    for (size_t i = 0; i < Ld.size(); ++i) {
        const auto& L = Ld[i].seg.seg.seg;
        double aL = angle(L.p0, L.p1);
        double LenL = std::hypot(L.p1.x-L.p0.x, L.p1.y-L.p0.y);
        double rL = Ld[i].radius_m;
        double dbar = mean_disp_along(disp, L.p0, L.p1, 2);
        if (std::isnan(dbar)) continue;

        for (size_t j = 0; j < Rd.size(); ++j) {
            const auto& Rseg = Rd[j].seg.seg.seg;
            double aR = angle(Rseg.p0, Rseg.p1);
            double da = std::fabs(std::fmod(aL - aR + CV_PI, CV_PI));
            da = std::min(da, CV_PI - da);
            if (da > atol) continue;
            double LenR = std::hypot(Rseg.p1.x-Rseg.p0.x, Rseg.p1.y-Rseg.p0.y);
            double lr = std::max(LenL, LenR) / std::max(1e-3, std::min(LenL, LenR));
            if (lr > cfg.len_tol) continue;

            double rR = Rd[j].radius_m;
            double rr = std::max(rL, rR) / std::max(1e-3, std::min(rL, rR));
            if (rr > cfg.radius_tol) continue;

            // Predicted R endpoints by shifting L by mean disparity.
            cv::Point2f Pa(L.p0.x - dbar, L.p0.y);
            cv::Point2f Pb(L.p1.x - dbar, L.p1.y);
            // Match orientation of R endpoints (closer=closer).
            double da00 = std::hypot(Pa.x-Rseg.p0.x, Pa.y-Rseg.p0.y) +
                          std::hypot(Pb.x-Rseg.p1.x, Pb.y-Rseg.p1.y);
            double da01 = std::hypot(Pa.x-Rseg.p1.x, Pa.y-Rseg.p1.y) +
                          std::hypot(Pb.x-Rseg.p0.x, Pb.y-Rseg.p0.y);
            cv::Point2f Ra, Rb;
            if (da00 <= da01) { Ra = Rseg.p0; Rb = Rseg.p1; }
            else              { Ra = Rseg.p1; Rb = Rseg.p0; }

            // Epipolar (rectified ⇒ y diff).
            double ey = 0.5 * (std::fabs(Pa.y - Ra.y) + std::fabs(Pb.y - Rb.y));
            if (ey > cfg.epi_tol_px) continue;

            // Sampson (here just |y_l - y_r|² since rectified).
            double samp = std::sqrt(0.5 * ((Pa.y-Ra.y)*(Pa.y-Ra.y) +
                                           (Pb.y-Rb.y)*(Pb.y-Rb.y)));
            if (samp > cfg.sampson_tol_px) continue;

            // Score: combined penalty.
            double score = ey + 0.5 * (lr - 1.0) * 4.0 + 0.5 * (rr - 1.0) * 4.0
                           + 0.3 * std::fabs(da) * 180.0 / CV_PI;
            if (score < best_score[i]) {
                best_score[i] = score; best_j[i] = (int)j; best_dbar[i] = dbar;
            }
            if (score < best_score_j[j]) {
                best_score_j[j] = score; best_i_for_j[j] = (int)i;
            }
        }
    }
    // Keep mutual-best matches.
    for (size_t i = 0; i < Ld.size(); ++i) {
        int j = best_j[i];
        if (j < 0) continue;
        if (best_i_for_j[j] != (int)i) continue;
        const auto& L = Ld[i].seg.seg.seg;
        const auto& Rseg = Rd[j].seg.seg.seg;
        double dbar = best_dbar[i];
        cv::Point2f Pa(L.p0.x - dbar, L.p0.y), Pb(L.p1.x - dbar, L.p1.y);
        double d00 = std::hypot(Pa.x-Rseg.p0.x, Pa.y-Rseg.p0.y) +
                     std::hypot(Pb.x-Rseg.p1.x, Pb.y-Rseg.p1.y);
        double d01 = std::hypot(Pa.x-Rseg.p1.x, Pa.y-Rseg.p1.y) +
                     std::hypot(Pb.x-Rseg.p0.x, Pb.y-Rseg.p0.y);
        PipeMatch m;
        m.left_idx  = (int)i;
        m.right_idx = j;
        m.l0 = L.p0; m.l1 = L.p1;
        if (d00 <= d01) { m.r0 = Rseg.p0; m.r1 = Rseg.p1; }
        else            { m.r0 = Rseg.p1; m.r1 = Rseg.p0; }
        m.mean_disparity_px = dbar;
        // Confidence: votes-weighted, score-weighted.
        int votes = std::min(Ld[i].seg.seg.votes, Rd[j].seg.seg.votes);
        m.confidence = std::max(0.0, std::min(1.0,
            0.4 * std::min(1, std::max(0, votes-1))
          + 0.6 * std::exp(-best_score[i] / 3.0)));
        out.push_back(m);
    }
    return out;
}

}  // namespace mate
