// pipe_parallel_pair.cpp — color-independent pipe detection.
#include "pipe_parallel_pair.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mate {

namespace {

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
// Signed perpendicular offset of point p from line (q0->q1), positive
// to the left of the direction.
double signedPerp(const cv::Point2f& p, const cv::Point2f& q0, const cv::Point2f& q1) {
    cv::Point2f v = q1 - q0;
    double L = std::hypot(v.x, v.y);
    if (L < 1e-6) return 0;
    return ((p.x - q0.x)*(-v.y) + (p.y - q0.y)*v.x) / L;
}

// Project p onto axis (q0->unit-dir d̂) → 1D scalar (origin = q0).
double projT(const cv::Point2f& p, const cv::Point2f& q0, const cv::Point2f& dhat) {
    return (p.x - q0.x)*dhat.x + (p.y - q0.y)*dhat.y;
}

// Mean and standard-deviation of grayscale in a thin band between two
// parallel lines (a = upper line, b = lower line, both already known to
// be parallel and same direction). We sample N points across the
// midline and one stripe each side.
void sampleBandStats(const cv::Mat& gray,
                      const cv::Point2f& a0, const cv::Point2f& a1,
                      const cv::Point2f& b0, const cv::Point2f& b1,
                      double& mean_in, double& std_in,
                      double& mean_out_a, double& mean_out_b) {
    int N = std::max(16, (int)(0.5 * (segLen(a0,a1) + segLen(b0,b1))));
    std::vector<float> in_vals;
    std::vector<float> outA_vals, outB_vals;
    in_vals.reserve(N); outA_vals.reserve(N); outB_vals.reserve(N);
    for (int k = 0; k <= N; ++k) {
        float t = (float)k / N;
        cv::Point2f mid_a = a0*(1-t) + a1*t;
        cv::Point2f mid_b = b0*(1-t) + b1*t;
        cv::Point2f mid   = (mid_a + mid_b) * 0.5f;
        cv::Point2f normal = mid_b - mid_a;
        double L = std::hypot(normal.x, normal.y);
        if (L < 1e-6) continue;
        cv::Point2f n_unit(normal.x / L, normal.y / L);
        cv::Point2f outA = mid_a - n_unit * (float)(L * 0.4);
        cv::Point2f outB = mid_b + n_unit * (float)(L * 0.4);
        auto sample = [&](cv::Point2f p) -> int {
            int xi = (int)p.x, yi = (int)p.y;
            if (xi<0||yi<0||xi>=gray.cols||yi>=gray.rows) return -1;
            return (int)gray.at<uchar>(yi, xi);
        };
        int v_in = sample(mid), v_oa = sample(outA), v_ob = sample(outB);
        if (v_in >= 0) in_vals.push_back(v_in);
        if (v_oa >= 0) outA_vals.push_back(v_oa);
        if (v_ob >= 0) outB_vals.push_back(v_ob);
    }
    auto stats = [&](const std::vector<float>& v, double& m, double& sd) {
        if (v.empty()) { m = 0; sd = 0; return; }
        double s = 0; for (auto x : v) s += x; m = s / v.size();
        double s2 = 0; for (auto x : v) s2 += (x - m)*(x - m);
        sd = std::sqrt(s2 / v.size());
    };
    double sd_oA, sd_oB;
    stats(in_vals,   mean_in,    std_in);
    stats(outA_vals, mean_out_a, sd_oA);
    stats(outB_vals, mean_out_b, sd_oB);
    (void)sd_oA; (void)sd_oB;
}

// Average Sobel-x gradient projected onto the perpendicular of a line.
// Used to verify the two edges of a candidate pair have opposite gradient
// signs (a real pipe = ridge profile, not two unrelated edges).
double meanGradAlongNormal(const cv::Mat& gx, const cv::Mat& gy,
                            const cv::Point2f& l0, const cv::Point2f& l1,
                            cv::Point2f n_unit) {
    int N = std::max(16, (int)segLen(l0, l1));
    double s = 0; int cnt = 0;
    for (int k = 0; k <= N; ++k) {
        float t = (float)k / N;
        cv::Point2f q = l0*(1-t) + l1*t;
        int xi = (int)q.x, yi = (int)q.y;
        if (xi<0||yi<0||xi>=gx.cols||yi>=gx.rows) continue;
        double g = gx.at<short>(yi,xi) * n_unit.x + gy.at<short>(yi,xi) * n_unit.y;
        s += g; ++cnt;
    }
    return cnt ? s / cnt : 0.0;
}

}  // anonymous

std::vector<PipePair>
detectPipeParallelPairs(const cv::Mat& bgr,
                         const std::vector<LinesMultiSegment>& lines,
                         double f_px, double Z,
                         double r_min_m, double r_max_m,
                         const cv::Mat& soft_mask,
                         const PipePairConfig& cfg) {
    std::vector<PipePair> out;
    if (bgr.empty() || lines.size() < 2 || f_px <= 0 || Z <= 0) return out;

    // Derive expected pipe-width band in pixels.
    double w_min_px = cfg.w_min_px_override > 0
        ? cfg.w_min_px_override : (2.0 * r_min_m * f_px / Z);
    double w_max_px = cfg.w_max_px_override > 0
        ? cfg.w_max_px_override : (2.0 * r_max_m * f_px / Z);

    cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gx, gy;
    cv::Sobel(gray, gx, CV_16S, 1, 0, 3);
    cv::Sobel(gray, gy, CV_16S, 0, 1, 3);

    const double atol = cfg.angle_tol_deg * CV_PI / 180.0;

    for (size_t i = 0; i < lines.size(); ++i) {
        const auto& A = lines[i].seg;
        double aA = segAngleRad(A.p0, A.p1);
        double LA = segLen(A.p0, A.p1);
        cv::Point2f dirA(std::cos(aA), std::sin(aA));
        // Unit normal (left of dir).
        cv::Point2f normA(-dirA.y, dirA.x);

        for (size_t j = i + 1; j < lines.size(); ++j) {
            const auto& B = lines[j].seg;
            double aB = segAngleRad(B.p0, B.p1);
            if (angleDist(aA, aB) > atol) continue;
            double LB = segLen(B.p0, B.p1);
            double lenRatio = std::max(LA, LB) / std::max(1e-3, std::min(LA, LB));
            if (lenRatio > cfg.len_ratio_tol) continue;

            // Spacing: average of perpendicular distances of B's endpoints from A.
            double dB0 = std::fabs(signedPerp(B.p0, A.p0, A.p1));
            double dB1 = std::fabs(signedPerp(B.p1, A.p0, A.p1));
            double spacing = 0.5 * (dB0 + dB1);
            if (spacing < w_min_px || spacing > w_max_px) continue;

            // Overlap: project both onto A's axis, compute overlap fraction.
            double tA0 = 0.0, tA1 = LA;
            double tB0 = projT(B.p0, A.p0, dirA);
            double tB1 = projT(B.p1, A.p0, dirA);
            if (tB0 > tB1) std::swap(tB0, tB1);
            double ov = std::max(0.0, std::min(tA1, tB1) - std::max(tA0, tB0));
            double minLen = std::min(LA, LB);
            if (ov / std::max(1.0, minLen) < cfg.min_overlap_frac) continue;

            // Order pair so a is on the "left" (normal points toward b).
            // Compute signed offset of B-midpoint from A.
            cv::Point2f Bmid = (B.p0 + B.p1) * 0.5f;
            double sB = signedPerp(Bmid, A.p0, A.p1);
            cv::Point2f a0 = A.p0, a1 = A.p1, b0 = B.p0, b1 = B.p1;
            cv::Point2f n_pair = normA;
            if (sB < 0) { n_pair = -normA; }     // flip normal to point at B

            // Opposite-sign gradient check (ridge profile).
            double gA = meanGradAlongNormal(gx, gy, a0, a1,  n_pair);
            double gB = meanGradAlongNormal(gx, gy, b0, b1, -n_pair);
            // Both should be POSITIVE (background→pipe) ⇒ ridge.
            // Accept also their sum > 0 (handles one weak edge).
            if (gA + gB < 4.0) continue;

            // Interior uniformity + exterior contrast.
            double mIn, sdIn, mOA, mOB;
            sampleBandStats(gray, a0, a1, b0, b1, mIn, sdIn, mOA, mOB);
            if (sdIn > cfg.interior_max_std) continue;
            double dIA = std::fabs(mIn - mOA);
            double dIB = std::fabs(mIn - mOB);
            if (std::min(dIA, dIB) < cfg.exterior_min_dI) continue;

            // Composite score (each term in 0..1).
            double s_ang  = 1.0 - angleDist(aA, aB) / atol;
            double s_len  = 1.0 - (lenRatio - 1.0) / (cfg.len_ratio_tol - 1.0);
            double s_ridge= std::min(1.0, (gA + gB) / 80.0);
            double s_uni  = std::max(0.0, 1.0 - sdIn / cfg.interior_max_std);
            double s_con  = std::min(1.0, (dIA + dIB) / 80.0);
            double score  = std::pow(s_ang * s_len * s_ridge * s_uni * s_con, 1.0/5.0);

            if (!soft_mask.empty()) {
                cv::Point2f mid = ((a0+a1+b0+b1) * 0.25f);
                int xi = (int)mid.x, yi = (int)mid.y;
                if (xi>=0 && yi>=0 && xi<soft_mask.cols && yi<soft_mask.rows
                    && soft_mask.at<uchar>(yi, xi)) {
                    score = std::min(1.0, score * 1.15);
                }
            }
            if (score < cfg.min_score) continue;

            // Build the medial line (mid-points of A and B endpoints).
            PipePair pp;
            pp.seg.p0 = (a0 + b0) * 0.5f;
            pp.seg.p1 = (a1 + b1) * 0.5f;
            pp.seg.length_px = segLen(pp.seg.p0, pp.seg.p1);
            cv::Point2f md = pp.seg.p1 - pp.seg.p0;
            pp.seg.angle_deg = std::atan2(md.y, md.x) * 180.0 / CV_PI;
            pp.width_px      = spacing;
            pp.score         = score;
            pp.interior_std  = sdIn;
            pp.exterior_diff = 0.5 * (dIA + dIB);
            pp.src_a = (int)i; pp.src_b = (int)j;
            out.push_back(pp);
        }
    }

    // Greedy non-maximum suppression: drop pairs whose midlines overlap
    // by > 70% with a higher-scoring pair.
    std::sort(out.begin(), out.end(), [](const PipePair& x, const PipePair& y){
        return x.score > y.score;
    });
    std::vector<bool> drop(out.size(), false);
    for (size_t i = 0; i < out.size(); ++i) {
        if (drop[i]) continue;
        for (size_t j = i + 1; j < out.size(); ++j) {
            if (drop[j]) continue;
            double dpa = std::fabs(signedPerp(out[j].seg.p0, out[i].seg.p0, out[i].seg.p1));
            double dpb = std::fabs(signedPerp(out[j].seg.p1, out[i].seg.p0, out[i].seg.p1));
            if (0.5*(dpa+dpb) > 0.5 * out[i].width_px) continue;
            // Overlap check.
            cv::Point2f dirI = out[i].seg.p1 - out[i].seg.p0;
            double LI = std::hypot(dirI.x, dirI.y);
            if (LI < 1e-3) continue;
            cv::Point2f dirIu(dirI.x / LI, dirI.y / LI);
            double tj0 = projT(out[j].seg.p0, out[i].seg.p0, dirIu);
            double tj1 = projT(out[j].seg.p1, out[i].seg.p0, dirIu);
            if (tj0 > tj1) std::swap(tj0, tj1);
            double LJ  = std::max(1.0, segLen(out[j].seg.p0, out[j].seg.p1));
            double ov  = std::max(0.0, std::min(LI, tj1) - std::max(0.0, tj0));
            // Drop j if it is largely contained in i — measure overlap as a
            // fraction of the *shorter* line so a small segment fully nested
            // inside a long one is correctly suppressed.
            if (ov / std::min(LI, LJ) > 0.7) drop[j] = true;
        }
    }
    std::vector<PipePair> finalOut;
    for (size_t i = 0; i < out.size(); ++i) if (!drop[i]) finalOut.push_back(out[i]);
    return finalOut;
}

}  // namespace mate
