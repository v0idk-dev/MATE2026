// sgbm_disparity.cpp — SGBM + (optional) WLS post-filter.
#include "sgbm_disparity.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

#if __has_include(<opencv2/ximgproc/disparity_filter.hpp>)
#  include <opencv2/ximgproc/disparity_filter.hpp>
#  define MATE_HAVE_XIMGPROC_DISPARITY 1
#else
#  define MATE_HAVE_XIMGPROC_DISPARITY 0
#endif

namespace mate {

namespace {
int round_to_multiple(int v, int m) { return ((v + m - 1) / m) * m; }
}

SgbmResult computeSgbmDisparity(const cv::Mat& Lr, const cv::Mat& Rr,
                                double f_px, double B_m,
                                const SgbmConfig& cfg) {
    SgbmResult R;
    if (Lr.empty() || Rr.empty() || Lr.size() != Rr.size()) return R;

    // Auto-size disparity range from (f, B, [Z_near, Z_far]):
    //   d = f * B / Z   →  d_max @ Z_near, d_min @ Z_far
    int num_disp = cfg.num_disparities_override;
    if (num_disp <= 0) {
        // Defensive floors: 5 cm minimum near, 100 m maximum far.
        // Without these, a user typo (negative or zero Z_near) would
        // explode num_disp and OOM the SGBM allocator.
        double Zn = std::max(0.05, cfg.Z_near_m);
        double Zf = std::min(100.0, cfg.Z_far_m);
        if (Zf <= Zn) std::swap(Zn, Zf);                  // tolerate user-swap
        Zf = std::max(Zn + 0.05, Zf);                     // ensure positive range
        double d_max = f_px * B_m / Zn;
        double d_min = f_px * B_m / Zf;
        int range   = std::max(16, (int)std::ceil(d_max - d_min));
        num_disp    = round_to_multiple(range, 16);
        num_disp    = std::min(num_disp, 256);
    }
    R.num_disparities_used = num_disp;

    int block = cfg.block_size_override;
    if (block <= 0) {
        block = std::max(3, std::min(Lr.cols, Lr.rows) / 200);
        if (block % 2 == 0) block++;
        block = std::min(block, 11);
    }
    R.block_size_used = block;

    int channels = Lr.channels();
    auto sgbm = cv::StereoSGBM::create(
        0, num_disp, block,
        cfg.P1_per_chan * channels * block * block,
        cfg.P2_per_chan * channels * block * block,
        cfg.disp12_max_diff,
        cfg.pre_filter_cap,
        cfg.uniqueness_ratio,
        cfg.speckle_window_size,
        cfg.speckle_range,
        cfg.use_3way_mode ? cv::StereoSGBM::MODE_SGBM_3WAY
                          : cv::StereoSGBM::MODE_SGBM);

    cv::Mat disp16;       // CV_16S, fixed-point ×16
    sgbm->compute(Lr, Rr, disp16);

    cv::Mat dispF;
#if MATE_HAVE_XIMGPROC_DISPARITY
    if (cfg.wls_enabled) {
        auto rmatcher = cv::ximgproc::createRightMatcher(sgbm);
        cv::Mat disp16R;
        rmatcher->compute(Rr, Lr, disp16R);
        auto wls = cv::ximgproc::createDisparityWLSFilter(sgbm);
        wls->setLambda(cfg.wls_lambda);
        wls->setSigmaColor(cfg.wls_sigma_color);
        cv::Mat disp_filtered;
        wls->filter(disp16, Lr, disp_filtered, disp16R);
        disp_filtered.convertTo(dispF, CV_32F, 1.0/16.0);
        cv::Mat conf = wls->getConfidenceMap();
        if (!conf.empty()) {
            cv::Mat conf8; conf.convertTo(conf8, CV_8U);
            R.confidence = conf8;
        }
    } else {
        disp16.convertTo(dispF, CV_32F, 1.0/16.0);
    }
#else
    disp16.convertTo(dispF, CV_32F, 1.0/16.0);
#endif

    // Mark invalid pixels as NaN.
    int valid = 0;
    for (int y = 0; y < dispF.rows; ++y) {
        float* dp = dispF.ptr<float>(y);
        const uchar* cp = R.confidence.empty() ? nullptr : R.confidence.ptr<uchar>(y);
        for (int x = 0; x < dispF.cols; ++x) {
            if (dp[x] <= 0.0f) { dp[x] = std::nanf(""); continue; }
            if (cp && cp[x] < cfg.min_confidence) { dp[x] = std::nanf(""); continue; }
            ++valid;
        }
    }
    R.disparity = dispF;
    R.pixels_valid = valid;
    R.pct_valid = 100.0 * valid / std::max(1, dispF.rows * dispF.cols);
    return R;
}

std::vector<CloudPoint> disparityToCloud(const cv::Mat& disp,
                                          const cv::Mat& Q,
                                          const cv::Mat& bgr,
                                          int max_points) {
    std::vector<CloudPoint> out;
    if (disp.empty() || Q.empty()) return out;
    cv::Mat xyz; cv::reprojectImageTo3D(disp, xyz, Q, true);
    int total_valid = 0;
    for (int y = 0; y < disp.rows; ++y) {
        const float* d = disp.ptr<float>(y);
        for (int x = 0; x < disp.cols; ++x)
            if (!std::isnan(d[x])) ++total_valid;
    }
    int stride = std::max(1, (int)std::ceil(std::sqrt(double(total_valid)/std::max(1, max_points))));
    out.reserve(std::min(max_points, total_valid));
    for (int y = 0; y < disp.rows; y += stride) {
        const cv::Vec3f* row = xyz.ptr<cv::Vec3f>(y);
        const cv::Vec3b* col = bgr.empty()? nullptr : bgr.ptr<cv::Vec3b>(y);
        const float* d       = disp.ptr<float>(y);
        for (int x = 0; x < disp.cols; x += stride) {
            if (std::isnan(d[x])) continue;
            const cv::Vec3f& p = row[x];
            if (!std::isfinite(p[2]) || p[2] <= 0 || p[2] > 50.0f) continue;
            CloudPoint cp;
            cp.p = cv::Point3f(p[0], p[1], p[2]);
            cp.bgr = col? col[x] : cv::Vec3b(255,255,255);
            cp.disparity = d[x];
            cp.u = x; cp.v = y;
            out.push_back(cp);
            if ((int)out.size() >= max_points) return out;
        }
    }
    return out;
}

}  // namespace mate
