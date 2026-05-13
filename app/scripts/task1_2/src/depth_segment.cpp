#include "depth_segment.hpp"
#include "stereo_math.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

#ifdef HAVE_OPENCV_XIMGPROC
#  include <opencv2/ximgproc/disparity_filter.hpp>
#endif

namespace mate {

static cv::Mat toGray(const cv::Mat& m) {
    if (m.empty() || m.channels() == 1) return m;
    cv::Mat g; cv::cvtColor(m, g, cv::COLOR_BGR2GRAY); return g;
}

DepthSegmentResult segmentForegroundByDepth(
        const RectifiedPair& rect,
        double baseline_m,
        const DepthSegmentConfig& cfg) {
    DepthSegmentResult R;
    if (rect.left.empty() || rect.right.empty()) return R;
    if (!(baseline_m > 0)) baseline_m = cfg.baseline_m_for_q;

    cv::Mat gL = toGray(rect.left), gR = toGray(rect.right);
    if (gL.size() != gR.size()) return R;

    R.baseline_m_used = baseline_m;

    // 1. Dense disparity via SGBM.
    int num_disp = ((cfg.sgbm_num_disp + 15) / 16) * 16;
    auto sgbm = cv::StereoSGBM::create(
        cfg.sgbm_min_disp, num_disp, cfg.sgbm_block_size,
        8  * 1 * cfg.sgbm_block_size * cfg.sgbm_block_size,
        32 * 1 * cfg.sgbm_block_size * cfg.sgbm_block_size,
        1, cfg.sgbm_pre_filter, cfg.sgbm_uniqueness,
        cfg.sgbm_speckle_win, cfg.sgbm_speckle_rng,
        cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disp16;
    sgbm->compute(gL, gR, disp16);

#ifdef HAVE_OPENCV_XIMGPROC
    if (cfg.use_wls_filter) {
        auto wls = cv::ximgproc::createDisparityWLSFilter(sgbm);
        auto rsgbm = cv::ximgproc::createRightMatcher(sgbm);
        cv::Mat disp16R;
        rsgbm->compute(gR, gL, disp16R);
        cv::Mat disp_filt;
        wls->setLambda(8000.0); wls->setSigmaColor(1.5);
        wls->filter(disp16, gL, disp_filt, disp16R);
        disp16 = disp_filt;
    }
#endif

    // OSD band: zero out disparity in top/bottom (burnt-in timestamps
    // create false matches at fixed pixel positions).
    if (cfg.osd_band_frac > 0) {
        const int top = static_cast<int>(cfg.osd_band_frac * disp16.rows);
        const int bot = disp16.rows - top;
        if (top > 0) disp16.rowRange(0, top).setTo(0);
        if (bot < disp16.rows) disp16.rowRange(bot, disp16.rows).setTo(0);
    }
    R.disp_int16 = disp16;

    // 2. CV_16S → CV_32F (SGBM returns disparity scaled by 16).
    cv::Mat disp32; disp16.convertTo(disp32, CV_32F, 1.0 / 16.0);
    cv::Mat valid = (disp32 > cfg.sgbm_min_disp + 0.5);

    double dmin, dmax;
    cv::minMaxLoc(disp32, &dmin, &dmax, nullptr, nullptr, valid);
    if (!std::isfinite(dmin) || !std::isfinite(dmax) || dmax <= dmin) return R;

    cv::Mat closeness;
    disp32.convertTo(closeness, CV_8U,
                     255.0 / (dmax - dmin),
                     -255.0 * dmin / (dmax - dmin));
    closeness.setTo(0, ~valid);
    R.closeness_u8 = closeness;

    // 3. Percentile FG threshold — INVARIANT to absolute scale.
    std::vector<float> dvals;
    dvals.reserve(static_cast<size_t>(cv::countNonZero(valid)));
    for (int y = 0; y < disp32.rows; ++y) {
        const float* d = disp32.ptr<float>(y);
        const uchar* v = valid.ptr<uchar>(y);
        for (int x = 0; x < disp32.cols; ++x)
            if (v[x]) dvals.push_back(d[x]);
    }
    if (dvals.empty()) return R;
    size_t kth = static_cast<size_t>(dvals.size() * cfg.fg_percentile);
    if (kth >= dvals.size()) kth = dvals.size() - 1;
    std::nth_element(dvals.begin(), dvals.begin() + kth, dvals.end());
    float fg_thresh = dvals[kth];

    cv::Mat fg = (disp32 > fg_thresh) & valid;

    // 4. Resolve Q matrix. Use rect.Q if present; otherwise synthesise
    //    from the user's baseline + the rectified-left intrinsics.
    cv::Mat Q;
    if (!rect.Q.empty()) {
        rect.Q.convertTo(Q, CV_64F);
    } else if (!rect.K_rect_left.empty()) {
        const double f  = focalFromK(rect.K_rect_left);
        const double cx = rect.K_rect_left.at<double>(0, 2);
        const double cy = rect.K_rect_left.at<double>(1, 2);
        if (std::isfinite(f) && f > 0)
            Q = makeQFromBaseline(f, cx, cy, baseline_m);
    } else {
        // No intrinsics either — estimate them from image size + 60° FOV.
        cv::Mat Kest = estimateIntrinsicsFromImage(rect.left.cols, rect.left.rows, 60.0);
        const double f  = focalFromK(Kest);
        const double cx = Kest.at<double>(0, 2);
        const double cy = Kest.at<double>(1, 2);
        Q = makeQFromBaseline(f, cx, cy, baseline_m);
    }
    R.focal_px_used = (Q.empty() ? 0.0 : Q.at<double>(2, 3));

    // 5. Reproject to 3D.
    cv::Mat depth32(disp32.size(), CV_32F, std::numeric_limits<float>::quiet_NaN());
    if (!Q.empty()) {
        cv::Mat xyz;
        cv::reprojectImageTo3D(disp32, xyz, Q, /*handleMissing=*/true);
        for (int y = 0; y < xyz.rows; ++y) {
            const cv::Vec3f* p = xyz.ptr<cv::Vec3f>(y);
            const uchar*     v = valid.ptr<uchar>(y);
            float* dz = depth32.ptr<float>(y);
            for (int x = 0; x < xyz.cols; ++x)
                if (v[x] && std::isfinite(p[x][2]) && p[x][2] > 0 && p[x][2] < 100.0f)
                    dz[x] = p[x][2];
        }
    }
    R.depth_m_32f = depth32;

    // 6. Morphological cleanup + drop tiny blobs.
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                    {cfg.morph_close_px, cfg.morph_close_px});
    cv::morphologyEx(fg, fg, cv::MORPH_CLOSE, se);
    cv::morphologyEx(fg, fg, cv::MORPH_OPEN,  se);

    cv::Mat labels, stats, cents;
    int n = cv::connectedComponentsWithStats(fg, labels, stats, cents);
    cv::Mat clean(fg.size(), CV_8U, cv::Scalar(0));
    int total = fg.rows * fg.cols;
    int min_blob = static_cast<int>(cfg.min_blob_frac * total);
    for (int i = 1; i < n; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < min_blob) continue;
        clean.setTo(255, labels == i);
    }
    R.foreground_u8 = clean;

    int fg_count = cv::countNonZero(clean);
    R.fg_area_frac = static_cast<double>(fg_count) / total;

    // 7. Auto-estimate subject distance Z = median FG depth.
    if (!Q.empty() && fg_count > 0) {
        std::vector<float> fdz; fdz.reserve(fg_count);
        std::vector<float> fdisp; fdisp.reserve(fg_count);
        for (int y = 0; y < depth32.rows; ++y) {
            const float* dz = depth32.ptr<float>(y);
            const float* dd = disp32.ptr<float>(y);
            const uchar* m  = clean.ptr<uchar>(y);
            for (int x = 0; x < depth32.cols; ++x) {
                if (!m[x]) continue;
                if (std::isfinite(dz[x])) fdz.push_back(dz[x]);
                if (dd[x] > 0)            fdisp.push_back(dd[x]);
            }
        }
        if (!fdz.empty()) {
            std::nth_element(fdz.begin(), fdz.begin() + fdz.size()/2, fdz.end());
            R.subject_distance_m_est = fdz[fdz.size()/2];
        }
        if (!fdisp.empty()) {
            std::nth_element(fdisp.begin(), fdisp.begin() + fdisp.size()/2, fdisp.end());
            R.median_disparity_px = fdisp[fdisp.size()/2];
        }
    }

    R.ok = R.fg_area_frac > 0.01;
    return R;
}

bool detectionInDepthBand(const cv::Mat& depth, const cv::Rect& roi,
                          double Z_est, double band_m) {
    if (depth.empty() || depth.type() != CV_32F || !(Z_est > 0)) return true;
    cv::Rect r = roi & cv::Rect(0, 0, depth.cols, depth.rows);
    if (r.area() <= 0) return false;
    std::vector<float> dz; dz.reserve(r.area());
    for (int y = r.y; y < r.y + r.height; ++y) {
        const float* d = depth.ptr<float>(y);
        for (int x = r.x; x < r.x + r.width; ++x)
            if (std::isfinite(d[x])) dz.push_back(d[x]);
    }
    if (dz.empty()) return false;
    std::nth_element(dz.begin(), dz.begin() + dz.size()/2, dz.end());
    float med = dz[dz.size()/2];
    return std::fabs(med - Z_est) <= band_m;
}

}  // namespace mate
