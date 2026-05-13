// pvc_segment.cpp — adaptive PVC segmentation + distance + skeleton.
#include "pvc_segment.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <vector>

namespace mate {

namespace {

// Zhang-Suen thinning (in-place on CV_8U {0,255} input).
void zhangSuenThinning(cv::Mat& img) {
    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8U);
    cv::Mat diff;
    cv::Mat marker;
    bool changed = true;
    while (changed) {
        changed = false;
        for (int iter = 0; iter < 2; ++iter) {
            marker = cv::Mat::zeros(img.size(), CV_8U);
            for (int y = 1; y < img.rows - 1; ++y) {
                const uchar* up = img.ptr<uchar>(y - 1);
                const uchar* cu = img.ptr<uchar>(y);
                const uchar* dn = img.ptr<uchar>(y + 1);
                uchar* mk = marker.ptr<uchar>(y);
                for (int x = 1; x < img.cols - 1; ++x) {
                    if (!cu[x]) continue;
                    int p2 = up[x] > 0, p3 = up[x + 1] > 0;
                    int p4 = cu[x + 1] > 0, p5 = dn[x + 1] > 0;
                    int p6 = dn[x] > 0, p7 = dn[x - 1] > 0;
                    int p8 = cu[x - 1] > 0, p9 = up[x - 1] > 0;
                    int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    if (B < 2 || B > 6) continue;
                    int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                            (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                            (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                            (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
                    if (A != 1) continue;
                    int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
                    int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);
                    if (m1 == 0 && m2 == 0) mk[x] = 1;
                }
            }
            for (int y = 0; y < img.rows; ++y) {
                uchar* ip = img.ptr<uchar>(y);
                const uchar* mp = marker.ptr<uchar>(y);
                for (int x = 0; x < img.cols; ++x)
                    if (mp[x]) { ip[x] = 0; changed = true; }
            }
        }
    }
}

}  // namespace

PvcSegmentResult segmentPvc(const cv::Mat& bgr, const PvcSegmentConfig& cfg) {
    PvcSegmentResult R;
    if (bgr.empty()) return R;

    // 1. BGR → CIELAB.
    cv::Mat lab; cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch; cv::split(lab, ch);
    cv::Mat L = ch[0], A = ch[1], B = ch[2];

    // 2. OSD-band exclusion mask (top/bottom band = "ignore").
    cv::Mat valid = cv::Mat::ones(bgr.size(), CV_8U) * 255;
    int band = static_cast<int>(cfg.osd_band_frac * bgr.rows);
    if (band > 0) {
        valid.rowRange(0, band).setTo(0);
        valid.rowRange(bgr.rows - band, bgr.rows).setTo(0);
    }

    // 3. Adaptive L* threshold (Otsu inside the valid band).
    int L_min = cfg.L_min_override;
    if (L_min <= 0) {
        // Otsu on the L* channel inside the OSD-stripped band, computed on
        // a column-vector built from sampled pixels (cv::threshold needs a
        // proper Mat, so we wrap a temporary buffer with explicit shape).
        std::vector<uchar> sample;
        sample.reserve((bgr.rows - 2*band) * bgr.cols / 4);
        for (int y = band; y < bgr.rows - band; y += 2)
            for (int x = 0; x < bgr.cols; x += 2)
                sample.push_back(L.at<uchar>(y, x));
        if (sample.empty()) sample.push_back(128);
        cv::Mat sm(static_cast<int>(sample.size()), 1, CV_8U, sample.data());
        cv::Mat dummy;
        double thr = cv::threshold(sm, dummy, 0, 255,
                                   cv::THRESH_BINARY | cv::THRESH_OTSU);
        // PVC is bright but we no longer hardcode a floor — Otsu on a
        // sane scene already returns >100; in pathological darkness the
        // chroma gate alone keeps the result sensible.
        L_min = std::max(60, static_cast<int>(thr));
    }
    R.L_min_used = L_min;

    cv::Mat brightMask;
    cv::threshold(L, brightMask, L_min, 255, cv::THRESH_BINARY);
    cv::bitwise_and(brightMask, valid, brightMask);

    // 4. Adaptive chroma threshold via MAD on bright pixels.
    double chroma_max = cfg.chroma_max_override;
    if (chroma_max <= 0.0) {
        std::vector<float> chromas;
        chromas.reserve(64000);
        for (int y = band; y < bgr.rows - band; y += 4)
            for (int x = 0; x < bgr.cols; x += 4) {
                if (!brightMask.at<uchar>(y, x)) continue;
                float a = (float)A.at<uchar>(y, x) - 128.f;
                float b = (float)B.at<uchar>(y, x) - 128.f;
                chromas.push_back(std::sqrt(a*a + b*b));
            }
        if (!chromas.empty()) {
            std::nth_element(chromas.begin(),
                              chromas.begin() + chromas.size()/2,
                              chromas.end());
            float median = chromas[chromas.size()/2];
            // MAD
            std::vector<float> dev(chromas.size());
            for (size_t i=0;i<chromas.size();++i) dev[i]=std::fabs(chromas[i]-median);
            std::nth_element(dev.begin(), dev.begin()+dev.size()/2, dev.end());
            float mad = dev[dev.size()/2];
            chroma_max = std::max(8.0, double(median + 2.5f * 1.4826f * mad));
        } else {
            chroma_max = 18.0;
        }
    }
    R.chroma_max_used = chroma_max;

    // 5. Build chroma mask.
    cv::Mat chromaMask(bgr.size(), CV_8U, cv::Scalar(0));
    for (int y = 0; y < bgr.rows; ++y) {
        const uchar* ap = A.ptr<uchar>(y);
        const uchar* bp = B.ptr<uchar>(y);
        uchar* cp = chromaMask.ptr<uchar>(y);
        for (int x = 0; x < bgr.cols; ++x) {
            float a = (float)ap[x] - 128.f;
            float b = (float)bp[x] - 128.f;
            if (std::sqrt(a*a + b*b) < chroma_max) cp[x] = 255;
        }
    }
    cv::Mat raw; cv::bitwise_and(brightMask, chromaMask, raw);

    // 6. Morphology open → close.
    int k = cfg.morph_kernel_override > 0
                ? cfg.morph_kernel_override
                : std::max(3, std::min(bgr.cols, bgr.rows) / 300);
    if (k % 2 == 0) k++;
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
    cv::morphologyEx(raw, raw, cv::MORPH_OPEN,  se);
    cv::morphologyEx(raw, raw, cv::MORPH_CLOSE, se);

    // 7. Filter small components.
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(raw, labels, stats, centroids, 8);
    cv::Mat keep = cv::Mat::zeros(bgr.size(), CV_8U);
    if (n <= 1) {
        // No PVC components found — short-circuit before the (now-pointless)
        // distance transform + skeletonisation, which would only burn CPU.
        R.mask = keep;
        R.pixels_kept = 0;
        if (cfg.compute_distance) R.dist_px = cv::Mat::zeros(bgr.size(), CV_32F);
        if (cfg.compute_skeleton) R.skeleton = cv::Mat::zeros(bgr.size(), CV_8U);
        R.median_radius_px = 0.0;
        return R;
    }
    int min_area = std::max(50, static_cast<int>(cfg.min_cc_area_frac * bgr.total()));
    for (int i = 1; i < n; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) >= min_area) {
            keep.setTo(255, labels == i);
        }
    }
    R.mask = keep;
    R.pixels_kept = cv::countNonZero(keep);

    // 8. Distance transform (Felzenszwalb, exact L2).
    if (cfg.compute_distance) {
        cv::distanceTransform(R.mask, R.dist_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    }

    // 9. Skeleton (Zhang-Suen).
    if (cfg.compute_skeleton) {
        cv::Mat sk = R.mask.clone();
        zhangSuenThinning(sk);
        R.skeleton = sk;
        if (!R.dist_px.empty()) {
            std::vector<float> rs; rs.reserve(8192);
            for (int y = 0; y < sk.rows; ++y) {
                const uchar* sp = sk.ptr<uchar>(y);
                const float* dp = R.dist_px.ptr<float>(y);
                for (int x = 0; x < sk.cols; ++x)
                    if (sp[x]) rs.push_back(dp[x]);
            }
            if (!rs.empty()) {
                std::nth_element(rs.begin(), rs.begin()+rs.size()/2, rs.end());
                R.median_radius_px = rs[rs.size()/2];
            }
        }
    }
    return R;
}

}  // namespace mate
