// pipe_pink_tape.cpp — adaptive LAB-based pink-tape blob detection.
#include "pipe_pink_tape.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mate {

std::vector<PinkBlob> detectPinkTape(const cv::Mat& bgr,
                                       cv::Mat* debug_mask,
                                       const PinkTapeConfig& cfg) {
    std::vector<PinkBlob> out;
    if (bgr.empty()) return out;

    // ── 1. LAB ────────────────────────────────────────────────────────
    cv::Mat lab; cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch; cv::split(lab, ch);
    const cv::Mat& L = ch[0]; const cv::Mat& A = ch[1]; const cv::Mat& B = ch[2];

    // ── 2. Pinkness score: 0..255 image. ─────────────────────────────
    //    raw = max(0, a* - 128) · gate(|b*-128| < |a*-128|) · gate(L* > L_min)
    //    Then we Otsu over the non-zero histogram.
    cv::Mat score(bgr.size(), CV_8U, cv::Scalar(0));
    for (int y = 0; y < bgr.rows; ++y) {
        const uchar* lp = L.ptr<uchar>(y);
        const uchar* ap = A.ptr<uchar>(y);
        const uchar* bp = B.ptr<uchar>(y);
        uchar* sp = score.ptr<uchar>(y);
        for (int x = 0; x < bgr.cols; ++x) {
            if (lp[x] < cfg.L_min) continue;
            int a = ap[x] - 128, b = bp[x] - 128;
            if (a <= 0) continue;                    // not red side
            if (std::abs(b) > std::abs(a)) continue; // more yellow/blue than red
            int s = a;                               // 0..127
            // Boost slightly for low |b*| (more pure red, like tape).
            int bonus = std::max(0, (std::abs(a) - std::abs(b))) / 2;
            sp[x] = (uchar)std::min(255, 2*(s + bonus));
        }
    }

    // Otsu on non-zero values to set the cut.
    std::vector<uchar> nz;
    nz.reserve(score.total() / 16);
    for (int y = 0; y < bgr.rows; y += 2)
        for (int x = 0; x < bgr.cols; x += 2)
            if (score.at<uchar>(y, x) > 0) nz.push_back(score.at<uchar>(y, x));
    int thr = 60;
    if (nz.size() > 64) {
        cv::Mat sm((int)nz.size(), 1, CV_8U, nz.data());
        cv::Mat dum;
        thr = (int)cv::threshold(sm, dum, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        thr = std::max(40, thr);
    }
    cv::Mat mask;
    cv::threshold(score, mask, thr, 255, cv::THRESH_BINARY);

    // ── 3. Morphology cleanup ────────────────────────────────────────
    int k = cfg.morph_kernel > 0
                ? cfg.morph_kernel
                : std::max(3, std::min(bgr.cols, bgr.rows) / 300);
    if (k % 2 == 0) ++k;
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  se);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, se);
    if (debug_mask) *debug_mask = mask.clone();

    // ── 4. Connected components → blobs with shape gating ────────────
    cv::Mat labels, stats, cents;
    int n = cv::connectedComponentsWithStats(mask, labels, stats, cents, 8);
    int min_area = std::max(8, (int)(cfg.min_area_frac * bgr.total()));
    int max_area = (int)(cfg.max_area_frac * bgr.total());
    for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < min_area || area > max_area) continue;
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        double aspect = (double)std::max(w, h) / std::max(1, std::min(w, h));
        if (aspect > cfg.aspect_max) continue;

        // Mean pinkness in this blob.
        cv::Mat blob_mask = (labels == i);
        double mean = cv::mean(score, blob_mask)[0];

        PinkBlob pb;
        pb.bbox = cv::Rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                            stats.at<int>(i, cv::CC_STAT_TOP), w, h);
        pb.centroid = cv::Point2f((float)cents.at<double>(i, 0),
                                   (float)cents.at<double>(i, 1));
        pb.area_px = area;
        pb.pinkness_mean = mean;
        // Confidence: combination of pinkness purity + area saturation +
        // aspect tightness.
        double cArea  = std::min(1.0, std::sqrt((double)area / max_area));
        double cPink  = std::min(1.0, mean / 200.0);
        double cAsp   = std::max(0.0, 1.0 - (aspect - 1.0) / (cfg.aspect_max - 1.0));
        pb.confidence = std::pow(cArea * cPink * cAsp, 1.0/3.0);
        out.push_back(pb);
    }

    // Sort by confidence desc.
    std::sort(out.begin(), out.end(),
              [](const PinkBlob& a, const PinkBlob& b){ return a.confidence > b.confidence; });
    return out;
}

}  // namespace mate
