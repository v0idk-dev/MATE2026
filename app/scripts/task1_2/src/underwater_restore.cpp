#include "underwater_restore.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace mate {

static cv::Vec3d estimateAirlight(const cv::Mat& bgr, const cv::Mat& dark) {
    // Top 0.1% of dark-channel pixels → take the brightest BGR among them.
    int N = dark.rows * dark.cols;
    int K = std::max(64, N / 1000);
    std::vector<float> dvals; dvals.reserve(N);
    for (int y = 0; y < dark.rows; ++y) {
        const uchar* d = dark.ptr<uchar>(y);
        for (int x = 0; x < dark.cols; ++x) dvals.push_back(d[x]);
    }
    std::nth_element(dvals.begin(), dvals.end() - K, dvals.end());
    float thr = dvals[dvals.size() - K];
    cv::Vec3d acc(0, 0, 0); int cnt = 0;
    double bright_max = -1;
    for (int y = 0; y < dark.rows; ++y) {
        const uchar* d = dark.ptr<uchar>(y);
        const cv::Vec3b* b = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < dark.cols; ++x) {
            if (d[x] < thr) continue;
            double br = 0.114 * b[x][0] + 0.587 * b[x][1] + 0.299 * b[x][2];
            if (br > bright_max) {
                bright_max = br;
                acc = cv::Vec3d(b[x][0], b[x][1], b[x][2]);
            }
            ++cnt;
        }
    }
    if (cnt == 0) return cv::Vec3d(220, 220, 220);
    return acc;
}

void underwaterRestore(cv::Mat& bgr, const UnderwaterCfg& cfg) {
    if (bgr.empty()) return;
    CV_Assert(bgr.type() == CV_8UC3);

    // 1. Dark channel
    cv::Mat dark(bgr.rows, bgr.cols, CV_8UC1);
    for (int y = 0; y < bgr.rows; ++y) {
        const cv::Vec3b* p = bgr.ptr<cv::Vec3b>(y);
        uchar* d = dark.ptr<uchar>(y);
        for (int x = 0; x < bgr.cols; ++x)
            d[x] = static_cast<uchar>(std::min({p[x][0], p[x][1], p[x][2]}));
    }
    cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, {15, 15});
    cv::erode(dark, dark, se);

    // 2. Airlight
    cv::Vec3d A = estimateAirlight(bgr, dark);
    A[0] = std::max(1.0, A[0]); A[1] = std::max(1.0, A[1]); A[2] = std::max(1.0, A[2]);

    // 3. Per-channel transmission, then recover.
    const double w[3] = { cfg.omega_blue, cfg.omega_green, cfg.omega_red };
    cv::Mat out = bgr.clone();
    for (int y = 0; y < bgr.rows; ++y) {
        const cv::Vec3b* in = bgr.ptr<cv::Vec3b>(y);
        cv::Vec3b* o = out.ptr<cv::Vec3b>(y);
        for (int x = 0; x < bgr.cols; ++x) {
            for (int c = 0; c < 3; ++c) {
                double t = 1.0 - w[c] * (in[x][c] / A[c]);
                t = std::max(cfg.t_min, t);
                double j = (in[x][c] - A[c]) / t + A[c];
                o[x][c] = cv::saturate_cast<uchar>(j);
            }
        }
    }

    // 4. CLAHE on L of LAB.
    cv::Mat lab; cv::cvtColor(out, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch; cv::split(lab, ch);
    auto clahe = cv::createCLAHE(cfg.clahe_clip, cv::Size(cfg.clahe_grid, cfg.clahe_grid));
    clahe->apply(ch[0], ch[0]);
    cv::merge(ch, lab);
    cv::cvtColor(lab, bgr, cv::COLOR_Lab2BGR);
}

void underwaterRestore(cv::Mat& bgr, double water_n) {
    UnderwaterCfg c; c.water_n = water_n;
    underwaterRestore(bgr, c);
}

}  // namespace mate
