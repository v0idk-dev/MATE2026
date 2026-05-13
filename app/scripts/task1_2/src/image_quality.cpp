// image_quality.cpp — see header for description.
#include "image_quality.hpp"
#include <opencv2/imgproc.hpp>
#include <sstream>

namespace mate {

namespace {

double laplacianVariance(const cv::Mat& gray) {
    cv::Mat lap; cv::Laplacian(gray, lap, CV_64F, 3);
    cv::Scalar mu, sd; cv::meanStdDev(lap, mu, sd);
    return sd[0] * sd[0];
}

double clippedFraction(const cv::Mat& gray) {
    if (gray.empty()) return 0.0;
    int clip = 0;
    const int N = (int)gray.total();
    for (int i = 0; i < N; ++i) {
        uchar v = gray.data[i];
        if (v == 0 || v == 255) ++clip;
    }
    return (double)clip / std::max(1, N);
}

cv::Mat toGray(const cv::Mat& img) {
    if (img.empty()) return img;
    if (img.channels() == 1) return img;
    cv::Mat g; cv::cvtColor(img, g, cv::COLOR_BGR2GRAY); return g;
}

}  // namespace

ImageQualityReport
checkImagePairQuality(const cv::Mat& L, const cv::Mat& R,
                      const ImageQualityConfig& cfg) {
    ImageQualityReport q;
    auto fail = [&](const std::string& s){ q.pass = false; q.reasons.push_back(s); };

    if (L.empty() || R.empty())                        { fail("empty image");         return q; }
    if (L.size() != R.size())                          { fail("L/R size mismatch");                }
    if (L.cols < cfg.min_width || L.rows < cfg.min_height ||
        R.cols < cfg.min_width || R.rows < cfg.min_height) {
        std::ostringstream os; os << "image too small (<" << cfg.min_width << "x" << cfg.min_height << ")";
        fail(os.str());
    }
    if ((L.channels() != 1 && L.channels() != 3) ||
        (R.channels() != 1 && R.channels() != 3)) {
        fail("unsupported channel count (need 1 or 3)");
    }

    cv::Mat gL = toGray(L), gR = toGray(R);
    if (gL.empty() || gR.empty()) return q;

    q.blur_var_left  = laplacianVariance(gL);
    q.blur_var_right = laplacianVariance(gR);
    q.mean_lum_left  = cv::mean(gL)[0];
    q.mean_lum_right = cv::mean(gR)[0];
    q.clip_frac_left  = clippedFraction(gL);
    q.clip_frac_right = clippedFraction(gR);

    if (q.blur_var_left  < cfg.blur_var_min) fail("left image too blurry");
    if (q.blur_var_right < cfg.blur_var_min) fail("right image too blurry");
    {
        double a = std::max(1e-6, std::min(q.blur_var_left, q.blur_var_right));
        double b = std::max(q.blur_var_left, q.blur_var_right);
        if (b / a > cfg.blur_var_ratio_max) fail("L/R blur asymmetry");
    }
    {
        double a = std::max(1.0, std::min(q.mean_lum_left, q.mean_lum_right));
        double b = std::max(q.mean_lum_left, q.mean_lum_right);
        if (b / a > cfg.exposure_ratio_max) fail("L/R exposure mismatch");
    }
    if (q.clip_frac_left  > cfg.clip_frac_max) fail("left image clipped (over/under-exposed)");
    if (q.clip_frac_right > cfg.clip_frac_max) fail("right image clipped (over/under-exposed)");

    return q;
}

}  // namespace mate
