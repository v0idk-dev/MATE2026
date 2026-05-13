#include "dense_mvs.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <cmath>

namespace mate {

namespace {
struct VoxKey { int x,y,z; bool operator==(const VoxKey& o) const {
    return x==o.x && y==o.y && z==o.z; } };
struct VoxHash { size_t operator()(const VoxKey& k) const {
    return ((size_t)(uint32_t)k.x * 73856093u) ^
           ((size_t)(uint32_t)k.y * 19349663u) ^
           ((size_t)(uint32_t)k.z * 83492791u); } };
}

std::vector<DensePoint>
computeDenseCloud(const cv::Mat& L, const cv::Mat& R,
                  const cv::Mat& Q, const DenseConfig& cfg) {
    std::vector<DensePoint> out;
    if (!cfg.enabled || L.empty() || R.empty() || Q.empty()) return out;

    cv::Mat gL, gR;
    cv::cvtColor(L, gL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(R, gR, cv::COLOR_BGR2GRAY);

    int nd = std::max(16, (cfg.num_disparities / 16) * 16);
    int bs = cfg.block_size | 1;  // ensure odd
    auto sgbm = cv::StereoSGBM::create(
        cfg.min_disparity, nd, bs,
        8 * 3 * bs * bs, 32 * 3 * bs * bs,
        1, 0, cfg.uniqueness_ratio,
        cfg.speckle_window_size, cfg.speckle_range,
        cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disp16;
    sgbm->compute(gL, gR, disp16);
    cv::Mat disp;
    disp16.convertTo(disp, CV_32F, 1.0 / 16.0);

    cv::Mat xyz;
    cv::reprojectImageTo3D(disp, xyz, Q, true);

    std::unordered_map<VoxKey, DensePoint, VoxHash> voxels;
    voxels.reserve(cfg.max_points * 2);
    const float vs = (float)cfg.voxel_size_m;

    for (int y = 0; y < xyz.rows; ++y) {
        const cv::Vec3f* row = xyz.ptr<cv::Vec3f>(y);
        const cv::Vec3b* crow = L.ptr<cv::Vec3b>(y);
        const float* drow = disp.ptr<float>(y);
        for (int x = 0; x < xyz.cols; ++x) {
            float d = drow[x];
            if (d <= 0 || !std::isfinite(d)) continue;
            const cv::Vec3f& p = row[x];
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) continue;
            if (std::abs(p[2]) > 100.0f) continue;  // sanity: drop crazy
            VoxKey k{ (int)std::floor(p[0]/vs), (int)std::floor(p[1]/vs), (int)std::floor(p[2]/vs) };
            auto it = voxels.find(k);
            if (it == voxels.end()) {
                DensePoint dp;
                dp.position = cv::Point3f(p[0], p[1], p[2]);
                dp.color = crow[x];
                voxels.emplace(k, dp);
                if ((int)voxels.size() >= cfg.max_points) break;
            }
        }
        if ((int)voxels.size() >= cfg.max_points) break;
    }
    out.reserve(voxels.size());
    for (auto& kv : voxels) out.push_back(kv.second);
    return out;
}

}  // namespace mate
