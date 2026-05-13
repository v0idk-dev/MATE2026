#pragma once
// Dense reconstruction via StereoSGBM disparity. Optional path; off by
// default (sparse wireframe meets the ±5cm spec). When enabled, returns
// a downsampled point cloud in left-camera frame, units = baseline unit.
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct DenseConfig {
    bool enabled = false;
    int  num_disparities = 128;     // multiple of 16
    int  block_size = 7;            // odd, 5..15
    int  speckle_window_size = 100;
    int  speckle_range = 32;
    int  min_disparity = 0;
    int  uniqueness_ratio = 10;
    double voxel_size_m = 0.02;     // 2cm voxel; smaller = denser cloud
    int max_points = 60000;         // hard cap to keep JSON small
};

struct DensePoint {
    cv::Point3f position;
    cv::Vec3b color;                // BGR
};

std::vector<DensePoint>
computeDenseCloud(const cv::Mat& left_rect, const cv::Mat& right_rect,
                  const cv::Mat& Q, const DenseConfig& cfg);

}  // namespace mate
