// image_undistort.cpp — step 1: cached undistort with optional Metal path.
#include "image_undistort.hpp"
#include "metal_compute.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <map>
#include <mutex>
#include <tuple>

namespace mate {

namespace {

struct Maps { cv::Mat mapx, mapy; };

// Cache key: pointer to the original K data + image size. We use the K
// matrix's data pointer as a coarse identity proxy; if the user reloads
// calibration the pointer changes and we recompute.
using Key = std::tuple<const double*, int, int, int>;
std::map<Key, Maps> g_cache;
std::mutex g_mu;

}  // namespace

void clearUndistortCache() { std::lock_guard<std::mutex> g(g_mu); g_cache.clear(); }

cv::Mat undistortImage(const cv::Mat& src,
                       const CameraIntrinsics& intr,
                       bool use_metal) {
    if (src.empty()) return src;
    Maps maps;
    const bool fisheye = (intr.model == DistortionModel::Fisheye);
    Key key{ intr.K.ptr<double>(), src.cols, src.rows, fisheye ? 1 : 0 };
    {
        std::lock_guard<std::mutex> g(g_mu);
        auto it = g_cache.find(key);
        if (it != g_cache.end()) maps = it->second;
    }
    if (maps.mapx.empty()) {
        cv::Size sz(src.cols, src.rows);
        cv::Mat newK = intr.K.clone();
        if (fisheye) {
            cv::fisheye::initUndistortRectifyMap(intr.K, intr.D, cv::Mat::eye(3,3,CV_64F),
                                                 newK, sz, CV_32FC1, maps.mapx, maps.mapy);
        } else {
            cv::initUndistortRectifyMap(intr.K, intr.D, cv::Mat(),
                                        newK, sz, CV_32FC1, maps.mapx, maps.mapy);
        }
        std::lock_guard<std::mutex> g(g_mu); g_cache[key] = maps;
    }
    if (use_metal && mate::metal::ensureInitialized())
        return mate::metal::remapBgr(src, maps.mapx, maps.mapy);
    cv::Mat dst;
    cv::remap(src, dst, maps.mapx, maps.mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return dst;
}

}  // namespace mate
