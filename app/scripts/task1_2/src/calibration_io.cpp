#include "calibration_io.hpp"
#include <opencv2/core/persistence.hpp>
#include <algorithm>
#include <cctype>

namespace mate {

namespace {

// Helper: read a node into a Mat if present; returns false if absent.
bool readMatIfPresent(const cv::FileNode& node, cv::Mat& out) {
    if (node.empty() || node.isNone()) return false;
    node >> out;
    return !out.empty();
}

// Helper: read a string node case-insensitively; returns lowered value or
// empty string. cv::FileNode.string() returns "" for non-string nodes.
std::string readStrLower(const cv::FileNode& node) {
    if (node.empty() || node.isNone() || !node.isString()) return {};
    std::string s = (std::string)node;
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

}  // namespace

std::optional<CameraIntrinsics>
loadCameraIntrinsicsYaml(const std::string& path) {
    cv::FileStorage fs;
    if (!fs.open(path, cv::FileStorage::READ)) return std::nullopt;

    CameraIntrinsics ci;
    ci.source_path = path;

    if (!readMatIfPresent(fs["K"], ci.K)) return std::nullopt;
    if (!readMatIfPresent(fs["D"], ci.D)) return std::nullopt;
    if (ci.K.rows != 3 || ci.K.cols != 3) return std::nullopt;

    // Convert to double for downstream math; FileStorage may emit either.
    if (ci.K.type() != CV_64F) ci.K.convertTo(ci.K, CV_64F);
    if (ci.D.type() != CV_64F) ci.D.convertTo(ci.D, CV_64F);

    // Distortion model — explicit "model" field wins; otherwise infer from
    // shape. Fisheye stores D as (4,1); pinhole as (1,N).
    std::string model = readStrLower(fs["model"]);
    if (model == "fisheye" || model == "equidistant") {
        ci.model = DistortionModel::Fisheye;
    } else if (model == "pinhole" || model == "brown" || model == "rational") {
        ci.model = DistortionModel::Pinhole;
    } else {
        // Heuristic: 4-element column vector → fisheye; otherwise pinhole.
        if (ci.D.total() == 4 && ci.D.rows == 4 && ci.D.cols == 1) {
            ci.model = DistortionModel::Fisheye;
        } else {
            ci.model = DistortionModel::Pinhole;
            // Normalize to a 1×N row for OpenCV's pinhole undistort.
            if (ci.D.rows > 1 && ci.D.cols == 1) {
                ci.D = ci.D.t();
            }
        }
    }

    if (!fs["image_width"].empty())  ci.image_width  = (int)fs["image_width"];
    if (!fs["image_height"].empty()) ci.image_height = (int)fs["image_height"];
    if (!fs["rms_px"].empty())       ci.rms_px = (double)fs["rms_px"];

    return ci;
}

std::optional<StereoExtrinsics>
loadStereoExtrinsicsYaml(const std::string& path) {
    cv::FileStorage fs;
    if (!fs.open(path, cv::FileStorage::READ)) return std::nullopt;

    StereoExtrinsics ex;
    ex.source_path = path;

    // Required: R, T. The rest may be absent (older calibrators).
    if (!readMatIfPresent(fs["R"], ex.R)) return std::nullopt;
    if (!readMatIfPresent(fs["T"], ex.T)) return std::nullopt;

    if (ex.R.type() != CV_64F) ex.R.convertTo(ex.R, CV_64F);
    if (ex.T.type() != CV_64F) ex.T.convertTo(ex.T, CV_64F);

    // Optional matrices.
    readMatIfPresent(fs["E"],  ex.E);
    readMatIfPresent(fs["F"],  ex.F);
    readMatIfPresent(fs["R1"], ex.R1);
    readMatIfPresent(fs["R2"], ex.R2);
    readMatIfPresent(fs["P1"], ex.P1);
    readMatIfPresent(fs["P2"], ex.P2);
    readMatIfPresent(fs["Q"],  ex.Q);

    // Scalars.
    if (!fs["image_width"].empty())          ex.image_width  = (int)fs["image_width"];
    if (!fs["image_height"].empty())         ex.image_height = (int)fs["image_height"];
    if (!fs["unit"].empty() && fs["unit"].isString()) ex.unit = (std::string)fs["unit"];
    if (!fs["stereo_rms_px"].empty())        ex.rms_px = (double)fs["stereo_rms_px"];
    if (!fs["avg_epipolar_err_px"].empty())  ex.avg_epipolar_err_px = (double)fs["avg_epipolar_err_px"];
    if (!fs["baseline"].empty())             ex.baseline = (double)fs["baseline"];
    if (!fs["pairs_used"].empty())           ex.pairs_used = (int)fs["pairs_used"];

    // Embedded fallback intrinsics. Both K_left and K_right must be present
    // for us to consider them usable.
    cv::Mat K_l, D_l, K_r, D_r;
    if (readMatIfPresent(fs["K_left"], K_l) && readMatIfPresent(fs["K_right"], K_r)) {
        readMatIfPresent(fs["D_left"], D_l);
        readMatIfPresent(fs["D_right"], D_r);
        if (K_l.type() != CV_64F) K_l.convertTo(K_l, CV_64F);
        if (K_r.type() != CV_64F) K_r.convertTo(K_r, CV_64F);
        if (!D_l.empty() && D_l.type() != CV_64F) D_l.convertTo(D_l, CV_64F);
        if (!D_r.empty() && D_r.type() != CV_64F) D_r.convertTo(D_r, CV_64F);
        ex.K_left_provided.K = K_l;
        ex.K_left_provided.D = D_l;
        ex.K_left_provided.image_width  = ex.image_width;
        ex.K_left_provided.image_height = ex.image_height;
        ex.K_left_provided.source_path  = path + "#K_left";
        ex.K_right_provided.K = K_r;
        ex.K_right_provided.D = D_r;
        ex.K_right_provided.image_width  = ex.image_width;
        ex.K_right_provided.image_height = ex.image_height;
        ex.K_right_provided.source_path  = path + "#K_right";
        // Heuristic on shape (we don't know whether stereo_calibrate.py was
        // run with fisheye intrinsics — at the moment our stereo_calibrate.py
        // assumes pinhole, but leaving this future-proof).
        for (auto* ci : {&ex.K_left_provided, &ex.K_right_provided}) {
            if (ci->D.total() == 4 && ci->D.rows == 4 && ci->D.cols == 1) {
                ci->model = DistortionModel::Fisheye;
            } else {
                ci->model = DistortionModel::Pinhole;
                if (!ci->D.empty() && ci->D.rows > 1 && ci->D.cols == 1) {
                    ci->D = ci->D.t();
                }
            }
        }
        ex.has_provided_intrinsics = true;
    }

    return ex;
}

double unitToMeters(const std::string& unit) {
    std::string u = unit;
    std::transform(u.begin(), u.end(), u.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (u == "cm") return 0.01;
    if (u == "mm") return 0.001;
    if (u == "in" || u == "inch" || u == "inches") return 0.0254;
    return 1.0;  // default: assume meters
}

}  // namespace mate
