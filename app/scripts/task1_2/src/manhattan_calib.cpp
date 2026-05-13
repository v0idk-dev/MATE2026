// manhattan_calib.cpp — texture-free auto-calibration via vanishing points.
// See manhattan_calib.hpp for the strategy overview.
#include "manhattan_calib.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace mate {
namespace {

// ── Line segment data ──────────────────────────────────────────────────
struct LineSeg {
    cv::Point2f a, b;
    cv::Vec3d   line_eq;     // homogeneous: a × b (image-space line)
    cv::Point2f mid;
    cv::Point2f dir;         // unit direction
    double      length = 0.0;
};

// Homogeneous line through two image points.
static inline cv::Vec3d lineFromPts(cv::Point2f a, cv::Point2f b) {
    cv::Vec3d va(a.x, a.y, 1.0), vb(b.x, b.y, 1.0);
    return va.cross(vb);
}

// ── Line detection (LSD primary, Hough fallback) ──────────────────────
// cv::createLineSegmentDetector still ships in OpenCV 4.x but throws at
// runtime when the build dropped its LSD impl due to license. Try/catch
// and fall back to HoughLinesP — slightly noisier but always available.
static std::vector<cv::Vec4f> detectLinesAny(const cv::Mat& gray, int min_len_px) {
    std::vector<cv::Vec4f> raw;
    // LSD path. Two failure modes to guard against on different OpenCV
    // builds: (a) the factory throws cv::Exception when the LSD impl was
    // dropped at build-time; (b) it silently returns a null cv::Ptr, in
    // which case calling ->detect() would segfault. Belt + suspenders.
    try {
        cv::Ptr<cv::LineSegmentDetector> lsd =
            cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
        if (!lsd.empty()) {
            lsd->detect(gray, raw);
            if (!raw.empty()) {
                std::cerr << "[manhattan] LSD: " << raw.size() << " segments\n";
                return raw;
            }
        } else {
            std::cerr << "[manhattan] LSD factory returned null — "
                         "falling back to Hough\n";
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[manhattan] LSD threw: " << e.what()
                  << " — falling back to Hough\n";
    } catch (const std::exception& e) {
        std::cerr << "[manhattan] LSD threw (std): " << e.what()
                  << " — falling back to Hough\n";
    }
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);
    std::vector<cv::Vec4i> houghi;
    cv::HoughLinesP(edges, houghi, 1, CV_PI / 180.0,
                    /*threshold=*/50, /*minLineLength=*/min_len_px,
                    /*maxLineGap=*/10);
    raw.reserve(houghi.size());
    for (auto& h : houghi) raw.push_back(cv::Vec4f((float)h[0],(float)h[1],(float)h[2],(float)h[3]));
    std::cerr << "[manhattan] Hough fallback: " << raw.size() << " segments\n";
    return raw;
}

static std::vector<LineSeg> buildSegs(const std::vector<cv::Vec4f>& raw, int min_len_px) {
    std::vector<LineSeg> out;
    out.reserve(raw.size());
    for (auto& v : raw) {
        cv::Point2f a(v[0], v[1]), b(v[2], v[3]);
        double len = cv::norm(a - b);
        if (len < min_len_px) continue;
        LineSeg s;
        s.a = a; s.b = b;
        s.line_eq = lineFromPts(a, b);
        s.mid     = cv::Point2f((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
        cv::Point2f d = b - a;
        s.dir = (len > 1e-6) ? cv::Point2f((float)(d.x / len), (float)(d.y / len))
                             : cv::Point2f(1.f, 0.f);
        s.length = len;
        out.push_back(s);
    }
    return out;
}

// ── Inlier test: angle between segment-midpoint→VP and segment direction.
// 0° = the segment points straight at the VP (perfect inlier).
static double segmentToVPAngleDeg(const LineSeg& s, const cv::Point3d& vp) {
    // Dehomogenize VP. If VP is at infinity (z ≈ 0), use direction only.
    cv::Point2f to_vp;
    if (std::abs(vp.z) < 1e-9) {
        to_vp = cv::Point2f((float)vp.x, (float)vp.y);
    } else {
        cv::Point2f vp2((float)(vp.x / vp.z), (float)(vp.y / vp.z));
        to_vp = vp2 - s.mid;
    }
    double n = std::hypot(to_vp.x, to_vp.y);
    if (n < 1e-6) return 90.0;
    cv::Point2f u((float)(to_vp.x / n), (float)(to_vp.y / n));
    // |sin θ| via cross, |cos θ| via dot — fold to [0, 90°] (line direction
    // is unsigned).
    double cosang = std::abs((double)u.x * s.dir.x + (double)u.y * s.dir.y);
    cosang = std::min(1.0, std::max(0.0, cosang));
    return std::acos(cosang) * 180.0 / CV_PI;
}

// ── Greedy RANSAC: find best-supported VP, mark inliers, repeat. ───────
struct VPCluster {
    cv::Point3d vp{0, 0, 0};
    std::vector<int> inliers;
};

static std::vector<VPCluster> findVPsRansac(
    const std::vector<LineSeg>& segs,
    int    max_vps,
    double inlier_angle_deg,
    int    n_iters,
    int    min_inliers)
{
    std::vector<VPCluster> result;
    std::vector<bool> used(segs.size(), false);
    std::mt19937 rng(0xC0DE);

    for (int round = 0; round < max_vps; ++round) {
        std::vector<int> avail;
        avail.reserve(segs.size());
        for (size_t i = 0; i < segs.size(); ++i) if (!used[i]) avail.push_back((int)i);
        if ((int)avail.size() < min_inliers) break;

        VPCluster best;
        std::uniform_int_distribution<int> di(0, (int)avail.size() - 1);
        for (int it = 0; it < n_iters; ++it) {
            int i1 = avail[di(rng)], i2 = avail[di(rng)];
            if (i1 == i2) continue;
            cv::Vec3d hv = segs[i1].line_eq.cross(segs[i2].line_eq);
            cv::Point3d vp(hv[0], hv[1], hv[2]);
            // Skip degenerate (parallel lines that produce a near-zero VP
            // before normalization — meaningless intersection).
            double mag = std::hypot(std::hypot(vp.x, vp.y), vp.z);
            if (mag < 1e-12) continue;

            std::vector<int> inl;
            inl.reserve(avail.size());
            for (int idx : avail) {
                if (segmentToVPAngleDeg(segs[idx], vp) < inlier_angle_deg)
                    inl.push_back(idx);
            }
            if ((int)inl.size() > (int)best.inliers.size()) {
                best.vp = vp;
                best.inliers = std::move(inl);
            }
        }
        if ((int)best.inliers.size() < min_inliers) break;
        for (int idx : best.inliers) used[idx] = true;
        std::cerr << "[manhattan] VP " << result.size()
                  << ": " << best.inliers.size() << " inliers\n";
        result.push_back(std::move(best));
    }
    return result;
}

// ── Caprile-Torre: focal from two orthogonal VPs (principal pt assumed
// at image center). Returns -1 on degenerate / non-positive f².
static double focalFromTwoVPs(const cv::Point3d& v1, const cv::Point3d& v2,
                              double cx, double cy)
{
    if (std::abs(v1.z) < 1e-9 || std::abs(v2.z) < 1e-9) return -1.0;
    double v1x = v1.x / v1.z, v1y = v1.y / v1.z;
    double v2x = v2.x / v2.z, v2y = v2.y / v2.z;
    double f2 = -((v1x - cx) * (v2x - cx) + (v1y - cy) * (v2y - cy));
    if (!std::isfinite(f2) || f2 <= 0) return -1.0;
    return std::sqrt(f2);
}

}  // namespace

ManhattanCalibResult deriveCalibrationFromManhattan(const ManhattanCalibInput& in) {
    ManhattanCalibResult r;
    if (in.sample_image.empty()) {
        r.note = "no sample image provided";
        return r;
    }
    const int W = in.sample_image.cols, H = in.sample_image.rows;
    const double cx = W * 0.5, cy = H * 0.5;

    // Greyscale + line detection. Explicitly handle 1/3/4 channel inputs
    // — current pipeline call path always passes 3-channel BGR (imread
    // with IMREAD_COLOR forces it), but the function is public and a
    // future caller might hand us a 4-channel BGRA frame, in which case
    // assigning straight to `gray` would crash Canny downstream.
    cv::Mat gray;
    const int ch = in.sample_image.channels();
    if      (ch == 1) gray = in.sample_image;
    else if (ch == 3) cv::cvtColor(in.sample_image, gray, cv::COLOR_BGR2GRAY);
    else if (ch == 4) cv::cvtColor(in.sample_image, gray, cv::COLOR_BGRA2GRAY);
    else {
        r.note = "unsupported image channel count: " + std::to_string(ch);
        return r;  // ok = false
    }
    int min_len = std::max(20, std::min(W, H) / 30);

    auto raw = detectLinesAny(gray, min_len);
    auto segs = buildSegs(raw, min_len);
    r.n_line_segments = (int)segs.size();
    if ((int)segs.size() < 8) {
        r.note = "too few line segments (" + std::to_string(segs.size()) + ") for VP recovery";
        // Continue with fallback focal — we still want to build a usable
        // K so the pipeline can run instead of bailing entirely.
    }

    // VP search. Tolerance of 2° works well for sharply-imaged PVC; loosen
    // to 3° on small sets.
    double tol = (segs.size() < 30) ? 3.0 : 2.0;
    auto vps = findVPsRansac(segs, /*max_vps=*/3, tol,
                             /*iters=*/400, /*min_inliers=*/6);
    r.n_vanishing_points = (int)vps.size();

    // Focal estimate: median over all orthogonal-pair candidates.
    double f_est = -1.0;
    if (vps.size() >= 2) {
        std::vector<double> fs;
        for (size_t i = 0; i < vps.size(); ++i)
            for (size_t j = i + 1; j < vps.size(); ++j) {
                double f = focalFromTwoVPs(vps[i].vp, vps[j].vp, cx, cy);
                if (f > 0) fs.push_back(f);
            }
        if (!fs.empty()) {
            std::sort(fs.begin(), fs.end());
            f_est = fs[fs.size() / 2];
        }
    }

    // Sanity bounds: typical lens focals span ~(0.3 W) to (4.0 W) in pixels.
    bool used_fallback = false;
    if (!(f_est > W * 0.3 && f_est < W * 4.0)) {
        // Fallback: assume 50° horizontal FOV → f = W / (2 tan 25°).
        double f_fallback = W / (2.0 * std::tan(25.0 * CV_PI / 180.0));
        std::cerr << "[manhattan] focal " << f_est
                  << " out of plausible range [" << W * 0.3 << ", " << W * 4.0
                  << "] → assuming 50° HFOV (" << f_fallback << " px)\n";
        f_est = f_fallback;
        used_fallback = true;
    }
    r.estimated_focal_px = f_est;
    r.focal_from_vps = !used_fallback;

    if (used_fallback) {
        r.note = "focal=" + std::to_string((int)f_est) +
                 "px (fallback: assumed 50° HFOV; "
                 + std::to_string(vps.size()) + " VPs found, " +
                 std::to_string(segs.size()) + " line segs)";
    } else {
        r.note = "focal=" + std::to_string((int)f_est) +
                 "px from " + std::to_string(vps.size()) + " VPs (" +
                 std::to_string(segs.size()) + " line segs)";
    }

    // Build K. Distortion left at zero — without a calibration target we
    // can't recover lens distortion; assume the cameras are reasonably
    // rectilinear (true for most ROV stereo rigs after the manufacturer's
    // factory calibration is baked into the sensor).
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 f_est, 0,    cx,
                 0,     f_est, cy,
                 0,     0,     1);
    // Distortion: row-vector 1×5 to match the codebase convention (the
    // YAML loader normalizes any column-vector D to row form before
    // handing it downstream — see calibration_io.cpp). cv::stereoRectify
    // accepts either, but staying consistent avoids surprising any later
    // helper that does shape introspection.
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);

    r.K_left.K = K.clone();
    r.K_left.D = D.clone();
    r.K_left.model = DistortionModel::Pinhole;
    r.K_left.image_width  = W;
    r.K_left.image_height = H;
    r.K_left.rms_px = -1.0;
    r.K_left.source_path = "manhattan-auto";
    r.K_right = r.K_left;  // rigid rig: shared intrinsics.

    // Stereo extrinsics: parallel-axis rig with horizontal baseline.
    // T is right-camera origin in left-camera frame; with R = I, the
    // standard convention is T = (-baseline, 0, 0).
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat T = (cv::Mat_<double>(3, 1) << -std::abs(in.rig_baseline_m), 0, 0);
    cv::Mat R1, R2, P1, P2, Q;
    try {
        cv::stereoRectify(K, D, K, D, cv::Size(W, H), R, T, R1, R2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY, /*alpha=*/0.0);
    } catch (const cv::Exception& e) {
        r.note += "; stereoRectify failed: " + std::string(e.what());
        return r;  // ok = false
    }

    r.extrinsics.R = R; r.extrinsics.T = T;
    r.extrinsics.R1 = R1; r.extrinsics.R2 = R2;
    r.extrinsics.P1 = P1; r.extrinsics.P2 = P2;
    r.extrinsics.Q  = Q;
    r.extrinsics.image_width  = W;
    r.extrinsics.image_height = H;
    r.extrinsics.unit = "m";
    r.extrinsics.baseline = std::abs(in.rig_baseline_m);
    r.extrinsics.rms_px = -1.0;
    r.extrinsics.K_left_provided  = r.K_left;
    r.extrinsics.K_right_provided = r.K_right;
    r.extrinsics.has_provided_intrinsics = true;
    r.extrinsics.source_path = "manhattan-auto";

    r.ok = true;
    std::cerr << "[manhattan] auto-calib OK: " << r.note
              << ", baseline=" << in.rig_baseline_m << "m\n";
    return r;
}

}  // namespace mate
