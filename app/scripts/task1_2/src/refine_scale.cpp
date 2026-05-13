#include "refine_scale.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <unordered_map>
#include <cmath>

namespace mate {

// Umeyama (1991) closed-form similarity alignment: finds (R, t, c) that
// minimizes Σ || c·R·src_i + t − dst_i ||².
struct Sim3 { Eigen::Matrix3d R; Eigen::Vector3d t; double c; };

static Sim3 umeyama(const std::vector<Eigen::Vector3d>& src,
                    const std::vector<Eigen::Vector3d>& dst) {
    const int n = static_cast<int>(src.size());
    Eigen::Vector3d mu_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d mu_d = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) { mu_s += src[i]; mu_d += dst[i]; }
    mu_s /= n; mu_d /= n;

    Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
    double var_s = 0.0;
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d xs = src[i] - mu_s;
        Eigen::Vector3d xd = dst[i] - mu_d;
        Sigma += xd * xs.transpose();
        var_s += xs.squaredNorm();
    }
    Sigma /= n; var_s /= n;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if (Sigma.determinant() < 0) S(2, 2) = -1;

    Sim3 out;
    out.R = svd.matrixU() * S * svd.matrixV().transpose();
    out.c = (var_s > 1e-12) ? (svd.singularValues().asDiagonal() * S).trace() / var_s : 1.0;
    out.t = mu_d - out.c * out.R * mu_s;
    return out;
}

ScaleRefinement refineModelScaleFromPlatePriors(
        Model3D& io,
        const std::vector<std::pair<int, cv::Vec3d>>& priors) {
    ScaleRefinement out;
    if (priors.size() < 3 || io.plates.empty() || io.sections.empty()) return out;

    // Build model-frame plate-center positions for the same ids as priors.
    std::unordered_map<int, Eigen::Vector3d> model_centers;
    for (const auto& p : io.plates) {
        const auto& s = io.sections[p.section_id];
        const double sx = s.size[0], sy = s.size[1], sz = s.size[2];
        const double ox = s.origin[0], oy = s.origin[1], oz = s.origin[2];
        Eigen::Vector3d c;
        // Plate::face is the canonical string ("+x", "-x", … , "-z");
        // translate once via the parseFace() bridge declared in model3d.hpp.
        switch (parseFace(p.face)) {
            case PlateFace::PosX: c = {ox+sx, oy + p.u*sy, oz + p.v*sz}; break;
            case PlateFace::NegX: c = {ox,    oy + p.u*sy, oz + p.v*sz}; break;
            case PlateFace::PosY: c = {ox + p.u*sx, oy+sy, oz + p.v*sz}; break;
            case PlateFace::NegY: c = {ox + p.u*sx, oy,    oz + p.v*sz}; break;
            case PlateFace::PosZ: c = {ox + p.u*sx, oy + p.v*sy, oz+sz}; break;
            case PlateFace::NegZ:
            default:              c = {ox + p.u*sx, oy + p.v*sy, oz};
        }
        model_centers[p.id] = c;
    }

    std::vector<Eigen::Vector3d> src, dst;
    for (const auto& [id, t_cam] : priors) {
        auto it = model_centers.find(id);
        if (it == model_centers.end()) continue;
        src.push_back(it->second);
        dst.emplace_back(t_cam[0], t_cam[1], t_cam[2]);
    }
    if (src.size() < 3) return out;

    Sim3 sim = umeyama(src, dst);

    // Compute residual in meters.
    double sse = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
        Eigen::Vector3d r = sim.c * sim.R * src[i] + sim.t - dst[i];
        sse += r.squaredNorm();
    }
    out.rms_m = std::sqrt(sse / src.size());
    out.k = sim.c;
    out.used_n = static_cast<int>(src.size());
    out.ok = std::isfinite(out.k) && out.k > 0;

    if (out.ok) {
        // Apply uniform scale to all section sizes/origins and totals.
        for (auto& s : io.sections) {
            for (auto& v : s.size)   v *= out.k;
            for (auto& v : s.origin) v *= out.k;
        }
        for (auto& p : io.plates) p.side_m *= out.k;
        // Model3D totals are top-level fields; ScaleInfo is plain (not
        // optional). Update both directly.
        io.total_length *= out.k;
        io.total_width  *= out.k;
        io.total_height *= out.k;
        io.scale.source = "plate-prior PnP (10 cm) + Umeyama";
        io.scale.reason = "rms_m=" + std::to_string(out.rms_m);
    }
    return out;
}

}  // namespace mate
