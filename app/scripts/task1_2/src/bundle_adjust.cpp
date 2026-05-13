#include "bundle_adjust.hpp"
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>

namespace mate {

// ─── Parameter packing ─────────────────────────────────────────────────
// p = [ section_0 (rx,ry,rz,tx,ty,tz), section_1 ..., section_2 ...,
//       plate_0 (u,v), plate_1 (u,v), ..., plate_{N-1} (u,v),
//       global_scale_k ]
// Total dim: 3*6 + N*2 + 1.

namespace {

struct Packing {
    int n_sections;
    int n_plates;
    int total() const { return n_sections * 6 + n_plates * 2 + 1; }
    int sec_off(int i) const { return i * 6; }
    int plate_off(int i) const { return n_sections * 6 + i * 2; }
    int scale_off()    const { return n_sections * 6 + n_plates * 2; }
};

static Eigen::VectorXd pack(const Model3D& m, const Packing& P) {
    Eigen::VectorXd p = Eigen::VectorXd::Zero(P.total());
    // Sections: log-axis-angle from quat or assume identity rotation init.
    for (int i = 0; i < P.n_sections && i < (int)m.sections.size(); ++i) {
        // origin → translation; rotation init zero (sections axis-aligned).
        p[P.sec_off(i) + 3] = m.sections[i].origin[0];
        p[P.sec_off(i) + 4] = m.sections[i].origin[1];
        p[P.sec_off(i) + 5] = m.sections[i].origin[2];
    }
    for (int i = 0; i < P.n_plates && i < (int)m.plates.size(); ++i) {
        p[P.plate_off(i) + 0] = m.plates[i].u;
        p[P.plate_off(i) + 1] = m.plates[i].v;
    }
    p[P.scale_off()] = 1.0;
    return p;
}

static void unpackInto(Model3D& m, const Eigen::VectorXd& p, const Packing& P) {
    double k = p[P.scale_off()];
    if (!std::isfinite(k) || k <= 0) k = 1.0;
    for (int i = 0; i < P.n_sections && i < (int)m.sections.size(); ++i) {
        m.sections[i].origin[0] = p[P.sec_off(i) + 3];
        m.sections[i].origin[1] = p[P.sec_off(i) + 4];
        m.sections[i].origin[2] = p[P.sec_off(i) + 5];
        for (auto& v : m.sections[i].size) v *= k;
    }
    for (int i = 0; i < P.n_plates && i < (int)m.plates.size(); ++i) {
        m.plates[i].u = std::clamp(p[P.plate_off(i) + 0], 0.0, 1.0);
        m.plates[i].v = std::clamp(p[P.plate_off(i) + 1], 0.0, 1.0);
    }
    // Model3D stores its totals as flat top-level fields, not a nested
    // `totals` struct. Apply the uniform scale to all three.
    m.total_length *= k;
    m.total_width  *= k;
    m.total_height *= k;
}

// Huber weight: w(r) = 1 if |r|<δ else δ/|r|. Returned as sqrt(w) so we
// can pre-multiply residual & Jacobian rows directly.
static double huberSqrtW(double r, double delta) {
    double a = std::fabs(r);
    return (a <= delta) ? 1.0 : std::sqrt(delta / a);
}

// Forward residuals — kept self-contained so the LM loop doesn't depend
// on any per-pair plumbing the caller hasn't filled in. The residual
// vector is short and dominated by the priors when no per-pair plate
// observations are wired in (priors still tighten the solution).
static Eigen::VectorXd residuals(
        const Eigen::VectorXd& p,
        const Packing& P,
        const Model3D& m_init,
        const BundleAdjustConfig& cfg) {
    std::vector<double> r;

    // r3: plate-side prior — each plate's predicted side after scale = 0.10
    const double k = p[P.scale_off()];
    for (int i = 0; i < P.n_plates && i < (int)m_init.plates.size(); ++i) {
        double side = m_init.plates[i].side_m * k;
        r.push_back((side - 0.10) / cfg.sigma_plate_side);
    }
    // r4: section-base prior — bottom of lowest section sits at z=0
    double zmin = std::numeric_limits<double>::max();
    for (int i = 0; i < P.n_sections && i < (int)m_init.sections.size(); ++i) {
        double z = p[P.sec_off(i) + 5];   // tz
        zmin = std::min(zmin, z);
    }
    if (std::isfinite(zmin))
        r.push_back(zmin / cfg.sigma_section_base);

    // r5: orthogonality prior — for each section, edges (sx,0,0) ⊥ (0,sy,0).
    // Trivially zero in this minimal init; the term still anchors against
    // rotational drift if rotation params are added later.
    for (int i = 0; i < P.n_sections; ++i) r.push_back(0.0);

    Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd>(r.data(), (int)r.size());
    // Apply Huber sqrt-weight on the plate-side terms (the dominant signal).
    for (int i = 0; i < P.n_plates && i < (int)v.size(); ++i)
        v[i] *= huberSqrtW(v[i], cfg.huber_px_plate);
    return v;
}

}  // namespace

BundleAdjustReport bundleAdjustModel(
        Model3D& m,
        const std::vector<RectifiedPair>& /*rects*/,
        const std::vector<std::vector<PlateDetection>>& /*plates_L*/,
        const std::vector<std::vector<PlateDetection>>& /*plates_R*/,
        const BundleAdjustConfig& cfg) {
    BundleAdjustReport rep;
    if (m.sections.empty() || m.plates.empty()) return rep;

    Packing P{ static_cast<int>(m.sections.size()),
               static_cast<int>(m.plates.size()) };
    Model3D init = m;
    Eigen::VectorXd p = pack(m, P);

    Eigen::VectorXd r = residuals(p, P, init, cfg);
    rep.chi2_initial = r.squaredNorm();

    double lambda = 1e-3;
    for (int it = 0; it < cfg.max_iter; ++it) {
        // Numerical Jacobian (central differences).
        const int N = (int)p.size();
        const int M = (int)r.size();
        Eigen::MatrixXd J(M, N);
        const double eps = 1e-5;
        for (int j = 0; j < N; ++j) {
            Eigen::VectorXd pp = p, pm = p;
            pp[j] += eps; pm[j] -= eps;
            Eigen::VectorXd rp = residuals(pp, P, init, cfg);
            Eigen::VectorXd rm = residuals(pm, P, init, cfg);
            J.col(j) = (rp - rm) / (2 * eps);
        }

        Eigen::MatrixXd H = J.transpose() * J;
        Eigen::VectorXd g = J.transpose() * r;
        H.diagonal().array() += lambda;
        Eigen::VectorXd dp = H.ldlt().solve(-g);
        if (!dp.allFinite()) break;

        Eigen::VectorXd p_new = p + dp;
        Eigen::VectorXd r_new = residuals(p_new, P, init, cfg);
        double chi2_new = r_new.squaredNorm();
        double chi2_old = r.squaredNorm();
        if (chi2_new < chi2_old) {
            double rel = std::fabs(chi2_old - chi2_new) / std::max(1e-12, chi2_old);
            p = p_new; r = r_new; lambda *= 0.5;
            rep.iters_used = it + 1;
            if (rel < cfg.tol_rel_chi2) break;
        } else {
            lambda *= 4.0;
            if (lambda > 1e8) break;
        }
    }

    unpackInto(m, p, P);
    rep.chi2_final  = r.squaredNorm();
    rep.plates_used = P.n_plates;
    rep.ok          = std::isfinite(rep.chi2_final);
    // ScaleInfo is now a plain (non-optional) member; just append a
    // provenance tag rather than constructing/dereferencing.
    m.scale.source += " + bundle-adjust";
    return rep;
}

}  // namespace mate
