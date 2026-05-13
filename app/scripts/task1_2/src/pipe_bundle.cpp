// pipe_bundle.cpp — Eigen-only LM bundle adjustment over pipe graph.
#include "pipe_bundle.hpp"
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace mate {

namespace {

Eigen::Vector2d projectE(const Eigen::Matrix<double,3,4>& P, const Eigen::Vector3d& X) {
    Eigen::Vector4d Xh; Xh << X, 1.0;
    Eigen::Vector3d x = P * Xh;
    return { x[0]/x[2], x[1]/x[2] };
}

Eigen::Matrix<double,3,4> toEigen34(const cv::Mat& P) {
    // Force CV_64F — the architect noted that a CV_32F input would make
    // .at<double>() return garbage. Convert defensively.
    cv::Mat Pd;
    if (P.type() != CV_64F) P.convertTo(Pd, CV_64F);
    else Pd = P;
    Eigen::Matrix<double,3,4> E;
    for (int r=0;r<3;++r) for (int c=0;c<4;++c) E(r,c) = Pd.at<double>(r,c);
    return E;
}

double huber(double r, double delta) {
    double a = std::fabs(r);
    return (a <= delta) ? 0.5*r*r : delta*(a - 0.5*delta);
}

}  // namespace

PipeBundleReport bundleAdjustPipes(PipeGraphResult& g,
                                    const cv::Mat& P1, const cv::Mat& P2,
                                    const PipeBundleConfig& cfg) {
    PipeBundleReport R;
    R.n_junctions = (int)g.junctions.size();
    R.n_pipes     = (int)g.pipes.size();
    if (g.junctions.empty() || g.pipes.empty()) return R;

    Eigen::Matrix<double,3,4> EP1 = toEigen34(P1), EP2 = toEigen34(P2);

    // Variable layout: [J_x J_y J_z]_{0..J-1} ++ [r]_{0..P-1}
    int J = (int)g.junctions.size();
    int P = (int)g.pipes.size();
    int N = 3*J + P;
    Eigen::VectorXd x(N);
    for (int j = 0; j < J; ++j) {
        x(3*j+0) = g.junctions[j].position.x;
        x(3*j+1) = g.junctions[j].position.y;
        x(3*j+2) = g.junctions[j].position.z;
    }
    for (int p = 0; p < P; ++p) x(3*J + p) = g.pipes[p].cyl.radius_m;

    // Cache: per-pipe initial junction positions (priors) and observed
    // 2D endpoints (use the cylinder's 3D endpoints projected through P1, P2 as observations).
    std::vector<Eigen::Vector3d> Jprior(J);
    for (int j = 0; j < J; ++j) Jprior[j] << g.junctions[j].position.x,
                                              g.junctions[j].position.y,
                                              g.junctions[j].position.z;
    std::vector<Eigen::Vector2d> obsLA(P), obsLB(P), obsRA(P), obsRB(P);
    for (int p = 0; p < P; ++p) {
        Eigen::Vector3d Xa(g.pipes[p].cyl.endpoint_a.x, g.pipes[p].cyl.endpoint_a.y, g.pipes[p].cyl.endpoint_a.z);
        Eigen::Vector3d Xb(g.pipes[p].cyl.endpoint_b.x, g.pipes[p].cyl.endpoint_b.y, g.pipes[p].cyl.endpoint_b.z);
        obsLA[p] = projectE(EP1, Xa); obsLB[p] = projectE(EP1, Xb);
        obsRA[p] = projectE(EP2, Xa); obsRB[p] = projectE(EP2, Xb);
    }

    auto Xj = [&](const Eigen::VectorXd& xv, int j){
        return Eigen::Vector3d(xv(3*j+0), xv(3*j+1), xv(3*j+2));
    };

    auto cost = [&](const Eigen::VectorXd& xv) {
        double c = 0;
        for (int p = 0; p < P; ++p) {
            int ja = g.pipes[p].junction_a, jb = g.pipes[p].junction_b;
            if (ja < 0 || jb < 0) continue;
            Eigen::Vector3d Xa = Xj(xv, ja), Xb = Xj(xv, jb);
            Eigen::Vector2d eLA = projectE(EP1, Xa) - obsLA[p];
            Eigen::Vector2d eLB = projectE(EP1, Xb) - obsLB[p];
            Eigen::Vector2d eRA = projectE(EP2, Xa) - obsRA[p];
            Eigen::Vector2d eRB = projectE(EP2, Xb) - obsRB[p];
            for (int k=0;k<2;++k) {
                c += huber(eLA[k]/cfg.sigma_px, cfg.huber_px/cfg.sigma_px);
                c += huber(eLB[k]/cfg.sigma_px, cfg.huber_px/cfg.sigma_px);
                c += huber(eRA[k]/cfg.sigma_px, cfg.huber_px/cfg.sigma_px);
                c += huber(eRB[k]/cfg.sigma_px, cfg.huber_px/cfg.sigma_px);
            }
            // radius prior in [r_min, r_max]
            double r = xv(3*J + p);
            double rmid = 0.018, rsig = cfg.sigma_radius_m;
            c += 0.5 * std::pow((r - rmid)/rsig, 2.0);
        }
        // Junction priors.
        for (int j = 0; j < J; ++j) {
            Eigen::Vector3d Xc = Xj(xv, j);
            Eigen::Vector3d e  = (Xc - Jprior[j]) / cfg.sigma_junction_prior_m;
            c += 0.5 * (e[0]*e[0]+e[1]*e[1]+e[2]*e[2]);
        }
        return c;
    };

    R.chi2_initial = cost(x);

    // Numerical gradient + diagonal Levenberg.
    double lambda = 1e-3;
    double prev = R.chi2_initial;
    Eigen::VectorXd grad(N);
    const double h_pos = 1e-4, h_r = 1e-5;
    for (int it = 0; it < cfg.max_iter; ++it) {
        R.iters_used = it + 1;
        for (int k = 0; k < N; ++k) {
            double h = (k < 3*J) ? h_pos : h_r;
            Eigen::VectorXd xp = x, xm = x; xp[k] += h; xm[k] -= h;
            grad[k] = (cost(xp) - cost(xm)) / (2*h);
        }
        // Step.
        Eigen::VectorXd step = -lambda * grad;
        Eigen::VectorXd xn = x + step;
        // Clamp radii non-negative.
        for (int p = 0; p < P; ++p) xn(3*J + p) = std::max(0.001, xn(3*J + p));
        double cur = cost(xn);
        if (cur < prev) {
            x = xn;
            if ((prev - cur)/std::max(1e-12, prev) < cfg.tol_rel_chi2) { prev = cur; break; }
            prev = cur; lambda *= 1.3;
        } else {
            lambda *= 0.5; if (lambda < 1e-9) break;
        }
    }
    R.chi2_final = prev;
    R.ok = (R.chi2_final <= R.chi2_initial);

    // Write back.
    for (int j = 0; j < J; ++j) {
        g.junctions[j].position.x = x(3*j+0);
        g.junctions[j].position.y = x(3*j+1);
        g.junctions[j].position.z = x(3*j+2);
    }
    for (int p = 0; p < P; ++p) {
        double r = std::max(0.001, x(3*J + p));
        g.pipes[p].cyl.radius_m = r;
        // Re-derive endpoints from the (possibly shifted) junctions.
        if (g.pipes[p].junction_a >= 0)
            g.pipes[p].cyl.endpoint_a = g.junctions[g.pipes[p].junction_a].position;
        if (g.pipes[p].junction_b >= 0)
            g.pipes[p].cyl.endpoint_b = g.junctions[g.pipes[p].junction_b].position;
        g.pipes[p].cyl.center = cv::Point3d(
            0.5*(g.pipes[p].cyl.endpoint_a.x + g.pipes[p].cyl.endpoint_b.x),
            0.5*(g.pipes[p].cyl.endpoint_a.y + g.pipes[p].cyl.endpoint_b.y),
            0.5*(g.pipes[p].cyl.endpoint_a.z + g.pipes[p].cyl.endpoint_b.z));
        cv::Vec3d ax(g.pipes[p].cyl.endpoint_b.x - g.pipes[p].cyl.endpoint_a.x,
                     g.pipes[p].cyl.endpoint_b.y - g.pipes[p].cyl.endpoint_a.y,
                     g.pipes[p].cyl.endpoint_b.z - g.pipes[p].cyl.endpoint_a.z);
        double L = std::sqrt(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]);
        g.pipes[p].cyl.length_m = L;
        if (L > 0) g.pipes[p].cyl.axis = cv::Vec3d(ax[0]/L, ax[1]/L, ax[2]/L);
    }
    return R;
}

}  // namespace mate
