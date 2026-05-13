// pipe_template.cpp — parametric template + Umeyama RANSAC registration.
#include "pipe_template.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <random>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace mate {

namespace {

// ─── helpers ─────────────────────────────────────────────────────────

int findOrAddNode(std::vector<TemplateNode>& nodes,
                  const cv::Point3d& p, double tol = 1e-6) {
    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& q = nodes[i].p;
        if (std::fabs(q.x-p.x)<tol && std::fabs(q.y-p.y)<tol && std::fabs(q.z-p.z)<tol)
            return (int)i;
    }
    TemplateNode n; n.p = p; n.degree_topo = 0;
    nodes.push_back(n);
    return (int)nodes.size() - 1;
}

void addCuboidEdges(std::vector<TemplateNode>& nodes,
                    std::vector<TemplateEdge>& edges,
                    double x0, double y0, double z0,
                    double W, double H, double D) {
    // 8 corners.
    cv::Point3d c[8] = {
        {x0,   y0,   z0  }, {x0+W, y0,   z0  },
        {x0,   y0+H, z0  }, {x0+W, y0+H, z0  },
        {x0,   y0,   z0+D}, {x0+W, y0,   z0+D},
        {x0,   y0+H, z0+D}, {x0+W, y0+H, z0+D},
    };
    int id[8];
    for (int k = 0; k < 8; ++k) id[k] = findOrAddNode(nodes, c[k]);
    auto E = [&](int a, int b, char axis, double L){
        edges.push_back({id[a], id[b], axis, L});
        nodes[id[a]].degree_topo++;
        nodes[id[b]].degree_topo++;
    };
    // 4 X-edges (bottom-front, bottom-back, top-front, top-back).
    E(0,1,'X',W); E(4,5,'X',W); E(2,3,'X',W); E(6,7,'X',W);
    // 4 Y-edges (vertical posts).
    E(0,2,'Y',H); E(1,3,'Y',H); E(4,6,'Y',H); E(5,7,'Y',H);
    // 4 Z-edges (depth).
    E(0,4,'Z',D); E(1,5,'Z',D); E(2,6,'Z',D); E(3,7,'Z',D);
}

// Umeyama 1991 closed-form similarity (R, t, s) from corresponding point
// sets. Returns (R, t, s) such that y ≈ s · R·x + t minimises sum
// |y_i - (sRx_i + t)|².
bool umeyama(const std::vector<cv::Point3d>& X,
             const std::vector<cv::Point3d>& Y,
             cv::Matx33d& R, cv::Vec3d& t, double& s) {
    if (X.size() < 3 || X.size() != Y.size()) return false;
    int n = (int)X.size();
    Eigen::Matrix<double,3,Eigen::Dynamic> Xe(3, n), Ye(3, n);
    for (int i = 0; i < n; ++i) {
        Xe(0,i) = X[i].x; Xe(1,i) = X[i].y; Xe(2,i) = X[i].z;
        Ye(0,i) = Y[i].x; Ye(1,i) = Y[i].y; Ye(2,i) = Y[i].z;
    }
    Eigen::Vector3d muX = Xe.rowwise().mean();
    Eigen::Vector3d muY = Ye.rowwise().mean();
    Xe.colwise() -= muX; Ye.colwise() -= muY;
    double sigmaX = Xe.squaredNorm() / n;
    if (sigmaX < 1e-12) return false;
    Eigen::Matrix3d cov = (Ye * Xe.transpose()) / n;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Eigen::Vector3d S = svd.singularValues();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    // Reflection correction (Umeyama 1991, eq. 39). When the covariance
    // is rank-deficient (S[2] ~= 0) the smallest singular vectors of U
    // and V are arbitrary, so flipping their sign is meaningless and
    // can introduce a spurious reflection. Only apply the flip when the
    // smallest singular value is non-trivial — otherwise the data is
    // collinear/coplanar and a unique 7-DOF similarity does not exist.
    if (S(2) > 1e-9 && U.determinant() * V.determinant() < 0) D(2,2) = -1;
    // Reject collinear/coplanar configurations entirely — solving for
    // similarity would over-fit the residual axis to noise.
    if (S(2) < 1e-9 * std::max(1.0, S(0))) return false;
    Eigen::Matrix3d Rmat = U * D * V.transpose();
    double scale = (S.array() * D.diagonal().array()).sum() / sigmaX;
    Eigen::Vector3d tvec = muY - scale * Rmat * muX;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R(r,c) = Rmat(r,c);
    t = cv::Vec3d(tvec[0], tvec[1], tvec[2]);
    s = scale;
    return true;
}

cv::Point3d applyRTS(const cv::Matx33d& R, const cv::Vec3d& t, double s, const cv::Point3d& x) {
    cv::Vec3d v(x.x, x.y, x.z);
    cv::Vec3d r = R * v;
    return { s*r[0]+t[0], s*r[1]+t[1], s*r[2]+t[2] };
}

// Match each detected junction to nearest template-projected node.
double scoreFit(const std::vector<cv::Point3d>& detJ,
                const std::vector<cv::Point3d>& tmplProj,
                double tol, std::vector<int>* assign_opt = nullptr) {
    double cost = 0; int inl = 0;
    if (assign_opt) { assign_opt->assign(detJ.size(), -1); }
    for (size_t i = 0; i < detJ.size(); ++i) {
        double best = 1e30; int bestj = -1;
        for (size_t j = 0; j < tmplProj.size(); ++j) {
            double dx = detJ[i].x - tmplProj[j].x;
            double dy = detJ[i].y - tmplProj[j].y;
            double dz = detJ[i].z - tmplProj[j].z;
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < best) { best = d2; bestj = (int)j; }
        }
        if (best < tol*tol) { cost += best; ++inl; if (assign_opt) (*assign_opt)[i] = bestj; }
        else                { cost += tol*tol; }
    }
    if (inl < 3) return 1e30;
    return cost - 1.0 * inl;   // inlier-favouring objective
}

}  // anonymous

// ─── default template ────────────────────────────────────────────────

PipeTemplate makeDefault3SectionTemplate(double W_side, double W_mid,
                                          double H_side, double H_mid,
                                          double D) {
    PipeTemplate T;
    T.D = D;
    T.sections = {{W_side, H_side}, {W_mid, H_mid}, {W_side, H_side}};

    // Place the three cuboids along X axis. Y up. Sections rest on Y=0.
    // L: x ∈ [0, W_side]
    // M: x ∈ [W_side, W_side + W_mid]
    // R: x ∈ [W_side + W_mid, 2*W_side + W_mid]
    addCuboidEdges(T.nodes, T.edges,  0.0,           0.0, 0.0, W_side, H_side, D);
    addCuboidEdges(T.nodes, T.edges,  W_side,        0.0, 0.0, W_mid,  H_mid,  D);
    addCuboidEdges(T.nodes, T.edges,  W_side+W_mid,  0.0, 0.0, W_side, H_side, D);

    // Provide labels (best-effort, not used in matching but useful for debug).
    for (size_t i = 0; i < T.nodes.size(); ++i) T.nodes[i].label = "n" + std::to_string(i);
    return T;
}

// ─── fit ─────────────────────────────────────────────────────────────

TemplateFitResult fitTemplate(const PipeGraphResult& graph,
                              const PipeTemplate&   tmpl,
                              const TemplateFitConfig& cfg) {
    TemplateFitResult R;
    if (graph.junctions.size() < 3 || tmpl.nodes.size() < 3) return R;

    // Detected junctions as 3D points.
    std::vector<cv::Point3d> detJ;
    detJ.reserve(graph.junctions.size());
    for (auto& j : graph.junctions) detJ.push_back(j.position);

    // Template node positions.
    std::vector<cv::Point3d> tmplP;
    tmplP.reserve(tmpl.nodes.size());
    for (auto& n : tmpl.nodes) tmplP.push_back(n.p);

    // ── Degree-bucketed sampling ─────────────────────────────────────
    // Pure random correspondence sampling has success probability ~1/N!.
    // Instead, bucket BOTH detected junctions and template nodes by
    // topological degree (1..6) and only ever pair items from matching
    // buckets. Two endpoints of an L-bend (degree 2 each) can only map
    // to template nodes of degree 2; T-fittings (degree 3) only to
    // template T-fittings; etc. Reduces the search space from N! to
    // (B_d!)^D where B_d = bucket size at degree d.
    //
    // Detected degree comes from graph.junctions[i].degree (filled by
    // pipe_graph). Template degree comes from edge incidences.
    std::vector<int> tmplDeg(tmpl.nodes.size(), 0);
    for (auto& e : tmpl.edges) { tmplDeg[e.a]++; tmplDeg[e.b]++; }

    constexpr int MAX_DEG = 8;
    std::vector<std::vector<size_t>> bucketDet(MAX_DEG + 1);
    std::vector<std::vector<size_t>> bucketTmpl(MAX_DEG + 1);
    for (size_t i = 0; i < graph.junctions.size(); ++i) {
        int d = std::min(MAX_DEG, std::max(0, graph.junctions[i].degree));
        bucketDet[d].push_back(i);
    }
    for (size_t i = 0; i < tmpl.nodes.size(); ++i) {
        int d = std::min(MAX_DEG, std::max(0, tmplDeg[i]));
        bucketTmpl[d].push_back(i);
    }

    std::mt19937 rng(0xCAFE);

    int K = std::max(cfg.min_correspondences, 3);
    K = std::min<int>(K, std::min<int>((int)detJ.size(), (int)tmplP.size()));

    double best_cost = 1e30;
    cv::Matx33d bestR; cv::Vec3d bestT; double bestS = 1.0;

    auto pickFromBucketsMatched = [&](std::vector<size_t>& di_out,
                                       std::vector<size_t>& ti_out) -> bool {
        di_out.clear(); ti_out.clear();
        // Prefer to draw correspondences from buckets that have ≥1 entry
        // on BOTH sides. Sample degree-by-degree until we have K pairs.
        std::vector<int> validDegs;
        for (int d = 1; d <= MAX_DEG; ++d)
            if (!bucketDet[d].empty() && !bucketTmpl[d].empty())
                validDegs.push_back(d);
        if (validDegs.empty()) return false;
        std::uniform_int_distribution<size_t> Udeg(0, validDegs.size() - 1);
        int safety = 100;
        while ((int)di_out.size() < K && safety-- > 0) {
            int d = validDegs[Udeg(rng)];
            std::uniform_int_distribution<size_t> Ud(0, bucketDet[d].size()  - 1);
            std::uniform_int_distribution<size_t> Ut(0, bucketTmpl[d].size() - 1);
            size_t dv = bucketDet[d][Ud(rng)];
            size_t tv = bucketTmpl[d][Ut(rng)];
            if (std::find(di_out.begin(), di_out.end(), dv) != di_out.end()) continue;
            if (std::find(ti_out.begin(), ti_out.end(), tv) != ti_out.end()) continue;
            di_out.push_back(dv); ti_out.push_back(tv);
        }
        return (int)di_out.size() == K;
    };

    // Fall-back: if degree info is unusable (all detected degree==0 from
    // failed graph pass), revert to fully-random correspondence sampling
    // — slower but still works given enough iterations.
    bool haveDegrees = false;
    for (auto& j : graph.junctions) if (j.degree > 0) { haveDegrees = true; break; }
    std::uniform_int_distribution<size_t> Udet(0, detJ.size() - 1);
    std::uniform_int_distribution<size_t> Utmpl(0, tmplP.size() - 1);

    for (int it = 0; it < cfg.ransac_iters; ++it) {
        std::vector<size_t> di, ti;
        bool ok = false;
        if (haveDegrees) ok = pickFromBucketsMatched(di, ti);
        if (!ok) {
            di.clear(); ti.clear();
            int safety = 100;
            while ((int)di.size() < K && safety-- > 0) {
                size_t v = Udet(rng);
                if (std::find(di.begin(), di.end(), v) == di.end()) di.push_back(v);
            }
            safety = 100;
            while ((int)ti.size() < K && safety-- > 0) {
                size_t v = Utmpl(rng);
                if (std::find(ti.begin(), ti.end(), v) == ti.end()) ti.push_back(v);
            }
            if ((int)di.size() < K || (int)ti.size() < K) continue;
        }
        std::vector<cv::Point3d> X, Y;
        for (int k = 0; k < K; ++k) { X.push_back(tmplP[ti[k]]); Y.push_back(detJ[di[k]]); }
        cv::Matx33d Rh; cv::Vec3d th; double sh = 1.0;
        if (!umeyama(X, Y, Rh, th, sh)) continue;
        if (!std::isfinite(sh) || sh < 1e-3 || sh > 1e3) continue;

        std::vector<cv::Point3d> proj; proj.reserve(tmplP.size());
        for (auto& p : tmplP) proj.push_back(applyRTS(Rh, th, sh, p));
        double c = scoreFit(detJ, proj, cfg.inlier_tol_m);
        if (c < best_cost) {
            best_cost = c;
            bestR = Rh; bestT = th; bestS = sh;
        }
    }
    if (best_cost >= 1e29) return R;

    // Final score with full assignment + RMS.
    std::vector<cv::Point3d> projFinal; projFinal.reserve(tmplP.size());
    for (auto& p : tmplP) projFinal.push_back(applyRTS(bestR, bestT, bestS, p));
    std::vector<int> assign;
    scoreFit(detJ, projFinal, cfg.inlier_tol_m, &assign);

    int inl = 0; double sse = 0;
    for (size_t i = 0; i < detJ.size(); ++i) {
        if (assign[i] < 0) continue;
        ++inl;
        cv::Point3d q = projFinal[assign[i]];
        double dx = detJ[i].x - q.x, dy = detJ[i].y - q.y, dz = detJ[i].z - q.z;
        sse += dx*dx + dy*dy + dz*dz;
    }

    R.R = bestR; R.t = bestT; R.s = bestS;
    R.s_axis = {1.0, 1.0, 1.0};
    R.inliers = inl; R.total = (int)detJ.size();
    R.rms_m = inl > 0 ? std::sqrt(sse / inl) : 0.0;
    R.assignment = assign;
    R.ok = (inl >= 3);

    // Optional per-axis scale refinement: solve a 3-DOF axis-aligned
    // re-scaling that shrinks/grows the *template* under the same R, t.
    if (R.ok && cfg.allow_anisotropic_scale && inl >= 4) {
        // For each inlier pair (X_i, Y_i): Y - t = R · diag(s_axis) · X
        // ⇒ R^T (Y - t) = diag(s_axis) · X  ⇒ s_axis_k = mean(LHS_k / X_k)
        // (per axis, weighted, ignoring near-zero X to avoid blowup).
        cv::Matx33d RT = bestR.t();
        Eigen::Vector3d num = Eigen::Vector3d::Zero();
        Eigen::Vector3d den = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < detJ.size(); ++i) {
            if (assign[i] < 0) continue;
            cv::Point3d X = tmplP[assign[i]];
            cv::Point3d Y = detJ[i];
            cv::Vec3d v(Y.x - bestT[0], Y.y - bestT[1], Y.z - bestT[2]);
            cv::Vec3d lhs = RT * v;
            double xv[3] = {X.x, X.y, X.z};
            double lv[3] = {lhs[0], lhs[1], lhs[2]};
            for (int k = 0; k < 3; ++k) {
                if (std::fabs(xv[k]) < 1e-3) continue;
                num[k] += lv[k] * xv[k];
                den[k] += xv[k] * xv[k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            if (den[k] > 0) R.s_axis[k] = num[k] / den[k];
        }
        // Sanity clamp.
        for (int k = 0; k < 3; ++k) {
            if (!std::isfinite(R.s_axis[k]) || R.s_axis[k] < 0.1 || R.s_axis[k] > 10.0)
                R.s_axis[k] = 1.0;
        }
    }
    return R;
}

cv::Point3d applyTemplateFit(const TemplateFitResult& fit, const cv::Point3d& x) {
    cv::Vec3d v(x.x * fit.s_axis[0], x.y * fit.s_axis[1], x.z * fit.s_axis[2]);
    cv::Vec3d r = fit.R * v;
    return { fit.s*r[0] + fit.t[0], fit.s*r[1] + fit.t[1], fit.s*r[2] + fit.t[2] };
}

// ─── injection ───────────────────────────────────────────────────────

PipeGraphResult injectPredictedPipes(const PipeGraphResult& detected,
                                      const PipeTemplate&    tmpl,
                                      const TemplateFitResult& fit,
                                      const TemplateFitConfig& cfg,
                                      InjectionReport* report) {
    PipeGraphResult out = detected;
    if (report) {
        report->n_detected_pipes  = (int)detected.pipes.size();
        report->n_template_edges  = (int)tmpl.edges.size();
        report->n_predicted_pipes = 0;
    }
    if (!fit.ok || !cfg.inject_missing_pipes) return out;

    // Project all template nodes into world coords.
    std::vector<cv::Point3d> proj; proj.reserve(tmpl.nodes.size());
    for (auto& n : tmpl.nodes) proj.push_back(applyTemplateFit(fit, n.p));

    // For each template node, find/append a junction in the output graph.
    std::vector<int> tmpl_to_junc(tmpl.nodes.size(), -1);
    for (size_t i = 0; i < tmpl.nodes.size(); ++i) {
        // Try to reuse an existing detected junction nearby.
        int best = -1; double bestd2 = cfg.inlier_tol_m * cfg.inlier_tol_m;
        for (size_t j = 0; j < out.junctions.size(); ++j) {
            double dx = out.junctions[j].position.x - proj[i].x;
            double dy = out.junctions[j].position.y - proj[i].y;
            double dz = out.junctions[j].position.z - proj[i].z;
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < bestd2) { bestd2 = d2; best = (int)j; }
        }
        if (best >= 0) tmpl_to_junc[i] = best;
        else {
            GraphJunction gj; gj.position = proj[i]; gj.degree = 0;
            out.junctions.push_back(gj);
            tmpl_to_junc[i] = (int)out.junctions.size() - 1;
        }
    }

    // For each template edge, check if an equivalent detected pipe exists;
    // if not, synthesise a predicted pipe.
    auto edgeMatchesAny = [&](int ja, int jb) {
        for (auto& p : out.pipes) {
            if ((p.junction_a == ja && p.junction_b == jb) ||
                (p.junction_a == jb && p.junction_b == ja)) return true;
        }
        return false;
    };

    for (auto& e : tmpl.edges) {
        int ja = tmpl_to_junc[e.a], jb = tmpl_to_junc[e.b];
        if (ja < 0 || jb < 0) continue;
        if (edgeMatchesAny(ja, jb)) continue;

        // Synthesise.
        GraphPipe gp;
        gp.cyl.endpoint_a = out.junctions[ja].position;
        gp.cyl.endpoint_b = out.junctions[jb].position;
        gp.cyl.center = cv::Point3d(
            0.5*(gp.cyl.endpoint_a.x + gp.cyl.endpoint_b.x),
            0.5*(gp.cyl.endpoint_a.y + gp.cyl.endpoint_b.y),
            0.5*(gp.cyl.endpoint_a.z + gp.cyl.endpoint_b.z));
        cv::Vec3d ax(gp.cyl.endpoint_b.x - gp.cyl.endpoint_a.x,
                     gp.cyl.endpoint_b.y - gp.cyl.endpoint_a.y,
                     gp.cyl.endpoint_b.z - gp.cyl.endpoint_a.z);
        double L = std::sqrt(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]);
        gp.cyl.length_m = L;
        if (L > 1e-9) gp.cyl.axis = cv::Vec3d(ax[0]/L, ax[1]/L, ax[2]/L);
        // Estimate radius from neighbouring detected pipes (median).
        std::vector<double> rs;
        for (auto& p : detected.pipes) if (p.cyl.radius_m > 0) rs.push_back(p.cyl.radius_m);
        double r_est = 0.018;
        if (!rs.empty()) {
            std::nth_element(rs.begin(), rs.begin()+rs.size()/2, rs.end());
            r_est = rs[rs.size()/2];
        }
        gp.cyl.radius_m = r_est;
        gp.cyl.confidence = cfg.missing_pipe_confidence;
        gp.cyl.ok = true;
        gp.junction_a = ja;
        gp.junction_b = jb;
        out.pipes.push_back(gp);
        out.junctions[ja].pipe_indices.push_back((int)out.pipes.size() - 1);
        out.junctions[ja].degree++;
        out.junctions[jb].pipe_indices.push_back((int)out.pipes.size() - 1);
        out.junctions[jb].degree++;
        if (report) report->n_predicted_pipes++;
    }
    return out;
}

}  // namespace mate
