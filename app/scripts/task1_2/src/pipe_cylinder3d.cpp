// pipe_cylinder3d.cpp — RANSAC + LM 3D cylinder fit.
#include "pipe_cylinder3d.hpp"
#include "sgbm_disparity.hpp"
#include <opencv2/imgproc.hpp>
#include <random>
#include <cmath>

namespace mate {

namespace {

cv::Vec3d cross3(const cv::Vec3d& a, const cv::Vec3d& b) {
    return { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] };
}
double norm3(const cv::Vec3d& v) { return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }

// Distance from point X to infinite cylinder (axis_pt p0, axis dir d̂, radius r).
double cylDist(const cv::Point3d& X, const cv::Point3d& p0, const cv::Vec3d& d, double r) {
    cv::Vec3d v(X.x - p0.x, X.y - p0.y, X.z - p0.z);
    cv::Vec3d perp = v - d * (v.dot(d));
    return std::fabs(norm3(perp) - r);
}

bool insideLineMask(const cv::Mat& mask, const cv::Point2f& p0, const cv::Point2f& p1,
                    int x, int y, double band_px) {
    if (mask.empty()) return false;
    if (x<0||y<0||x>=mask.cols||y>=mask.rows) return false;
    if (!mask.at<uchar>(y, x)) return false;
    cv::Point2f v = p1 - p0;
    double L = std::hypot(v.x, v.y);
    if (L < 1e-3) return false;
    double t = ((x - p0.x)*v.x + (y - p0.y)*v.y) / (L*L);
    if (t < -0.05 || t > 1.05) return false;
    double d = std::fabs(((x-p0.x)*(-v.y) + (y-p0.y)*v.x) / L);
    return d <= band_px;
}

}  // namespace

Cylinder3D fitCylinderFromCloud(const PipeMatch& m,
                                const std::vector<CloudPoint>& cloud,
                                const cv::Mat& mask_left,
                                const cv::Mat& Q,
                                int img_w, int img_h,
                                const CylinderConfig& cfg) {
    Cylinder3D R;
    if (cloud.empty()) return R;

    // CloudPoint carries the original (u,v) of its parent pixel in the
    // LEFT rectified view (set by disparityToCloud). We use that directly
    // — no Q^-1 inversion needed. Saves an O(N) 4×4 matrix multiply per
    // point and avoids numerical drift when Q is near-singular.
    (void)Q;

    double band = std::max(4.0, std::hypot(m.l1.x - m.l0.x, m.l1.y - m.l0.y) * 0.04);
    std::vector<cv::Point3d> pts;
    pts.reserve(cloud.size() / 4);
    for (const auto& c : cloud) {
        if (insideLineMask(mask_left, m.l0, m.l1, c.u, c.v, band))
            pts.emplace_back(c.p.x, c.p.y, c.p.z);
    }
    (void)img_w; (void)img_h;
    if ((int)pts.size() < cfg.min_inliers) return R;

    // ── RANSAC ──
    std::mt19937 rng(0xBEEF);
    std::uniform_int_distribution<size_t> U(0, pts.size() - 1);
    int    best_in = 0;
    cv::Vec3d  best_axis;
    cv::Point3d best_p0;
    double best_r = 0;

    for (int it = 0; it < cfg.ransac_iters; ++it) {
        size_t i = U(rng), j = U(rng);
        if (i == j) continue;
        cv::Vec3d d(pts[j].x - pts[i].x, pts[j].y - pts[i].y, pts[j].z - pts[i].z);
        double L = norm3(d); if (L < 0.01) continue;
        d = d / L;

        // Project pts onto plane perpendicular to d̂ through pts[i].
        // Compute centroid of projections, then radius = mean perpendicular distance.
        cv::Vec3d sum(0,0,0);
        std::vector<cv::Vec3d> perps; perps.reserve(pts.size());
        for (auto& p : pts) {
            cv::Vec3d v(p.x-pts[i].x, p.y-pts[i].y, p.z-pts[i].z);
            cv::Vec3d perp = v - d * (v.dot(d));
            perps.push_back(perp);
            sum += perp;
        }
        cv::Vec3d c = sum * (1.0 / pts.size());
        // Recentre.
        double rsum = 0;
        for (auto& q : perps) rsum += norm3(q - c);
        double rmean = rsum / pts.size();
        if (rmean > cfg.max_radius_m) continue;

        cv::Point3d p0 = cv::Point3d(pts[i].x + c[0], pts[i].y + c[1], pts[i].z + c[2]);
        int in = 0;
        for (auto& p : pts)
            if (cylDist(p, p0, d, rmean) < cfg.ransac_inlier_tol_m) ++in;
        if (in > best_in) {
            best_in = in; best_axis = d; best_p0 = p0; best_r = rmean;
        }
    }
    if (best_in < cfg.min_inliers) return R;

    // ── Enforce gauge: re-anchor p0 to the cloud centroid projected onto
    // the plane perpendicular to the axis through itself (kills the 1-D
    // sliding freedom along d̂). All subsequent residuals are then
    // gauge-invariant.
    // Defensive: if the axis somehow has near-zero norm (degenerate
    // RANSAC outcome) we skip the re-anchor rather than producing NaNs.
    {
        double an = std::sqrt(best_axis[0]*best_axis[0] +
                              best_axis[1]*best_axis[1] +
                              best_axis[2]*best_axis[2]);
        if (an > 1e-9) {
            best_axis[0] /= an; best_axis[1] /= an; best_axis[2] /= an;
            cv::Point3d cen(0,0,0);
            for (auto& p : pts) { cen.x += p.x; cen.y += p.y; cen.z += p.z; }
            cen.x /= pts.size(); cen.y /= pts.size(); cen.z /= pts.size();
            cv::Vec3d v(cen.x - best_p0.x, cen.y - best_p0.y, cen.z - best_p0.z);
            double t = v[0]*best_axis[0] + v[1]*best_axis[1] + v[2]*best_axis[2];
            best_p0.x += t * best_axis[0];
            best_p0.y += t * best_axis[1];
            best_p0.z += t * best_axis[2];
        }
    }

    // ── LM polish (numerical Jacobian, 5 unknowns: 2 angles + 3 p0 minus 1 gauge) ──
    auto residuals = [&](const cv::Vec3d& d, const cv::Point3d& p0, double r) {
        double s = 0; int n = 0;
        for (auto& p : pts) {
            double e = cylDist(p, p0, d, r);
            double e2 = e * e;
            // Huber.
            double H = (e <= cfg.lm_huber_m) ? 0.5 * e2
                                              : cfg.lm_huber_m * (e - 0.5*cfg.lm_huber_m);
            s += H; ++n;
        }
        return s / std::max(1, n);
    };

    cv::Vec3d d = best_axis;
    cv::Point3d p0 = best_p0;
    double r = best_r;
    double prev = residuals(d, p0, r);
    double lambda = 1e-2;
    for (int it = 0; it < cfg.lm_max_iter; ++it) {
        const int N = 5;
        cv::Mat J(1, N, CV_64F);
        double base = prev;
        double h_ang = 1e-4, h_pos = 1e-4, h_r = 1e-5;
        // theta(rot d about x), phi(rot d about y), p0.x, p0.y, r
        auto rotate = [&](cv::Vec3d& v, double tx, double ty){
            // Small-angle rotation: v' ≈ v + tx·(0,0,-vy) + ... (linearised)
            cv::Vec3d v2 = v;
            v2[1] -= tx * v[2]; v2[2] += tx * v[1];
            v2[0] += ty * v[2]; v2[2] -= ty * v[0];
            double n = norm3(v2); if (n>0) v2 = v2 * (1.0/n);
            return v2;
        };
        double rs[5];
        for (int k = 0; k < N; ++k) {
            cv::Vec3d dk = d; cv::Point3d pk = p0; double rk = r;
            if (k==0) dk = rotate(d, h_ang, 0);
            else if (k==1) dk = rotate(d, 0, h_ang);
            else if (k==2) pk.x += h_pos;
            else if (k==3) pk.y += h_pos;
            else if (k==4) rk += h_r;
            rs[k] = residuals(dk, pk, rk);
        }
        double grad[5] = {
            (rs[0]-base)/h_ang, (rs[1]-base)/h_ang,
            (rs[2]-base)/h_pos, (rs[3]-base)/h_pos,
            (rs[4]-base)/h_r
        };
        // Steepest-descent step damped by lambda.
        cv::Vec3d dn = rotate(d, -lambda*grad[0], -lambda*grad[1]);
        cv::Point3d pn = p0;
        pn.x -= lambda * grad[2];
        pn.y -= lambda * grad[3];
        double rn = std::max(0.001, r - lambda * grad[4]);
        double cur = residuals(dn, pn, rn);
        if (cur < prev) {
            d = dn; p0 = pn; r = rn;
            // Relative-improvement convergence test. (Older code had a
            // broken nested-ternary version where `<` bound tighter than
            // `?:`, making the outer `if` always-true and the real check
            // dead-code-style nested. Flatten to one explicit test.)
            if ((prev - cur) / std::max(1e-12, prev) < 1e-7) {
                prev = cur;
                break;
            }
            prev = cur; lambda *= 1.5;
        } else {
            lambda *= 0.5; if (lambda < 1e-8) break;
        }
    }

    // Length & endpoints from inlier projections on axis.
    double tmin = 1e30, tmax = -1e30;
    int    in_count = 0; double sum_e2 = 0;
    for (auto& p : pts) {
        cv::Vec3d v(p.x-p0.x, p.y-p0.y, p.z-p0.z);
        double t = v.dot(d);
        cv::Vec3d perp = v - d * t;
        double e = std::fabs(norm3(perp) - r);
        if (e > cfg.ransac_inlier_tol_m * 2) continue;
        ++in_count; sum_e2 += e*e;
        if (t < tmin) tmin = t;
        if (t > tmax) tmax = t;
    }
    if (in_count < cfg.min_inliers) return R;

    R.center     = cv::Point3d(p0.x + 0.5*(tmin+tmax)*d[0],
                                p0.y + 0.5*(tmin+tmax)*d[1],
                                p0.z + 0.5*(tmin+tmax)*d[2]);
    R.axis       = d;
    R.radius_m   = r;
    R.length_m   = tmax - tmin;
    R.endpoint_a = cv::Point3d(p0.x + tmin*d[0], p0.y + tmin*d[1], p0.z + tmin*d[2]);
    R.endpoint_b = cv::Point3d(p0.x + tmax*d[0], p0.y + tmax*d[1], p0.z + tmax*d[2]);
    R.inliers    = in_count;
    R.rms_m      = std::sqrt(sum_e2 / in_count);
    R.confidence = std::min(1.0, 0.5*m.confidence
                                + 0.5*std::exp(-R.rms_m / 0.005));
    R.ok         = true;
    return R;
}

}  // namespace mate
